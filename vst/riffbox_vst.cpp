/* =============================================================================
 * riffbox_vst.cpp - RiffBox: an electric guitar for the MPC OS plugin host (Force, MPC
 * Live/One/X/Key) that plays a whole chord per note - power chords, octaves, barre chords -
 * through five amp models, with aftertouch and a tempo delay. VST2 instrument, armhf.
 *
 * The strings are extended Karplus-Strong (the principle of DISTRHO Kars / Chris Cannam's
 * karplong; Karplus & Strong 1983, Jaffe & Smith 1983), written new for this plugin: cubic-
 * interpolated delay lines so bends and vibrato stay in tune, a pick-position comb on the
 * excitation, per-string decay, palm mute, a feedback "bloom" on aftertouch.
 *
 *   note on   the chord shape on the played root, strummed low->high (DOWN), high->low (UP)
 *             or alternating (ALT); one guitar: a new chord takes over the strings
 *   note off  the strings are damped (RELEASE); the sustain pedal holds them
 *   pressure  channel or poly aftertouch -> vibrato, bend (up to a whole tone), feedback
 *             (endless sustain, blooming to the octave), wah, extra gain - each with an amount
 *   bend/mod  pitch bend +-2 semitones, mod wheel -> vibrato
 *   amp       CLEAN, CLASSIC ROCK, BRIT GARAGE, FUZZ, STONER: pre-shaping, 2x oversampled
 *             clipping, tone stack (bass/mid/treble), cabinet; DOUBLE adds a second, detuned
 *             and later strum through its own amp, panned apart
 *   pedal     a RAT in front of the amp (rat_core.h): RAT, TURBO (LEDs), GE, RUETZ
 *   delay     tempo-synced from the host (1/16 .. 1/2, dotted, triplet), ping-pong
 *
 * MIT license (see ../LICENSE).
 * ========================================================================== */
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "params.h"
#include "popup.h"    /* mpc-vst-plugins wrapper/popup.h, copied into build/ by build.sh */
#include "rat_core.h" /* the RAT pedal (same model as mpc-vst-rat) */

/* ---- VST2 ABI (hand-written; no Steinberg SDK) ---------------------------- */
struct AEffect;
typedef intptr_t (*audioMasterCallback)(AEffect *, int32_t, int32_t, intptr_t, void *, float);
struct AEffect {
    int32_t magic;
    intptr_t (*dispatcher)(AEffect *, int32_t, int32_t, intptr_t, void *, float);
    void (*process)(AEffect *, float **, float **, int32_t);
    void (*setParameter)(AEffect *, int32_t, float);
    float (*getParameter)(AEffect *, int32_t);
    int32_t numPrograms, numParams, numInputs, numOutputs, flags;
    intptr_t resvd1, resvd2;
    int32_t initialDelay, realQualities, offQualities;
    float ioRatio;
    void *object, *user;
    int32_t uniqueID, version;
    void (*processReplacing)(AEffect *, float **, float **, int32_t);
    void (*processDoubleReplacing)(AEffect *, double **, double **, int32_t);
    char future[56];
};
typedef struct { int32_t type, byteSize, deltaFrames, flags; char data[16]; } VstEvent;
typedef struct {
    int32_t type, byteSize, deltaFrames, flags, noteLength, noteOffset;
    unsigned char midiData[4];
    char detune, noteOffVelocity, reserved1, reserved2;
} VstMidiEvent;
typedef struct { int32_t numEvents; intptr_t reserved; VstEvent *events[2]; } VstEvents;

enum {
    effOpen = 0, effClose = 1, effGetParamLabel = 6, effGetParamDisplay = 7, effGetParamName = 8,
    effSetSampleRate = 10, effSetBlockSize = 11, effMainsChanged = 12, effGetChunk = 23,
    effSetChunk = 24, effProcessEvents = 25, effCanBeAutomated = 26, effGetPlugCategory = 35,
    effGetEffectName = 45, effGetVendorString = 47, effGetProductString = 48,
    effGetVendorVersion = 49, effCanDo = 51, effGetVstVersion = 58,
};
enum { audioMasterAutomate = 0, audioMasterGetTime = 7, audioMasterUpdateDisplay = 42 };
enum { kVstTransportPlaying = 1 << 1, kVstPpqPosValid = 1 << 9, kVstTempoValid = 1 << 10, kVstTimeSigValid = 1 << 13 };
typedef struct {
    double samplePos, sampleRate, nanoSeconds, ppqPos, tempo, barStartPos, cycleStartPos, cycleEndPos;
    int32_t timeSigNumerator, timeSigDenominator, smpteOffset, smpteFrameRate, samplesToNextClock, flags;
} VstTimeInfo;
enum { effFlagsCanReplacing = 1 << 4, effFlagsProgramChunks = 1 << 5, effFlagsIsSynth = 1 << 8 };

static FILE *g_log;
static std::mutex g_logMutex;
#define LOG(...) do { std::lock_guard<std::mutex> lk_(g_logMutex); \
    if (g_log) { std::fprintf(g_log, __VA_ARGS__); std::fflush(g_log); } } while (0)

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }
static float db2lin(float db) { return std::pow(10.0f, db / 20.0f); }

/* ---- filters (RBJ cookbook) ----------------------------------------------------- */
struct Biquad {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    inline float run(float x) { float y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
    void set(double B0, double B1, double B2, double A0, double A1, double A2) {
        b0 = (float)(B0 / A0); b1 = (float)(B1 / A0); b2 = (float)(B2 / A0); a1 = (float)(A1 / A0); a2 = (float)(A2 / A0);
    }
    void lowpass(double f, double q, double sr) {
        double w = 2 * M_PI * std::min(f, sr * 0.49) / sr, al = std::sin(w) / (2 * q), c = std::cos(w);
        set((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
    }
    void highpass(double f, double q, double sr) {
        double w = 2 * M_PI * f / sr, al = std::sin(w) / (2 * q), c = std::cos(w);
        set((1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al);
    }
    void bandpass(double f, double q, double sr) {   /* 0 dB peak */
        double w = 2 * M_PI * std::min(f, sr * 0.45) / sr, al = std::sin(w) / (2 * q), c = std::cos(w);
        set(al, 0, -al, 1 + al, -2 * c, 1 - al);
    }
    void peak(double f, double q, double db, double sr) {
        double A = std::pow(10, db / 40), w = 2 * M_PI * std::min(f, sr * 0.45) / sr, al = std::sin(w) / (2 * q), c = std::cos(w);
        set(1 + al * A, -2 * c, 1 - al * A, 1 + al / A, -2 * c, 1 - al / A);
    }
    void lowshelf(double f, double db, double sr) {
        double A = std::pow(10, db / 40), w = 2 * M_PI * f / sr, c = std::cos(w), s = std::sin(w), al = s / 2 * std::sqrt(2.0), r = 2 * std::sqrt(A) * al;
        set(A * ((A + 1) - (A - 1) * c + r), 2 * A * ((A - 1) - (A + 1) * c), A * ((A + 1) - (A - 1) * c - r),
            (A + 1) + (A - 1) * c + r, -2 * ((A - 1) + (A + 1) * c), (A + 1) + (A - 1) * c - r);
    }
    void highshelf(double f, double db, double sr) {
        double A = std::pow(10, db / 40), w = 2 * M_PI * std::min(f, sr * 0.45) / sr, c = std::cos(w), s = std::sin(w), al = s / 2 * std::sqrt(2.0), r = 2 * std::sqrt(A) * al;
        set(A * ((A + 1) + (A - 1) * c + r), -2 * A * ((A - 1) + (A + 1) * c), A * ((A + 1) + (A - 1) * c - r),
            (A + 1) - (A - 1) * c + r, 2 * ((A - 1) - (A + 1) * c), (A + 1) - (A - 1) * c - r);
    }
};

/* tanh, rational approximation (exact enough for a clipper, cheap on ARM) */
static inline float soft(float x) {
    x = clampf(x, -3.0f, 3.0f);
    const float x2 = x * x;
    return x * (27 + x2) / (27 + 9 * x2);
}
/* a harder knee: cubic soft clip, flat beyond +-1 */
static inline float hard(float x) {
    if (x > 1) return 2.0f / 3.0f;
    if (x < -1) return -2.0f / 3.0f;
    return x - x * x * x / 3.0f;
}

/* ---- chord shapes: semitones above the root, one per string, low to high (E-shape barre) ---- */
static const int SHAPES[11][6] = {
    {0, 7, -1, -1, -1, -1},      /* POWER 5 */
    {0, 7, 12, -1, -1, -1},      /* POWER 8 */
    {0, 12, 24, -1, -1, -1},     /* OCTAVES */
    {0, 7, 12, 16, 19, 24},      /* MAJOR */
    {0, 7, 12, 15, 19, 24},      /* MINOR */
    {0, 7, 12, 14, 19, 24},      /* SUS2 */
    {0, 7, 12, 17, 19, 24},      /* SUS4 */
    {0, 7, 10, 16, 19, 24},      /* DOM7 */
    {0, 7, 10, 15, 19, 24},      /* MIN7 */
    {0, 7, 11, 16, 19, 24},      /* MAJ7 */
    {0, 7, 12, 16, 19, 26},      /* ADD9 */
};

/* ---- one string --------------------------------------------------------------------- */
static const int SBUF = 8192;              /* E1 at 96 kHz fits */
struct String {
    float buf[SBUF];
    int w = 0;
    bool active = false;
    float freq = 110, delay = 400;           /* loop delay in samples (before the filter's) */
    float S = 0.3f;                          /* loop filter: y = (1-S) x + S x[-1] */
    float g = 0.999f, g_target = 0.999f;     /* loop gain (decay), smoothed toward the target */
    float xm1 = 0, env = 0;
    int wait = 0;                            /* strum: samples until the pluck */
    float vel = 1, pick = 0.13f, tone = 0.5f;
    Biquad harm;                             /* feedback bloom: the octave of the string */
    float harm_f = 0;
    uint32_t seed = 1;
    inline float noise() { seed = seed * 1664525u + 1013904223u; return (int32_t)seed * (1.0f / 2147483648.0f); }
};

/* ---- the amp ---------------------------------------------------------------------- */
enum { CLEAN, CLASSIC, BRIT, FUZZ, STONER };
struct AmpModel {
    float pre_hp, pre_lp, pre_pk_f, pre_pk_db;       /* input shaping */
    float drive_lo, drive_hi;                        /* gain knob 0..100 -> linear drive */
    int stages; bool hardclip; float asym;
    float post_lp;                                   /* after the clipper (fuzz fizz) */
    float bass, mid, treble;                         /* the stack's own voicing, dB */
    float cab_hp, cab_lo_f, cab_lo_db, cab_pk_f, cab_pk_db, cab_lp;
    float level;                                     /* output trim, measured (host_test) */
};
static const AmpModel MODELS[5] = {
    /* CLEAN */        {70, 9000, 1000, 0,    1.0f, 6.0f,   1, false, 0.00f, 12000,  2,  0,  2,  70, 120, 1, 2800, 2, 7500, 3.85f},
    /* CLASSIC ROCK */ {140, 7000, 900, 4,    4.0f, 70.0f,  2, false, 0.10f, 9000,   0,  2,  1,  80, 120, 3, 2200, 4, 5200, 0.254f},
    /* BRIT GARAGE */  {100, 9000, 2600, 6,   2.0f, 30.0f,  1, false, 0.25f, 10000,  -1, 1,  3,  90, 140, 1, 3000, 3, 6000, 0.608f},
    /* FUZZ */         {40, 4000, 800, 0,    30.0f, 400.0f, 2, true,  0.35f, 3500,   1, -2,  0,  70, 110, 2, 1800, 3, 4600, 0.392f},
    /* STONER */       {50, 6000, 700, 0,    40.0f, 500.0f, 2, true,  0.15f, 5000,   6, -9,  1,  60,  90, 4, 1500, 2, 3800, 0.533f},
};
static const char *const MODEL_NAMES[5] = {"clean", "classic rock", "brit garage", "fuzz", "stoner"};

struct Amp {
    Biquad pre_hp, pre_lp, pre_pk, wah;
    Biquad up1, up2, dn1, dn2;                       /* 2x oversampling around the clipper */
    Biquad post_lp, dc, bass, mid, treble, cab_hp, cab_lo, cab_pk, cab_lp1, cab_lp2;
    const AmpModel *m = &MODELS[1];
    float drive = 1, out = 1;
    void configure(int model, float gain01, float b, float md, float tr, float sr) {
        m = &MODELS[clampi(model, 0, 4)];
        pre_hp.highpass(m->pre_hp, 0.7, sr);
        pre_lp.lowpass(m->pre_lp, 0.7, sr);
        pre_pk.peak(m->pre_pk_f, 0.8, m->pre_pk_db, sr);
        drive = m->drive_lo * std::pow(m->drive_hi / m->drive_lo, gain01);
        up1.lowpass(0.45 * sr, 0.54, 2 * sr); up2.lowpass(0.45 * sr, 1.31, 2 * sr);
        dn1.lowpass(0.45 * sr, 0.54, 2 * sr); dn2.lowpass(0.45 * sr, 1.31, 2 * sr);
        post_lp.lowpass(m->post_lp, 0.7, sr);
        dc.highpass(20, 0.7, sr);
        bass.lowshelf(120, m->bass + b, sr);
        mid.peak(700, 0.8, m->mid + md, sr);
        treble.highshelf(3000, m->treble + tr, sr);
        cab_hp.highpass(m->cab_hp, 0.7, sr);
        cab_lo.peak(m->cab_lo_f, 1.2, m->cab_lo_db, sr);
        cab_pk.peak(m->cab_pk_f, 1.0, m->cab_pk_db, sr);
        cab_lp1.lowpass(m->cab_lp, 0.54, sr); cab_lp2.lowpass(m->cab_lp, 1.31, sr);
        /* the clipper's output sits near 0.7 whatever the drive: a clean amp gets quieter
         * with less gain, as a real one does */
        out = m->level * (m->hardclip || m->stages > 1 ? 1.0f : std::min(1.0f, 0.35f + drive / 8.0f));
    }
    inline float clip(float x) const {
        if (m->hardclip) {
            float y = hard(x + m->asym) - hard(m->asym);
            if (m->stages > 1) y = hard(3.0f * y + m->asym * 0.5f) - hard(m->asym * 0.5f);
            return y;
        }
        float y = soft(x + m->asym) - soft(m->asym);
        if (m->stages > 1) y = soft(2.5f * y + m->asym * 0.5f) - soft(m->asym * 0.5f);
        return y;
    }
    /* wah01 < 0: no wah; drive_mul: aftertouch gain */
    inline float process(float x, float wah_mix, float drive_mul) {
        x = pre_hp.run(x);
        if (wah_mix > 0) x = x * (1 - wah_mix) + wah.run(x) * 3.0f * wah_mix;
        x = pre_pk.run(pre_lp.run(x)) * drive * drive_mul;
        /* 2x: x, 0 through the anti-imaging filter, clip both, back down */
        float a = up2.run(up1.run(2 * x)), b = up2.run(up1.run(0.0f));
        a = dn2.run(dn1.run(clip(a)));
        b = dn2.run(dn1.run(clip(b)));
        (void)b;
        float y = dc.run(post_lp.run(a));
        y = treble.run(mid.run(bass.run(y)));
        y = cab_lp2.run(cab_lp1.run(cab_pk.run(cab_lo.run(cab_hp.run(y)))));
        return y * out;
    }
};

/* ---- the plugin ----------------------------------------------------------------- */
static const int NSTR = 12;                  /* per guitar: 6 sounding + 6 dying from the last chord */
static const double DLY_BEATS[7] = {0.25, 1.0 / 3.0, 0.5, 0.75, 1.0, 1.5, 2.0};

struct Ev { int32_t delta; uint8_t b[3]; };

struct Plugin {
    AEffect fx;
    audioMasterCallback master = nullptr;
    std::atomic<float> cache[NPARAMS];
    std::atomic<int> notify[NPARAMS];
    float open[NPARAMS] = {0};
    volatile int release[NPARAMS] = {0};
    float sr = 44100;
    String str[2][NSTR];                     /* [guitar][slot] - guitar 1 only with DOUBLE */
    Amp amp[2];
    rat::Core ped[2];
    std::atomic<bool> amp_dirty{true};
    /* performance state (audio thread) */
    int cur_note = -1;                       /* the note whose chord sounds */
    bool held = false, pedal = false;
    bool next_up = false;                    /* ALT: the next stroke */
    float at = 0, at_s = 0;                  /* aftertouch 0..1, smoothed */
    float bend = 0;                          /* pitch bend, semitones */
    float modw = 0;
    double lfo = 0;
    /* delay */
    std::vector<float> dl, dr;
    int dw = 0;
    float dtime = 0;                         /* samples, slewed */
    Biquad dlp[2], dhp[2];
    Ev evq[256];
    int ev_n = 0;
    std::mutex evMutex;
    std::vector<uint8_t> chunk;
    std::mt19937 rng;
};

static int IDX_PEDAL, IDX_PED_DIST, IDX_PED_FILTER, IDX_PED_LEVEL;
static int IDX_SHAPE, IDX_OCT, IDX_STRUM, IDX_STROKE, IDX_MUTE, IDX_SUS, IDX_BRIGHT, IDX_REL, IDX_DOUBLE,
    IDX_AMP, IDX_GAIN, IDX_BASS, IDX_MID, IDX_TREB, IDX_VOL, IDX_AT_VIB, IDX_AT_BEND, IDX_AT_FB, IDX_AT_WAH,
    IDX_AT_GAIN, IDX_DLY_TIME, IDX_DLY_FB, IDX_DLY_MIX, IDX_DLY_PP;

static int param_index(const char *key) {
    for (int i = 0; i < NPARAMS; i++) if (!std::strcmp(PARAMS[i].key, key)) return i;
    return -1;
}
static float clamp01(float v) { return v < 0 ? 0 : v > 1 ? 1 : v; }
static void copy_str(void *dst, const char *s, size_t max) {
    std::strncpy((char *)dst, s, max - 1);
    ((char *)dst)[max - 1] = 0;
}
static int norm_to_ui(const param_t *p, float n) {
    if (p->nopts) return (int)std::lround(clamp01(n) * (p->nopts - 1));
    return (int)std::lround(p->min + (p->max - p->min) * clamp01(n));
}
static float ui_to_norm(const param_t *p, double v) {
    if (p->nopts) return p->nopts > 1 ? clamp01((float)(v / (p->nopts - 1))) : 0.0f;
    return p->max > p->min ? clamp01((float)((v - p->min) / (p->max - p->min))) : 0.0f;
}
static int ui(Plugin *w, int i) { return i < 0 ? 0 : norm_to_ui(&PARAMS[i], w->cache[i].load()); }
static float u01(Plugin *w, int i) { return ui(w, i) / 100.0f; }
static void set_ui(Plugin *w, int i, int v) {
    if (i < 0) return;
    w->cache[i].store(ui_to_norm(&PARAMS[i], v));
    w->notify[i].store(1);
}

/* ---- strings ----------------------------------------------------------------------- */
/* decay (T60, s) of a string at freq: SUSTAIN, shortened by palm mute, high strings shorter */
static float t60(Plugin *w, float freq) {
    float t = 0.8f + 11.0f * u01(w, IDX_SUS) * u01(w, IDX_SUS);
    t *= std::pow(110.0f / std::max(freq, 30.0f), 0.5f);
    const float m = u01(w, IDX_MUTE);
    return t * (1 - m) + 0.10f * m;
}
static float loop_gain(float freq, float T, float sr) {
    (void)sr;
    return std::pow(10.0f, -3.0f / (freq * std::max(T, 0.01f)));   /* per period */
}
static void damp(Plugin *w, String &s, float T) {
    s.g_target = loop_gain(s.freq, T, w->sr);
    s.S = std::max(s.S, 0.42f);
}

/* Pluck string s at freq; its old vibration is replaced (the excitation adds to what is left
 * after the damping the new chord gave it). */
static void pluck(Plugin *w, String &s, float freq, float vel) {
    s.freq = freq;
    s.delay = w->sr / freq - s.S;
    const float mute = u01(w, IDX_MUTE), bright = u01(w, IDX_BRIGHT);
    s.S = 0.12f + 0.30f * (1 - bright) + 0.08f * mute;
    s.delay = std::max(4.0f, w->sr / freq - s.S);
    s.g = s.g_target = loop_gain(freq, t60(w, freq), w->sr);
    s.vel = vel;
    s.env = 1;
    s.active = true;
    /* excitation over one period: filtered noise, a pick-position comb, no DC */
    const int D = std::min((int)s.delay, SBUF - 4);
    std::vector<float> e((size_t)D);
    const float lp = 0.15f + 0.8f * bright * (0.4f + 0.6f * vel) * (1 - 0.7f * mute);
    float z = 0, mean = 0;
    for (int i = 0; i < D; i++) { z += lp * (s.noise() - z); e[(size_t)i] = z; }
    const int P = std::max(1, (int)(s.pick * D));
    for (int i = D - 1; i >= P; i--) e[(size_t)i] -= e[(size_t)(i - P)];
    for (int i = 0; i < D; i++) mean += e[(size_t)i];
    mean /= D;
    float peak = 1e-6f;
    for (int i = 0; i < D; i++) { e[(size_t)i] -= mean; peak = std::max(peak, std::fabs(e[(size_t)i])); }
    const float amp = vel / peak;
    for (int i = 0; i < D; i++) s.buf[(s.w - D + i) & (SBUF - 1)] += e[(size_t)i] * amp;
    s.harm.bandpass(2 * freq, 8, w->sr);
    s.harm_f = 2 * freq;
}

/* A chord on root note: takes over the guitar's strings, strummed */
static void chord(Plugin *w, int note, int vel) {
    const int shape = clampi(ui(w, IDX_SHAPE), 0, 10);
    const int root = note + 12 * ui(w, IDX_OCT);
    const int stroke = ui(w, IDX_STROKE);
    bool up = stroke == 1 || (stroke == 2 && w->next_up);
    if (stroke == 2) w->next_up = !w->next_up;
    const float strum = ui(w, IDX_STRUM) / 1000.0f * w->sr;
    int ints[6], n = 0;
    for (int k = 0; k < 6; k++) if (SHAPES[shape][k] >= 0) ints[n++] = SHAPES[shape][k];
    const bool dbl = ui(w, IDX_DOUBLE) != 0;
    std::uniform_real_distribution<float> jit(0.9f, 1.1f);
    for (int g = 0; g < (dbl ? 2 : 1); g++) {
        /* the last chord's strings: damped, they die over ~50 ms */
        for (auto &s : w->str[g]) if (s.active) damp(w, s, 0.05f);
        for (int k = 0; k < n; k++) {
            /* a free slot, else the quietest */
            int best = 0;
            float be = 1e9f;
            for (int j = 0; j < NSTR; j++) {
                const String &s = w->str[g][j];
                const float e = s.active ? s.env + (s.wait > 0 ? 10 : 0) : -1;
                if (e < be) { be = e; best = j; }
            }
            String &s = w->str[g][best];
            const int order = up ? n - 1 - k : k;
            const float detune = g ? std::pow(2.0f, 7.0f / 1200.0f) : 1.0f;
            const float f = 440.0f * std::pow(2.0f, (root + ints[k] - 69) / 12.0f) * detune;
            s.wait = (int)(order * strum) + (g ? (int)(0.012f * w->sr) : 0) + 1;
            s.freq = f;
            s.vel = clampf(vel / 127.0f * jit(w->rng) * (up ? 0.85f : 1.0f), 0.05f, 1.0f);
            s.active = true;
            s.env = 1;
        }
    }
    w->cur_note = note;
    w->held = true;
}
static void release_chord(Plugin *w) {
    const float T = 0.02f + 1.5f * u01(w, IDX_REL) * u01(w, IDX_REL);
    for (auto &gs : w->str)
        for (auto &s : gs) if (s.active) { if (s.wait > 0) s.active = false; else damp(w, s, T); }
    w->held = false;
}

static void handle(Plugin *w, const uint8_t *b) {
    const uint8_t t = b[0] & 0xF0;
    if (t == 0x90 && b[2] > 0) { chord(w, b[1], b[2]); return; }
    if (t == 0x80 || (t == 0x90 && b[2] == 0)) {
        if (b[1] == w->cur_note) { w->cur_note = -1; if (!w->pedal) release_chord(w); else w->held = false; }
        return;
    }
    if (t == 0xD0) { w->at = b[1] / 127.0f; return; }
    if (t == 0xA0) { if (b[1] == w->cur_note || w->cur_note < 0) w->at = b[2] / 127.0f; return; }
    if (t == 0xE0) { w->bend = ((b[2] << 7 | b[1]) - 8192) / 8192.0f * 2.0f; return; }
    if (t == 0xB0) {
        if (b[1] == 1) w->modw = b[2] / 127.0f;
        else if (b[1] == 64) {
            w->pedal = b[2] >= 64;
            if (!w->pedal && w->cur_note < 0 && w->held == false) release_chord(w);
        } else if (b[1] == 120 || b[1] == 123) { for (auto &gs : w->str) for (auto &s : gs) damp(w, s, 0.02f); w->cur_note = -1; }
    }
}

/* ---- audio ---------------------------------------------------------------------- */
static void render(Plugin *w, float *L, float *R, int n) {
    if (w->amp_dirty.exchange(false)) {
        for (auto &a : w->amp)
            a.configure(ui(w, IDX_AMP), u01(w, IDX_GAIN), (float)ui(w, IDX_BASS), (float)ui(w, IDX_MID), (float)ui(w, IDX_TREB), w->sr);
        const int pm = ui(w, IDX_PEDAL);
        for (auto &p : w->ped) {
            p.dist = u01(w, IDX_PED_DIST); p.filter = u01(w, IDX_PED_FILTER); p.volume = u01(w, IDX_PED_LEVEL);
            p.diodes = pm == 2 ? rat::LED : pm == 3 ? rat::GERMANIUM : rat::SILICON;
            p.ruetz = pm == 4;
            p.configure();
        }
    }
    const bool pedal = ui(w, IDX_PEDAL) != 0;
    const bool dbl = ui(w, IDX_DOUBLE) != 0;
    const float vib_amt = u01(w, IDX_AT_VIB), bend_amt = u01(w, IDX_AT_BEND), fb_amt = u01(w, IDX_AT_FB),
                wah_amt = u01(w, IDX_AT_WAH), gain_amt = u01(w, IDX_AT_GAIN);
    const float vol = db2lin(-30.0f + 36.0f * u01(w, IDX_VOL));
    const float as = 1 - std::exp(-1.0f / (0.02f * w->sr));
    const double lfo_step = 2 * M_PI * 5.5 / w->sr;
    /* control rate: per 32 samples */
    for (int i0 = 0; i0 < n; i0 += 32) {
        const int m = std::min(32, n - i0);
        w->at_s += (w->at - w->at_s) * std::min(1.0f, as * m);
        const float at = w->at_s;
        const float vib_c = 50.0f * std::max(at * vib_amt, w->modw);           /* cents */
        const float lfo = (float)std::sin(w->lfo);
        w->lfo += lfo_step * m;
        if (w->lfo > 2 * M_PI) w->lfo -= 2 * M_PI;
        const float semis = w->bend + 2.0f * at * bend_amt + vib_c * lfo / 100.0f;
        const float ratio = std::pow(2.0f, semis / 12.0f);
        const float fb = at * fb_amt;
        const float wah = at * wah_amt;
        const float drive_mul = 1 + 3 * at * gain_amt;
        if (wah > 0) for (auto &a : w->amp) a.wah.bandpass(450.0f * std::pow(2200.0f / 450.0f, at), 3.0, w->sr);
        float gin[2][32] = {{0}};
        for (int g = 0; g < (dbl ? 2 : 1); g++) {
            for (auto &s : w->str[g]) {
                if (!s.active) continue;
                float *o = gin[g];
                for (int i = 0; i < m; i++) {
                    if (s.wait > 0 && --s.wait == 0) pluck(w, s, s.freq, s.vel);
                    if (s.wait > 0) continue;
                    /* sustain with feedback: the loop gain goes to 1, the octave grows */
                    const float g_eff = s.g + (0.99995f - s.g) * fb * (w->held || w->pedal ? 1.0f : 0.0f);
                    const float d = std::min(std::max(2.0f, s.delay / ratio), (float)(SBUF - 4));
                    const float rp = (float)s.w - d;
                    const int ip = (int)std::floor(rp);
                    const float fr = rp - ip;
                    const float y0 = s.buf[(ip - 1) & (SBUF - 1)], y1 = s.buf[ip & (SBUF - 1)],
                                y2 = s.buf[(ip + 1) & (SBUF - 1)], y3 = s.buf[(ip + 2) & (SBUF - 1)];
                    const float c1 = 0.5f * (y2 - y0), c2 = y0 - 2.5f * y1 + 2 * y2 - 0.5f * y3, c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
                    const float x = ((c3 * fr + c2) * fr + c1) * fr + y1;
                    float y = ((1 - s.S) * x + s.S * s.xm1) * g_eff;
                    s.xm1 = x;
                    if (fb > 0.001f) {
                        y += fb * 0.004f * s.harm.run(x);
                        y += fb * (soft(y) - y);           /* keeps the endless note bounded */
                    }
                    s.buf[s.w & (SBUF - 1)] = y;
                    s.w++;
                    o[i] += x;
                    s.env += 0.002f * (std::fabs(x) - s.env);
                }
                s.g += (s.g_target - s.g) * 0.2f;
                if (s.wait <= 0 && s.env < 1e-5f) { s.active = false; std::memset(s.buf, 0, sizeof s.buf); s.xm1 = 0; }
            }
        }
        /* strings -> wah -> pedal -> amp */
        auto chain = [&](int g, float x) {
            x *= 0.3f;
            if (wah > 0) x = x * (1 - wah) + w->amp[g].wah.run(x) * 3.0f * wah;
            if (pedal) x = w->ped[g].process(x);
            return w->amp[g].process(x, 0.0f, drive_mul);
        };
        for (int i = 0; i < m; i++) {
            const float a = chain(0, gin[0][i]);
            if (dbl) {
                const float b = chain(1, gin[1][i]);
                L[i0 + i] = (0.85f * a + 0.15f * b) * vol;
                R[i0 + i] = (0.15f * a + 0.85f * b) * vol;
            } else {
                L[i0 + i] = R[i0 + i] = a * vol;
            }
        }
    }
}

/* tempo delay after the amp */
static void delay(Plugin *w, float *L, float *R, int n, double tempo) {
    const float mix = u01(w, IDX_DLY_MIX);
    if (mix <= 0 && w->dtime <= 0) return;
    const int size = (int)w->dl.size();
    const float target = std::min((float)(DLY_BEATS[clampi(ui(w, IDX_DLY_TIME), 0, 6)] * 60.0 / tempo * w->sr), (float)(size - 4));
    if (w->dtime <= 0) w->dtime = target;
    const float fbk = ui(w, IDX_DLY_FB) / 100.0f;
    const bool pp = ui(w, IDX_DLY_PP) != 0;
    for (int i = 0; i < n; i++) {
        w->dtime += (target - w->dtime) * 0.0005f;      /* glide on tempo/time changes */
        float rp = (float)w->dw - w->dtime;
        int ip = (int)std::floor(rp);
        float fr = rp - ip;
        auto rd = [&](const std::vector<float> &b) {
            return b[(size_t)((ip % size + size) % size)] * (1 - fr) + b[(size_t)(((ip + 1) % size + size) % size)] * fr;
        };
        const float el = rd(w->dl), er = rd(w->dr);
        const float in = 0.5f * (L[i] + R[i]);
        float nl, nr;
        if (pp) { nl = in + fbk * er; nr = fbk * el; }
        else { nl = L[i] + fbk * el; nr = R[i] + fbk * er; }
        w->dl[(size_t)w->dw] = w->dhp[0].run(w->dlp[0].run(nl));
        w->dr[(size_t)w->dw] = w->dhp[1].run(w->dlp[1].run(nr));
        w->dw = (w->dw + 1) % size;
        L[i] += mix * el;
        R[i] += mix * er;
    }
}

struct NoDenormals {   /* filter tails decaying into denormals cost a lot on ARM */
#if defined(__arm__) && defined(__ARM_FP)
    uint32_t old = 0;
    NoDenormals() { asm volatile("vmrs %0, fpscr" : "=r"(old)); asm volatile("vmsr fpscr, %0" : : "r"(old | (1u << 24))); }
    ~NoDenormals() { asm volatile("vmsr fpscr, %0" : : "r"(old)); }
#elif defined(__x86_64__) || defined(__i386__)
    unsigned old = __builtin_ia32_stmxcsr();
    NoDenormals() { __builtin_ia32_ldmxcsr(old | 0x8040); }
    ~NoDenormals() { __builtin_ia32_ldmxcsr(old); }
#endif
};

static void processReplacing(AEffect *e, float **in, float **out, int32_t n) {
    (void)in;
    Plugin *w = (Plugin *)e->object;
    NoDenormals nd;
    Ev q[256];
    int qn;
    {
        std::lock_guard<std::mutex> lk(w->evMutex);
        qn = w->ev_n;
        std::memcpy(q, w->evq, sizeof(Ev) * (size_t)qn);
        w->ev_n = 0;
    }
    std::stable_sort(q, q + qn, [](const Ev &a, const Ev &b) { return a.delta < b.delta; });
    int32_t pos = 0;
    for (int k = 0; k < qn; k++) {
        int32_t at = clampi(q[k].delta, 0, n);
        if (at > pos) { render(w, out[0] + pos, out[1] + pos, at - pos); pos = at; }
        handle(w, q[k].b);
    }
    if (pos < n) render(w, out[0] + pos, out[1] + pos, n - pos);
    double tempo = 120;
    VstTimeInfo *ti = (VstTimeInfo *)w->master(&w->fx, audioMasterGetTime, 0, kVstTempoValid, 0, 0);
    if (ti && (ti->flags & kVstTempoValid) && ti->tempo > 20) tempo = ti->tempo;
    delay(w, out[0], out[1], n, tempo);
    for (int c = 0; c < 2; c++)                      /* output safety: a soft knee above -3 dBFS */
        for (int i = 0; i < n; i++) {
            const float x = out[c][i], ax = std::fabs(x);
            if (ax > 0.7f) out[c][i] = std::copysign(0.7f + 0.3f * soft((ax - 0.7f) / 0.3f), x);
        }
    bool any = false;
    for (int i = 0; i < NPARAMS; i++) {
        if (w->release[i]) { w->release[i] = 0; any = true; w->master(&w->fx, audioMasterAutomate, i, 0, 0, 0.0f); }
        if (!w->notify[i].exchange(0)) continue;
        any = true;
        w->master(&w->fx, audioMasterAutomate, i, 0, 0, w->cache[i].load());
    }
    if (any) w->master(&w->fx, audioMasterUpdateDisplay, 0, 0, 0, 0.0f);
}

/* ---- parameters ------------------------------------------------------------------ */
static void setParameter(AEffect *e, int32_t i, float n) {
    Plugin *w = (Plugin *)e->object;
    if (i < 0 || i >= NPARAMS) return;
    const param_t *p = &PARAMS[i];
    if (popup_set(w->open, i, n)) return;
    bool nudge = false;
    if (p->nopts > 1) {   /* a Q-Link nudge lands between options -> step one option */
        float pos = clamp01(n) * (p->nopts - 1);
        if (std::fabs(pos - std::round(pos)) > 0.001f) {
            float cur = w->cache[i].load() * (p->nopts - 1);
            int idx = clampi((int)std::lround(cur) + (pos > cur ? 1 : -1), 0, p->nopts - 1);
            n = (float)idx / (p->nopts - 1);
            nudge = true;
        }
    }
    w->cache[i].store(clamp01(n));
    if (i == IDX_AMP || i == IDX_GAIN || i == IDX_BASS || i == IDX_MID || i == IDX_TREB || i == IDX_PEDAL ||
        i == IDX_PED_DIST || i == IDX_PED_FILTER || i == IDX_PED_LEVEL) w->amp_dirty.store(true);
    if (!nudge) popup_picked(w->open, w->release, i);
}
static float getParameter(AEffect *e, int32_t i) {
    Plugin *w = (Plugin *)e->object;
    if (i < 0 || i >= NPARAMS) return 0.0f;
    if (popup_is(i)) return w->open[i];
    return w->cache[i].load();
}

/* ---- project chunk: "RIFF1", then key=value; for every parameter ------------------- */
static intptr_t get_chunk(Plugin *w, void **ptr) {
    std::string t = "RBOX1;";
    char buf[64];
    for (int i = 0; i < NPARAMS; i++) {
        std::snprintf(buf, sizeof buf, "%s=%d;", PARAMS[i].key, ui(w, i));
        t += buf;
    }
    w->chunk.assign(t.begin(), t.end());
    *ptr = w->chunk.data();
    return (intptr_t)w->chunk.size();
}
static intptr_t set_chunk(Plugin *w, const void *data, intptr_t len) {
    std::string t((const char *)data, (size_t)len);
    if (t.compare(0, 6, "RBOX1;")) return 0;
    size_t pos = 6;
    while (pos < t.size()) {
        size_t semi = t.find(';', pos);
        if (semi == std::string::npos) break;
        std::string kv = t.substr(pos, semi - pos);
        pos = semi + 1;
        size_t eq = kv.find('=');
        if (eq == std::string::npos) continue;
        int i = param_index(kv.substr(0, eq).c_str());
        if (i >= 0) set_ui(w, i, std::atoi(kv.c_str() + eq + 1));
    }
    w->amp_dirty.store(true);
    return 1;
}

/* ---- dispatcher ---------------------------------------------------------------- */
static void set_rate(Plugin *w, float sr) {
    w->sr = sr;
    w->dl.assign((size_t)(sr * 2.2f), 0.0f);
    w->dr.assign((size_t)(sr * 2.2f), 0.0f);
    w->dw = 0;
    w->dtime = 0;
    for (int c = 0; c < 2; c++) { w->dlp[c].lowpass(3500, 0.7, sr); w->dhp[c].highpass(150, 0.7, sr); }
    for (auto &gs : w->str) for (auto &s : gs) { s.active = false; std::memset(s.buf, 0, sizeof s.buf); }
    for (auto &p : w->ped) p.init(sr);
    w->amp_dirty.store(true);
}

static intptr_t dispatcher(AEffect *e, int32_t op, int32_t idx, intptr_t v, void *p, float o) {
    Plugin *w = (Plugin *)e->object;
    switch (op) {
    case effOpen: return 1;
    case effClose: delete w; return 1;
    case effGetPlugCategory: return 2;
    case effGetEffectName:
    case effGetProductString: copy_str(p, PLUG_NAME, 32); return 1;
    case effGetVendorString: copy_str(p, PLUG_VENDOR, 32); return 1;
    case effGetVendorVersion: return PLUG_VERSION;
    case effGetVstVersion: return 2400;
    case effCanBeAutomated: return idx >= 0 && idx < NPARAMS;
    case effGetParamName: if (idx >= 0 && idx < NPARAMS) copy_str(p, PARAMS[idx].name, 32); return 1;
    case effGetParamLabel: if (idx >= 0 && idx < NPARAMS) copy_str(p, PARAMS[idx].unit, 8); return 1;
    case effGetParamDisplay: {
        if (idx < 0 || idx >= NPARAMS) return 0;
        const param_t *pp = &PARAMS[idx];
        const int u = popup_is(idx) ? norm_to_ui(pp, w->open[idx]) : ui(w, idx);
        char buf[32];
        if (pp->nopts) std::snprintf(buf, sizeof buf, "%s", pp->opts[u]);
        else if (idx == IDX_STRUM) std::snprintf(buf, sizeof buf, "%d ms", u);
        else if (idx == IDX_BASS || idx == IDX_MID || idx == IDX_TREB) std::snprintf(buf, sizeof buf, "%+d dB", u);
        else if (idx == IDX_OCT) std::snprintf(buf, sizeof buf, "%+d", u);
        else std::snprintf(buf, sizeof buf, "%d", u);
        copy_str(p, buf, 24);
        return 1;
    }
    case effSetSampleRate: if (o > 0) set_rate(w, o); return 1;
    case effSetBlockSize: return 1;
    case effMainsChanged: return 1;
    case effProcessEvents: {
        VstEvents *ev = (VstEvents *)p;
        std::lock_guard<std::mutex> lk(w->evMutex);
        for (int i = 0; ev && i < ev->numEvents && w->ev_n < 256; i++) {
            if (ev->events[i]->type != 1) continue;
            const VstMidiEvent *me = (const VstMidiEvent *)ev->events[i];
            Ev &q = w->evq[w->ev_n++];
            q.delta = me->deltaFrames;
            std::memcpy(q.b, me->midiData, 3);
        }
        return 1;
    }
    case effCanDo: {
        const char *s = (const char *)p;
        return (!std::strcmp(s, "receiveVstEvents") || !std::strcmp(s, "receiveVstMidiEvent")) ? 1 : -1;
    }
    case effGetChunk: return get_chunk(w, (void **)p);
    case effSetChunk: return set_chunk(w, p, v);
    default: return 0;
    }
}

static void start_values(Plugin *w) {
    set_ui(w, IDX_OCT, 0);
    set_ui(w, IDX_STRUM, 12);
    set_ui(w, IDX_SUS, 60);
    set_ui(w, IDX_BRIGHT, 60);
    set_ui(w, IDX_REL, 20);
    set_ui(w, IDX_AMP, 1);
    set_ui(w, IDX_GAIN, 50);
    set_ui(w, IDX_BASS, 0);
    set_ui(w, IDX_MID, 0);
    set_ui(w, IDX_TREB, 0);
    set_ui(w, IDX_VOL, 65);
    set_ui(w, IDX_AT_VIB, 40);
    set_ui(w, IDX_AT_FB, 60);
    set_ui(w, IDX_AT_GAIN, 30);
    set_ui(w, IDX_DLY_TIME, 3);   /* 3/16 */
    set_ui(w, IDX_DLY_FB, 35);
    set_ui(w, IDX_DLY_MIX, 20);
    set_ui(w, IDX_DLY_PP, 1);
    set_ui(w, IDX_PED_DIST, 50);
    set_ui(w, IDX_PED_FILTER, 30);
    set_ui(w, IDX_PED_LEVEL, 60);
}

extern "C" __attribute__((visibility("default"))) AEffect *VSTPluginMain(audioMasterCallback master) {
    {
        std::lock_guard<std::mutex> lk(g_logMutex);
        if (!g_log) g_log = std::fopen("/tmp/riffbox_vst.log", "a");
    }
    static std::once_flag once;
    std::call_once(once, [] {
        IDX_SHAPE = param_index("shape"); IDX_OCT = param_index("octave"); IDX_STRUM = param_index("strum");
        IDX_STROKE = param_index("stroke"); IDX_MUTE = param_index("mute"); IDX_SUS = param_index("sustain");
        IDX_BRIGHT = param_index("bright"); IDX_REL = param_index("release"); IDX_DOUBLE = param_index("double");
        IDX_AMP = param_index("amp"); IDX_GAIN = param_index("gain"); IDX_BASS = param_index("bass");
        IDX_MID = param_index("mid"); IDX_TREB = param_index("treble"); IDX_VOL = param_index("volume");
        IDX_AT_VIB = param_index("at_vib"); IDX_AT_BEND = param_index("at_bend"); IDX_AT_FB = param_index("at_fb");
        IDX_AT_WAH = param_index("at_wah"); IDX_AT_GAIN = param_index("at_gain"); IDX_DLY_TIME = param_index("dly_time");
        IDX_DLY_FB = param_index("dly_fb"); IDX_DLY_MIX = param_index("dly_mix"); IDX_DLY_PP = param_index("dly_pp");
        IDX_PEDAL = param_index("pedal"); IDX_PED_DIST = param_index("ped_dist"); IDX_PED_FILTER = param_index("ped_filter");
        IDX_PED_LEVEL = param_index("ped_level");
    });
    Plugin *w = new Plugin();
    w->master = master;
    w->rng.seed((unsigned)(uintptr_t)w);
    for (int i = 0; i < NPARAMS; i++) { w->cache[i].store(PARAMS[i].def); w->notify[i].store(0); }
    start_values(w);
    uint32_t seed = 12345;
    for (auto &gs : w->str) for (auto &s : gs) { std::memset(s.buf, 0, sizeof s.buf); s.seed = (seed += 7919); s.pick = 0.11f + 0.04f * (seed % 5) / 4.0f; }
    set_rate(w, 44100);
    AEffect *e = &w->fx;
    std::memset(e, 0, sizeof *e);
    e->magic = 0x56737450;
    e->dispatcher = dispatcher;
    e->setParameter = setParameter;
    e->getParameter = getParameter;
    e->processReplacing = processReplacing;
    e->numParams = NPARAMS;
    e->numInputs = 0;
    e->numOutputs = 2;
    e->flags = effFlagsCanReplacing | effFlagsIsSynth | effFlagsProgramChunks;
    e->uniqueID = PLUG_UID;
    e->version = PLUG_VERSION;
    e->object = w;
    LOG("[riffbox_vst] up, %d params\n", NPARAMS);
    return e;
}
