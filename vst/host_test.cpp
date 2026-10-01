/* Offline x86 test of riffbox_vst.cpp (test.sh builds it with ASan/UBSan): strings in tune, chord
 * shapes, the five amps (distortion order, levels), strum, damping on note off, a new chord taking
 * over, aftertouch bend/vibrato/feedback/wah, the tempo delay, DOUBLE in stereo, chunk restore,
 * NaN/denormal-free output. With "bench" as the second argument it only measures CPU load.
 * Prints PASSED/FAILED; exit code follows. */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <dlfcn.h>
#include <sys/stat.h>
#include <unistd.h>

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
typedef struct {
    int32_t type, byteSize, deltaFrames, flags, noteLength, noteOffset;
    unsigned char midiData[4];
    char detune, noteOffVelocity, reserved1, reserved2;
} VstMidiEvent;
typedef struct { int32_t numEvents; intptr_t reserved; void *events[64]; } VstEvents;

typedef struct {
    double samplePos, sampleRate, nanoSeconds, ppqPos, tempo, barStartPos, cycleStartPos, cycleEndPos;
    int32_t timeSigNumerator, timeSigDenominator, smpteOffset, smpteFrameRate, samplesToNextClock, flags;
} VstTimeInfo;
static long g_samples = 0;     /* the test's transport: 120 BPM, 4/4, always playing */
static VstTimeInfo g_ti;
static int automated = 0;
static intptr_t master(AEffect *, int32_t op, int32_t, intptr_t, void *, float) {
    if (op == 0) automated++;
    if (op == 7) {
        std::memset(&g_ti, 0, sizeof g_ti);
        g_ti.sampleRate = 44100; g_ti.samplePos = (double)g_samples;
        g_ti.ppqPos = g_samples / 44100.0 * 2.0; g_ti.tempo = 120;
        g_ti.timeSigNumerator = 4; g_ti.timeSigDenominator = 4;
        g_ti.flags = (1 << 1) | (1 << 9) | (1 << 10) | (1 << 13);
        return (intptr_t)&g_ti;
    }
    return 0;
}
static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static const int BS = 256;
static const float SR = 44100.0f;
static void midi(AEffect *e, std::vector<std::vector<uint8_t>> msgs, int delta = 0);

static void midi(AEffect *e, std::vector<std::vector<uint8_t>> msgs, int delta) {
    static VstMidiEvent ev[64];
    static VstEvents list;
    list.numEvents = 0;
    for (auto &m : msgs) {
        VstMidiEvent &v = ev[list.numEvents];
        std::memset(&v, 0, sizeof v);
        v.type = 1; v.byteSize = sizeof v; v.deltaFrames = delta;
        for (size_t k = 0; k < m.size() && k < 3; k++) v.midiData[k] = m[k];
        list.events[list.numEvents++] = &v;
    }
    e->dispatcher(e, 25, 0, 0, &list, 0);
}

/* renders seconds of audio; returns RMS, flags NaN/Inf/denormals */
static double render(AEffect *e, double seconds, bool *bad = nullptr, std::vector<float> *keep = nullptr) {
    std::vector<float> l(BS), r(BS);
    float *out[2] = {l.data(), r.data()};
    double sum = 0; long n = 0;
    int blocks = (int)(seconds * SR / BS);
    for (int b = 0; b < blocks; b++) {
        e->processReplacing(e, nullptr, out, BS);
        g_samples += BS;
        if (keep) keep->insert(keep->end(), l.begin(), l.end());
        for (int i = 0; i < BS; i++) {
            for (float s : {l[i], r[i]}) {
                if (!std::isfinite(s) || std::fpclassify(s) == FP_SUBNORMAL) {
                    if (bad && !*bad) std::printf("  (bad sample %g)\n", (double)s);
                    if (bad) *bad = true;
                }
                sum += (double)s * s; n++;
            }
        }
    }
    return n ? std::sqrt(sum / n) : 0;
}


/* renders seconds after sending msgs; returns the left channel */
static std::vector<float> capture(AEffect *e, std::vector<std::vector<uint8_t>> msgs, double seconds) {
    midi(e, msgs);
    std::vector<float> l(BS), r(BS), all;
    float *out[2] = {l.data(), r.data()};
    int blocks = (int)(seconds * SR / BS);
    for (int b = 0; b < blocks; b++) {
        e->processReplacing(e, nullptr, out, BS);
        g_samples += BS;
        for (int i = 0; i < BS; i++) all.push_back(0.5f * (l[i] + r[i]));
    }
    return all;
}
static double rms(const std::vector<float> &a, double t0 = 0, double t1 = 1e9) {
    size_t i0 = (size_t)(t0 * SR), i1 = std::min(a.size(), (size_t)(t1 * SR));
    double s = 0;
    for (size_t i = i0; i < i1; i++) s += (double)a[i] * a[i];
    return i1 > i0 ? std::sqrt(s / (i1 - i0)) : 0;
}
/* amplitude of harmonics 1..n-1 of f0 (Goertzel over 0.2-0.7 s); h[0] = 0 */
static std::vector<double> harmonics(const std::vector<float> &a, double f0, int n, double t0 = 0.2, double t1 = 0.7) {
    std::vector<double> h((size_t)n, 0.0);
    size_t i0 = (size_t)(t0 * SR), i1 = std::min(a.size(), (size_t)(t1 * SR));
    for (int k = 1; k < n; k++) {
        double w = 2 * M_PI * f0 * k / SR, c = std::cos(w), s1 = 0, s2 = 0;
        for (size_t i = i0; i < i1; i++) { double s0 = a[i] + 2 * c * s1 - s2; s2 = s1; s1 = s0; }
        h[(size_t)k] = std::sqrt(std::max(0.0, s1 * s1 + s2 * s2 - 2 * c * s1 * s2)) / (i1 - i0);
    }
    return h;
}
/* how far the spectral centroid of 110 Hz harmonics wanders in 100 ms windows (std dev, Hz) */
static double movement(const std::vector<float> &a) {
    std::vector<double> c;
    for (double t = 0.1; t + 0.1 < a.size() / (double)SR; t += 0.1) {
        std::vector<double> h = harmonics(a, 110.0, 40, t, t + 0.1);
        double num = 0, den = 0;
        for (int k = 1; k < 40; k++) { num += 110.0 * k * h[k]; den += h[k]; }
        if (den > 1e-7) c.push_back(num / den);
    }
    if (c.size() < 2) return 0;
    double m = 0, v = 0;
    for (double x : c) m += x;
    m /= c.size();
    for (double x : c) v += (x - m) * (x - m);
    return std::sqrt(v / c.size());
}
static int param(AEffect *e, const char *name) {
    char buf[64];
    for (int i = 0; i < e->numParams; i++) {
        buf[0] = 0;
        e->dispatcher(e, 8, i, 0, buf, 0);
        if (!std::strcmp(buf, name)) return i;
    }
    std::printf("FAIL: no parameter %s\n", name);
    fails++;
    return 0;
}
static std::string display(AEffect *e, int i) {
    char buf[64] = {0};
    e->dispatcher(e, 7, i, 0, buf, 0);
    return buf;
}
static void choose(AEffect *e, const char *name, int v) {
    int i = param(e, name);
    char buf[64];
    /* find the range from the display of 0 and 1 is not possible: ranges are known here */
    struct R { const char *n; int lo, hi; };
    static const R ranges[] = {{"Chord", 0, 10}, {"Octave", -2, 2}, {"Strum", 0, 60}, {"Stroke", 0, 2}, {"Palm Mute", 0, 100},
        {"Sustain", 0, 100}, {"Pick Tone", 0, 100}, {"Release", 0, 100}, {"Double", 0, 1}, {"Amp", 0, 4}, {"Gain", 0, 100},
        {"Bass", -12, 12}, {"Mid", -12, 12}, {"Treble", -12, 12}, {"Volume", 0, 100}, {"AT Vibrato", 0, 100},
        {"AT Bend", 0, 100}, {"AT Feedback", 0, 100}, {"AT Wah", 0, 100}, {"AT Gain", 0, 100}, {"Delay Time", 0, 6},
        {"Delay Fdbk", 0, 95}, {"Delay Mix", 0, 100}, {"Ping Pong", 0, 1}, {"Pedal", 0, 4}, {"Ped Dist", 0, 100},
        {"Ped Filter", 0, 100}, {"Ped Level", 0, 100}};
    for (auto &r : ranges)
        if (!std::strcmp(r.n, name)) { e->setParameter(e, i, (float)(v - r.lo) / (r.hi - r.lo)); return; }
    (void)buf;
    std::printf("FAIL: range of %s\n", name);
    fails++;
}
/* fundamental by autocorrelation over [t0, t1], 40..fmax Hz */
static double pitch(const std::vector<float> &a, double t0, double t1, double fmax = 1000) {
    size_t i0 = (size_t)(t0 * SR), i1 = std::min(a.size(), (size_t)(t1 * SR));
    int lo = (int)(SR / fmax), hi = (int)(SR / 40);
    std::vector<double> r((size_t)hi + 2, 0.0);
    for (int L = lo - 1; L <= hi + 1; L++) {
        double s = 0;
        for (size_t i = i0; i + (size_t)L < i1; i++) s += (double)a[i] * a[i + (size_t)L];
        r[(size_t)L] = s;
    }
    /* the first lag where the ACF comes within 90 % of its best */
    double best = 0;
    for (int L = lo; L <= hi; L++) best = std::max(best, r[(size_t)L]);
    int bl = lo;
    for (int L = lo; L <= hi; L++)
        if (r[(size_t)L] > 0.9 * best && r[(size_t)L] >= r[(size_t)L - 1] && r[(size_t)L] >= r[(size_t)L + 1]) { bl = L; break; }
    double y0 = r[(size_t)bl - 1], y1 = r[(size_t)bl], y2 = r[(size_t)bl + 1], d = y0 - 2 * y1 + y2;
    double off = d != 0 ? 0.5 * (y0 - y2) / d : 0;
    return SR / (bl + off);
}
static double goertzel(const std::vector<float> &a, double f, double t0, double t1) {
    size_t i0 = (size_t)(t0 * SR), i1 = std::min(a.size(), (size_t)(t1 * SR));
    double w = 2 * M_PI * f / SR, c = std::cos(w), s1 = 0, s2 = 0;
    for (size_t i = i0; i < i1; i++) { double s0 = a[i] + 2 * c * s1 - s2; s2 = s1; s1 = s0; }
    return std::sqrt(std::max(0.0, s1 * s1 + s2 * s2 - 2 * c * s1 * s2)) / (i1 - i0);
}
static double peakabs(const std::vector<float> &a, double t0, double t1) {
    double p = 0;
    for (size_t i = (size_t)(t0 * SR); i < std::min(a.size(), (size_t)(t1 * SR)); i++) p = std::max(p, (double)std::fabs(a[i]));
    return p;
}
static void stop(AEffect *e, int note, bool *bad) {
    midi(e, {{0x80, (uint8_t)note, 0}, {0xD0, 0, 0}});
    render(e, 1.5, bad);
}

int main(int argc, char **argv) {
    const char *so = argc > 1 ? argv[1] : "./riffbox.so";
    const bool bench = argc > 2 && !std::strcmp(argv[2], "bench");
    void *h = dlopen(so, RTLD_NOW | RTLD_LOCAL);
    if (!h) { std::printf("FAILED: dlopen %s\n", dlerror()); return 1; }
    auto mainf = (AEffect * (*)(audioMasterCallback)) dlsym(h, "VSTPluginMain");
    if (!mainf) { std::printf("FAILED: no VSTPluginMain\n"); return 1; }
    AEffect *e = mainf(master);
    CHECK(e && e->magic == 0x56737450, "magic");
    e->dispatcher(e, 0, 0, 0, nullptr, 0);
    e->dispatcher(e, 10, 0, 0, nullptr, SR);
    e->dispatcher(e, 11, 0, BS, nullptr, 0);
    bool bad = false;

    if (bench) {   /* worst case: 6-string chord, DOUBLE, STONER, delay, aftertouch feedback + wah */
        choose(e, "Chord", 3); choose(e, "Double", 1); choose(e, "Amp", 4);
        choose(e, "AT Wah", 50); choose(e, "AT Feedback", 100);
        midi(e, {{0x90, 40, 110}, {0xD0, 90, 0}});
        auto t0 = std::chrono::steady_clock::now();
        render(e, 5.0);
        double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("bench (this CPU, one core): 6-string chord, double, stoner, delay, aftertouch: %.1f %%\n", 100 * s / 5.0);
        choose(e, "Double", 0); choose(e, "Chord", 0); choose(e, "AT Wah", 0);
        midi(e, {{0x90, 40, 110}});
        t0 = std::chrono::steady_clock::now();
        render(e, 5.0);
        s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("bench (this CPU, one core): power chord, single, stoner, delay: %.1f %%\n", 100 * s / 5.0);
        e->dispatcher(e, 1, 0, 0, nullptr, 0);
        return 0;
    }

    /* clean, no delay, no strum for the measurements */
    choose(e, "Amp", 0); choose(e, "Gain", 0); choose(e, "Delay Mix", 0); choose(e, "Strum", 0);
    choose(e, "Sustain", 60);

    /* 1. in tune: octaves on A2 and A3 */
    choose(e, "Chord", 2);
    for (int note : {45, 57, 40}) {
        std::vector<float> a = capture(e, {{0x90, (uint8_t)note, 100}}, 1.0);
        stop(e, note, &bad);
        const double want = 440 * std::pow(2.0, (note - 69) / 12.0), got = pitch(a, 0.2, 0.8);
        std::printf("  note %d: %.2f Hz (want %.2f), rms %.4f\n", note, got, want, rms(a));
        CHECK(std::fabs(got / want - 1) < 0.006, "note %d out of tune: %.2f Hz", note, got);
        CHECK(rms(a) > 0.005, "note %d too quiet", note);
    }

    /* 2. chord shapes: the third of E major vs E minor */
    for (int shape : {3, 4}) {
        choose(e, "Chord", shape);
        std::vector<float> a = capture(e, {{0x90, 40, 100}}, 1.0);
        stop(e, 40, &bad);
        const double maj = goertzel(a, 207.65, 0.1, 0.9), min = goertzel(a, 196.00, 0.1, 0.9);
        std::printf("  %s: G#3 %.5f, G3 %.5f\n", shape == 3 ? "MAJOR" : "MINOR", maj, min);
        CHECK(shape == 3 ? maj > 3 * min : min > 3 * maj, "%s: wrong third", shape == 3 ? "MAJOR" : "MINOR");
    }

    /* 3. amps: more crunch from CLEAN to STONER (crest factor falls), levels close together */
    choose(e, "Chord", 0);
    choose(e, "Gain", 50);
    double crest[5], level[5];
    for (int m = 0; m < 5; m++) {
        choose(e, "Amp", m);
        std::vector<float> a = capture(e, {{0x90, 40, 110}}, 1.2);
        stop(e, 40, &bad);
        level[m] = rms(a, 0.1, 1.0);
        crest[m] = peakabs(a, 0.1, 1.0) / std::max(level[m], 1e-9);
        std::printf("  amp %-14s rms %.4f (%.1f dBFS), crest %.2f\n", display(e, param(e, "Amp")).c_str(), level[m],
                    20 * std::log10(level[m] + 1e-12), crest[m]);
    }
    CHECK(crest[0] > crest[1] && crest[1] > crest[3] && crest[0] > crest[4], "the amps do not get crunchier");
    for (int m = 0; m < 5; m++)
        CHECK(std::fabs(20 * std::log10(level[m] / level[1])) < 6, "amp %d level %.1f dB off classic", m, 20 * std::log10(level[m] / level[1]));

    /* 4. strum: 60 ms spreads the attack */
    choose(e, "Amp", 0); choose(e, "Gain", 0); choose(e, "Chord", 3);
    double first[2];
    for (int k = 0; k < 2; k++) {
        choose(e, "Strum", k ? 60 : 0);
        std::vector<float> a = capture(e, {{0x90, 40, 100}}, 0.6);
        stop(e, 40, &bad);
        first[k] = rms(a, 0.0, 0.02);
    }
    std::printf("  strum: first 20 ms rms %.4f (0 ms) vs %.4f (60 ms)\n", first[0], first[1]);
    CHECK(first[1] < 0.6 * first[0], "strum does not spread the strings");
    choose(e, "Strum", 0);

    /* 5. note off damps; 6. a new chord takes over */
    choose(e, "Chord", 2);
    {
        std::vector<float> a = capture(e, {{0x90, 45, 100}}, 0.8);
        std::vector<float> b = capture(e, {{0x80, 45, 0}}, 0.8);
        std::printf("  note off: rms held %.4f, 0.3-0.6 s after %.5f\n", rms(a, 0.4, 0.8), rms(b, 0.3, 0.6));
        CHECK(rms(b, 0.3, 0.6) < 0.05 * rms(a, 0.4, 0.8), "note off does not damp");
        render(e, 1.0);
        a = capture(e, {{0x90, 45, 100}}, 0.5);
        b = capture(e, {{0x90, 52, 100}}, 0.8);
        const double f = pitch(b, 0.3, 0.8);
        std::printf("  A2 then E3: %.1f Hz after the change\n", f);
        CHECK(std::fabs(f / 164.81 - 1) < 0.01, "the new chord did not take over (%.1f Hz)", f);
        midi(e, {{0x80, 45, 0}});
        stop(e, 52, &bad);
        render(e, 0.2);
    }

    /* 7. aftertouch: bend a whole tone, vibrato, feedback sustain, wah */
    choose(e, "AT Bend", 100); choose(e, "AT Vibrato", 0); choose(e, "AT Feedback", 0);
    {
        std::vector<float> a = capture(e, {{0x90, 45, 100}}, 0.5);
        std::vector<float> b = capture(e, {{0xD0, 127, 0}}, 0.8);
        const double f0 = pitch(a, 0.1, 0.5), f1 = pitch(b, 0.3, 0.8);
        std::printf("  AT bend: %.1f Hz -> %.1f Hz (want %.1f)\n", f0, f1, f0 * std::pow(2.0, 2 / 12.0));
        CHECK(std::fabs(f1 / (f0 * std::pow(2.0, 2 / 12.0)) - 1) < 0.01, "AT bend: %.1f Hz", f1);
        stop(e, 45, &bad);
    }
    choose(e, "AT Bend", 0); choose(e, "AT Vibrato", 100);
    {
        midi(e, {{0x90, 45, 100}, {0xD0, 127, 0}});
        std::vector<double> p;
        std::vector<float> a = capture(e, {}, 1.2);
        for (double t = 0.2; t < 1.1; t += 0.05) p.push_back(pitch(a, t, t + 0.08, 150));
        double lo = 1e9, hi = 0;
        for (double x : p) { lo = std::min(lo, x); hi = std::max(hi, x); }
        std::printf("  AT vibrato: %.1f .. %.1f Hz\n", lo, hi);
        CHECK(hi / lo > 1.02 && hi / lo < 1.12, "AT vibrato: %.1f .. %.1f Hz", lo, hi);
        stop(e, 45, &bad);
    }
    choose(e, "AT Vibrato", 0); choose(e, "Sustain", 10);
    {
        double late[2], pk = 0;
        for (int k = 0; k < 2; k++) {
            choose(e, "AT Feedback", k ? 100 : 0);
            std::vector<float> a = capture(e, {{0x90, 45, 100}, {0xD0, (uint8_t)(k ? 127 : 0), 0}}, 4.0);
            late[k] = rms(a, 3.0, 4.0);
            pk = std::max(pk, peakabs(a, 0, 4));
            stop(e, 45, &bad);
        }
        std::printf("  AT feedback: rms at 3-4 s %.5f without, %.5f with (peak %.2f)\n", late[0], late[1], pk);
        CHECK(late[1] > 5 * late[0] && late[1] > 0.002, "AT feedback does not sustain");
        CHECK(pk < 2.0, "AT feedback runs away");
    }
    choose(e, "AT Feedback", 0); choose(e, "Sustain", 60); choose(e, "AT Wah", 100);
    {
        std::vector<float> a = capture(e, {{0x90, 40, 100}, {0xD0, 0, 0}}, 0.6);
        std::vector<float> b = capture(e, {{0xD0, 127, 0}}, 0.6);
        auto hiratio = [&](const std::vector<float> &x) { return goertzel(x, 1975.5, 0.2, 0.6) / std::max(1e-9, goertzel(x, 493.9, 0.2, 0.6)); };
        std::printf("  AT wah: 2 kHz / 500 Hz %.3f heel, %.3f toe\n", hiratio(a), hiratio(b));
        CHECK(hiratio(b) > 3 * hiratio(a), "AT wah does not sweep");
        stop(e, 40, &bad);
        choose(e, "AT Wah", 0);
    }

    /* 8. delay: 1/8 at 120 BPM = 0.25 s */
    choose(e, "Delay Mix", 100); choose(e, "Delay Fdbk", 0); choose(e, "Delay Time", 2); choose(e, "Ping Pong", 0);
    choose(e, "Release", 0); choose(e, "Palm Mute", 100);
    {
        render(e, 0.5);
        std::vector<float> a = capture(e, {{0x90, 45, 110}}, 0.05);
        std::vector<float> b = capture(e, {{0x80, 45, 0}}, 0.6);
        a.insert(a.end(), b.begin(), b.end());
        const double dry = rms(a, 0.0, 0.05), gap = rms(a, 0.15, 0.2), echo = rms(a, 0.25, 0.30);
        std::printf("  delay: rms dry %.4f, gap %.5f, echo %.4f\n", dry, gap, echo);
        CHECK(echo > 5 * gap && echo > 0.2 * dry, "no echo at 0.25 s");
    }
    choose(e, "Palm Mute", 0); choose(e, "Release", 20); choose(e, "Delay Mix", 20);

    /* 9. DOUBLE: two guitars, left and right */
    choose(e, "Double", 1); choose(e, "Amp", 1);
    {
        midi(e, {{0x90, 40, 100}});
        std::vector<float> l(BS), r(BS);
        float *out[2] = {l.data(), r.data()};
        double diff = 0, sum = 0;
        for (int b = 0; b < 100; b++) {
            e->processReplacing(e, nullptr, out, BS);
            g_samples += BS;
            for (int i = 0; i < BS; i++) { diff += std::fabs(l[i] - r[i]); sum += std::fabs(l[i]) + std::fabs(r[i]); }
        }
        std::printf("  double: L/R difference %.2f\n", diff / std::max(sum, 1e-9));
        CHECK(diff > 0.1 * sum, "DOUBLE is not stereo");
        stop(e, 40, &bad);
    }

    /* 10. the RAT pedal in front of a clean amp: crunch from the pedal alone */
    choose(e, "Double", 0); choose(e, "Amp", 0); choose(e, "Gain", 0); choose(e, "Chord", 0); choose(e, "Delay Mix", 0);
    {
        double cr[5], lv[5];
        const char *pn[5] = {"OFF", "RAT", "TURBO RAT", "GE RAT", "RUETZ RAT"};
        choose(e, "Ped Dist", 80);
        for (int pm = 0; pm < 5; pm++) {
            choose(e, "Pedal", pm);
            std::vector<float> a = capture(e, {{0x90, 40, 110}}, 1.2);
            stop(e, 40, &bad);
            lv[pm] = rms(a, 0.1, 1.0);
            cr[pm] = peakabs(a, 0.1, 1.0) / std::max(lv[pm], 1e-9);
            std::printf("  pedal %-9s rms %.4f, crest %.2f\n", pn[pm], lv[pm], cr[pm]);
            CHECK(lv[pm] > 0.01, "pedal %s silent", pn[pm]);
        }
        CHECK(cr[1] < 0.7 * cr[0], "the RAT does not distort the clean amp");
        choose(e, "Pedal", 0);
    }

    /* 11. chunk into a second instance */
    choose(e, "Amp", 3); choose(e, "Chord", 6); choose(e, "Bass", -5); choose(e, "Pedal", 2);
    void *chunk = nullptr;
    intptr_t len = e->dispatcher(e, 23, 0, 0, &chunk, 0);
    std::vector<uint8_t> copy((uint8_t *)chunk, (uint8_t *)chunk + len);
    AEffect *e2 = mainf(master);
    e2->dispatcher(e2, 10, 0, 0, nullptr, SR);
    e2->dispatcher(e2, 24, 0, (intptr_t)copy.size(), copy.data(), 0);
    for (const char *k : {"Amp", "Chord", "Bass", "Double", "Pedal"})
        CHECK(display(e2, param(e2, k)) == display(e, param(e, k)), "chunk: %s %s vs %s", k, display(e2, param(e2, k)).c_str(),
              display(e, param(e, k)).c_str());
    std::printf("  chunk %ld bytes: %s %s %s\n", (long)len, display(e2, param(e2, "Amp")).c_str(),
                display(e2, param(e2, "Chord")).c_str(), display(e2, param(e2, "Bass")).c_str());
    e2->dispatcher(e2, 1, 0, 0, nullptr, 0);

    CHECK(!bad, "NaN, Inf or denormals in the output");
    CHECK(automated > 0, "the host was never told about changed values");
    e->dispatcher(e, 1, 0, 0, nullptr, 0);
    dlclose(h);
    std::printf(fails ? "FAILED (%d)\n" : "PASSED\n", fails);
    return fails ? 1 : 0;
}
