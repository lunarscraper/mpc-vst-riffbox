/* =============================================================================
 * rat_core.h - a ProCo RAT distortion modelled from its circuit, one channel.
 * Shared by mpc-vst-rat (the effect) and mpc-vst-riffbox (its PEDAL stage). MIT license.
 *
 *   in -> LM308 non-inverting stage: gain 1 + Zf/Zg
 *           Zf = DISTORTION pot (100k, log) || 100 pF
 *           Zg = (47 R + 2.2 uF) || (560 R + 4.7 uF)        (RUETZ: the 560 R leg removed)
 *         the LM308's gain-bandwidth (~1 MHz, so high gain darkens) and slew rate (0.3 V/us),
 *         its output swinging to about +-4 V on 9 V
 *      -> 1 k into two diodes to ground: 1N914 (RAT), LEDs (TURBO), germanium (GE), none (OP-AMP)
 *      -> FILTER: 1.5 k + 100 k pot into 3.3 nF, a low-pass from ~32 kHz down to ~475 Hz
 *      -> VOLUME
 * The op-amp stage, the clipping and the slew limit run 4x oversampled. 1.0 full scale = 1 V.
 * ========================================================================== */
#pragma once
#include <algorithm>
#include <cmath>

namespace rat {

struct Biquad {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
    inline float run(float x) { float y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
    void lowpass(double f, double q, double sr) {
        double w = 2 * M_PI * std::min(f, sr * 0.49) / sr, al = std::sin(w) / (2 * q), c = std::cos(w), a0 = 1 + al;
        b0 = (float)((1 - c) / 2 / a0); b1 = (float)((1 - c) / a0); b2 = b0; a1 = (float)(-2 * c / a0); a2 = (float)((1 - al) / a0);
    }
    void highpass(double f, double q, double sr) {
        double w = 2 * M_PI * f / sr, al = std::sin(w) / (2 * q), c = std::cos(w), a0 = 1 + al;
        b0 = (float)((1 + c) / 2 / a0); b1 = (float)(-(1 + c) / a0); b2 = b0; a1 = (float)(-2 * c / a0); a2 = (float)((1 - al) / a0);
    }
};

enum Diodes { SILICON, LED, GERMANIUM, NONE };

struct Core {
    /* settings (set, then configure()) */
    float dist = 0.5f;     /* DISTORTION 0..1 */
    float filter = 0.3f;   /* FILTER 0..1, clockwise = darker */
    float volume = 0.7f;   /* VOLUME 0..1 */
    int diodes = SILICON;
    bool ruetz = false;

    void init(float sr) {
        fs = sr;
        fs4 = 4 * sr;
        static const double Q6[3] = {0.5176, 0.7071, 1.9319};   /* 6th-order Butterworth */
        for (int k = 0; k < 3; k++) { up[k].lowpass(0.45 * sr, Q6[k], fs4); dn[k].lowpass(0.45 * sr, Q6[k], fs4); }
        dc_in.highpass(10, 0.7, sr);
        dc_out.highpass(10, 0.7, sr);
        for (auto &s : x3) s = 0;
        for (auto &s : y3) s = 0;
        last = 0; lp = 0; tone = 0;
        configure();
    }

    /* coefficients from the settings; cheap enough to call per block */
    void configure() {
        /* the op-amp stage: G(s) = N(s)/D(s), bilinear at 4x */
        const double Rd = 100e3 * (std::pow(100.0, dist) - 1) / 99, Cf = 100e-12;
        const double R1 = 47, C1 = 2.2e-6, R2 = 560, C2 = ruetz ? 0.0 : 4.7e-6;
        const double a = Rd * Cf, b = R1 * C1, c = R2 * C2;
        double D[4] = {1, a + b + c, a * b + a * c + b * c, a * b * c};
        double N[4] = {D[0], D[1] + Rd * (C1 + C2), D[2] + Rd * (C1 * c + C2 * b), D[3]};
        bilinear(N, D, 2.0 * fs4);
        /* gain-bandwidth: one pole at GBW / (mid-band gain) */
        const double zg = ruetz ? R1 : R1 * R2 / (R1 + R2);
        const double gmid = 1 + Rd / zg;
        const double fc = std::min(1.0e6 / gmid, 0.45 * fs);
        gbw_a = (float)(1 - std::exp(-2 * M_PI * fc / fs4));
        slew = (float)(0.3e6 / fs4);
        /* FILTER: one pole, 1.5 k + 100 k * pot into 3.3 nF (linear pot) */
        const double ft = 1.0 / (2 * M_PI * (1.5e3 + 100e3 * filter) * 3.3e-9);
        tone_a = (float)(1 - std::exp(-2 * M_PI * std::min(ft, 0.45 * fs) / fs));
        /* diodes: knee voltage; the output trimmed so each type ends up at the same level */
        knee = diodes == SILICON ? 0.6f : diodes == LED ? 1.7f : diodes == GERMANIUM ? 0.35f : 4.0f;
        const float vdb = volume <= 0 ? -200.0f : -36.0f + 42.0f * volume;   /* log pot */
        out = std::pow(10.0f, vdb / 20.0f) * 0.6f / knee;
    }

    inline float process(float x) {
        x = dc_in.run(x);
        float acc = 0;
        for (int k = 0; k < 4; k++) {
            float u = (k == 0 ? 4 * x : 0.0f);
            for (auto &f : up) u = f.run(u);
            /* op-amp stage */
            float y = (float)(nb[0] * u + nb[1] * x3[0] + nb[2] * x3[1] + nb[3] * x3[2]
                              - da[1] * y3[0] - da[2] * y3[1] - da[3] * y3[2]);
            x3[2] = x3[1]; x3[1] = x3[0]; x3[0] = u;
            y3[2] = y3[1]; y3[1] = y3[0]; y3[0] = y;
            lp += gbw_a * (y - lp);                           /* GBW */
            float v = std::max(last - slew, std::min(last + slew, lp));   /* slew rate */
            v = 4.0f * soft(v / 4.0f);                        /* the rails */
            last = v;
            /* the diodes after 1 k: x / sqrt(1 + (x/knee)^2), germanium softer */
            float d;
            if (diodes == GERMANIUM) d = v / (1 + std::fabs(v) / knee);
            else if (diodes == NONE) d = v;
            else d = v / std::sqrt(1 + (v / knee) * (v / knee));
            for (auto &f : dn) d = f.run(d);
            if (k == 0) acc = d;
        }
        tone += tone_a * (acc - tone);                        /* FILTER */
        return dc_out.run(tone) * out;
    }

private:
    float fs = 44100, fs4 = 176400;
    Biquad up[3], dn[3], dc_in, dc_out;
    double nb[4] = {1, 0, 0, 0}, da[4] = {1, 0, 0, 0};
    double x3[3] = {0, 0, 0}, y3[3] = {0, 0, 0};
    float gbw_a = 1, slew = 1, last = 0, lp = 0, tone_a = 1, tone = 0, knee = 0.6f, out = 1;

    static inline float soft(float x) {
        x = std::max(-3.0f, std::min(3.0f, x));
        const float x2 = x * x;
        return x * (27 + x2) / (27 + 9 * x2);
    }
    /* N(s)/D(s), third order -> z, s = K (1 - z^-1) / (1 + z^-1) */
    void bilinear(const double *N, const double *D, double K) {
        double n[4] = {0, 0, 0, 0}, d[4] = {0, 0, 0, 0};
        for (int k = 0; k < 4; k++) {
            /* (1 - z)^k (1 + z)^(3-k) */
            double p[4] = {1, 0, 0, 0};
            int len = 1;
            for (int i = 0; i < 3; i++) {
                const double s = i < k ? -1.0 : 1.0;
                for (int j = len; j > 0; j--) p[j] += s * p[j - 1];
                len++;
            }
            const double kk = std::pow(K, k);
            for (int j = 0; j < 4; j++) { n[j] += N[k] * kk * p[j]; d[j] += D[k] * kk * p[j]; }
        }
        for (int j = 0; j < 4; j++) { nb[j] = n[j] / d[0]; da[j] = d[j] / d[0]; }
    }
};

}  // namespace rat
