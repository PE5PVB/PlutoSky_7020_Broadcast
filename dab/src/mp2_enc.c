// SPDX-License-Identifier: Apache-2.0
// MPEG-1 Layer II encoder for DAB. Structure (ISO/IEC 11172-3, 2.4):
//   1. polyphase analysis filterbank: 32 subbands, 36 samples per subband per frame (3 parts of 12)
//   2. scale factors per part (63 steps of 2 dB), scale factor selection information (scfsi) per subband
//   3. psychoacoustic model 1 (ISO/IEC 11172-3, D.1): 1024-point FFT, tonal and noise maskers, spreading function, absolute
//   threshold -> signal-to-mask ratio per subband
//   4. bit allocation: repeatedly give the subband with the lowest noise-to-mask ratio one quantizer class more, while it fits
//   5. the frame: header, CRC-16, allocation, scfsi, scale factors, samples (3-sample groups for 3, 5 and 9 levels), then the
//      DAB end: X-PAD, ScF-CRC, F-PAD
// Stereo only (mode 00), 48 kHz; bit allocation tables B.2a (56..192 kbit/s per channel, 27 subbands) and B.2c (32/48, 8
// subbands).
#include "mp2_enc.h"
#include "bitwriter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NSB 32
#define NCH 2

// analysis window: our own 512-tap prototype, a Kaiser-windowed sinc. Its two parameters were fitted to the standard synthesis
// filterbank (measured as the impulse response of a decoder, libmpg123): with cutoff 0.0089 fs and beta 11.3 the aliasing between
// neighbouring subbands cancels in the standard decoder to better than -83 dB (a tone 250 Hz from a subband edge); with the
// textbook cutoff fs/128 it stayed at -35 dB. WIN_GAIN makes the analysis + standard synthesis chain unity gain.
#define WIN_BETA 11.3
#define WIN_FC   0.0089
#define WIN_GAIN 2.0

// bit allocation tables (ISO/IEC 11172-3, B.2a and B.2c): per subband the bits of the allocation field and the quantizer
// classes (number of levels) it selects
typedef struct {
    int nbal;
    int steps[16];
} alloc_row;
// clang-format off
#define A0 { 4, { 0, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767, 65535 } }
#define A1 { 4, { 0, 3, 5, 7, 9, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 65535 } }
#define A2 { 3, { 0, 3, 5, 7, 9, 15, 31, 65535 } }
#define A3 { 2, { 0, 3, 5, 65535 } }
static const alloc_row TAB_A[27] = {
    A0, A0, A0, A1, A1, A1, A1, A1, A1, A1, A1, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A3, A3, A3, A3
};
#define C0 { 4, { 0, 3, 5, 9, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767 } }
#define C1 { 3, { 0, 3, 5, 9, 15, 31, 63, 127 } }
static const alloc_row TAB_C[8] = {
    C0, C0, C1, C1, C1, C1, C1, C1
};
// clang-format on

#define PSY_N 1024          // FFT length of the psychoacoustic model
#define PSY_H (PSY_N / 2)

struct mp2enc {
    int kbps, bytes, br_index, sblimit, scfcrc_len;
    const alloc_row *tab;
    float C[512];           // analysis window
    double M[NSB][64];      // matrixing cosines
    float X[NCH][512];      // filterbank input history
    float sb[NCH][36][NSB]; // subband samples of the frame
    double scf_val[63];
    // psychoacoustic model 1 (ISO/IEC 11172-3, D.1)
    float hann[PSY_N], tw_re[PSY_H], tw_im[PSY_H];
    short brev[PSY_N];
    float z[PSY_H + 1];     // critical band rate (bark) of each FFT line
    float ath[PSY_H + 1];   // absolute threshold of each line, dB (a full-scale sine = 96 dB)
    // allocation stability: an audio bandwidth per bitrate (subbands above it get no bits, so they are not switched on and off
    // from frame to frame), and the subbands that had bits in the previous frame
    int bw_sb;
    unsigned char prev_on[NCH][NSB];
    // the ScF-CRC of a frame protects the scale factors of the NEXT frame (TS 103 466, B.3): a frame is held back one frame
    uint8_t held[3 * 192];
    int have_held;
};

static const int BR_TAB[15] = { 0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384 };

int mp2enc_valid_kbps(int kbps) { return kbps == 64 || kbps == 96 || kbps == 112 || kbps == 128 || kbps == 160 || kbps == 192; }

static double bark(double f);
static double ath_db(double f);

static double bessel_i0(double x){
    double s = 1, t = 1;
    for (int k = 1; k < 40; k++){
        t *= (x / (2 * k)) * (x / (2 * k));
        s += t;
    }
    return s;
}

mp2enc *mp2enc_open(int kbps){
    if (!mp2enc_valid_kbps(kbps)) return NULL;
    mp2enc *e = calloc(1, sizeof *e);
    if (!e) return NULL;
    e->kbps = kbps;
    e->bytes = 3 * kbps;
    for (int i = 1; i < 15; i++)
        if (BR_TAB[i] == kbps) e->br_index = i;
    if (kbps / 2 >= 56){
        e->tab = TAB_A;
        e->sblimit = 27;
        // 750 Hz per subband: 15 kHz at 192 kbit/s (as FM), 14.25 at 160, 12.75 at 128, 12 at 112 and below
        e->bw_sb = kbps >= 192 ? 20 : kbps >= 160 ? 19 : kbps >= 128 ? 17 : 16;
    } else {
        e->tab = TAB_C;
        e->sblimit = 8;
        e->bw_sb = 8;
    }
    e->scfcrc_len = kbps < 112 ? 2 : 4;
    // prototype lowpass h[n], n = 0..511, centre 256, h[0] = 0 as in the standard; C[n] = h[n] with the sign of the
    // standard window (negative where floor(n / 64) is odd)
    for (int n = 0; n < 512; n++){
        double t = n - 256.0, s = t == 0 ? 2 * WIN_FC : sin(2 * M_PI * WIN_FC * t) / (M_PI * t);
        double r = t / 256.0, w = fabs(r) >= 1 ? 0 : bessel_i0(WIN_BETA * sqrt(1 - r * r)) / bessel_i0(WIN_BETA);
        double h = WIN_GAIN * s * w;
        e->C[n] = (float)(((n / 64) & 1) ? -h : h);
    }
    for (int k = 0; k < NSB; k++)
        for (int i = 0; i < 64; i++) e->M[k][i] = cos((2 * k + 1) * (i - 16) * M_PI / 64.0);
    for (int i = 0; i < 63; i++) e->scf_val[i] = pow(2.0, 1.0 - i / 3.0);
    for (int n = 0; n < PSY_N; n++){
        e->hann[n] = (float)(0.5 - 0.5 * cos(2 * M_PI * n / PSY_N));
        int r = 0;
        for (int b = 0; b < 10; b++)
            if (n & (1 << b)) r |= 1 << (9 - b);
        e->brev[n] = (short)r;
    }
    for (int k = 0; k < PSY_H; k++){
        e->tw_re[k] = (float)cos(2 * M_PI * k / PSY_N);
        e->tw_im[k] = (float)-sin(2 * M_PI * k / PSY_N);
    }
    for (int k = 0; k <= PSY_H; k++){
        const double f = k * 48000.0 / PSY_N;
        e->z[k] = (float)bark(f);
        e->ath[k] = (float)ath_db(f);
    }
    return e;
}

void mp2enc_close(mp2enc *e) { free(e); }

// one step of the analysis filterbank: 32 new samples of channel c (pcm with stride 2) -> 32 subband samples
static void analyse(mp2enc *e, int c, const float *pcm, float *out){
    float *X = e->X[c];
    memmove(X + 32, X, 480 * sizeof(float));
    for (int i = 0; i < 32; i++) X[31 - i] = pcm[2 * i];
    double Y[64];
    for (int i = 0; i < 64; i++){
        double y = 0;
        for (int j = 0; j < 8; j++) y += (double)e->C[i + 64 * j] * X[i + 64 * j];
        Y[i] = y;
    }
    for (int k = 0; k < NSB; k++){
        double s = 0;
        for (int i = 0; i < 64; i++) s += e->M[k][i] * Y[i];
        out[k] = (float)s;
    }
}

// bits per sample of an ungrouped quantizer: 7 -> 3, 65535 -> 16
static int nbits_steps(int steps){
    int b = 0;
    while ((1 << b) < steps + 1) b++;
    return b;
}
static int grouped(int steps) { return steps == 3 || steps == 5 || steps == 9; }
static int group_bits(int steps) { return steps == 3 ? 5 : steps == 5 ? 7 : 10; }
static int sample_bits(int steps) { return steps == 0 ? 0 : grouped(steps) ? 12 * group_bits(steps) : 36 * nbits_steps(steps); }
static double snr_db(int steps){
    switch (steps){
    case 0: return 0;
    case 3: return 7.00;
    case 5: return 11.00;
    case 7: return 16.00;
    case 9: return 20.84;
    case 15: return 25.28;
    case 31: return 31.59;
    case 63: return 37.75;
    case 127: return 43.84;
    case 255: return 49.89;
    case 511: return 55.93;
    case 1023: return 61.96;
    case 2047: return 67.98;
    case 4095: return 74.01;
    case 8191: return 80.03;
    case 16383: return 86.05;
    case 32767: return 92.01;
    default: return 98.01;
    }
}
static double bark(double f) { return 13.0 * atan(0.00076 * f) + 3.5 * atan((f / 7500.0) * (f / 7500.0)); }
static double ath_db(double f){ // absolute threshold, dB SPL (0 dBFS sine = 96 dB SPL)
    double k = f / 1000.0;
    if (k < 0.02) k = 0.02;
    return 3.64 * pow(k, -0.8) - 6.5 * exp(-0.6 * (k - 3.3) * (k - 3.3)) + 1e-3 * k * k * k * k;
}

// ---- psychoacoustic model 1 (ISO/IEC 11172-3, Annex D.1), for one channel ------------------------------------------------
// 1024-point FFT (Hann) over the middle of the frame, levels in dB with a full-scale sine at 96 dB. Tonal components (local
// maxima at least 7 dB above their neighbours, D.1 step 4) and one non-tonal (noise) component per critical band; components
// below the absolute threshold and the weaker of two tonal ones within 0.5 bark are dropped. Each masks with the masking index
// av (tonal -6.025 - 0.275 z, noise -2.025 - 0.175 z) and the spreading function vf over -3..+8 bark; the global threshold is
// the power sum with the absolute threshold, and its minimum within a subband is that subband's threshold. smr = Lsb - minimum,
// with Lsb the larger of the subband's strongest line and the level its scale factor allows (20 log(scf * 32768) - 10).
static void fft1024(const mp2enc *e, float *re, float *im){
    for (int len = 2; len <= PSY_N; len <<= 1){
        const int half = len >> 1, step = PSY_N / len;
        for (int s = 0; s < PSY_N; s += len)
            for (int k = 0; k < half; k++){
                const float wr = e->tw_re[k * step], wi = e->tw_im[k * step];
                const int a = s + k, b = a + half;
                const float tr = re[b] * wr - im[b] * wi, ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
    }
}
static inline float db2p(float db) { return expf(db * 0.23025851f); }
static inline float p2db(float p) { return 4.3429448f * logf(p + 1e-30f); }

typedef struct {
    short k;
    float spl, z;
    char tonal;
} masker;

static void psy1(const mp2enc *e, const float *pcm, int c, int sbl, const int scf[NSB][3], double *smr){
    static float re[PSY_N], im[PSY_N], X[PSY_H + 1];
    static char used[PSY_H + 1];
    static masker m[PSY_H];
    for (int n = 0; n < PSY_N; n++){
        re[e->brev[n]] = pcm[2 * (64 + n) + c] * e->hann[n];
        im[e->brev[n]] = 0;
    }
    fft1024(e, re, im);
    // a full-scale sine: peak |X| = N/4 = 256 with the Hann window -> 96 dB
    for (int k = 0; k <= PSY_H; k++) X[k] = p2db((re[k] * re[k] + im[k] * im[k]) / 65536.0f) + 96.0f;
    memset(used, 0, sizeof used);
    int nm = 0;
    // tonal components
    for (int k = 3; k < 500; k++){
        if (!(X[k] > X[k - 1] && X[k] >= X[k + 1])) continue;
        const int jmax = k < 63 ? 2 : k < 127 ? 3 : k < 255 ? 6 : 12;
        int ok = 1;
        for (int j = 2; j <= jmax && ok; j++)
            if (X[k] - X[k - j] < 7.0f || (k + j <= PSY_H && X[k] - X[k + j] < 7.0f)) ok = 0;
        if (!ok) continue;
        const float spl = p2db(db2p(X[k - 1]) + db2p(X[k]) + db2p(X[k + 1]));
        for (int j = -jmax; j <= jmax; j++)
            if (k + j >= 0 && k + j <= PSY_H) used[k + j] = 1;
        m[nm++] = (masker){ (short)k, spl, e->z[k], 1 };
    }
    // non-tonal: the rest of each critical band, at its geometric centre
    for (int b0 = 1; b0 <= PSY_H;){
        const int cb = (int)e->z[b0];
        int b1 = b0;
        float p = 0;
        while (b1 <= PSY_H && (int)e->z[b1] == cb){
            if (!used[b1]) p += db2p(X[b1]);
            b1++;
        }
        if (p > 0){
            int k = (int)lrint(sqrt((double)b0 * (b1 - 1)));
            if (k < 1) k = 1;
            m[nm++] = (masker){ (short)k, p2db(p), e->z[k], 0 };
        }
        b0 = b1;
    }
    // decimation: below the absolute threshold, and the weaker of two tonal components closer than 0.5 bark
    int keep = 0;
    for (int i = 0; i < nm; i++)
        if (m[i].spl >= e->ath[m[i].k]) m[keep++] = m[i];
    nm = keep;
    for (int i = 0; i < nm; i++){
        if (!m[i].tonal || m[i].spl < -900) continue;
        for (int j = i + 1; j < nm; j++)
            if (m[j].tonal && m[j].spl > -900 && fabsf(m[j].z - m[i].z) < 0.5f){
                if (m[j].spl > m[i].spl){
                    m[i].spl = -999;
                    break;
                }
                m[j].spl = -999;
            }
    }
    // the global threshold on a subsampled grid (every line up to 48, every 2nd up to 96, every 4th above), its minimum per subband
    float ltmin[NSB];
    for (int b = 0; b < NSB; b++) ltmin[b] = 1e9f;
    for (int i = 1; i < PSY_H;){
        const float zi = e->z[i];
        float sum = db2p(e->ath[i]);
        for (int j = 0; j < nm; j++){
            if (m[j].spl < -900) continue;
            const float dz = zi - m[j].z, x = m[j].spl;
            if (dz < -3.0f || dz >= 8.0f) continue;
            float vf;
            if (dz < -1.0f) vf = 17.0f * (dz + 1.0f) - (0.4f * x + 6.0f);
            else if (dz < 0.0f) vf = (0.4f * x + 6.0f) * dz;
            else if (dz < 1.0f) vf = -17.0f * dz;
            else vf = -(dz - 1.0f) * (17.0f - 0.15f * x) - 17.0f;
            const float av = m[j].tonal ? -6.025f - 0.275f * m[j].z : -2.025f - 0.175f * m[j].z;
            sum += db2p(x + av + vf);
        }
        const float lt = p2db(sum);
        if (lt < ltmin[i / 16]) ltmin[i / 16] = lt;
        i += i < 48 ? 1 : i < 96 ? 2 : 4;
    }
    for (int b = 0; b < sbl; b++){
        float xmax = -999;
        for (int k = 16 * b; k < 16 * b + 16; k++)
            if (X[k] > xmax) xmax = X[k];
        int si = scf[b][0];
        if (scf[b][1] < si) si = scf[b][1];
        if (scf[b][2] < si) si = scf[b][2];
        const float lscf = (float)(20.0 * log10(e->scf_val[si] * 32768.0)) - 10.0f;
        const float lsb = xmax > lscf ? xmax : lscf;
        smr[b] = lsb - ltmin[b];
    }
}

static void crc_bits(unsigned *crc, const uint8_t *buf, size_t from, size_t n){
    for (size_t i = from; i < from + n; i++){
        int bit = (buf[i >> 3] >> (7 - (i & 7))) & 1;
        int top = (*crc >> 15) & 1;
        *crc = (*crc << 1) & 0xFFFF;
        if (top ^ bit) *crc ^= 0x8005;
    }
}

// one frame into out, and the ScF-CRC words of its own scale factors into scfcrc[0..3] (group 0..3)
static int encode_one(mp2enc *e, const float *pcm, const uint8_t *pad, int padlen, uint8_t *out, uint8_t *scfcrc){
    const int SBL = e->sblimit;
    // 1. filterbank
    for (int s = 0; s < 36; s++)
        for (int c = 0; c < NCH; c++) analyse(e, c, pcm + s * 64 + c, e->sb[c][s]);

    // 2. scale factors (3 parts of 12) and scfsi
    int scf[NCH][NSB][3], scfsi[NCH][NSB], nscf[NCH][NSB];
    for (int c = 0; c < NCH; c++)
        for (int b = 0; b < SBL; b++){
            for (int p = 0; p < 3; p++){
                double mx = 0;
                for (int s = 12 * p; s < 12 * p + 12; s++){
                    double v = fabs(e->sb[c][s][b]);
                    if (v > mx) mx = v;
                }
                int i = 62;
                while (i > 0 && e->scf_val[i] < mx) i--; // the smallest scale factor that still holds the peak
                scf[c][b][p] = i;
            }
            int *f = scf[c][b];
            // equal (or within one step: take the larger) neighbours share one scale factor
            if (abs(f[0] - f[1]) <= 1 && abs(f[1] - f[2]) <= 1 && abs(f[0] - f[2]) <= 1){
                int m = f[0] < f[1] ? f[0] : f[1];
                if (f[2] < m) m = f[2];
                f[0] = f[1] = f[2] = m;
                scfsi[c][b] = 2;
                nscf[c][b] = 1;
            } else if (abs(f[0] - f[1]) <= 1){
                int m = f[0] < f[1] ? f[0] : f[1];
                f[0] = f[1] = m;
                scfsi[c][b] = 1;
                nscf[c][b] = 2;
            } else if (abs(f[1] - f[2]) <= 1){
                int m = f[1] < f[2] ? f[1] : f[2];
                f[1] = f[2] = m;
                scfsi[c][b] = 3;
                nscf[c][b] = 2;
            } else {
                scfsi[c][b] = 0;
                nscf[c][b] = 3;
            }
        }

    // 3. signal-to-mask ratio per subband (psychoacoustic model 1, each channel on its own)
    double smr[NCH][NSB];
    for (int c = 0; c < NCH; c++) psy1(e, pcm, c, SBL, (const int (*)[3])scf[c], smr[c]);

    // 4. bit allocation
    int tail = (padlen > 2 ? padlen : 2) + e->scfcrc_len;
    int fixed = 32 + 16;
    for (int b = 0; b < SBL; b++) fixed += NCH * e->tab[b].nbal;
    int avail = e->bytes * 8 - tail * 8 - fixed;
    if (avail < 0) return -1;
    int al[NCH][NSB];
    memset(al, 0, sizeof al);
    for (;;){
        int bc = -1, bb = -1, bcost = 0;
        double best = 1e9;
        for (int c = 0; c < NCH; c++)
            for (int b = 0; b < SBL && b < e->bw_sb; b++){
                const alloc_row *r = &e->tab[b];
                const int maxi = (1 << r->nbal) - 1;
                if (al[c][b] >= maxi) continue;
                const int cur = r->steps[al[c][b]], nxt = r->steps[al[c][b] + 1];
                int cost = sample_bits(nxt) - sample_bits(cur) + (cur == 0 ? 2 + 6 * nscf[c][b] : 0);
                if (cost > avail) continue;
                double mnr = snr_db(cur) - smr[c][b];
                // hysteresis: a subband that had bits in the previous frame is switched on first (switching a subband on and off
                // from frame to frame is heard as 'birdies', a gurgling sound)
                if (cur == 0 && e->prev_on[c][b]) mnr -= 6.0;
                if (mnr < best){
                    best = mnr;
                    bc = c;
                    bb = b;
                    bcost = cost;
                }
            }
        if (bc < 0) break;
        al[bc][bb]++;
        avail -= bcost;
    }
    for (int c = 0; c < NCH; c++)
        for (int b = 0; b < NSB; b++) e->prev_on[c][b] = al[c][b] > 0;

    // 5. the frame
    bitwriter w;
    bw_init(&w, out, (size_t)e->bytes);
    bw_put(&w, 0xFFF, 12);
    bw_put(&w, 1, 1);
    bw_put(&w, 2, 2);
    bw_put(&w, 0, 1); // sync, MPEG-1, layer II, protected (CRC follows)
    bw_put(&w, (uint32_t)e->br_index, 4);
    bw_put(&w, 1, 2);
    bw_put(&w, 0, 1);
    bw_put(&w, 0, 1); // bitrate, 48 kHz, no padding, private
    bw_put(&w, 0, 2);
    bw_put(&w, 0, 2);
    bw_put(&w, 0, 1);
    bw_put(&w, 1, 1);
    bw_put(&w, 0, 2);  // stereo, mode ext, copyright, original, emphasis
    bw_put(&w, 0, 16); // CRC, filled in below
    const size_t crc_from = bw_bits(&w);
    for (int b = 0; b < SBL; b++)
        for (int c = 0; c < NCH; c++) bw_put(&w, (uint32_t)al[c][b], e->tab[b].nbal);
    for (int b = 0; b < SBL; b++)
        for (int c = 0; c < NCH; c++)
            if (al[c][b]) bw_put(&w, (uint32_t)scfsi[c][b], 2);
    const size_t crc_to = bw_bits(&w);
    for (int b = 0; b < SBL; b++)
        for (int c = 0; c < NCH; c++)
            if (al[c][b]){
                const int *f = scf[c][b];
                switch (scfsi[c][b]){
                case 0:
                    bw_put(&w, (uint32_t)f[0], 6);
                    bw_put(&w, (uint32_t)f[1], 6);
                    bw_put(&w, (uint32_t)f[2], 6);
                    break;
                case 1:
                    bw_put(&w, (uint32_t)f[0], 6);
                    bw_put(&w, (uint32_t)f[2], 6);
                    break;
                case 2: bw_put(&w, (uint32_t)f[0], 6); break;
                default:
                    bw_put(&w, (uint32_t)f[0], 6);
                    bw_put(&w, (uint32_t)f[1], 6);
                    break;
                }
            }
    for (int g = 0; g < 12; g++)
        for (int b = 0; b < SBL; b++)
            for (int c = 0; c < NCH; c++){
                if (!al[c][b]) continue;
                const int steps = e->tab[b].steps[al[c][b]], part = g / 4;
                const double sc = e->scf_val[scf[c][b][part]];
                int q[3];
                for (int k = 0; k < 3; k++){
                    double x = e->sb[c][3 * g + k][b] / sc;
                    int v = (int)floor((x * steps + steps - 1) / 2.0 + 0.5);
                    q[k] = v < 0 ? 0 : v > steps - 1 ? steps - 1 : v;
                }
                if (grouped(steps)) bw_put(&w, (uint32_t)(q[0] + steps * (q[1] + steps * q[2])), group_bits(steps));
                else {
                    const int nb = nbits_steps(steps);
                    for (int k = 0; k < 3; k++) bw_put(&w, (uint32_t)q[k], nb);
                }
            }
    if (w.overflow || (int)bw_bytes(&w) > e->bytes - tail) return -1;
    unsigned crc = 0xFFFF;
    crc_bits(&crc, out, 16, 16);
    crc_bits(&crc, out, crc_from, crc_to - crc_from);
    out[4] = (uint8_t)(crc >> 8);
    out[5] = (uint8_t)crc;
    // the DAB end of the frame: X-PAD (as given, its first byte nearest the ScF-CRC), ScF-CRC (filled in by mp2enc_frame), F-PAD
    uint8_t *end = out + e->bytes;
    if (padlen >= 2){
        end[-2] = pad[padlen - 2];
        end[-1] = pad[padlen - 1];
        memcpy(end - 2 - e->scfcrc_len - (padlen - 2), pad, (size_t)(padlen - 2));
    }
    // ScF-CRC words of this frame (TS 103 466, B.3): CRC-8, x^8 + x^4 + x^3 + x^2 + 1, initial state 0, over the 3 most
    // significant bits of every transmitted scale factor of a group of subbands, in bit stream order
    static const int grp[5] = { 0, 4, 8, 16, 27 };
    for (int g = 0; g < 4; g++){
        unsigned c8 = 0;
        for (int b = grp[g]; b < grp[g + 1] && b < SBL; b++)
            for (int c = 0; c < NCH; c++){
                if (!al[c][b]) continue;
                const int *f = scf[c][b];
                int v[3], nv = 0;
                switch (scfsi[c][b]){
                case 0: v[nv++] = f[0]; v[nv++] = f[1]; v[nv++] = f[2]; break;
                case 2: v[nv++] = f[0]; break;
                default: v[nv++] = f[0]; v[nv++] = f[2]; break;   // scfsi 1 and 3: two scale factors are transmitted
                }
                for (int i = 0; i < nv; i++)
                    for (int bit = 5; bit >= 3; bit--){
                        const unsigned top = (c8 >> 7) & 1, in = (unsigned)(v[i] >> bit) & 1;
                        c8 = (c8 << 1) & 0xFF;
                        if (top ^ in) c8 ^= 0x1D;
                    }
            }
        scfcrc[g] = (uint8_t)c8;
    }
    return e->bytes;
}

int mp2enc_frame(mp2enc *e, const float *pcm, const uint8_t *pad, int padlen, uint8_t *out){
    static uint8_t cur[3 * 192];
    uint8_t crc[4];
    if (!e->have_held){ // the first call: a silent frame goes first (it carries the ScF-CRC of the first real frame)
        static const float zero[MP2_FRAME * 2];
        uint8_t dummy[4];
        if (encode_one(e, zero, NULL, 0, e->held, dummy) < 0) return -1;
        e->have_held = 1;
    }
    if (encode_one(e, pcm, pad, padlen, cur, crc) < 0) return -1;
    // the held frame carries the ScF-CRC of this one, in reverse order just before the F-PAD: CRC3, CRC2, CRC1, CRC0
    uint8_t *slot = e->held + e->bytes - 2 - e->scfcrc_len;
    for (int i = 0; i < e->scfcrc_len; i++) slot[i] = crc[e->scfcrc_len - 1 - i];
    memcpy(out, e->held, (size_t)e->bytes);
    memcpy(e->held, cur, (size_t)e->bytes);
    return e->bytes;
}
