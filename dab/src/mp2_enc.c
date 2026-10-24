// SPDX-License-Identifier: Apache-2.0
// MPEG-1 Layer II encoder for DAB. Structure (ISO/IEC 11172-3, 2.4):
//   1. polyphase analysis filterbank: 32 subbands, 36 samples per subband per frame (3 parts of 12)
//   2. scale factors per part (63 steps of 2 dB), scale factor selection information (scfsi) per subband
//   3. a simple psychoacoustic model: subband levels, masking spread over the critical bands, absolute threshold -> SMR per subband
//   4. bit allocation: repeatedly give the subband with the lowest noise-to-mask ratio one quantizer class more, while it fits
//   5. the frame: header, CRC-16, allocation, scfsi, scale factors, samples (3-sample groups for 3, 5 and 9 levels), then the
//      DAB end: X-PAD, ScF-CRC, F-PAD
// Stereo only (mode 00), 48 kHz; bit allocation tables B.2a (56..192 kbit/s per channel, 27 subbands) and B.2c (32/48, 8 subbands).
#include "mp2_enc.h"
#include "bitwriter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NSB   32
#define NCH   2

// analysis window: our own 512-tap prototype, a Kaiser-windowed sinc. Its two parameters were fitted to the standard synthesis
// filterbank (measured as the impulse response of a decoder, libmpg123): with cutoff 0.0089 fs and beta 11.3 the aliasing between
// neighbouring subbands cancels in the standard decoder to better than -83 dB (a tone 250 Hz from a subband edge); with the
// textbook cutoff fs/128 it stayed at -35 dB. WIN_GAIN makes the analysis + standard synthesis chain unity gain.
#define WIN_BETA   11.3
#define WIN_FC     0.0089
#define WIN_GAIN   2.0

// bit allocation tables (ISO/IEC 11172-3, B.2a and B.2c): per subband the bits of the allocation field and the quantizer
// classes (number of levels) it selects
typedef struct { int nbal; int steps[16]; } alloc_row;
static const alloc_row TAB_A[27] = {
#define A0 { 4, { 0, 3, 7, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767, 65535 } }
#define A1 { 4, { 0, 3, 5, 7, 9, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 65535 } }
#define A2 { 3, { 0, 3, 5, 7, 9, 15, 31, 65535 } }
#define A3 { 2, { 0, 3, 5, 65535 } }
    A0, A0, A0, A1, A1, A1, A1, A1, A1, A1, A1, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A3, A3, A3, A3
};
static const alloc_row TAB_C[8] = {
#define C0 { 4, { 0, 3, 5, 9, 15, 31, 63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383, 32767 } }
#define C1 { 3, { 0, 3, 5, 9, 15, 31, 63, 127 } }
    C0, C0, C1, C1, C1, C1, C1, C1
};

struct mp2enc {
    int kbps, bytes, br_index, sblimit, scfcrc_len;
    const alloc_row *tab;
    float C[512];                           // analysis window
    double M[NSB][64];                      // matrixing cosines
    float X[NCH][512];                      // filterbank input history
    float sb[NCH][36][NSB];                 // subband samples of the frame
    double scf_val[63];
};

static const int BR_TAB[15] = { 0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384 };

int mp2enc_valid_kbps(int kbps){ return kbps == 64 || kbps == 96 || kbps == 112 || kbps == 128 || kbps == 160 || kbps == 192; }

static double bessel_i0(double x){ double s = 1, t = 1; for (int k = 1; k < 40; k++){ t *= (x / (2 * k)) * (x / (2 * k)); s += t; } return s; }

mp2enc *mp2enc_open(int kbps){
    if (!mp2enc_valid_kbps(kbps)) return NULL;
    mp2enc *e = calloc(1, sizeof *e);
    if (!e) return NULL;
    e->kbps = kbps; e->bytes = 3 * kbps;
    for (int i = 1; i < 15; i++) if (BR_TAB[i] == kbps) e->br_index = i;
    if (kbps / 2 >= 56){ e->tab = TAB_A; e->sblimit = 27; } else { e->tab = TAB_C; e->sblimit = 8; }
    e->scfcrc_len = kbps < 112 ? 2 : 4;
    // prototype lowpass h[n], n = 0..511, centre 256, h[0] = 0 as in the standard; C[n] = h[n] with the sign of the
    // standard window (negative where floor(n / 64) is odd)
    for (int n = 0; n < 512; n++){
        double t = n - 256.0, s = t == 0 ? 2 * WIN_FC : sin(2 * M_PI * WIN_FC * t) / (M_PI * t);
        double r = t / 256.0, w = fabs(r) >= 1 ? 0 : bessel_i0(WIN_BETA * sqrt(1 - r * r)) / bessel_i0(WIN_BETA);
        double h = WIN_GAIN * s * w;
        e->C[n] = (float)(((n / 64) & 1) ? -h : h);
    }
    for (int k = 0; k < NSB; k++) for (int i = 0; i < 64; i++) e->M[k][i] = cos((2 * k + 1) * (i - 16) * M_PI / 64.0);
    for (int i = 0; i < 63; i++) e->scf_val[i] = pow(2.0, 1.0 - i / 3.0);
    return e;
}

void mp2enc_close(mp2enc *e){ free(e); }
int  mp2enc_tail_bytes(const mp2enc *e){ return e->scfcrc_len + 2; }

// one step of the analysis filterbank: 32 new samples of channel c (pcm with stride 2) -> 32 subband samples
static void analyse(mp2enc *e, int c, const float *pcm, float *out){
    float *X = e->X[c];
    memmove(X + 32, X, 480 * sizeof(float));
    for (int i = 0; i < 32; i++) X[31 - i] = pcm[2 * i];
    double Y[64];
    for (int i = 0; i < 64; i++){ double y = 0; for (int j = 0; j < 8; j++) y += (double)e->C[i + 64 * j] * X[i + 64 * j]; Y[i] = y; }
    for (int k = 0; k < NSB; k++){ double s = 0; for (int i = 0; i < 64; i++) s += e->M[k][i] * Y[i]; out[k] = (float)s; }
}

static int nbits_steps(int steps){ int b = 0; while ((1 << b) < steps + 1) b++; return b; }       // 7 -> 3, 65535 -> 16
static int grouped(int steps){ return steps == 3 || steps == 5 || steps == 9; }
static int group_bits(int steps){ return steps == 3 ? 5 : steps == 5 ? 7 : 10; }
static int sample_bits(int steps){ return steps == 0 ? 0 : grouped(steps) ? 12 * group_bits(steps) : 36 * nbits_steps(steps); }
static double snr_db(int steps){
    switch (steps){
    case 0: return 0; case 3: return 7.00; case 5: return 11.00; case 7: return 16.00; case 9: return 20.84; case 15: return 25.28;
    case 31: return 31.59; case 63: return 37.75; case 127: return 43.84; case 255: return 49.89; case 511: return 55.93;
    case 1023: return 61.96; case 2047: return 67.98; case 4095: return 74.01; case 8191: return 80.03; case 16383: return 86.05;
    case 32767: return 92.01; default: return 98.01;
    }
}
static double bark(double f){ return 13.0 * atan(0.00076 * f) + 3.5 * atan((f / 7500.0) * (f / 7500.0)); }
static double ath_db(double f){                                       // absolute threshold, dB SPL (0 dBFS sine = 96 dB SPL)
    double k = f / 1000.0; if (k < 0.02) k = 0.02;
    return 3.64 * pow(k, -0.8) - 6.5 * exp(-0.6 * (k - 3.3) * (k - 3.3)) + 1e-3 * k * k * k * k;
}

static void crc_bits(unsigned *crc, const uint8_t *buf, size_t from, size_t n){
    for (size_t i = from; i < from + n; i++){
        int bit = (buf[i >> 3] >> (7 - (i & 7))) & 1;
        int top = (*crc >> 15) & 1;
        *crc = (*crc << 1) & 0xFFFF;
        if (top ^ bit) *crc ^= 0x8005;
    }
}

int mp2enc_frame(mp2enc *e, const float *pcm, const uint8_t *pad, int padlen, uint8_t *out){
    const int SBL = e->sblimit;
    // 1. filterbank
    for (int s = 0; s < 36; s++) for (int c = 0; c < NCH; c++) analyse(e, c, pcm + s * 64 + c, e->sb[c][s]);

    // 2. scale factors (3 parts of 12) and scfsi
    int scf[NCH][NSB][3], scfsi[NCH][NSB], nscf[NCH][NSB];
    double lvl[NCH][NSB];
    for (int c = 0; c < NCH; c++) for (int b = 0; b < SBL; b++){
        double pw = 0;
        for (int p = 0; p < 3; p++){
            double mx = 0;
            for (int s = 12 * p; s < 12 * p + 12; s++){ double v = fabs(e->sb[c][s][b]); if (v > mx) mx = v; pw += (double)e->sb[c][s][b] * e->sb[c][s][b]; }
            int i = 62; while (i > 0 && e->scf_val[i] < mx) i--;              // the smallest scale factor that still holds the peak
            scf[c][b][p] = i;
        }
        lvl[c][b] = 96.0 + 10 * log10(pw / 36.0 * 2.0 + 1e-20);               // level of an equivalent sine, dB SPL
        int *f = scf[c][b];
        // equal (or within one step: take the larger) neighbours share one scale factor
        if (abs(f[0] - f[1]) <= 1 && abs(f[1] - f[2]) <= 1 && abs(f[0] - f[2]) <= 1){ int m = f[0] < f[1] ? f[0] : f[1]; if (f[2] < m) m = f[2]; f[0] = f[1] = f[2] = m; scfsi[c][b] = 2; nscf[c][b] = 1; }
        else if (abs(f[0] - f[1]) <= 1){ int m = f[0] < f[1] ? f[0] : f[1]; f[0] = f[1] = m; scfsi[c][b] = 1; nscf[c][b] = 2; }
        else if (abs(f[1] - f[2]) <= 1){ int m = f[1] < f[2] ? f[1] : f[2]; f[1] = f[2] = m; scfsi[c][b] = 3; nscf[c][b] = 2; }
        else { scfsi[c][b] = 0; nscf[c][b] = 3; }
    }

    // 3. masking threshold per subband (both channels' maskers count for each channel only within that channel)
    double smr[NCH][NSB];
    for (int c = 0; c < NCH; c++) for (int b = 0; b < SBL; b++){
        const double fb = (b + 0.5) * 750.0, zb = bark(fb);
        double t = pow(10.0, ath_db(fb) / 10.0);
        for (int j = 0; j < SBL; j++){
            const double dz = zb - bark((j + 0.5) * 750.0);
            double m = lvl[c][j] - 14.0 - (dz >= 0 ? 10.0 * dz : 25.0 * -dz);   // masker j spreads up 10 dB/bark, down 25 dB/bark
            t += pow(10.0, m / 10.0);
        }
        smr[c][b] = lvl[c][b] - 10 * log10(t);
    }

    // 4. bit allocation
    int tail = (padlen > 2 ? padlen : 2) + e->scfcrc_len;
    int fixed = 32 + 16; for (int b = 0; b < SBL; b++) fixed += NCH * e->tab[b].nbal;
    int avail = e->bytes * 8 - tail * 8 - fixed;
    if (avail < 0) return -1;
    int al[NCH][NSB]; memset(al, 0, sizeof al);
    for (;;){
        int bc = -1, bb = -1, bcost = 0; double best = 1e9;
        for (int c = 0; c < NCH; c++) for (int b = 0; b < SBL; b++){
            const alloc_row *r = &e->tab[b]; const int maxi = (1 << r->nbal) - 1;
            if (al[c][b] >= maxi) continue;
            const int cur = r->steps[al[c][b]], nxt = r->steps[al[c][b] + 1];
            int cost = sample_bits(nxt) - sample_bits(cur) + (cur == 0 ? 2 + 6 * nscf[c][b] : 0);
            if (cost > avail) continue;
            double mnr = snr_db(cur) - smr[c][b];
            if (mnr < best){ best = mnr; bc = c; bb = b; bcost = cost; }
        }
        if (bc < 0) break;
        al[bc][bb]++; avail -= bcost;
    }

    // 5. the frame
    bitwriter w; bw_init(&w, out, (size_t)e->bytes);
    bw_put(&w, 0xFFF, 12); bw_put(&w, 1, 1); bw_put(&w, 2, 2); bw_put(&w, 0, 1);    // sync, MPEG-1, layer II, protected (CRC follows)
    bw_put(&w, (uint32_t)e->br_index, 4); bw_put(&w, 1, 2); bw_put(&w, 0, 1); bw_put(&w, 0, 1);   // bitrate, 48 kHz, no padding, private
    bw_put(&w, 0, 2); bw_put(&w, 0, 2); bw_put(&w, 0, 1); bw_put(&w, 1, 1); bw_put(&w, 0, 2);   // stereo, mode ext, copyright, original, emphasis
    bw_put(&w, 0, 16);                                                       // CRC, filled in below
    const size_t crc_from = bw_bits(&w);
    for (int b = 0; b < SBL; b++) for (int c = 0; c < NCH; c++) bw_put(&w, (uint32_t)al[c][b], e->tab[b].nbal);
    for (int b = 0; b < SBL; b++) for (int c = 0; c < NCH; c++) if (al[c][b]) bw_put(&w, (uint32_t)scfsi[c][b], 2);
    const size_t crc_to = bw_bits(&w);
    for (int b = 0; b < SBL; b++) for (int c = 0; c < NCH; c++) if (al[c][b]){
        const int *f = scf[c][b];
        switch (scfsi[c][b]){
        case 0: bw_put(&w, (uint32_t)f[0], 6); bw_put(&w, (uint32_t)f[1], 6); bw_put(&w, (uint32_t)f[2], 6); break;
        case 1: bw_put(&w, (uint32_t)f[0], 6); bw_put(&w, (uint32_t)f[2], 6); break;
        case 2: bw_put(&w, (uint32_t)f[0], 6); break;
        default: bw_put(&w, (uint32_t)f[0], 6); bw_put(&w, (uint32_t)f[1], 6); break;
        }
    }
    for (int g = 0; g < 12; g++) for (int b = 0; b < SBL; b++) for (int c = 0; c < NCH; c++){
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
        else { const int nb = nbits_steps(steps); for (int k = 0; k < 3; k++) bw_put(&w, (uint32_t)q[k], nb); }
    }
    if (w.overflow || (int)bw_bytes(&w) > e->bytes - tail) return -1;
    unsigned crc = 0xFFFF;
    crc_bits(&crc, out, 16, 16); crc_bits(&crc, out, crc_from, crc_to - crc_from);
    out[4] = (uint8_t)(crc >> 8); out[5] = (uint8_t)crc;
    // the DAB end of the frame: X-PAD (as given, its first byte nearest the ScF-CRC), ScF-CRC (left 0), F-PAD
    uint8_t *end = out + e->bytes;
    if (padlen >= 2){
        end[-2] = pad[padlen - 2]; end[-1] = pad[padlen - 1];
        memcpy(end - 2 - e->scfcrc_len - (padlen - 2), pad, (size_t)(padlen - 2));
    }
    return e->bytes;
}
