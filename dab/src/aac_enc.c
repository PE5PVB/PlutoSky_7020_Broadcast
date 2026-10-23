// SPDX-License-Identifier: Apache-2.0
// AAC-LC encoder, frame length 960 (DAB+). Own design; only the normative code books come from aac_huff_tab.c.
//
// Block switching: ONLY_LONG, LONG_START, EIGHT_SHORT (8 x 120 lines, grouped), LONG_STOP, decided with one frame of
// look-ahead (attack detector on the high-passed input). Everything after the transform works on a list of "bands":
// a band is one scale factor band of one window group and occupies a contiguous range of the coefficient array,
// because short-window coefficients are stored interleaved exactly as they are transmitted (group, band, window, line).
#include "aac_enc.h"
#include "aac_mdct.h"
#include "aac_huff_tab.h"
#include "bitwriter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NL         AAC_FRAME                 // long window lines (960)
#define NS         120                       // short window lines
#define NSFB_L     49
#define NSFB_S     14
#define MAXBANDS   (8 * NSFB_S)              // >= NSFB_L
#define ZERO_HCB   0
#define ESC_HCB    11
#define NOISE_HCB  13                     // perceptual noise substitution
#define NBOOK      14
#define MAGIC      0.4054f                   // rounding offset of the x^(3/4) quantiser
#define MAXQ       8191

enum { ONLY_LONG = 0, LONG_START = 1, EIGHT_SHORT = 2, LONG_STOP = 3 };

// scale factor band offsets, frame length 960, 48 and 32 kHz (ISO/IEC 14496-3, 4.5.4)
static const int swb_l[NSFB_L + 1] = {
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80, 88, 96, 108, 120,
    132, 144, 160, 176, 196, 216, 240, 264, 292, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640,
    672, 704, 736, 768, 800, 832, 864, 896, 928, 960
};
static const int swb_s[NSFB_S + 1] = { 0, 4, 8, 12, 16, 20, 28, 36, 44, 56, 68, 80, 96, 112, 120 };

static const struct { int dim, lav, sign; } hcb[12] = {
    {0, 0, 0}, {4, 1, 1}, {4, 1, 1}, {4, 2, 0}, {4, 2, 0}, {2, 4, 1}, {2, 4, 1},
    {2, 7, 0}, {2, 7, 0}, {2, 12, 0}, {2, 12, 0}, {2, 16, 0}
};

typedef struct {               // the layout of one frame
    int seq;                   // window sequence
    int ngroups, glen[8];      // window groups (short), 1 group of 1 for long
    int max_sfb;               // per group
    int nbands;                // ngroups * max_sfb
    int start[MAXBANDS + 1];   // band b covers [start[b], start[b+1]) of the coefficient array
} layout;

typedef struct {
    int sf[MAXBANDS];
    int cb[MAXBANDS];
    int pns[MAXBANDS];         // band sent as noise (book 13) with energy nrg
    int nrg[MAXBANDS];
    int q[NL];
    int global_gain;
} chdata;

struct aacenc {
    int sr, ch, sr_index, max_sfb_l, max_sfb_s, no_short, force_short, no_pns;
    mdct_plan mdct_l, mdct_s;
    float win_l[2 * NL], win_s[2 * NS], win_start[2 * NL], win_stop[2 * NL];
    float old[2][NL], mid[2][NL];              // input frames n-2 and n-1 (the block being coded is old|mid)
    float sub_e[16];                           // high-passed energy of 120-sample sub-blocks: [0..7] = mid, [8..15] = new
    float hp_prev[2];
    int   prev_seq, next_short, next_attack;   // window sequence of the last block; the next block must be short, and where
    layout L;
    float X[2][NL], Xa34[2][NL];
    float en[2][MAXBANDS], thr[2][MAXBANDS], thr_prev_l[2][NSFB_L], tonal[2][MAXBANDS];
    float band_hz[MAXBANDS];                   // lowest frequency of each band
    double last_off;                           // rate control offset of the last frame
    int   ms[MAXBANDS];
    float ath_l[NSFB_L], ath_s[NSFB_S], bark_l[NSFB_L], bark_s[NSFB_S];
    double spw_l[NSFB_L * NSFB_L], spw_s[NSFB_S * NSFB_S];   // spreading weight of band m on band b ([b * nsfb + m]), 0 beyond 60 dB
    float in_gain;
    chdata cd[2];
    int   sf_guess[2][MAXBANDS];               // last scale factor found per band: the start of the next search
    uint8_t dse[512]; int dse_len;             // data stream element for the next frame (DAB+ PAD)
    aacenc_stats st;
};

static float tab_pow43[MAXQ + 1], tab_sf_g[256], tab_sf_q[256];   // |q|^(4/3), 2^((sf-100)/4), 2^(-3(sf-100)/16)
static void init_tables(void){
    static int done = 0; if (done) return;
    for (int i = 0; i <= MAXQ; i++) tab_pow43[i] = powf((float)i, 4.0f / 3.0f);
    for (int i = 0; i < 256; i++){ tab_sf_g[i] = powf(2.0f, 0.25f * (i - 100)); tab_sf_q[i] = powf(2.0f, -0.1875f * (i - 100)); }
    done = 1;
}

// ---------------------------------------------------------------------------------------------------------------------
// spectral Huffman coding

static int hcb_index(int cb, const int *v){
    switch (cb){
    case 1: case 2: return 27 * (v[0] + 1) + 9 * (v[1] + 1) + 3 * (v[2] + 1) + (v[3] + 1);
    case 3: case 4: return 27 * abs(v[0]) + 9 * abs(v[1]) + 3 * abs(v[2]) + abs(v[3]);
    case 5: case 6: return 9 * (v[0] + 4) + (v[1] + 4);
    case 7: case 8: return 8 * abs(v[0]) + abs(v[1]);
    case 9: case 10: return 13 * abs(v[0]) + abs(v[1]);
    default: { int y = abs(v[0]) > 15 ? 16 : abs(v[0]), z = abs(v[1]) > 15 ? 16 : abs(v[1]); return 17 * y + z; }
    }
}

static int esc_bits(int a){ if (a < 16) return 0; int n = 0; while ((a >> (n + 5)) > 0) n++; return 2 * n + 5; }

static void put_esc(bitwriter *w, int a){
    if (a < 16) return;
    int n = 0; while ((a >> (n + 5)) > 0) n++;
    for (int i = 0; i < n; i++) bw_put(w, 1, 1);
    bw_put(w, 0, 1);
    bw_put(w, (uint32_t)(a - (1 << (n + 4))), n + 4);
}

static int code_band(const int *q, int n, int cb, bitwriter *w){
    const int dim = hcb[cb].dim, lav = hcb[cb].lav;
    int bits = 0;
    for (int i = 0; i < n; i += dim){
        for (int k = 0; k < dim; k++) if (cb != ESC_HCB && abs(q[i + k]) > lav) return 1 << 28;
        int idx = hcb_index(cb, q + i), len = aac_hcb_len[cb][idx];
        bits += len;
        if (w) bw_put(w, aac_hcb_code[cb][idx], len);
        if (!hcb[cb].sign) for (int k = 0; k < dim; k++) if (q[i + k]){ bits++; if (w) bw_put(w, q[i + k] < 0, 1); }
        if (cb == ESC_HCB) for (int k = 0; k < dim; k++){ int a = abs(q[i + k]); bits += esc_bits(a); if (w) put_esc(w, a); }
    }
    return bits;
}

// bits of one band in every spectral book at once (books whose lav is below maxabs get 'inf'): the books come in pairs that
// share the index, the sign bits are the same for all unsigned books, so one pass over the band is enough. Equals code_band().
static void band_costs(const int *q, int n, int maxabs, int inf, int *cost){
    for (int cb = 1; cb <= ESC_HCB; cb++) cost[cb] = maxabs > hcb[cb].lav && cb != ESC_HCB ? inf : 0;
    int nz = 0, esc = 0;
    for (int i = 0; i < n; i++){ int a = abs(q[i]); nz += a != 0; esc += esc_bits(a); }
    if (maxabs <= 2){                                                   // books 1..4 (dim 4)
        const uint8_t *l1 = aac_hcb_len[1], *l2 = aac_hcb_len[2], *l3 = aac_hcb_len[3], *l4 = aac_hcb_len[4];
        int c1 = 0, c2 = 0, c3 = 0, c4 = 0;
        for (int i = 0; i < n; i += 4){
            const int *v = q + i;
            int iu = 27 * abs(v[0]) + 9 * abs(v[1]) + 3 * abs(v[2]) + abs(v[3]);
            c3 += l3[iu]; c4 += l4[iu];
            if (maxabs <= 1){ int is = 27 * (v[0] + 1) + 9 * (v[1] + 1) + 3 * (v[2] + 1) + (v[3] + 1); c1 += l1[is]; c2 += l2[is]; }
        }
        if (maxabs <= 1){ cost[1] = c1; cost[2] = c2; }
        cost[3] = c3 + nz; cost[4] = c4 + nz;
    }
    int c5 = 0, c6 = 0, c7 = 0, c8 = 0, c9 = 0, c10 = 0, c11 = 0;      // books 5..11 (dim 2)
    const uint8_t *l5 = aac_hcb_len[5], *l6 = aac_hcb_len[6], *l7 = aac_hcb_len[7], *l8 = aac_hcb_len[8];
    const uint8_t *l9 = aac_hcb_len[9], *l10 = aac_hcb_len[10], *l11 = aac_hcb_len[11];
    for (int i = 0; i < n; i += 2){
        int a0 = abs(q[i]), a1 = abs(q[i + 1]);
        if (maxabs <= 4){ int is = 9 * (q[i] + 4) + (q[i + 1] + 4); c5 += l5[is]; c6 += l6[is]; }
        if (maxabs <= 7){ int iu = 8 * a0 + a1; c7 += l7[iu]; c8 += l8[iu]; }
        if (maxabs <= 12){ int iu = 13 * a0 + a1; c9 += l9[iu]; c10 += l10[iu]; }
        c11 += l11[17 * (a0 > 15 ? 16 : a0) + (a1 > 15 ? 16 : a1)];
    }
    if (maxabs <= 4){ cost[5] = c5; cost[6] = c6; }
    if (maxabs <= 7){ cost[7] = c7 + nz; cost[8] = c8 + nz; }
    if (maxabs <= 12){ cost[9] = c9 + nz; cost[10] = c10 + nz; }
    cost[11] = c11 + nz + esc;
}

// ---------------------------------------------------------------------------------------------------------------------
// sectioning, scale factors, bit count and writing of one channel

static int lenbits(const layout *L){ return L->seq == EIGHT_SHORT ? 3 : 5; }
static int sect_hdr_bits(const layout *L, int len){ int lb = lenbits(L), esc = (1 << lb) - 1; return 4 + lb * (len / esc + 1); }

static int band_nonzero(const int *q, int a, int n){ for (int i = 0; i < n; i++) if (q[a + i]) return 1; return 0; }

// code books (Viterbi per group), scale factors of zero bands inside real sections, global gain; returns the ICS bits
// (global gain, section data, scale factors, spectral data and the three tool flags; not ics_info)
static int section_and_count(const layout *L, chdata *c){
    static const int INF = 1 << 29;
    int cost[MAXBANDS][NBOOK], best[NSFB_L][NBOOK], from[NSFB_L][NBOOK];
    const int hdr = 4 + lenbits(L);
    for (int b = 0; b < L->nbands; b++){
        const int a = L->start[b], n = L->start[b + 1] - a;
        int maxabs = 0; for (int i = 0; i < n; i++){ int v = abs(c->q[a + i]); if (v > maxabs) maxabs = v; }
        for (int cb = 0; cb < NBOOK; cb++) cost[b][cb] = INF;
        if (c->pns[b]) cost[b][NOISE_HCB] = 0;                             // noise bands: book 13 only, no spectral bits
        else { if (!maxabs) cost[b][ZERO_HCB] = 0; band_costs(c->q + a, n, maxabs, INF, cost[b]); }
    }
    for (int g = 0; g < L->ngroups; g++){
        const int b0 = g * L->max_sfb;
        for (int s = 0; s < L->max_sfb; s++){
            int pmin = 0;                                                   // cheapest book of the previous band (the first of equals)
            if (s > 0) for (int p = 1; p < NBOOK; p++) if (best[s - 1][p] < best[s - 1][pmin]) pmin = p;
            for (int cb = 0; cb < NBOOK; cb++){
                const int cc = cost[b0 + s][cb];
                if (cc >= INF){ best[s][cb] = INF; continue; }
                if (s == 0){ best[s][cb] = cc + hdr; from[s][cb] = -1; continue; }
                int m = best[s - 1][cb], arg = cb;                         // continue the section, or start a new one after pmin
                if (best[s - 1][pmin] < INF && best[s - 1][pmin] + hdr < m){ m = best[s - 1][pmin] + hdr; arg = pmin; }
                best[s][cb] = m >= INF ? INF : m + cc; from[s][cb] = arg;
            }
        }
        if (L->max_sfb > 0){
            int cb = 0; for (int k = 1; k < NBOOK; k++) if (best[L->max_sfb - 1][k] < best[L->max_sfb - 1][cb]) cb = k;
            for (int s = L->max_sfb - 1; s >= 0; s--){ c->cb[b0 + s] = cb; cb = from[s][cb]; }
        }
    }
    // zero bands inside a real section still send a scale factor: give them the neighbour's (difference 0)
    int first = -1;
    for (int b = 0; b < L->nbands && first < 0; b++) if (band_nonzero(c->q, L->start[b], L->start[b + 1] - L->start[b])) first = c->sf[b];
    int prevsf = first < 0 ? 100 : first;
    for (int b = 0; b < L->nbands; b++){
        if (band_nonzero(c->q, L->start[b], L->start[b + 1] - L->start[b])) prevsf = c->sf[b];
        else if (c->cb[b] != ZERO_HCB && c->cb[b] != NOISE_HCB) c->sf[b] = prevsf;
    }
    int bits = 8 + 3;                                                   // global_gain, pulse/tns/gain-control flags
    for (int g = 0; g < L->ngroups; g++){
        const int b0 = g * L->max_sfb;
        for (int s = 0; s < L->max_sfb; ){
            int e2 = s; while (e2 < L->max_sfb && c->cb[b0 + e2] == c->cb[b0 + s]) e2++;
            bits += sect_hdr_bits(L, e2 - s); s = e2;
        }
    }
    int last = -1;
    for (int b = 0; b < L->nbands; b++) if (c->cb[b] != ZERO_HCB && c->cb[b] != NOISE_HCB){
        if (last < 0){ c->global_gain = c->sf[b]; last = c->sf[b]; }
        bits += aac_scf_len[c->sf[b] - last + 60]; last = c->sf[b];
    }
    if (last < 0) c->global_gain = 100;
    // noise energies: their own chain, the first as a 9-bit value relative to global_gain - 90, then Huffman differences
    int nl = -1000;
    for (int b = 0; b < L->nbands; b++) if (c->cb[b] == NOISE_HCB){
        if (nl == -1000){ int v = c->nrg[b] - (c->global_gain - 90); if (v < -256) v = -256; if (v > 255) v = 255; c->nrg[b] = c->global_gain - 90 + v; bits += 9; }
        else { int d = c->nrg[b] - nl; if (d < -60) d = -60; if (d > 60) d = 60; c->nrg[b] = nl + d; bits += aac_scf_len[d + 60]; }
        nl = c->nrg[b];
    }
    for (int b = 0; b < L->nbands; b++) if (c->cb[b] != ZERO_HCB && c->cb[b] != NOISE_HCB) bits += cost[b][c->cb[b]];
    return bits;
}

static void write_ics_info(const layout *L, bitwriter *w){
    bw_put(w, 0, 1);                                                    // ics_reserved_bit
    bw_put(w, (uint32_t)L->seq, 2);
    bw_put(w, 0, 1);                                                    // window_shape: sine
    if (L->seq == EIGHT_SHORT){
        bw_put(w, (uint32_t)L->max_sfb, 4);
        uint32_t grouping = 0; int win = 0;                             // bit for window 1..7: 1 = same group as the previous
        for (int g = 0; g < L->ngroups; g++) for (int k = 0; k < L->glen[g]; k++, win++) if (win > 0) grouping = (grouping << 1) | (k > 0);
        bw_put(w, grouping, 7);
    } else {
        bw_put(w, (uint32_t)L->max_sfb, 6);
        bw_put(w, 0, 1);                                                // predictor_data_present
    }
}

static int ics_info_bits(const layout *L){ return L->seq == EIGHT_SHORT ? 4 + 4 + 7 : 4 + 6 + 1; }

static void write_ics(const layout *L, const chdata *c, bitwriter *w, int with_ics_info){
    bw_put(w, (uint32_t)c->global_gain, 8);
    if (with_ics_info) write_ics_info(L, w);
    const int lb = lenbits(L), esc = (1 << lb) - 1;
    for (int g = 0; g < L->ngroups; g++){                              // section_data
        const int b0 = g * L->max_sfb;
        for (int s = 0; s < L->max_sfb; ){
            int e2 = s; while (e2 < L->max_sfb && c->cb[b0 + e2] == c->cb[b0 + s]) e2++;
            int len = e2 - s;
            bw_put(w, (uint32_t)c->cb[b0 + s], 4);
            while (len >= esc){ bw_put(w, (uint32_t)esc, lb); len -= esc; }
            bw_put(w, (uint32_t)len, lb);
            s = e2;
        }
    }
    int last = c->global_gain, nl = -1000;                              // scale_factor_data (and noise energies)
    for (int b = 0; b < L->nbands; b++){
        if (c->cb[b] == ZERO_HCB) continue;
        if (c->cb[b] == NOISE_HCB){
            if (nl == -1000) bw_put(w, (uint32_t)(c->nrg[b] - (c->global_gain - 90) + 256), 9);
            else { int d = c->nrg[b] - nl + 60; bw_put(w, aac_scf_code[d], aac_scf_len[d]); }
            nl = c->nrg[b];
        } else { int d = c->sf[b] - last + 60; bw_put(w, aac_scf_code[d], aac_scf_len[d]); last = c->sf[b]; }
    }
    bw_put(w, 0, 1); bw_put(w, 0, 1); bw_put(w, 0, 1);                  // pulse, tns, gain control: none
    for (int b = 0; b < L->nbands; b++) if (c->cb[b] != ZERO_HCB && c->cb[b] != NOISE_HCB) code_band(c->q + L->start[b], L->start[b + 1] - L->start[b], c->cb[b], w);
}

// ---------------------------------------------------------------------------------------------------------------------
// quantisation

static void quant_band(const float *xa34, const float *x, int n, int sf, int *q){
    const float s = tab_sf_q[sf];
    for (int i = 0; i < n; i++){ int v = (int)(xa34[i] * s + MAGIC); if (v > MAXQ) v = MAXQ; q[i] = x[i] < 0 ? -v : v; }
}

static float band_noise(const float *x, const int *q, int n, int sf){
    const float g = tab_sf_g[sf];
    float e = 0;
    for (int i = 0; i < n; i++){ float r = tab_pow43[abs(q[i])] * g; float d = fabsf(x[i]) - r; e += d * d; }
    return e;
}

// the coarsest scale factor whose quantisation noise stays within 'allowed' (the noise grows with sf). The search starts at
// *guess (the band's result of the last search: the rate control calls this about ten times per frame with small changes),
// steps outwards with doubling steps and then halves the bracket; usually 3-4 trial quantisations instead of 9.
static int choose_sf(const float *x, const float *xa34, int n, float en, float allowed, int *q, int *guess){
    if (en <= allowed) return -1;
    float mx = 0; for (int i = 0; i < n; i++) if (xa34[i] > mx) mx = xa34[i];
    int lo = (int)ceil(100.0 + log2f(mx / MAXQ) / 0.1875f); if (lo < 0) lo = 0; if (lo > 255) lo = 255;
    int g = *guess < lo ? lo : *guess > 255 ? 255 : *guess;
    int good = -1, bad = -1;                                            // good: noise ok; bad: too much noise (good < bad)
    #define SF_OK(sf) (quant_band(xa34, x, n, (sf), q), band_noise(x, q, n, (sf)) <= allowed)
    if (SF_OK(g)){
        good = g;
        for (int step = 1; good < 255; step *= 2){ int t = good + step > 255 ? 255 : good + step; if (SF_OK(t)) good = t; else { bad = t; break; } }
        if (bad < 0) bad = 256;
    } else {
        bad = g;
        for (int step = 1; bad > lo; step *= 2){ int t = bad - step < lo ? lo : bad - step; if (SF_OK(t)){ good = t; break; } bad = t; }
        if (good < 0) good = lo - 1;                                    // nothing fits: the finest allowed (lo), as before
    }
    while (bad - good > 1){ int mid = (good + bad) / 2; if (SF_OK(mid)) good = mid; else bad = mid; }
    #undef SF_OK
    int bestsf = good < lo ? lo : good;
    *guess = bestsf;
    quant_band(xa34, x, n, bestsf, q);
    return band_nonzero(q, 0, n) ? bestsf : -1;
}

static int element_bits(const aacenc *e){
    const layout *L = &e->L;
    int bits = 3 + 4 + 3;                                               // element id, tag, ID_END
    if (e->dse_len) bits += 3 + 4 + 1 + 8 + (e->dse_len >= 255 ? 8 : 0) + 8 * e->dse_len;
    if (e->ch == 2){ bits += 1 + ics_info_bits(L) + 2; int any = 0; for (int b = 0; b < L->nbands; b++) any |= e->ms[b]; if (any) bits += L->nbands; }
    else bits += ics_info_bits(L);
    return bits;
}

static int quantise_all(aacenc *e, double off_db){
    const layout *L = &e->L;
    const float f = (float)pow(10.0, off_db / 10.0);
    for (int c = 0; c < e->ch; c++){
        chdata *d = &e->cd[c];
        memset(d->q, 0, sizeof d->q);
        for (int b = 0; b < L->nbands; b++){
            const int a = L->start[b], n = L->start[b + 1] - a;
            int sf = choose_sf(e->X[c] + a, e->Xa34[c] + a, n, e->en[c][b], e->thr[c][b] * f, d->q + a, &e->sf_guess[c][b]);
            d->pns[b] = 0;
            // noise substitution: a noise-like band above 4 kHz (long windows, not M/S) that would be emptied or coded with
            // a noise of more than half its energy is sent as "noise with this energy" instead
            if (!e->no_pns && L->seq != EIGHT_SHORT && !e->ms[b] && e->band_hz[b] >= 4000.0f && e->tonal[c][b] < 0.3f && e->en[c][b] > e->thr[c][b]
                && (sf < 0 || band_noise(e->X[c] + a, d->q + a, n, sf) > 0.5f * e->en[c][b])){
                d->pns[b] = 1; sf = -1;
                d->nrg[b] = (int)lrint(2.0 * log2((double)e->en[c][b] + 1e-9));
            }
            d->sf[b] = sf;
            if (sf < 0) memset(d->q + a, 0, sizeof(int) * (size_t)n);
        }
        // consecutive coded scale factors within +-60: make the earlier one finer
        for (int changed = 1; changed; ){
            changed = 0;
            int prev = -1;
            for (int b = 0; b < L->nbands; b++){
                if (d->sf[b] < 0) continue;
                if (prev >= 0){
                    if (d->sf[b] - d->sf[prev] > 60){ d->sf[b] = d->sf[prev] + 60; changed = 1; }
                    else if (d->sf[prev] - d->sf[b] > 60){ d->sf[prev] = d->sf[b] + 60; changed = 1; }
                }
                prev = b;
            }
            if (changed) for (int b = 0; b < L->nbands; b++) if (d->sf[b] >= 0)
                quant_band(e->Xa34[c] + L->start[b], e->X[c] + L->start[b], L->start[b + 1] - L->start[b], d->sf[b], d->q + L->start[b]);
        }
        for (int b = 0; b < L->nbands; b++) if (d->sf[b] < 0) d->sf[b] = 0;
    }
    int bits = element_bits(e);
    for (int c = 0; c < e->ch; c++) bits += section_and_count(L, &e->cd[c]);
    return bits;
}

// ---------------------------------------------------------------------------------------------------------------------
// psychoacoustics on one transform: band energy, spreading over Bark, tonality (spectral flatness), absolute threshold

static void psy_bands(const float *X, const int *swb, int nsfb, const float *ath, const float *bark, const double *spw, float *en, float *thr, float *tonal){
    for (int b = 0; b < nsfb; b++){
        const int a = swb[b], n = swb[b + 1] - a;
        // geometric mean through a running product kept in range with frexp: one logarithm per band instead of one per line
        double s = 0, prod = 1.0; int ex = 0;
        for (int i = 0; i < n; i++){
            double p = (double)X[a + i] * X[a + i]; s += p;
            prod *= p + 1e-9;
            if ((i & 7) == 7){ int k; prod = frexp(prod, &k); ex += k; }
        }
        double sl = (log2(prod) + ex) * 0.30102999566398120;               // sum of log10(p + 1e-9)
        en[b] = (float)s;
        double sfm_db = 10.0 * sl / n - 10.0 * log10(s / n + 1e-9);
        double t = sfm_db / -40.0; tonal[b] = (float)(t < 0 ? 0 : t > 1 ? 1 : t);
    }
    for (int b = 0; b < nsfb; b++){
        double spread = 0;
        const double *w = spw + b * nsfb;
        for (int m = 0; m < nsfb; m++) if (w[m] > 0) spread += en[m] * w[m];
        double off = tonal[b] * (14.5 + bark[b]) + (1.0 - tonal[b]) * 5.5;   // SMR offset (Johnston)
        double t = spread * pow(10.0, -off / 10.0);
        thr[b] = (float)(t < ath[b] ? ath[b] : t);
    }
}

// the spreading function of psy_bands: 12 dB per Bark downwards, 27 dB per Bark upwards, cut at 60 dB
static void spread_weights(int nsfb, const float *bark, double *spw){
    for (int b = 0; b < nsfb; b++) for (int m = 0; m < nsfb; m++){
        double dz = bark[b] - bark[m], att = dz >= 0 ? 12.0 * dz : 27.0 * -dz;
        spw[b * nsfb + m] = att < 60.0 ? pow(10.0, -att / 10.0) : 0.0;
    }
}

static void ath_bark(int nsfb, const int *swb, int lines, double binhz, double e_line_fs, float *ath, float *bark){
    for (int b = 0; b < nsfb; b++){
        double fl = swb[b] * binhz, fh = swb[b + 1] * binhz, fc = 0.5 * (fl + fh), mn = 1e9;
        for (double f = (fl < 20 ? 20 : fl); f <= fh; f += binhz * 0.5){
            double k = f / 1000.0, a = 3.64 * pow(k, -0.8) - 6.5 * exp(-0.6 * (k - 3.3) * (k - 3.3)) + 1e-3 * pow(k, 4);
            if (a < mn) mn = a;
        }
        if (mn > 1e8) mn = 0;
        ath[b] = (float)(e_line_fs * pow(10.0, (mn - 96.0) / 10.0) * (swb[b + 1] - swb[b]));
        bark[b] = (float)(13.0 * atan(0.00076 * fc) + 3.5 * atan(pow(fc / 7500.0, 2)));
        (void)lines;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

aacenc *aacenc_open(int sample_rate, int channels, int bandwidth_hz){
    if ((sample_rate != 48000 && sample_rate != 32000) || channels < 1 || channels > 2) return NULL;
    aacenc *e = calloc(1, sizeof *e);
    if (!e) return NULL;
    e->sr = sample_rate; e->ch = channels; e->sr_index = sample_rate == 48000 ? 3 : 5;
    e->no_short = getenv("AACENC_NOSHORT") != NULL;                    // tests: compare with long windows only
    e->force_short = getenv("AACENC_FORCESHORT") != NULL;              // tests: every block short (scaling check)
    e->no_pns = getenv("AACENC_NOPNS") != NULL;                        // tests: without noise substitution
    init_tables();
    if (mdct_init(&e->mdct_l, NL) || mdct_init(&e->mdct_s, NS)){ free(e); return NULL; }
    for (int n = 0; n < 2 * NL; n++) e->win_l[n] = (float)sin(M_PI * (n + 0.5) / (2 * NL));
    for (int n = 0; n < 2 * NS; n++) e->win_s[n] = (float)sin(M_PI * (n + 0.5) / (2 * NS));
    // LONG_START: long rise, 420 ones, short fall, 420 zeros; LONG_STOP mirrored
    const int flat = (NL - NS) / 2;
    for (int n = 0; n < 2 * NL; n++){
        float s;
        if (n < NL) s = e->win_l[n];
        else if (n < NL + flat) s = 1.0f;
        else if (n < NL + flat + NS) s = e->win_s[NS + (n - NL - flat)];
        else s = 0.0f;
        e->win_start[n] = s;
        e->win_stop[2 * NL - 1 - n] = s;
    }
    // bandwidth -> max_sfb (long and short)
    const double bin_l = (double)sample_rate / (2 * NL), bin_s = (double)sample_rate / (2 * NS);
    while (e->max_sfb_l < NSFB_L && swb_l[e->max_sfb_l + 1] * bin_l <= bandwidth_hz + 1e-6) e->max_sfb_l++;
    while (e->max_sfb_s < NSFB_S && swb_s[e->max_sfb_s + 1] * bin_s <= bandwidth_hz + 1e-6) e->max_sfb_s++;
    // scale: an input of +-1.0 corresponds to +-32768 in the decoder's 16-bit domain; with this MDCT (no normalisation)
    // the decoder's spectral scale is reached with a factor 2 (measured with tests/check_aac.py: gain 1.000)
    e->in_gain = 32768.0f * 2.0f;
    // absolute threshold: a full-scale sine is 96 dB SPL; its line energy is about (gain * M/2)^2
    ath_bark(NSFB_L, swb_l, NL, bin_l, pow(e->in_gain * NL / 2.0, 2), e->ath_l, e->bark_l);
    ath_bark(NSFB_S, swb_s, NS, bin_s, pow(e->in_gain * NS / 2.0, 2), e->ath_s, e->bark_s);
    spread_weights(NSFB_L, e->bark_l, e->spw_l); spread_weights(NSFB_S, e->bark_s, e->spw_s);
    e->prev_seq = ONLY_LONG;
    return e;
}

void aacenc_close(aacenc *e){ if (!e) return; mdct_free(&e->mdct_l); mdct_free(&e->mdct_s); free(e); }

void aacenc_get_stats(const aacenc *e, aacenc_stats *s){ *s = e->st; }

int aacenc_set_dse(aacenc *e, const uint8_t *data, int len){
    if (len < 0 || len > 510) return -1;
    memcpy(e->dse, data, (size_t)len); e->dse_len = len;
    return 0;
}

int aacenc_asc(const aacenc *e, uint8_t asc[2]){
    uint16_t v = (uint16_t)((2 << 11) | (e->sr_index << 7) | (e->ch << 3) | (1 << 2));   // LC, rate, channels, 960
    asc[0] = (uint8_t)(v >> 8); asc[1] = (uint8_t)v;
    return 2;
}

static int write_fill(bitwriter *w, int bytes){
    for (;;){
        int r = bytes * 8 - (int)bw_bits(w) - 3;
        if (r < 7) break;
        int cnt;
        if (r - 7 < 8 * 15){ cnt = (r - 7) / 8; bw_put(w, 6, 3); bw_put(w, (uint32_t)cnt, 4); }
        else { cnt = (r - 15) / 8; if (cnt > 269) cnt = 269; bw_put(w, 6, 3); bw_put(w, 15, 4); bw_put(w, (uint32_t)(cnt - 14), 8); }
        if (cnt > 0){ bw_put(w, 0, 4); bw_put(w, 0, 4); for (int i = 1; i < cnt; i++) bw_put(w, 0xA5, 8); }
        if (cnt == 0) break;
    }
    bw_put(w, 7, 3);
    bw_align(w);
    return (int)bw_bytes(w) == bytes ? 0 : -1;
}

// attack detector: energy of the high-passed sum signal in 120-sample sub-blocks of the new frame -> sub_e[8..15]
static void analyse_new(aacenc *e, const float *pcm){
    memmove(e->sub_e, e->sub_e + 8, 8 * sizeof(float));
    for (int s = 0; s < 8; s++){
        double acc = 0;
        for (int n = s * NS; n < (s + 1) * NS; n++){
            for (int c = 0; c < e->ch; c++){ float x = pcm[n * e->ch + c], y = x - e->hp_prev[c]; e->hp_prev[c] = x; acc += (double)y * y; }
        }
        e->sub_e[8 + s] = (float)acc;
    }
}

// is there an attack in the short-window zone of the block (mid|new)? sub-blocks 4..12 of sub_e, 4 sub-blocks of history
static int attack_in_next(const aacenc *e, int *where){
    const float floor_e = 1e-6f * NS;                                  // about -60 dBFS
    for (int i = 4; i <= 12; i++){
        float h = 0; for (int k = i - 4; k < i; k++) if (e->sub_e[k] > h) h = e->sub_e[k];
        if (e->sub_e[i] > floor_e && e->sub_e[i] > 10.0f * h){ if (where) *where = i - 4; return 1; }
    }
    return 0;
}

static void build_layout_long(aacenc *e, int seq){
    layout *L = &e->L;
    L->seq = seq; L->ngroups = 1; L->glen[0] = 1; L->max_sfb = e->max_sfb_l; L->nbands = L->max_sfb;
    for (int b = 0; b <= L->max_sfb; b++) L->start[b] = swb_l[b];
    for (int b = 0; b < L->max_sfb; b++) e->band_hz[b] = (float)(swb_l[b] * (double)e->sr / (2 * NL));
}

static void build_layout_short(aacenc *e, int attack_win){
    layout *L = &e->L;
    L->seq = EIGHT_SHORT; L->max_sfb = e->max_sfb_s; L->ngroups = 0;
    // groups: windows before the attack, the attack window, the windows after it
    int a = attack_win < 0 ? 0 : attack_win > 7 ? 7 : attack_win;
    if (a > 0) L->glen[L->ngroups++] = a;
    L->glen[L->ngroups++] = 1;
    if (a < 7) L->glen[L->ngroups++] = 7 - a;
    L->nbands = L->ngroups * L->max_sfb;
    int pos = 0, b = 0;
    for (int g = 0; g < L->ngroups; g++)                               // only the coded bands are stored, back to back
        for (int s = 0; s < L->max_sfb; s++){ e->band_hz[b] = (float)(swb_s[s] * (double)e->sr / (2 * NS)); L->start[b++] = pos; pos += L->glen[g] * (swb_s[s + 1] - swb_s[s]); }
    L->start[b] = pos;
}

int aacenc_frame(aacenc *e, const float *pcm, int target_bytes, int max_bytes, int pad_to, uint8_t *out){
    // 1. window sequence of the block old|mid, with mid|new as look-ahead
    analyse_new(e, pcm);
    const int cur_short = e->next_short, cur_attack = e->next_attack;  // decided one frame ago for this block
    int nxt_win = -1;
    const int nxt = e->force_short || (!e->no_short && attack_in_next(e, &nxt_win));
    int seq;
    if (cur_short) seq = EIGHT_SHORT;
    else if (e->prev_seq == EIGHT_SHORT) seq = nxt ? EIGHT_SHORT : LONG_STOP;   // a long window may not follow a short one directly
    else if (nxt) seq = LONG_START;
    else seq = ONLY_LONG;
    e->next_short = (seq == LONG_START) || (seq == EIGHT_SHORT && nxt);
    e->next_attack = nxt ? nxt_win : 7;

    // 2. transform the block old|mid
    float blk[2 * NL];
    if (seq == EIGHT_SHORT) build_layout_short(e, cur_short ? cur_attack : 7);
    else build_layout_long(e, seq);
    const layout *L = &e->L;
    for (int c = 0; c < e->ch; c++){
        for (int n = 0; n < NL; n++){ blk[n] = e->old[c][n]; blk[NL + n] = e->mid[c][n]; }
        if (seq != EIGHT_SHORT){
            const float *w = seq == LONG_START ? e->win_start : seq == LONG_STOP ? e->win_stop : e->win_l;
            float wb[2 * NL]; for (int n = 0; n < 2 * NL; n++) wb[n] = blk[n] * w[n];
            mdct_run(&e->mdct_l, wb, e->X[c]);
            for (int k = swb_l[L->max_sfb]; k < NL; k++) e->X[c][k] = 0;
            float en[NSFB_L], thr[NSFB_L], ton[NSFB_L];
            psy_bands(e->X[c], swb_l, NSFB_L, e->ath_l, e->bark_l, e->spw_l, en, thr, ton);
            for (int b = 0; b < L->nbands; b++){
                float t = thr[b], lim = 2.0f * e->thr_prev_l[c][b];       // pre-echo: at most 3 dB above the last long frame
                if (seq == ONLY_LONG && e->thr_prev_l[c][b] > 0 && t > lim && lim > e->ath_l[b]) t = lim;
                e->en[c][b] = en[b]; e->thr[c][b] = t; e->thr_prev_l[c][b] = t; e->tonal[c][b] = ton[b];
            }
        } else {
            float Xs[8][NS], en_w[8][NSFB_S], thr_w[8][NSFB_S], ton_w[8][NSFB_S];
            const int off = (NL - NS) / 2;                              // 420: first short window
            for (int j = 0; j < 8; j++){
                float wb[2 * NS];
                for (int n = 0; n < 2 * NS; n++) wb[n] = blk[off + j * NS + n] * e->win_s[n];
                mdct_run(&e->mdct_s, wb, Xs[j]);
                for (int k = swb_s[L->max_sfb]; k < NS; k++) Xs[j][k] = 0;
                psy_bands(Xs[j], swb_s, NSFB_S, e->ath_s, e->bark_s, e->spw_s, en_w[j], thr_w[j], ton_w[j]);
            }
            // interleave (group, band, window, line) and combine the per-window thresholds per group band
            int win = 0, b = 0;
            memset(e->X[c], 0, sizeof e->X[c]);
            for (int g = 0; g < L->ngroups; g++){
                for (int s = 0; s < L->max_sfb; s++, b++){
                    int pos = L->start[b];
                    float en = 0, tmin = 1e30f;
                    for (int k = 0; k < L->glen[g]; k++){
                        const int wj = win + k;
                        for (int i = swb_s[s]; i < swb_s[s + 1]; i++) e->X[c][pos++] = Xs[wj][i];
                        en += en_w[wj][s]; if (thr_w[wj][s] < tmin) tmin = thr_w[wj][s];
                    }
                    e->en[c][b] = en; e->thr[c][b] = tmin * L->glen[g]; e->tonal[c][b] = ton_w[win][s];
                }
                win += L->glen[g];
            }
            for (int b2 = 0; b2 < NSFB_L; b2++) e->thr_prev_l[c][b2] = 0;   // no pre-echo limit right after short blocks
        }
        for (int n = 0; n < NL; n++){ e->old[c][n] = e->mid[c][n]; e->mid[c][n] = pcm[n * e->ch + c] * e->in_gain; }
    }
    e->prev_seq = seq;

    // 3. M/S decision per band
    memset(e->ms, 0, sizeof e->ms);
    e->st.ms_bands = 0;
    if (e->ch == 2){
        for (int b = 0; b < L->nbands; b++){
            const int a = L->start[b], n = L->start[b + 1] - a;
            double eM = 0, eS = 0;
            for (int i = 0; i < n; i++){ double m = 0.5 * (e->X[0][a + i] + e->X[1][a + i]), s = 0.5 * (e->X[0][a + i] - e->X[1][a + i]); eM += m * m; eS += s * s; }
            double tL = e->thr[0][b], tR = e->thr[1][b], tMS = 0.5 * (tL < tR ? tL : tR);
            double peLR = log2(1.0 + e->en[0][b] / tL) + log2(1.0 + e->en[1][b] / tR);
            double peMS = log2(1.0 + eM / tMS) + log2(1.0 + eS / tMS);
            if (peMS < peLR){
                e->ms[b] = 1; e->st.ms_bands++;
                for (int i = 0; i < n; i++){ float l = e->X[0][a + i], r = e->X[1][a + i]; e->X[0][a + i] = 0.5f * (l + r); e->X[1][a + i] = 0.5f * (l - r); }
                e->en[0][b] = (float)eM; e->en[1][b] = (float)eS; e->thr[0][b] = e->thr[1][b] = (float)tMS;
            }
        }
    }
    for (int c = 0; c < e->ch; c++) for (int k = 0; k < NL; k++) e->Xa34[c][k] = powf(fabsf(e->X[c][k]), 0.75f);

    // 4. rate control: the lowest threshold offset (best quality) whose bits fit the target
    const int target_bits = target_bytes * 8 - 7, max_bits = max_bytes * 8 - 7;
    // search around the last frame's offset first (6 halvings: 24 dB down to 0.4 dB), the full range only when that bracket does not hold it
    double lo = e->last_off - 12.0, hi = e->last_off + 12.0, best = 60.0;
    if (lo < -20.0) lo = -20.0;
    if (hi > 60.0) hi = 60.0;
    if (quantise_all(e, hi) > target_bits){ lo = hi; hi = 60.0; }
    else { best = hi; if (quantise_all(e, lo) <= target_bits){ hi = lo; lo = -20.0; } }
    for (int it = 0; it < 6; it++){
        double m = 0.5 * (lo + hi);
        if (quantise_all(e, m) <= target_bits){ best = m; hi = m; } else lo = m;
    }
    e->last_off = best;
    int bits = quantise_all(e, best);
    layout Lw = *L;
    if (bits > max_bits){                                               // emergency: an empty frame
        Lw.max_sfb = 0; Lw.nbands = 0;
        for (int c = 0; c < e->ch; c++){ memset(&e->cd[c], 0, sizeof e->cd[c]); e->cd[c].global_gain = 100; }
        memset(e->ms, 0, sizeof e->ms);
    }

    double worst = -1e9;
    for (int c = 0; c < e->ch; c++) for (int b = 0; b < Lw.nbands; b++){
        const int a = Lw.start[b], n = Lw.start[b + 1] - a;
        double nz = e->cd[c].cb[b] == ZERO_HCB ? e->en[c][b] : band_noise(e->X[c] + a, e->cd[c].q + a, n, e->cd[c].sf[b]);
        double r = 10.0 * log10((nz + 1e-12) / (e->thr[c][b] + 1e-12)); if (r > worst) worst = r;
    }

    // 5. bitstream
    bitwriter w; bw_init(&w, out, (size_t)max_bytes);
    if (e->dse_len){                                                    // DSE first (DAB+ PAD), byte aligned at the AU start
        bw_put(&w, 4, 3); bw_put(&w, 0, 4); bw_put(&w, 1, 1);
        if (e->dse_len < 255) bw_put(&w, (uint32_t)e->dse_len, 8); else { bw_put(&w, 255, 8); bw_put(&w, (uint32_t)(e->dse_len - 255), 8); }
        for (int i = 0; i < e->dse_len; i++) bw_put(&w, e->dse[i], 8);
        e->dse_len = 0;
    }
    if (e->ch == 2){
        bw_put(&w, 1, 3); bw_put(&w, 0, 4); bw_put(&w, 1, 1);             // ID_CPE, tag, common_window
        write_ics_info(&Lw, &w);
        int any = 0; for (int b = 0; b < Lw.nbands; b++) any |= e->ms[b];
        bw_put(&w, any ? 1 : 0, 2);
        if (any) for (int b = 0; b < Lw.nbands; b++) bw_put(&w, (uint32_t)e->ms[b], 1);   // order: group, band
        write_ics(&Lw, &e->cd[0], &w, 0);
        write_ics(&Lw, &e->cd[1], &w, 0);
    } else {
        bw_put(&w, 0, 3); bw_put(&w, 0, 4);
        write_ics(&Lw, &e->cd[0], &w, 1);
    }
    e->st.bits_used = (int)bw_bits(&w) + 3;
    e->st.max_sfb = Lw.max_sfb; e->st.offset_db = best; e->st.nmr_db = worst;
    e->st.pns_bands = 0; for (int c = 0; c < e->ch; c++) for (int b = 0; b < Lw.nbands; b++) e->st.pns_bands += e->cd[c].cb[b] == NOISE_HCB;
    e->st.window_sequence = seq;
    if (pad_to > 0){
        if (pad_to > max_bytes || write_fill(&w, pad_to) != 0 || w.overflow) return -1;
    } else { bw_put(&w, 7, 3); bw_align(&w); }
    if (w.overflow) return -1;
    return (int)bw_bytes(&w);
}
