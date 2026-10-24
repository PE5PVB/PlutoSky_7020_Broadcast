// SPDX-License-Identifier: Apache-2.0
// DAB mode I transmitter (ETSI EN 300 401). Own implementation; the normative tables (puncturing vectors, phase
// reference table) are data of the standard.
#include "dab_mod.h"
#include "dab_crc.h"
#include "aac_mdct.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define K          1536                    // carriers
#define NFFT       2048
#define TGUARD     504
#define TNULL      2656
#define NSYM       76                      // OFDM symbols per frame (phase reference + 3 FIC + 72 MSC)
#define CIF_BITS   55296                   // 864 CU of 64 bits
#define FIC_BITS   2304                    // per CIF after coding (mode I)
#define FIB_BYTES  32

// puncturing vectors PI_1 .. PI_24 (EN 300 401, 11.1.2): applied MSB first to consecutive groups of 32 coded bits
static const uint32_t PI[25] = { 0,
    0xc8888888, 0xc888c888, 0xc8c8c888, 0xc8c8c8c8, 0xccc8c8c8, 0xccc8ccc8, 0xccccccc8, 0xcccccccc,
    0xeccccccc, 0xeccceccc, 0xecececcc, 0xecececec, 0xeeececec, 0xeeeceeec, 0xeeeeeeec, 0xeeeeeeee,
    0xfeeeeeee, 0xfeeefeee, 0xfefefeee, 0xfefefefe, 0xfffefefe, 0xfffefffe, 0xfffffffe, 0xffffffff };
#define PI_TAIL 0xccccccu                  // the 24 coded tail bits

// phase reference symbol, mode I (EN 300 401, 14.3.2): h tables and per 32-carrier block (i, n)
static const uint8_t PRS_H[4][32] = {
    { 0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1, 1, 0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1, 1 },
    { 0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3, 0, 0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3, 0 },
    { 0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1, 3, 0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1, 3 },
    { 0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3, 2, 0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3, 2 } };
static const uint8_t PRS_IN[48][2] = {
    {0,3},{3,1},{2,1},{1,1},{0,2},{3,2},{2,1},{1,0},{0,2},{3,2},{2,3},{1,3},
    {0,0},{3,2},{2,1},{1,3},{0,3},{3,3},{2,3},{1,0},{0,3},{3,0},{2,1},{1,1},
    {0,1},{1,2},{2,0},{3,1},{0,3},{1,2},{2,2},{3,3},{0,2},{1,1},{2,2},{3,3},
    {0,1},{1,2},{2,3},{3,3},{0,2},{1,2},{2,2},{3,1},{0,1},{1,3},{2,1},{3,2} };
// carrier array order used below: index 0..767 = carriers +1..+768, 768..1535 = carriers -768..-1

static const int TI_DELAY[16] = { 0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15 };

typedef struct { int nbits; uint32_t pat; } prule;

typedef struct {
    int bitrate, n, cu_start, cu_size, prot;
    prule rules[2]; int nrules;
    int in_bits, out_bits;                 // per logical frame: information bits, coded bits (= cu_size * 64)
    uint8_t *ti[16]; int ti_pos;           // time interleaver history (coded bits)
} subch;

struct dab_mod {
    dab_ensemble ens;
    subch sc[DAB_MAX_SVC];
    int cu_used;
    int cif_count, cif_in_frame;
    int phases_only;                       // 1: only the phase words (the FPGA does the OFDM), no IQ frame
    uint8_t *frame_bits;                  // FIC (4 x 2304) + MSC (4 x 55296) bits of the frame being built
    int fidx[K];                           // frequency interleaver: QPSK symbol n -> carrier index
    dab_cf prs[K];
    uint8_t prs_ph[K];                     // phase reference in units of 90 degrees
    uint32_t phw[NSYM * K / 16];           // FPGA form of the frame: per symbol and carrier the 2-bit phase index p
    fft_plan fft;
    dab_cf *iq;
    // FIG carousel
    uint8_t figs[64][32]; int figlen[64], nfigs, fig_next;
    // a multiplex reconfiguration announced in FIG 0/0 (change flags, occurrence change) and taking effect at CIF count change_at
    dab_ensemble next_ens; subch next_sc[DAB_MAX_SVC]; int next_cu, change_at, change_flags;
};

// ---------------------------------------------------------------------------------------------------------------------
// bit helpers

static void bytes_to_bits(const uint8_t *in, int nbytes, uint8_t *bits){ for (int i = 0; i < nbytes * 8; i++) bits[i] = (in[i >> 3] >> (7 - (i & 7))) & 1; }

static void prbs_xor(uint8_t *bits, int n){                 // energy dispersal: x^9 + x^5 + 1, all ones at the start
    unsigned r = 0x1FF;
    for (int i = 0; i < n; i++){ unsigned b = ((r >> 8) ^ (r >> 4)) & 1u; r = ((r << 1) | b) & 0x1FFu; bits[i] ^= (uint8_t)b; }
}

static int parity7(unsigned v){ v &= 0x7F; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1; return (int)(v & 1); }

// mother code K=7, R=1/4 (octal 133, 171, 145, 133), 6 tail bits: out gets 4*(n+6) bits
static void conv_encode(const uint8_t *in, int n, uint8_t *out){
    unsigned mem = 0; int o = 0;
    for (int i = 0; i < n + 6; i++){
        unsigned b = i < n ? in[i] : 0;
        mem = (mem >> 1) | (b << 6);
        out[o++] = (uint8_t)parity7(mem & 0x5b); out[o++] = (uint8_t)parity7(mem & 0x79);
        out[o++] = (uint8_t)parity7(mem & 0x65); out[o++] = (uint8_t)parity7(mem & 0x5b);
    }
}

static int puncture(const uint8_t *c, const prule *rules, int nrules, uint8_t *out){
    int ip = 0, op = 0;
    for (int r = 0; r < nrules; r++)
        for (int k = 0; k < rules[r].nbits; k++, ip++) if ((rules[r].pat >> (31 - (k & 31))) & 1u) out[op++] = c[ip];
    for (int k = 0; k < 24; k++, ip++) if ((PI_TAIL >> (23 - k)) & 1u) out[op++] = c[ip];
    return op;
}

// ---------------------------------------------------------------------------------------------------------------------
// FIC: FIGs and FIBs

static void label16(uint8_t *dst, const char *s){ int i = 0; for (; i < 16 && s[i]; i++) dst[i] = (uint8_t)s[i]; for (; i < 16; i++) dst[i] = ' '; }

static void fig_add(dab_mod *m, const uint8_t *data, int len){ memcpy(m->figs[m->nfigs], data, (size_t)len); m->figlen[m->nfigs++] = len; }

// FIG 0/1 sub-channel organisation (long form, EEP-A), several sub-channels per FIG when they fit; cn = 1: the next configuration
static void figs_subch(dab_mod *m, const subch *sc, int nsvc, int cn){
    uint8_t f[32]; int p;
    for (int i = 0; i < nsvc; ){
        p = 0; f[p++] = 0; f[p++] = (uint8_t)((cn << 7) | 0x01);       // header filled below; C/N, OE=0, P/D=0, ext=1
        while (i < nsvc && p + 4 <= 30){
            const subch *s = &sc[i];
            f[p++] = (uint8_t)((i << 2) | (s->cu_start >> 8)); f[p++] = (uint8_t)s->cu_start;
            f[p++] = (uint8_t)(0x80 | ((s->prot > 4 ? 1 : 0) << 4) | (((s->prot - 1) & 3) << 2) | (s->cu_size >> 8));   // long form, option 0 (A) / 1 (B), level
            f[p++] = (uint8_t)s->cu_size;
            i++;
        }
        f[0] = (uint8_t)((0 << 5) | (p - 1)); fig_add(m, f, p);
    }
}
// FIG 0/2 service organisation: one audio component per service (DAB+ or MPEG-1 Layer II)
static void figs_service(dab_mod *m, const dab_ensemble *e, int cn){
    uint8_t f[32]; int p;
    for (int i = 0; i < e->nsvc; ){
        p = 0; f[p++] = 0; f[p++] = (uint8_t)((cn << 7) | 0x02);
        while (i < e->nsvc && p + 5 <= 30){
            f[p++] = (uint8_t)(e->svc[i].sid >> 8); f[p++] = (uint8_t)e->svc[i].sid;
            f[p++] = 0x01;                                      // local 0, CAId 0, 1 component
            f[p++] = (uint8_t)((0 << 6) | (e->svc[i].mp2 ? 0 : 63));   // TMId 00 (MSC stream audio), ASCTy 63 (DAB+) or 0 (DAB, MPEG-1 Layer II)
            f[p++] = (uint8_t)((i << 2) | 0x02);                // SubChId, primary, no CA
            i++;
        }
        f[0] = (uint8_t)((0 << 5) | (p - 1)); fig_add(m, f, p);
    }
}
static void figs_service_labels(dab_mod *m, const dab_ensemble *e){
    uint8_t f[32];
    for (int i = 0; i < e->nsvc; i++){
        int p = 0; f[p++] = 0; f[p++] = 0x01; f[p++] = (uint8_t)(e->svc[i].sid >> 8); f[p++] = (uint8_t)e->svc[i].sid;
        label16(f + p, e->svc[i].label); p += 16; f[p++] = 0xFF; f[p++] = 0x00;
        f[0] = (uint8_t)((1 << 5) | (p - 1)); fig_add(m, f, p);
    }
}

static void build_figs(dab_mod *m){
    const dab_ensemble *e = &m->ens;
    uint8_t f[32]; int p;
    m->nfigs = 0; m->fig_next = 0;
    figs_subch(m, m->sc, e->nsvc, 0);
    figs_service(m, e, 0);
    // FIG 0/13 user application information: SlideShow (user application type 0x002) in X-PAD, application type 12 (MOT,
    // start of data group), DSCTy 60 (MOT) - per service whose PAD carries one (TS 101 756, EN 300 401 8.1.20)
    for (int i = 0; i < e->nsvc; i++) if (e->svc[i].sls){
        p = 0; f[p++] = 0; f[p++] = 0x0D;                       // C/N=0 OE=0 P/D=0 (16-bit SId), extension 13
        f[p++] = (uint8_t)(e->svc[i].sid >> 8); f[p++] = (uint8_t)e->svc[i].sid;
        f[p++] = 0x01;                                          // SCIdS 0, one user application
        f[p++] = 0x00; f[p++] = 0x42;                           // user application type 0x002 (SlideShow), data length 2
        f[p++] = 0x0C;                                          // CA 0, CA Org 0, X-PAD application type 12
        f[p++] = 0x3C;                                          // DG flag 0 (data groups with CRC), DSCTy 60 (MOT)
        f[0] = (uint8_t)((0 << 5) | (p - 1)); fig_add(m, f, p);
    }
    if (m->change_at >= 0){                                     // announced reconfiguration: the next organisation as well
        if (m->change_flags & 1) figs_subch(m, m->next_sc, m->next_ens.nsvc, 1);
        if (m->change_flags & 2){ figs_service(m, &m->next_ens, 1); figs_service_labels(m, &m->next_ens); }
    }
    // FIG 0/9 country, LTO, international table
    { int lto = e->lto_halfhours; p = 0; f[p++] = 0; f[p++] = 0x09;
      f[p++] = (uint8_t)(0x40 | (lto < 0 ? 0x20 : 0) | (abs(lto) & 0x1F));   // ext flag 0, LTO unique, sign, value
      f[p++] = e->ecc; f[p++] = 0x01;
      f[0] = (uint8_t)(p - 1); fig_add(m, f, p); }
    // FIG 1/0 ensemble label, FIG 1/1 programme service labels (EBU Latin, short label = first 8 characters)
    p = 0; f[p++] = 0; f[p++] = 0x00; f[p++] = (uint8_t)(e->eid >> 8); f[p++] = (uint8_t)e->eid; label16(f + p, e->label); p += 16; f[p++] = 0xFF; f[p++] = 0x00;
    f[0] = (uint8_t)((1 << 5) | (p - 1)); fig_add(m, f, p);
    figs_service_labels(m, e);
}

// the 3 FIBs of one CIF: FIG 0/0 first, then FIGs from the carousel while they fit
static void build_fibs(dab_mod *m, uint8_t fib[3][FIB_BYTES]){
    memset(fib, 0, 3 * FIB_BYTES);
    int start_fig = m->fig_next, guard = 0;
    for (int b = 0; b < 3; b++){
        int p = 0;
        if (b == 0){                                             // FIG 0/0 ensemble information with the CIF count
            uint8_t *f = fib[b];
            const int chg = m->change_at >= 0 ? m->change_flags : 0;
            f[p++] = chg ? 0x06 : 0x05; f[p++] = 0x00; f[p++] = (uint8_t)(m->ens.eid >> 8); f[p++] = (uint8_t)m->ens.eid;
            f[p++] = (uint8_t)((chg << 6) | (((m->cif_count / 250) % 20) & 0x1F));   // change flags, AL 0, CIF count high
            f[p++] = (uint8_t)(m->cif_count % 250);
            if (chg) f[p++] = (uint8_t)(m->change_at % 250);     // occurrence change: the CIF count (low part) of the new configuration
        }
        while (guard < 2 * m->nfigs){
            int l = m->figlen[m->fig_next];
            if (p + l > 30) break;
            memcpy(fib[b] + p, m->figs[m->fig_next], (size_t)l); p += l;
            m->fig_next = (m->fig_next + 1) % m->nfigs; guard++;
            if (m->fig_next == start_fig && b == 2) break;
        }
        if (p < 30){ fib[b][p++] = 0xFF; while (p < 30) fib[b][p++] = 0x00; }   // end marker, padding
        uint16_t crc = dab_crc16(fib[b], 30);
        fib[b][30] = (uint8_t)(crc >> 8); fib[b][31] = (uint8_t)crc;
    }
}

static void encode_fic(dab_mod *m, uint8_t *out_bits){
    uint8_t fib[3][FIB_BYTES], bits[768], coded[4 * (768 + 6)];
    build_fibs(m, fib);
    bytes_to_bits(&fib[0][0], 96, bits);
    prbs_xor(bits, 768);
    conv_encode(bits, 768, coded);
    const prule r[2] = { { 21 * 128, PI[16] }, { 3 * 128, PI[15] } };
    puncture(coded, r, 2, out_bits);                             // 2304 bits
}

// ---------------------------------------------------------------------------------------------------------------------

// prot 1..4 = EEP-1A..4A (any multiple of 8 kbit/s), 5..8 = EEP-1B..4B (multiples of 32 kbit/s); EN 300 401 tables 7 and 8
static int setup_subch(subch *s, int bitrate, int prot, int cu_start){
    if (bitrate < 8 || bitrate % 8 || prot < 1 || prot > 8 || (prot > 4 && bitrate % 32)) return -1;
    const int n = bitrate / 8;
    s->bitrate = bitrate; s->n = n; s->prot = prot; s->cu_start = cu_start; s->nrules = 2;
    if (prot > 4){                                                  // EEP-B: n = bitrate / 32, 24n - 3 and 3 blocks of 128 bits
        static const int pi1[4] = { 10, 6, 4, 2 }, pi2[4] = { 9, 5, 3, 1 }, cus[4] = { 27, 21, 18, 15 };
        const int nb = bitrate / 32, l = prot - 5;
        s->rules[0] = (prule){ (24 * nb - 3) * 128, PI[pi1[l]] }; s->rules[1] = (prule){ 3 * 128, PI[pi2[l]] }; s->cu_size = cus[l] * nb;
    } else switch (prot){
    case 1: s->rules[0] = (prule){ (6 * n - 3) * 128, PI[24] }; s->rules[1] = (prule){ 3 * 128, PI[23] }; s->cu_size = 12 * n; break;
    case 2: if (n == 1){ s->rules[0] = (prule){ 5 * 128, PI[13] }; s->rules[1] = (prule){ 1 * 128, PI[12] }; }
            else { s->rules[0] = (prule){ (2 * n - 3) * 128, PI[14] }; s->rules[1] = (prule){ (4 * n + 3) * 128, PI[13] }; }
            s->cu_size = 8 * n; break;
    case 3: s->rules[0] = (prule){ (6 * n - 3) * 128, PI[8] }; s->rules[1] = (prule){ 3 * 128, PI[7] }; s->cu_size = 6 * n; break;
    default: s->rules[0] = (prule){ (4 * n - 3) * 128, PI[3] }; s->rules[1] = (prule){ (2 * n + 3) * 128, PI[2] }; s->cu_size = 4 * n; break;
    }
    s->in_bits = 24 * bitrate; s->out_bits = s->cu_size * 64;
    for (int i = 0; i < 16; i++){ s->ti[i] = calloc((size_t)s->out_bits, 1); if (!s->ti[i]) return -1; }
    return 0;
}

int dab_cu_size(int bitrate, int prot){
    if (bitrate < 8 || bitrate % 8 || prot < 1 || prot > 8 || (prot > 4 && bitrate % 32)) return -1;
    static const int a[4] = { 12, 8, 6, 4 }, b[4] = { 27, 21, 18, 15 };
    return prot > 4 ? b[prot - 5] * (bitrate / 32) : a[prot - 1] * (bitrate / 8);
}

dab_mod *dab_mod_open(const dab_ensemble *ens){
    if (ens->nsvc < 1 || ens->nsvc > DAB_MAX_SVC) return NULL;
    dab_mod *m = calloc(1, sizeof *m);
    if (!m) return NULL;
    m->ens = *ens;
    int cu = 0;
    for (int i = 0; i < ens->nsvc; i++){
        if (setup_subch(&m->sc[i], ens->svc[i].bitrate, ens->svc[i].prot_level, cu)){ dab_mod_close(m); return NULL; }
        cu += m->sc[i].cu_size;
    }
    if (cu > 864){ dab_mod_close(m); return NULL; }
    m->cu_used = cu;
    m->frame_bits = calloc(4 * FIC_BITS + 4 * CIF_BITS, 1);
    m->iq = calloc(DAB_FRAME_SAMPLES, sizeof(dab_cf));
    if (!m->frame_bits || !m->iq || fft_init(&m->fft, NFFT)){ dab_mod_close(m); return NULL; }
    // frequency interleaver (EN 300 401, 14.6.1): PI(i) = (13 PI(i-1) + 511) mod 2048, keep 256..1792 except 1024
    int perm = 0, n = 0;
    for (int j = 1; j < NFFT && n < K; j++){
        perm = (13 * perm + 511) & (NFFT - 1);
        if (perm >= 256 && perm <= 1792 && perm != 1024) m->fidx[n++] = perm > 1024 ? perm - 1025 : perm + 512;
    }
    for (int a = 0; a < K; a++){
        int row = a / 32, k = a % 32, ph = (PRS_H[PRS_IN[row][0]][k] + PRS_IN[row][1]) & 3;
        static const float re[4] = { 1, 0, -1, 0 }, im[4] = { 0, 1, 0, -1 };
        m->prs[a].re = re[ph]; m->prs[a].im = im[ph]; m->prs_ph[a] = (uint8_t)ph;
    }
    m->change_at = -1;
    build_figs(m);
    return m;
}

void dab_mod_close(dab_mod *m){
    if (!m) return;
    for (int i = 0; i < DAB_MAX_SVC; i++) for (int k = 0; k < 16; k++){ free(m->sc[i].ti[k]); free(m->next_sc[i].ti[k]); }
    free(m->frame_bits); free(m->iq); fft_free(&m->fft); free(m);
}

int dab_mod_lf_bytes(const dab_mod *m, int i){ return 3 * m->sc[i].bitrate; }
int dab_mod_cu_used(const dab_mod *m){ return m->cu_used; }
const dab_cf *dab_mod_frame(const dab_mod *m){ return m->iq; }
const uint32_t *dab_mod_phase_words(const dab_mod *m){ return m->phw; }
void dab_mod_set_phases_only(dab_mod *m, int on){ m->phases_only = on ? 1 : 0; }

static void ofdm_symbol(dab_mod *m, const dab_cf *z, dab_cf *out){
    static cpx in[NFFT], X[NFFT];
    memset(in, 0, sizeof in);
    for (int a = 0; a < K; a++){
        int bin = a < K / 2 ? a + 1 : NFFT - K + a;              // +1..+768 -> bins 1..768, -768..-1 -> bins 1280..2047
        in[bin].re = z[a].re; in[bin].im = -z[a].im;              // inverse FFT via conj(FFT(conj(x)))
    }
    fft_run(&m->fft, in, X);
    const float g = 1.0f / sqrtf((float)K);
    for (int t = 0; t < NFFT; t++){ out[TGUARD + t].re = X[t].re * g; out[TGUARD + t].im = -X[t].im * g; }
    memcpy(out, out + NFFT, sizeof(dab_cf) * TGUARD);              // cyclic prefix
}

// FPGA form: symbol l carries per carrier zp = 45 degrees x (l + 2p) (the differential QPSK adds 45 degrees per symbol),
// so p = (zp - l) / 2 is sent, 2 bits per carrier, 16 carriers per word (carrier a in bits 2(a%16)+1..2(a%16))
static void pack_phases(dab_mod *m, int l, const uint8_t *zp){
    uint32_t *w = m->phw + (size_t)l * (K / 16);
    for (int i = 0; i < K / 16; i++){
        uint32_t v = 0;
        for (int k = 0; k < 16; k++) v |= (uint32_t)(((zp[16 * i + k] - l) & 7) >> 1) << (2 * k);
        w[i] = v;
    }
}

static void modulate_frame(dab_mod *m){
    if (m->phases_only){
        static uint8_t zp[K], yc[K];
        for (int a = 0; a < K; a++) zp[a] = (uint8_t)(2 * m->prs_ph[a]);
        pack_phases(m, 0, zp);
        for (int l = 0; l < NSYM - 1; l++){
            const uint8_t *b = m->frame_bits + (size_t)l * 2 * K;
            static const uint8_t code[4] = { 1, 7, 3, 5 };
            for (int n = 0; n < K; n++) yc[m->fidx[n]] = code[2 * b[n] + b[n + K]];
            for (int a = 0; a < K; a++) zp[a] = (uint8_t)((zp[a] + yc[a]) & 7);
            pack_phases(m, l + 1, zp);
        }
        return;
    }
    dab_cf *o = m->iq;
    memset(o, 0, sizeof(dab_cf) * TNULL); o += TNULL;               // null symbol
    static dab_cf z[K], y[K];
    static uint8_t zp[K], yc[K];                                    // the same phases in units of 45 degrees
    memcpy(z, m->prs, sizeof z);
    for (int a = 0; a < K; a++) zp[a] = (uint8_t)(2 * m->prs_ph[a]);
    pack_phases(m, 0, zp);
    ofdm_symbol(m, z, o); o += TGUARD + NFFT;
    const float h = (float)M_SQRT1_2;
    for (int l = 0; l < NSYM - 1; l++){
        const uint8_t *b = m->frame_bits + (size_t)l * 2 * K;
        for (int n = 0; n < K; n++){
            dab_cf q = { h * (1 - 2 * b[n]), h * (1 - 2 * b[n + K]) }; y[m->fidx[n]] = q;
            static const uint8_t code[4] = { 1, 7, 3, 5 };            // (b_re, b_im) = 00: 45, 01: 315, 10: 135, 11: 225 degrees
            yc[m->fidx[n]] = code[2 * b[n] + b[n + K]];
        }
        for (int a = 0; a < K; a++) zp[a] = (uint8_t)((zp[a] + yc[a]) & 7);
        pack_phases(m, l + 1, zp);
        for (int a = 0; a < K; a++){                               // differential modulation
            float re = z[a].re * y[a].re - z[a].im * y[a].im, im = z[a].re * y[a].im + z[a].im * y[a].re;
            z[a].re = re; z[a].im = im;
        }
        ofdm_symbol(m, z, o); o += TGUARD + NFFT;
    }
}

// the announced configuration takes effect with this CIF
static void apply_change(dab_mod *m){
    // a service that keeps its bitrate and protection keeps its time interleaver history (only its address may move), so its
    // audio runs on without a gap; only the changed services start afresh
    for (int i = 0; i < m->next_ens.nsvc; i++) for (int j = 0; j < m->ens.nsvc; j++)
        if (m->ens.svc[j].sid == m->next_ens.svc[i].sid && m->sc[j].bitrate == m->next_sc[i].bitrate && m->sc[j].prot == m->next_sc[i].prot && m->sc[j].ti[0]){
            for (int k = 0; k < 16; k++){ uint8_t *t = m->next_sc[i].ti[k]; m->next_sc[i].ti[k] = m->sc[j].ti[k]; m->sc[j].ti[k] = t; }
            m->next_sc[i].ti_pos = m->sc[j].ti_pos;
            break;
        }
    for (int i = 0; i < DAB_MAX_SVC; i++) for (int k = 0; k < 16; k++){ free(m->sc[i].ti[k]); m->sc[i].ti[k] = NULL; }
    memcpy(m->sc, m->next_sc, sizeof m->sc); memset(m->next_sc, 0, sizeof m->next_sc);
    m->ens = m->next_ens; m->cu_used = m->next_cu; m->change_at = -1; m->change_flags = 0;
    build_figs(m);
}

int dab_mod_reconfigure(dab_mod *m, const dab_ensemble *next, int at_cif){
    if (m->change_at >= 0 || next->nsvc < 1 || next->nsvc > DAB_MAX_SVC) return -1;
    subch ns[DAB_MAX_SVC]; memset(ns, 0, sizeof ns);
    int cu = 0, flags = next->nsvc != m->ens.nsvc ? 3 : 0;          // services added or removed: both organisations change
    for (int i = 0; i < next->nsvc; i++){
        if (setup_subch(&ns[i], next->svc[i].bitrate, next->svc[i].prot_level, cu)) goto fail;
        cu += ns[i].cu_size;
        if (i >= m->ens.nsvc) continue;
        if (ns[i].bitrate != m->sc[i].bitrate || ns[i].prot != m->sc[i].prot || ns[i].cu_start != m->sc[i].cu_start) flags |= 1;
        if (next->svc[i].sid != m->ens.svc[i].sid || next->svc[i].mp2 != m->ens.svc[i].mp2) flags |= 2;   // the codec is in FIG 0/2
    }
    if (cu > 864 || !flags) goto fail;
    memcpy(m->next_sc, ns, sizeof ns); m->next_ens = *next; m->next_cu = cu;
    m->change_flags = flags; m->change_at = ((at_cif % 5000) + 5000) % 5000;
    build_figs(m);
    return 0;
fail:
    for (int i = 0; i < DAB_MAX_SVC; i++) for (int k = 0; k < 16; k++) free(ns[i].ti[k]);
    return -1;
}
int dab_mod_change_pending(const dab_mod *m){ return m->change_at >= 0; }
int dab_mod_cif_count(const dab_mod *m){ return m->cif_count; }

int dab_mod_set_meta(dab_mod *m, const dab_ensemble *e){
    if (m->change_at >= 0 || e->nsvc != m->ens.nsvc) return -1;
    for (int i = 0; i < e->nsvc; i++) if (e->svc[i].sid != m->ens.svc[i].sid) return -1;
    snprintf(m->ens.label, sizeof m->ens.label, "%s", e->label);
    m->ens.eid = e->eid; m->ens.ecc = e->ecc; m->ens.lto_halfhours = e->lto_halfhours;
    for (int i = 0; i < e->nsvc; i++){
        snprintf(m->ens.svc[i].label, sizeof m->ens.svc[i].label, "%s", e->svc[i].label);
        m->ens.svc[i].sls = e->svc[i].sls; m->ens.svc[i].pty = e->svc[i].pty;
    }
    build_figs(m);
    return 0;
}

void dab_mod_set_labels(dab_mod *m, const char *ensemble, const char *service){
    if (ensemble){ snprintf(m->ens.label, sizeof m->ens.label, "%s", ensemble); snprintf(m->next_ens.label, sizeof m->next_ens.label, "%s", ensemble); }
    if (service){ snprintf(m->ens.svc[0].label, sizeof m->ens.svc[0].label, "%s", service); snprintf(m->next_ens.svc[0].label, sizeof m->next_ens.svc[0].label, "%s", service); }
    build_figs(m);
}

int dab_mod_push_cif(dab_mod *m, const uint8_t *const lf[]){
    if (m->change_at >= 0 && m->cif_count == m->change_at) apply_change(m);
    const int c = m->cif_in_frame;
    encode_fic(m, m->frame_bits + c * FIC_BITS);
    uint8_t *cif = m->frame_bits + 4 * FIC_BITS + (size_t)c * CIF_BITS;
    memset(cif, 0, CIF_BITS);
    prbs_xor(cif, CIF_BITS);                                         // unused capacity: the dispersal sequence
    static uint8_t bits[24 * 192 + 8], coded[4 * (24 * 192 + 6)], punct[864 * 64];
    for (int i = 0; i < m->ens.nsvc; i++){
        subch *s = &m->sc[i];
        bytes_to_bits(lf[i], s->in_bits / 8, bits);
        prbs_xor(bits, s->in_bits);
        conv_encode(bits, s->in_bits, coded);
        int nb = puncture(coded, s->rules, s->nrules, punct);
        while (nb < s->out_bits) punct[nb++] = 0;
        // time interleaving: bit i leaves TI_DELAY[i mod 16] CIFs later
        s->ti_pos = (s->ti_pos + 15) % 16;
        memcpy(s->ti[s->ti_pos], punct, (size_t)s->out_bits);
        uint8_t *dst = cif + s->cu_start * 64;
        for (int k = 0; k < s->out_bits; k++) dst[k] = s->ti[(s->ti_pos + TI_DELAY[k & 15]) % 16][k];
    }
    m->cif_count = (m->cif_count + 1) % 5000;
    if (++m->cif_in_frame < 4) return 0;
    m->cif_in_frame = 0;
    modulate_frame(m);
    return 1;
}
