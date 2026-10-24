// SPDX-License-Identifier: Apache-2.0
// skypluto-dab: the DAB+ transmitter on the Pluto's ARM. A multiplex of up to 8 services (dab/src/mux_conf.h): at most one is
// encoded here (stereo audio from the I2S input through the FPGA's audio FIFO -> sample rate adaptation to the transmitter clock
// -> own AAC-LC encoder -> DAB+ super frames, or own MPEG-1 Layer II encoder -> DAB audio frames), the others arrive already
// encoded from ODR-AudioEnc over ZeroMQ and are passed on unchanged. Then the DAB mode I multiplex -> OFDM phase words -> the
// FPGA's OFDM block (IFFT, guard interval, 3/2 interpolation) -> the modulator's I/Q path.
//
// The transmission runs on the FPGA's sample clock (the AD9361), the audio on the I2S master's clock (the Pico). The two
// drift apart, so the audio goes through a polyphase resampler whose ratio is steered by the latency of the audio.
//
//   skypluto-dab -m mux.conf [-t DLS file] [-P slideshow dir] [-S status file] [-g gain 0..3] [-L level] [-T Hz] [-v]
//   -m file: the multiplex layout, re-read when it changes: labels at once, everything else (services added or removed,
//            bitrate, protection, SId, source) through an announced multiplex reconfiguration ~3 s later, so receivers follow
//            without retuning; a service that keeps SId, bitrate and protection runs on without a gap
//   skypluto-dab [-b kbit/s] [-s service] [-e ensemble] [-i service id] [-x outside.conf] ...   (one service, no layout file)
//   -t file: the DLS of the service encoded here (re-read when it changes)
//   -P dir:  the slideshow of the service encoded here: images 1.jpg .. 8.jpg (or .png) sent in turn in the PAD, a new or changed
//            one first; dir/sls.conf may hold int=<seconds> between the starts of two slides (default 20). Only while the
//            service has the slideshow flag in the layout (that is what announces it in FIG 0/13).
//   -S file: a status line once a second (fifo= uf= rst= aud= ppm= sil= cpu= svc= cu= [chg=] [sls=] and z<i>= per outside
//            service), read by skypluto-ctl (?DAB); the keys are parsed there, keep them stable
//   -T Hz:   test tone for the service encoded here (left Hz, right 1.5 x Hz, -12 dBFS) instead of the I2S input
//   skypluto-dab -f in.f32 -o phases.bin [-p ppm] [-n seconds] [-X recorded.zmq] [-R next.conf -W seconds] ...
//            simulation on a PC: no hardware; -X feeds every outside service from a recording, -R moves another layout onto
//            the -m file at that time
//
// Registers (WFM block at 0x7C440000): 0x64 DAB control ([0] DAB mode, [1] reset, [3:2] gain 2^n), 0x68 write = phase word,
// read = status ([13:0] OFDM FIFO level, [16] underflow, [17] CDC FIFO full), 0x6C read = audio sample {L16, R16} (pops),
// 0x70 = audio samples waiting, 0x08 = carrier level (16 bit, 65535 = x1), 0x00 bit 0 = enable.
#include "../src/dabplus_enc.h"
#include "../src/dab_mot.h"
#include "../src/dab_mod.h"
#include "../src/ext_stream.h"
#include "../src/mux_conf.h"
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define WFM_BASE 0x7C440000UL
#define MAP_LEN  0x1000
#define R_LEVEL  0x08
#define R_DAB    0x64
#define R_PHASE  0x68
#define R_AUDIO  0x6C
#define R_AUDCNT 0x70

#define FS            48000
#define SF_SAMPLES    5760            // 120 ms at 48 kHz: one DAB+ super frame
#define FRAME_WORDS   DAB_PHASE_WORDS // 7296 phase words per 96 ms transmission frame
#define WORDS_PER_S   76000.0         // 7296 / 0.096
#define OFDM_FIFO_MAX 8000            // fill the FPGA's OFDM FIFO (8192 words) up to here
#define SF_MAX        (120 * 24)      // the largest super frame (192 kbit/s)

// ---- single producer / single consumer rings --------------------------------------------------------------------------------
#define AUD_RING 32768 // stereo samples (0.68 s)
#define PH_RING  (FRAME_WORDS * 4)
static float aud_ring[AUD_RING][2];
static uint32_t ph_ring[PH_RING];
static atomic_uint aud_w, aud_r, ph_w, ph_r; // free-running indices
static unsigned aud_fill(void) { return atomic_load(&aud_w) - atomic_load(&aud_r); }
static unsigned ph_fill(void) { return atomic_load(&ph_w) - atomic_load(&ph_r); }

// ---- configuration and state ------------------------------------------------------------------------------------------------
static int o_kbps = 96, o_gain = 1, o_level = -1, o_verbose = 0;
static const char *o_svc = "PlutoSky", *o_ens = "PlutoSky DAB", *o_dls = NULL, *o_dlsfile = NULL, *o_status = NULL,
                  *o_extconf = NULL;
static const char *o_slsdir = NULL;
static const char *o_muxfile = NULL, *o_simstream = NULL, *o_simnext = NULL;
static double o_simnext_t = 0;
static unsigned o_sid = 0x8F01;
static const char *o_simin = NULL, *o_simout = NULL;
static double o_ppm = 0.0, o_secs = 0.0, o_tone = 0.0;
static volatile sig_atomic_t g_stop = 0;
static volatile uint32_t *g_reg = NULL;
static atomic_uint st_underflow, st_silence, st_audio_ovf, st_resets, st_resync;
static double g_ppm_now = 0.0;
#define AUD_TARGET (SF_SAMPLES * 2) // audio buffer fill the rate control aims for (about 240 ms)

static uint32_t rd(unsigned off) { return g_reg[off / 4]; }
static void wr(unsigned off, uint32_t v) { g_reg[off / 4] = v; }

// ---- polyphase resampler (Kaiser windowed sinc, 16 taps, 256 phases, linear between phases) ---------------------------------
#define RS_TAPS 16
#define RS_PH   256
static float rs_h[RS_PH + 1][RS_TAPS];
static double rs_pos = 0.0; // read position relative to aud_r, in input samples

static double bessel_i0(double x){
    double s = 1, t = 1;
    for (int k = 1; k < 30; k++){
        t *= (x / (2 * k)) * (x / (2 * k));
        s += t;
    }
    return s;
}
static void rs_init(void){
    const double beta = 8.0, fc = 0.92; // cutoff 0.92 x Nyquist (22 kHz): above the AAC bandwidth
    for (int p = 0; p <= RS_PH; p++){
        double sum = 0, frac = (double)p / RS_PH;
        for (int k = 0; k < RS_TAPS; k++){
            double t = (k - (RS_TAPS / 2 - 1)) - frac; // tap k sits at input index floor(pos) - 7 + k
            double w = fabs(t) >= RS_TAPS / 2
                               ? 0
                               : bessel_i0(beta * sqrt(1 - (t / (RS_TAPS / 2)) * (t / (RS_TAPS / 2)))) / bessel_i0(beta);
            double s = t == 0 ? fc : sin(M_PI * fc * t) / (M_PI * t);
            rs_h[p][k] = (float)(s * w);
            sum += s * w;
        }
        for (int k = 0; k < RS_TAPS; k++) rs_h[p][k] /= (float)sum;
    }
}
// n output samples with input step 'ratio'; consumes the input it passes. Needs aud_fill() >= n * ratio + RS_TAPS.
static void rs_run(float *out, int n, double ratio){
    unsigned base = atomic_load(&aud_r);
    for (int i = 0; i < n; i++){
        int ip = (int)rs_pos;
        double fr = (rs_pos - ip) * RS_PH;
        int ph = (int)fr;
        float a = (float)(fr - ph);
        float l = 0, r = 0;
        for (int k = 0; k < RS_TAPS; k++){
            float h = rs_h[ph][k] + a * (rs_h[ph + 1][k] - rs_h[ph][k]);
            const float *x = aud_ring[(base + ip + k) % AUD_RING]; // index floor(pos) - 7 + k, with 7 samples of history kept
            l += h * x[0];
            r += h * x[1];
        }
        out[2 * i] = l;
        out[2 * i + 1] = r;
        rs_pos += ratio;
    }
    int adv = (int)rs_pos;
    rs_pos -= adv;
    atomic_store(&aud_r, base + adv);
}

// ---- DLS from a file (re-read when it changes) ------------------------------------------------------------------------------
static char g_dls_text[129]; // the current DLS, for a new encoder after a change of codec
static void dls_poll(dabplus_enc *enc){
    static struct timespec last;
    static int have = 0;
    struct stat st;
    if (!o_dlsfile || stat(o_dlsfile, &st) != 0) return;
    if (have && st.st_mtim.tv_sec == last.tv_sec && st.st_mtim.tv_nsec == last.tv_nsec) return;
    FILE *f = fopen(o_dlsfile, "rb");
    if (!f) return;
    char buf[129];
    size_t n = fread(buf, 1, 128, f);
    fclose(f);
    buf[n] = 0;
    while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = 0;
    dabplus_enc_set_dls(enc, buf, 15);
    snprintf(g_dls_text, sizeof g_dls_text, "%s", buf);
    last = st.st_mtim;
    have = 1;
    if (o_verbose) fprintf(stderr, "DLS: %s\n", buf);
}

// ---- producer state ---------------------------------------------------------------------------------------------------------
static dabplus_enc *g_enc;
static dab_mod *g_mod;
static dabplus_cfg g_cfg;
static float *g_pcm;
static uint8_t g_sf[SF_MAX];
static double g_integ = 0.0, g_lat_target = 0.0;
static double g_seed_ppm = 0.0;
static atomic_int g_seed_ready; // measured input clock offset, handed to the rate control once
static int g_primed = 0, g_cifs = 0;
static unsigned g_sim_fifo = 0; // simulation: the OFDM FIFO level
#define RC_KP  0.35             // ppm per sample of latency error
#define RC_KI  1.8e-4           // ppm per sample per super frame
#define RC_MAX 2000.0           // ppm (the PicoAudio clocks its 48 kHz from the MPX clock: +977 ppm)
#define RESYNC 4800             // samples (100 ms) of excess audio that are skipped at once

static unsigned ofdm_fifo_level(void) { return g_reg ? (rd(R_PHASE) & 0x3FFF) : g_sim_fifo; }
// the audio latency in samples: audio waiting + audio already encoded but not yet transmitted (CIFs in the modulator, phase words
// in the ring and in the FPGA's FIFO). The audio buffer alone jumps by thousands of samples with the beat of super frames (120
// ms) and transmission frames (96 ms); this sum only moves with the clock difference.
static double latency(void){
    return (double)aud_fill() + (ph_fill() + ofdm_fifo_level()) * (FS / WORDS_PER_S) + (g_cifs % 4) * (FS * 0.024);
}

static double now_s(void){
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
static double g_t_enc = 0.0, g_t_mod = 0.0; // seconds spent in the AAC encoder and in the DAB multiplex/modulation (-v)

// ---- the multiplex: its services and where their super frames come from -----------------------------------------------------
typedef struct {
    ext_stream *x;
    int silent_made;
    uint8_t sf[SF_MAX];
} svc_rt;
static mux_conf g_mux, g_next; // on the air; announced (while g_chg_cif >= 0)
static svc_rt g_rt[MUX_MAX_SVC];
static pthread_mutex_t g_rt_lock = PTHREAD_MUTEX_INITIALIZER; // the I/O thread polls the streams, the producer replaces them
static int g_local = -1;                                      // the service encoded here (index in g_mux), -1 none
static double g_tone = 0.0;                                   // its test tone (Hz), 0 = the I2S input
static int g_chg_cif = -1;                                    // the CIF at which g_next takes effect
#define CHG_LEAD 125 // CIFs between the announcement and the change (3 s; FIG 0/0 allows up to 6 s)

static void mux_to_ens(const mux_conf *c, dab_ensemble *e){
    memset(e, 0, sizeof *e);
    e->eid = (uint16_t)c->eid;
    e->ecc = (uint8_t)c->ecc;
    e->lto_halfhours = c->lto;
    snprintf(e->label, sizeof e->label, "%s", c->label);
    e->nsvc = c->n;
    for (int i = 0; i < c->n; i++){
        e->svc[i].sid = (uint16_t)c->s[i].sid;
        snprintf(e->svc[i].label, sizeof e->svc[i].label, "%s", c->s[i].label);
        e->svc[i].bitrate = c->s[i].kbps;
        e->svc[i].prot_level = c->s[i].prot;
        e->svc[i].sls = c->s[i].sls;
        e->svc[i].mp2 = c->s[i].mp2;
        e->svc[i].pty = c->s[i].pty;
    }
}
// the same services in the same sub-channels (only labels, slideshow flags, programme types or the test tone differ)
static int same_structure(const mux_conf *a, const mux_conf *b){
    if (a->n != b->n) return 0;
    for (int i = 0; i < a->n; i++){
        const mux_service *x = &a->s[i], *y = &b->s[i];
        if (x->src != y->src || x->port != y->port || x->sid != y->sid || x->kbps != y->kbps || x->prot != y->prot ||
            x->mp2 != y->mp2)
            return 0;
    }
    return 1;
}
static double tone_of(const mux_conf *c, int local){
    return o_tone > 0 ? o_tone : (local >= 0 && c->s[local].src == MUX_SRC_TONE ? c->s[local].tone_hz : 0.0);
}

static ext_stream *open_stream(const mux_service *s){
    ext_stream *x = ext_open_params(s->port, s->kbps, s->sid, s->label, s->sls, o_simstream);
    if (!x)
        fprintf(stderr, "service %04X \"%s\": port %d cannot be opened, the service stays silent\n", s->sid, s->label, s->port);
    return x;
}
// a silent super frame of a bitrate (for an outside service whose port could not be opened)
static void silent_superframe(int kbps, uint8_t *out){
    dabplus_cfg c = { kbps, 1, 0, 0, 1 };
    dabplus_enc *e = dabplus_enc_open(&c, 0);
    float *z = calloc(6 * 960 * 2, sizeof(float));
    if (e && z){
        for (int i = 0; i < 3; i++) dabplus_enc_superframe(e, z, out);
    } else {
        memset(out, 0, (size_t)(120 * kbps / 8));
    }
    dabplus_enc_close(e);
    free(z);
}
// the encoder of the service encoded here, at its bitrate and codec (AAC for DAB+, MPEG-1 Layer II for DAB); a change of codec
// makes a new encoder, which takes over the DLS (a slide being sent starts again with the next one)
static int local_encoder(int kbps, int mp2){
    if (g_enc && dabplus_enc_is_mp2(g_enc) != mp2){
        dabplus_enc_close(g_enc);
        g_enc = NULL;
    }
    if (!g_enc){
        g_cfg = (dabplus_cfg){ kbps, 1, 0, 0, 1 };
        g_enc = mp2 ? dabplus_enc_open_mp2(kbps) : dabplus_enc_open(&g_cfg, 0);
        if (!g_enc) return -1;
        if (g_dls_text[0]) dabplus_enc_set_dls(g_enc, g_dls_text, 15);
        else if (o_dls) dabplus_enc_set_dls(g_enc, o_dls, 15);
        return 0;
    }
    if (g_cfg.bitrate_kbps == kbps) return 0;
    if (dabplus_enc_set_bitrate(g_enc, kbps)) return -1;
    g_cfg.bitrate_kbps = kbps;
    return 0;
}

// ---- slideshow of the service encoded here (-P dir) -------------------------------------------------------------------------
#define SLS_MAX 8
static int g_sls_type[SLS_MAX + 1]; // per slide 1..8: 0 none, DAB_MOT_JFIF or DAB_MOT_PNG
static struct timespec g_sls_mt[SLS_MAX + 1];
static int g_sls_cur = 0, g_sls_prio = 0, g_sls_int = 20, g_sls_count = 0;
static long g_sls_sf = 1L << 30, g_sls_scan = 0; // super frames since the current slide started, since the last scan
static unsigned g_sls_tid = 0;
static int g_sls_init = 0;
static atomic_int g_sls_busy; // a slide is being sent: set by the producer, read for the status line
static void sls_rescan(void){
    g_sls_count = 0;
    for (int n = 1; n <= SLS_MAX; n++){
        char fn[300];
        struct stat st;
        int ty = 0;
        snprintf(fn, sizeof fn, "%s/%d.jpg", o_slsdir, n);
        if (!stat(fn, &st)) ty = DAB_MOT_JFIF;
        else {
            snprintf(fn, sizeof fn, "%s/%d.png", o_slsdir, n);
            if (!stat(fn, &st)) ty = DAB_MOT_PNG;
        }
        if (ty){
            g_sls_count++;
            if (ty != g_sls_type[n] || st.st_mtim.tv_sec != g_sls_mt[n].tv_sec || st.st_mtim.tv_nsec != g_sls_mt[n].tv_nsec){
                if (g_sls_init) g_sls_prio = n; // new or replaced (not at the start): next on the air
                if (n == g_sls_cur && g_enc) dabplus_enc_slide_abort(g_enc);
                g_sls_mt[n] = st.st_mtim;
            }
        } else if (n == g_sls_cur && g_sls_type[n] && g_enc) dabplus_enc_slide_abort(g_enc); // removed while on the air
        g_sls_type[n] = ty;
    }
    g_sls_init = 1;
    char fn[300];
    snprintf(fn, sizeof fn, "%s/sls.conf", o_slsdir);
    FILE *f = fopen(fn, "r");
    if (f){
        char l[64];
        while (fgets(l, sizeof l, f))
            if (!strncmp(l, "int=", 4)){
                int v = atoi(l + 4);
                if (v >= 5 && v <= 3600) g_sls_int = v;
            }
        fclose(f);
    }
}
// once per super frame, before the local service is encoded
static void sls_poll(void){
    if (!o_slsdir || g_local < 0 || !g_enc) return;
    if (!g_mux.s[g_local].sls){
        if (dabplus_enc_slide_busy(g_enc)) dabplus_enc_slide_abort(g_enc);
        return;
    }
    if (!g_sls_tid) g_sls_tid = (unsigned)time(NULL) & 0xFFFF;
    if (g_sls_scan-- <= 0){ // about every 2 s
        sls_rescan();
        g_sls_scan = 16;
    }
    g_sls_sf++;
    if (dabplus_enc_slide_busy(g_enc) || !g_sls_count) return;
    if (!g_sls_prio && g_sls_sf * 120 < g_sls_int * 1000L) return;
    int n = g_sls_prio;
    if (!n || !g_sls_type[n]){
        n = g_sls_cur;
        do n = n % SLS_MAX + 1;
        while (!g_sls_type[n]);
    }
    g_sls_prio = 0;
    g_sls_sf = 0;
    g_sls_cur = n;
    char fn[300];
    snprintf(fn, sizeof fn, "%s/%d.%s", o_slsdir, n, g_sls_type[n] == DAB_MOT_PNG ? "png" : "jpg");
    static uint8_t img[DAB_MOT_MAX_BODY + 1];
    FILE *f = fopen(fn, "rb");
    if (!f) return;
    int len = (int)fread(img, 1, sizeof img, f);
    fclose(f);
    if (len < 1 || len > DAB_MOT_MAX_BODY){
        if (o_verbose) fprintf(stderr, "slide %d: %d bytes, not sent\n", n, len);
        return;
    }
    g_sls_tid = (g_sls_tid + 1) & 0xFFFF;
    if (!g_sls_tid) g_sls_tid = 1;
    char name[32];
    snprintf(name, sizeof name, "slide%d.%s", n, g_sls_type[n] == DAB_MOT_PNG ? "png" : "jpg");
    if (dabplus_enc_set_slide(g_enc, img, len, g_sls_type[n], name, g_sls_tid))
        fprintf(stderr, "slide %d: could not be queued\n", n);
    else if (o_verbose)
        fprintf(stderr, "slide %d: %d bytes, transport id %u, about %d s on the air\n", n, len, g_sls_tid,
                (len + 200) / (dabplus_enc_slide_rate(g_enc) > 0 ? dabplus_enc_slide_rate(g_enc) : 1) + 1);
}

static void log_mux(const char *what, const mux_conf *c){
    fprintf(stderr, "%s: \"%s\" %04X, %d service(s), %d of 864 CU\n", what, c->label, c->eid, c->n, mux_cu_total(c));
    static const char *const pr[9] = { "", "1A", "2A", "3A", "4A", "1B", "2B", "3B", "4B" };
    for (int i = 0; i < c->n; i++){
        const mux_service *s = &c->s[i];
        fprintf(stderr, "    %d: %04X \"%s\" %d kbit/s %s%s%s, ", i + 1, s->sid, s->label, s->kbps, pr[s->prot],
                s->mp2 ? " MP2" : "", s->sls ? " slideshow" : "");
        if (s->src == MUX_SRC_ZMQ) fprintf(stderr, "ZeroMQ port %d\n", s->port);
        else if (s->src == MUX_SRC_TONE) fprintf(stderr, "test tone %d Hz\n", s->tone_hz);
        else fprintf(stderr, "I2S input\n");
    }
}

// the layout file changed: labels at once, the rest announced for a super frame boundary 3 s ahead (one change at a time)
static struct timespec g_mux_mtime;
static off_t g_mux_size = -1;
static void mux_poll(void){
    struct stat st;
    if (!o_muxfile || g_chg_cif >= 0 || stat(o_muxfile, &st)) return;
    if (st.st_mtim.tv_sec == g_mux_mtime.tv_sec && st.st_mtim.tv_nsec == g_mux_mtime.tv_nsec && st.st_size == g_mux_size) return;
    g_mux_mtime = st.st_mtim;
    g_mux_size = st.st_size;
    mux_conf c;
    char err[160];
    if (mux_read(o_muxfile, &c, err, sizeof err)){
        fprintf(stderr, "multiplex file not taken: %s\n", err);
        return;
    }
    if (same_structure(&c, &g_mux)){
        dab_ensemble e;
        mux_to_ens(&c, &e);
        if (dab_mod_set_meta(g_mod, &e) == 0){
            g_mux = c;
            g_tone = tone_of(&g_mux, g_local);
            log_mux("labels changed", &g_mux);
        }
        return;
    }
    dab_ensemble next;
    mux_to_ens(&c, &next);
    // a super frame boundary (super frames start at counts 0 mod 5)
    int at = (dab_mod_cif_count(g_mod) + CHG_LEAD + 4) / 5 * 5 % 5000;
    if (dab_mod_reconfigure(g_mod, &next, at)){
        fprintf(stderr, "multiplex file not taken: the reconfiguration was refused\n");
        return;
    }
    g_next = c;
    g_chg_cif = at;
    fprintf(stderr, "reconfiguration announced for CIF %d\n", at);
    log_mux("next", &g_next);
}

// the announced layout takes effect with this super frame: streams kept (same port and bitrate), closed or opened; the local
// encoder changes bitrate (or starts)
static void apply_switch(void){
    ext_stream *nx[MUX_MAX_SVC];   // the stream of each service of the new layout
    int kept[MUX_MAX_SVC] = { 0 }; // per service of the old layout: its stream goes on in the new one
    for (int i = 0; i < g_next.n; i++){
        nx[i] = NULL;
        if (g_next.s[i].src != MUX_SRC_ZMQ) continue;
        for (int j = 0; j < g_mux.n; j++)
            if (!kept[j] && g_rt[j].x && g_mux.s[j].src == MUX_SRC_ZMQ && g_mux.s[j].port == g_next.s[i].port){
                nx[i] = g_rt[j].x;
                kept[j] = 1;
                break; // the same sender: the connection stays, only the super frame size may change
            }
    }
    pthread_mutex_lock(&g_rt_lock);
    for (int i = 0; i < g_next.n; i++)
        if (nx[i] && ext_set_kbps(nx[i], g_next.s[i].kbps))
            fprintf(stderr, "service %d: bitrate %d kbit/s refused by its stream\n", i + 1, g_next.s[i].kbps);
    for (int j = 0; j < g_mux.n; j++)
        if (g_rt[j].x && !kept[j]){
            ext_close(g_rt[j].x);
            g_rt[j].x = NULL;
        }
    for (int i = 0; i < g_next.n; i++)
        if (g_next.s[i].src == MUX_SRC_ZMQ && !nx[i]) nx[i] = open_stream(&g_next.s[i]);
    for (int i = 0; i < MUX_MAX_SVC; i++){
        g_rt[i].x = i < g_next.n ? nx[i] : NULL;
        g_rt[i].silent_made = 0;
    }
    g_mux = g_next;
    pthread_mutex_unlock(&g_rt_lock);
    int nl = mux_local(&g_mux);
    if (nl >= 0 && local_encoder(g_mux.s[nl].kbps, g_mux.s[nl].mp2))
        fprintf(stderr, "the local encoder could not change to %d kbit/s %s\n", g_mux.s[nl].kbps,
                g_mux.s[nl].mp2 ? "MP2" : "AAC");
    g_local = nl;
    g_tone = tone_of(&g_mux, g_local);
    g_chg_cif = -1;
    log_mux("reconfiguration done", &g_mux);
}

// ---- one super frame (5 CIFs = 1.25 transmission frames). Returns the transmission frames completed. ------------------------
static int produce_superframe(void){
    double ratio = 1.0 + g_ppm_now * 1e-6;
    int was_primed = g_primed;
    if (g_local >= 0 && g_tone > 0){
        // test tone: generated here on the transmitter's clock, so no resampling and no rate control
        static double tone_ph_l = 0, tone_ph_r = 0;
        for (int i = 0; i < SF_SAMPLES; i++){
            g_pcm[2 * i] = 0.25f * (float)sin(tone_ph_l);
            g_pcm[2 * i + 1] = 0.25f * (float)sin(tone_ph_r);
            tone_ph_l = fmod(tone_ph_l + 2 * M_PI * g_tone / FS, 2 * M_PI);
            tone_ph_r = fmod(tone_ph_r + 2 * M_PI * 1.5 * g_tone / FS, 2 * M_PI);
        }
    } else if (g_local >= 0){
        unsigned need = (unsigned)ceil(SF_SAMPLES * ratio + rs_pos) + RS_TAPS;
        // start (and restart after a gap) only with AUD_TARGET in the buffer
        if (!g_primed && aud_fill() >= AUD_TARGET + need) g_primed = 1;
        // a gap in the audio: silence until the buffer is full again; the clock estimate (g_integ) stays
        if (g_primed && aud_fill() < need) g_primed = 0;
        if (g_primed){
            // far too much audio (the start, or a stall of this program): skip the excess instead of slewing it away at the rate
            // limit. Once running this is judged on the latency: the audio buffer alone swings by thousands of samples with the
            // 120/96 ms beat and would trigger it without reason. The rate control keeps its frequency estimate (g_integ).
            double excess = was_primed ? latency() - g_lat_target : (double)aud_fill() - (double)(AUD_TARGET + need);
            int nearly_full = aud_fill() > AUD_RING - RS_TAPS - 2 * SF_SAMPLES;
            if (excess > RESYNC || nearly_full){
                unsigned room = aud_fill() - need, skip = excess > 0 ? (unsigned)excess : 0;
                if (nearly_full && skip < aud_fill() - (AUD_TARGET + need)) skip = aud_fill() - (AUD_TARGET + need);
                if (skip > room) skip = room;
                atomic_fetch_add(&aud_r, skip);
                atomic_fetch_add(&st_resync, 1);
                was_primed = 0;
            }
            rs_run(g_pcm, SF_SAMPLES, ratio);
        } else {
            memset(g_pcm, 0, sizeof(float) * SF_SAMPLES * 2);
            atomic_fetch_add(&st_silence, 1);
        }
    }
    double t0 = now_s();
    mux_poll();
    // this super frame is the first one of the new layout
    if (g_chg_cif >= 0 && dab_mod_cif_count(g_mod) == g_chg_cif) apply_switch();
    if (g_local >= 0){
        if (!g_enc) return -1; // a change of codec could not open the new encoder
        dls_poll(g_enc);
        sls_poll();
        if (dabplus_enc_superframe(g_enc, g_pcm, g_sf)) return -1;
        atomic_store(&g_sls_busy, dabplus_enc_slide_busy(g_enc));
    }
    const uint8_t *src[MUX_MAX_SVC];
    for (int i = 0; i < g_mux.n; i++){
        if (i == g_local){
            src[i] = g_sf;
            continue;
        }
        if (g_rt[i].x){ // the outside service's next super frame (or silence)
            memcpy(g_rt[i].sf, ext_next(g_rt[i].x), (size_t)(120 * g_mux.s[i].kbps / 8));
        } else if (!g_rt[i].silent_made){
            silent_superframe(g_mux.s[i].kbps, g_rt[i].sf);
            g_rt[i].silent_made = 1;
        }
        src[i] = g_rt[i].sf;
    }
    double t1 = now_s();
    g_t_enc += t1 - t0;
    int frames = 0;
    for (int k = 0; k < 5; k++){
        const uint8_t *lf[MUX_MAX_SVC];
        // logical frame k of 5: 24 ms, 3 bytes per kbit/s
        for (int i = 0; i < g_mux.n; i++) lf[i] = src[i] + k * 3 * g_mux.s[i].kbps;
        if (dab_mod_push_cif(g_mod, lf)){
            const uint32_t *w = dab_mod_phase_words(g_mod);
            unsigned p = atomic_load(&ph_w);
            for (int i = 0; i < FRAME_WORDS; i++) ph_ring[(p + i) % PH_RING] = w[i];
            atomic_store(&ph_w, p + FRAME_WORDS);
            frames++;
        }
        g_cifs++;
    }
    g_t_mod += now_s() - t1;
    if (g_local >= 0 && g_tone <= 0 && g_primed){
        // rate control (PI, critically damped, time constant about a minute): hold the latency where it was at the start;
        // positive error = audio piling up = consume faster
        double L = latency();
        if (!was_primed) g_lat_target = L;
        double e = L - g_lat_target;
        // the I2S source's clock offset, measured by the I/O loop over the first seconds: start the integrator there instead
        // of letting it climb for minutes (+1000 ppm takes ~2 min from zero)
        if (atomic_exchange(&g_seed_ready, 0)) g_integ = (g_seed_ppm - RC_KP * e) / RC_KI;
        else g_integ += e;
        g_ppm_now = RC_KP * e + RC_KI * g_integ;
        if (g_ppm_now > RC_MAX){
            g_ppm_now = RC_MAX;
            g_integ -= e;
        }
        if (g_ppm_now < -RC_MAX){
            g_ppm_now = -RC_MAX;
            g_integ -= e;
        }
    }
    return frames;
}
// room in the phase ring for the next super frame
static int producer_wants(void) { return ph_fill() + FRAME_WORDS * 5 / 4 <= PH_RING - FRAME_WORDS; }

static void *producer_thread(void *arg){
    (void)arg;
    while (!g_stop){
        if (producer_wants()){
            if (produce_superframe() < 0){
                fprintf(stderr, "encoder failed\n");
                g_stop = 1;
            }
        } else usleep(5000);
    }
    return NULL;
}

static void poll_streams(void){
    pthread_mutex_lock(&g_rt_lock);
    for (int i = 0; i < MUX_MAX_SVC; i++)
        if (g_rt[i].x) ext_poll(g_rt[i].x);
    pthread_mutex_unlock(&g_rt_lock);
}

// ---- hardware I/O: audio FIFO -> ring, phase ring -> OFDM FIFO --------------------------------------------------------------
static unsigned g_words_out = 0; // phase words written since the last (re)start, for the frame alignment
static void hw_restart(void){
    // after an underflow the OFDM block must restart at a frame boundary: reset it and drop the rest of the frame in progress
    wr(R_DAB, (uint32_t)(0x3 | (o_gain << 2)));
    usleep(20); // hold the reset: both sides of the CDC FIFO, synchronised to l_clk
    unsigned rest = (FRAME_WORDS - g_words_out % FRAME_WORDS) % FRAME_WORDS;
    if (rest > ph_fill()) rest = ph_fill();
    atomic_fetch_add(&ph_r, rest);
    g_words_out = 0;
    wr(R_DAB, (uint32_t)(0x1 | (o_gain << 2)));
    atomic_fetch_add(&st_resets, 1);
}
static unsigned long long g_in_total = 0; // audio words read from the FPGA
static unsigned g_in_nmax = 0;            // the largest batch since the last report (-v)
// diagnosis: SIGUSR1 records the next 10 s of the I2S input as read from the FPGA (with the size of each read batch) to
// /tmp/dab_i2s.raw: per batch a word 0xBA7C0000 | n and the CLOCK_MONOTONIC time in us (2 words, low first), then the
// n samples {L16, R16}
#define DUMP_WORDS (48000 * 11)
static volatile sig_atomic_t g_dump_req = 0;
static uint32_t *g_dump;
static atomic_uint g_dump_n;
static atomic_int g_dump_done;
static void hw_io_step(void){
    poll_streams(); // the outside services' super frames from the network
    unsigned n = rd(R_AUDCNT);
    // Bitstream B1D0001E (fixed in B1D0001F; current B1D00021) reports 1024 while the FIFO's write pointer has wrapped and its
    // read pointer not yet (23 times per second at 48 kHz, for a few ms). Kept as a guard for old bitstreams: never read more
    // than can have arrived since the last read (98 % of 48 kHz; the rest stays for the next round). Only a stall of this
    // thread can then still overflow the FIFO.
    static double t_prev = 0.0;
    double t_now = now_s();
    if (n >= 1023 && t_prev > 0){
        unsigned est = (unsigned)((t_now - t_prev) * FS * 0.98);
        if (est < n) n = est;
        else atomic_fetch_add(&st_audio_ovf, 1);
    }
    t_prev = t_now;
    g_in_total += n;
    if (n > g_in_nmax) g_in_nmax = n;
    // the audio is read in any case (the FIFO must not fill), kept only when it is used
    const int use = g_local >= 0 && g_tone <= 0;
    unsigned w = atomic_load(&aud_w), room = AUD_RING - RS_TAPS - aud_fill(), kept = 0;
    if (g_dump_req && !g_dump){
        g_dump = calloc(DUMP_WORDS, 4);
        g_dump_req = 0;
    }
    if (g_dump && !atomic_load(&g_dump_done)){ // batch header: marker | n, then the time in us (two words)
        unsigned k = atomic_load(&g_dump_n);
        if (k + 3 < DUMP_WORDS){
            uint64_t us = (uint64_t)(now_s() * 1e6);
            g_dump[k] = 0xBA7C0000u | (n & 0xFFFF);
            g_dump[k + 1] = (uint32_t)us;
            g_dump[k + 2] = (uint32_t)(us >> 32);
            atomic_store(&g_dump_n, k + 3);
        }
    }
    for (unsigned i = 0; i < n; i++){
        uint32_t v = rd(R_AUDIO); // every read pops; keep what fits (the rate control prevents a full ring)
        if (g_dump && !atomic_load(&g_dump_done)){
            unsigned k = atomic_load(&g_dump_n);
            if (k < DUMP_WORDS){
                g_dump[k] = v;
                atomic_store(&g_dump_n, k + 1);
            } else {
                atomic_store(&g_dump_done, 1);
            }
        }
        if (!use || kept == room) continue;
        aud_ring[(w + kept) % AUD_RING][0] = (float)(int16_t)(v >> 16) * (1.0f / 32768.0f);
        aud_ring[(w + kept) % AUD_RING][1] = (float)(int16_t)(v & 0xFFFF) * (1.0f / 32768.0f);
        kept++;
    }
    atomic_store(&aud_w, w + kept);
    uint32_t s = rd(R_PHASE);
    if (s & (1u << 16)){
        atomic_fetch_add(&st_underflow, 1);
        hw_restart();
        return;
    }
    unsigned lvl = s & 0x3FFF, space = lvl < OFDM_FIFO_MAX ? OFDM_FIFO_MAX - lvl : 0, avail = ph_fill();
    if (space > avail) space = avail;
    unsigned r = atomic_load(&ph_r);
    for (unsigned i = 0; i < space; i++) wr(R_PHASE, ph_ring[(r + i) % PH_RING]);
    atomic_store(&ph_r, r + space);
    g_words_out += space;
}

// the outside services, for the status line (verbose = 0) or the log (verbose = 1). Status line form:
//   z<n>=<port>:<buffered>/<received>/<silent>/<dropped>/<unusable>/<sender>/<kbit/s the sender uses>
static void print_streams(FILE *f, int verbose){
    pthread_mutex_lock(&g_rt_lock);
    for (int i = 0; i < g_mux.n; i++){
        if (g_mux.s[i].src != MUX_SRC_ZMQ) continue;
        ext_stats es;
        memset(&es, 0, sizeof es);
        if (g_rt[i].x) ext_get_stats(g_rt[i].x, &es);
        if (verbose)
            fprintf(f,
                    "    service %d (%04X, port %d): %d buffered, %lu received, %lu sent, %lu silent, %lu dropped, "
                    "%lu unusable, from %s\n",
                    i + 1, g_mux.s[i].sid, g_mux.s[i].port, es.fill, es.received, es.used, es.silent, es.dropped, es.bad,
                    es.peer ? es.peer : "-");
        else
            fprintf(f, " z%d=%d:%d/%lu/%lu/%lu/%lu/%s/%d", i + 1, g_mux.s[i].port, es.fill, es.received, es.silent, es.dropped,
                    es.bad, es.peer ? es.peer : "-", g_rt[i].x ? es.kbps_in : 0);
    }
    pthread_mutex_unlock(&g_rt_lock);
}

static void on_signal(int sig){
    (void)sig;
    g_stop = 1;
}
static void on_usr1(int sig){
    (void)sig;
    g_dump_req = 1;
}

static void usage(void){
    fprintf(stderr,
            "skypluto-dab -m mux.conf [-t DLS file] [-P slideshow dir] [-S status file] [-g gain 0..3] [-L level] "
            "[-T tone Hz] [-v]\n"
            "skypluto-dab [-b kbit/s] [-s service] [-e ensemble] [-i service id hex] [-d DLS text] [-x outside.conf] ...   "
            "(without a layout file)\n"
            "skypluto-dab -f in.f32 -o phases.bin [-p audio clock ppm] [-n seconds] [-X recorded.zmq] ...   "
            "(simulation, no hardware)\n");
}

int main(int argc, char **argv){
    int c;
    while ((c = getopt(argc, argv, "m:b:s:e:i:d:t:P:x:X:R:W:g:L:T:S:vf:o:p:n:h")) != -1){
        switch (c){
        case 'm': o_muxfile = optarg; break;
        case 'b': o_kbps = atoi(optarg); break;
        case 's': o_svc = optarg; break;
        case 'e': o_ens = optarg; break;
        case 'd': o_dls = optarg; break;
        case 't': o_dlsfile = optarg; break;
        case 'P': o_slsdir = optarg; break;
        case 'g': o_gain = atoi(optarg) & 3; break;
        case 'L': o_level = atoi(optarg); break;
        case 'T': o_tone = atof(optarg); break;
        case 'i': o_sid = (unsigned)strtoul(optarg, NULL, 16) & 0xFFFF; break;
        case 'S': o_status = optarg; break;
        case 'x': o_extconf = optarg; break;
        case 'X': o_simstream = optarg; break;
        case 'R': o_simnext = optarg; break;
        case 'W': o_simnext_t = atof(optarg); break;
        case 'v': o_verbose = 1; break;
        case 'f': o_simin = optarg; break;
        case 'o': o_simout = optarg; break;
        case 'p': o_ppm = atof(optarg); break;
        case 'n': o_secs = atof(optarg); break;
        default: usage(); return 2;
        }
    }
    // the layout: from the file, or one service from the options (plus an outside one from -x)
    if (o_muxfile){
        char err[160];
        struct stat st;
        if (mux_read(o_muxfile, &g_mux, err, sizeof err)){
            fprintf(stderr, "%s: %s\n", o_muxfile, err);
            return 2;
        }
        if (!stat(o_muxfile, &st)){
            g_mux_mtime = st.st_mtim;
            g_mux_size = st.st_size;
        }
    } else {
        if (o_kbps < 8 || o_kbps > 192 || o_kbps % 8){
            fprintf(stderr, "bitrate: 8..192 kbit/s, multiple of 8\n");
            return 2;
        }
        mux_default(&g_mux);
        snprintf(g_mux.label, sizeof g_mux.label, "%.16s", o_ens);
        g_mux.s[0].sid = o_sid;
        snprintf(g_mux.s[0].label, sizeof g_mux.s[0].label, "%.16s", o_svc);
        g_mux.s[0].kbps = o_kbps;
        if (o_extconf){
            ext_stream *x = ext_open(o_extconf);
            if (!x) fprintf(stderr, "outside service: %s unusable (port in use, bad bitrate?), going on without it\n", o_extconf);
            else {
                mux_service *s = &g_mux.s[1];
                g_mux.n = 2;
                s->src = MUX_SRC_ZMQ;
                s->port = ext_port(x);
                s->sid = ext_sid(x);
                s->kbps = ext_kbps(x);
                s->prot = 3;
                s->sls = ext_sls(x);
                snprintf(s->label, sizeof s->label, "%s", ext_label(x));
                g_rt[1].x = x;
            }
        }
    }
    g_local = mux_local(&g_mux);
    g_tone = tone_of(&g_mux, g_local);
    if (g_local >= 0 && local_encoder(g_mux.s[g_local].kbps, g_mux.s[g_local].mp2)){
        fprintf(stderr, "encoder: bad configuration\n");
        return 2;
    }
    for (int i = 0; i < g_mux.n; i++)
        if (g_mux.s[i].src == MUX_SRC_ZMQ && !g_rt[i].x) g_rt[i].x = open_stream(&g_mux.s[i]);
    dab_ensemble ens;
    mux_to_ens(&g_mux, &ens);
    g_mod = dab_mod_open(&ens);
    if (!g_mod){
        fprintf(stderr, "multiplex: bad configuration\n");
        return 2;
    }
    dab_mod_set_phases_only(g_mod, 1); // the FPGA does the OFDM
    log_mux("multiplex", &g_mux);
    g_pcm = calloc(SF_SAMPLES * 2, sizeof(float));
    rs_init();
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGUSR1, on_usr1);

    if (o_simin){
        // ---- simulation: audio arrives at 48 kHz x (1 + ppm), the OFDM block takes 76000 words/s; steps of 4 ms -------------
        FILE *fi = fopen(o_simin, "rb"), *fo = o_simout ? fopen(o_simout, "wb") : NULL;
        if (!fi || !fo){
            perror("file");
            return 2;
        }
        double acc = 0, wacc = 0, t = 0;
        long words = 0;
        int eof = 0;
        unsigned fifo = 0; // simulated OFDM FIFO level (g_sim_fifo for latency())
        while (!eof && (o_secs <= 0 || t < o_secs)){
            acc += FS * (1 + o_ppm * 1e-6) * 0.004;
            int n = (int)acc;
            acc -= n;
            for (int i = 0; i < n; i++){
                float s[2];
                if (fread(s, sizeof s, 1, fi) != 1){
                    eof = 1;
                    break;
                }
                if (aud_fill() < AUD_RING - RS_TAPS){
                    unsigned w = atomic_load(&aud_w);
                    aud_ring[w % AUD_RING][0] = s[0];
                    aud_ring[w % AUD_RING][1] = s[1];
                    atomic_store(&aud_w, w + 1);
                } else atomic_fetch_add(&st_audio_ovf, 1);
            }
            poll_streams();
            if (o_simnext && o_muxfile && t >= o_simnext_t){ // -R/-W: another layout at that time
                rename(o_simnext, o_muxfile);
                o_simnext = NULL;
            }
            while (producer_wants())
                if (produce_superframe() < 0) return 1;
            // the FPGA consumes
            wacc += WORDS_PER_S * 0.004;
            int take = (int)wacc;
            wacc -= take;
            if ((unsigned)take > fifo){
                if (words > FRAME_WORDS) atomic_fetch_add(&st_underflow, 1);
                fifo = 0;
            } else fifo -= take;
            unsigned space = OFDM_FIFO_MAX - fifo, avail = ph_fill();
            if (space > avail) space = avail;
            unsigned r = atomic_load(&ph_r);
            for (unsigned i = 0; i < space; i++){
                uint32_t v = ph_ring[(r + i) % PH_RING];
                fwrite(&v, 4, 1, fo);
            }
            atomic_store(&ph_r, r + space);
            fifo += space;
            words += space;
            g_sim_fifo = fifo;
            t += 0.004;
            if (o_verbose && fmod(t, 5.0) < 0.004)
                fprintf(stderr,
                        "t=%.0f s audio %u (target %d) latency %.0f (target %.0f) ratio %+.1f ppm "
                        "silence %u resync %u underflow %u\n",
                        t, aud_fill(), AUD_TARGET, latency(), g_lat_target, g_ppm_now, atomic_load(&st_silence),
                        atomic_load(&st_resync), atomic_load(&st_underflow));
        }
        fclose(fi);
        fclose(fo);
        printf("simulation: %.1f s, %ld phase words (%ld frames), final ratio %+.1f ppm, "
               "silent super frames %u, underflows %u, audio overflows %u\n",
               t, words, words / FRAME_WORDS, g_ppm_now, atomic_load(&st_silence), atomic_load(&st_underflow),
               atomic_load(&st_audio_ovf));
        printf("time per second of audio: AAC encoder %.0f ms, DAB multiplex %.0f ms\n", 1000 * g_t_enc / t, 1000 * g_t_mod / t);
        return 0;
    }

    // ---- hardware -----------------------------------------------------------------------------------------------------------
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0){
        perror("open /dev/mem");
        return 1;
    }
    void *m = mmap(NULL, MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, WFM_BASE);
    if (m == MAP_FAILED){
        perror("mmap");
        return 1;
    }
    g_reg = (volatile uint32_t *)m;
    if (o_level >= 0) wr(R_LEVEL, (uint32_t)o_level);
    wr(R_DAB, (uint32_t)(0x3 | (o_gain << 2))); // DAB mode with the OFDM block in reset: no output until the first frame is ready
    for (int i = 0; i < 2; i++)
        if (produce_superframe() < 0) return 1;
    wr(R_DAB, (uint32_t)(0x1 | (o_gain << 2))); // DAB mode
    while (rd(R_AUDCNT)) (void)rd(R_AUDIO);     // stale audio
    if (g_local >= 0 && g_tone > 0) fprintf(stderr, "test tone %.0f Hz left, %.0f Hz right (-12 dBFS)\n", g_tone, 1.5 * g_tone);
    pthread_t th;
    pthread_create(&th, NULL, producer_thread, NULL);
    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    double last = 0;
    while (!g_stop){
        hw_io_step();
        usleep(3000);
        struct timespec t1;
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double t = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;
        // the input clock against this CPU's clock over 10 s (from 2 s on): the start value of the rate control
        {
            static int seed_state = 0;
            static double seed_t0;
            static unsigned long long seed_n0;
            if (g_local >= 0 && g_tone <= 0 && seed_state == 0 && t >= 2.0){
                seed_t0 = t;
                seed_n0 = g_in_total;
                seed_state = 1;
            } else if (seed_state == 1 && t - seed_t0 >= 10.0){
                double ppm = ((double)(g_in_total - seed_n0) / (t - seed_t0) / FS - 1.0) * 1e6;
                if (fabs(ppm) < RC_MAX){
                    g_seed_ppm = ppm;
                    atomic_store(&g_seed_ready, 1);
                }
                fprintf(stderr, "    input clock %+.0f ppm (measured over 10 s)\n", ppm);
                seed_state = 2;
            }
        }
        static double last_st = 0;
        if (o_status && t - last_st >= 1.0){
            // one line for skypluto-ctl (?DAB), replaced as a whole (rename) so a reader never sees half a line
            last_st = t;
            if (g_dump && atomic_load(&g_dump_done)){ // the I2S recording is complete: write it out and stop
                FILE *df = fopen("/tmp/dab_i2s.raw", "wb");
                if (df){
                    fwrite(g_dump, 4, atomic_load(&g_dump_n), df);
                    fclose(df);
                }
                fprintf(stderr, "I2S recording written: %u words\n", atomic_load(&g_dump_n));
                uint32_t *b = g_dump;
                g_dump = NULL;
                atomic_store(&g_dump_n, 0);
                atomic_store(&g_dump_done, 0);
                free(b);
            }
            char tmp[256];
            snprintf(tmp, sizeof tmp, "%s.new", o_status);
            FILE *sf = fopen(tmp, "w");
            if (sf){
                // the layout is replaced by the producer thread (apply_switch, under g_rt_lock): read it under the lock,
                // and never touch g_enc here (a change of codec frees it)
                pthread_mutex_lock(&g_rt_lock);
                const int nsvc = g_mux.n, cu = mux_cu_total(&g_mux), loc = g_local;
                const int sls_on = o_slsdir && loc >= 0 && loc < nsvc && g_mux.s[loc].sls;
                pthread_mutex_unlock(&g_rt_lock);
                fprintf(sf, "fifo=%u uf=%u rst=%u aud=%u ppm=%+.0f sil=%u cpu=%.0f svc=%d cu=%d", rd(R_PHASE) & 0x3FFF,
                        atomic_load(&st_underflow), atomic_load(&st_resets), aud_fill(), g_ppm_now, atomic_load(&st_silence),
                        100 * (g_t_enc + g_t_mod) / t, nsvc, cu);
                if (g_chg_cif >= 0) fprintf(sf, " chg=%d", g_chg_cif);
                if (sls_on) fprintf(sf, " sls=%d/%d/%d", g_sls_count, g_sls_cur, atomic_load(&g_sls_busy));
                print_streams(sf, 0);
                fputc('\n', sf);
                fclose(sf);
                rename(tmp, o_status);
            }
        }
        if (o_verbose && t - last >= 5.0){
            static unsigned long long in_last = 0;
            fprintf(stderr, "    audio in %.0f samples/s, largest batch %u\n", (g_in_total - in_last) / (t - last), g_in_nmax);
            print_streams(stderr, 1);
            in_last = g_in_total;
            g_in_nmax = 0;
            last = t;
            fprintf(stderr,
                    "t=%.0f s audio %u ratio %+.1f ppm phase queue %u OFDM FIFO %u silence %u resync %u underflow %u "
                    "restarts %u audio-ovf %u cpu enc %.0f%% mux %.0f%%\n",
                    t, aud_fill(), g_ppm_now, ph_fill(), rd(R_PHASE) & 0x3FFF, atomic_load(&st_silence), atomic_load(&st_resync),
                    atomic_load(&st_underflow), atomic_load(&st_resets), atomic_load(&st_audio_ovf), 100 * g_t_enc / t,
                    100 * g_t_mod / t);
        }
    }
    pthread_join(th, NULL);
    // back to FM: OFDM block in reset, then DAB mode off
    wr(R_DAB, 0x2);
    usleep(20);
    wr(R_DAB, 0x0);
    munmap(m, MAP_LEN);
    close(fd);
    return 0;
}
