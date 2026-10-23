// SPDX-License-Identifier: Apache-2.0
// skypluto-dab: the DAB+ transmitter on the Pluto's ARM. Stereo audio from the I2S input (through the FPGA's audio FIFO) ->
// sample rate adaptation to the transmitter clock -> own AAC-LC encoder -> DAB+ super frames -> DAB mode I multiplex ->
// OFDM phase words -> the FPGA's OFDM block (IFFT, guard interval, 3/2 interpolation) -> the modulator's I/Q path.
//
// The transmission runs on the FPGA's sample clock (the AD9361), the audio on the I2S master's clock (the Pico). The two
// drift apart, so the audio goes through a polyphase resampler whose ratio is steered by the fill of the audio buffer.
//
//   skypluto-dab [-b kbit/s] [-s service] [-e ensemble] [-i service id] [-d DLS text] [-t DLS file] [-g gain 0..3] [-L level] [-T Hz] [-S file] [-v]
//   -S file: a status line once a second (fifo= uf= rst= aud= ppm= sil= cpu=), read by skypluto-ctl for ?DAB
//   -c file: br= sid= svc= ens= lines, read every super frame: labels change at once, bitrate and service id through an
//            announced multiplex reconfiguration (~3 s later), so receivers follow without retuning
//   -T Hz: test tone instead of the I2S input (left Hz, right 1.5 x Hz, -12 dBFS), for a first test without a DAB source
//   skypluto-dab -f in.f32 -o phases.bin [-p ppm] [-n seconds] ...     (simulation on a PC: no hardware)
//
// Registers (WFM block at 0x7C440000): 0x64 DAB control ([0] DAB mode, [1] reset, [3:2] gain 2^n), 0x68 write = phase word,
// read = status ([13:0] OFDM FIFO level, [16] underflow, [17] CDC FIFO full), 0x6C read = audio sample {L16, R16} (pops),
// 0x70 = audio samples waiting, 0x08 = carrier level (16 bit, 65535 = x1), 0x00 bit 0 = enable.
#include "../src/dabplus_enc.h"
#include "../src/dab_mod.h"
#include <errno.h>
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

#define WFM_BASE      0x7C440000UL
#define MAP_LEN       0x1000
#define R_CTRL        0x00
#define R_LEVEL       0x08
#define R_DAB         0x64
#define R_PHASE       0x68
#define R_AUDIO       0x6C
#define R_AUDCNT      0x70

#define FS            48000
#define SF_SAMPLES    5760                 // 120 ms at 48 kHz: one DAB+ super frame
#define FRAME_WORDS   DAB_PHASE_WORDS      // 7296 phase words per 96 ms transmission frame
#define WORDS_PER_S   76000.0              // 7296 / 0.096
#define OFDM_FIFO_MAX 8000                 // fill the FPGA's OFDM FIFO (8192 words) up to here

// ---- single producer / single consumer rings -------------------------------------------------------------------------------------
#define AUD_RING      32768                // stereo samples (0.68 s)
#define PH_RING       (FRAME_WORDS * 4)
static float      aud_ring[AUD_RING][2];
static uint32_t   ph_ring[PH_RING];
static atomic_uint aud_w, aud_r, ph_w, ph_r;    // free-running indices
static unsigned aud_fill(void){ return atomic_load(&aud_w) - atomic_load(&aud_r); }
static unsigned ph_fill(void){ return atomic_load(&ph_w) - atomic_load(&ph_r); }

// ---- configuration and state ---------------------------------------------------------------------------------------------------
static int   o_kbps = 96, o_gain = 1, o_level = -1, o_verbose = 0;
static const char *o_svc = "PlutoSky", *o_ens = "PlutoSky DAB", *o_dls = NULL, *o_dlsfile = NULL, *o_status = NULL, *o_cfgfile = NULL;
static unsigned o_sid = 0x8F01;
static const char *o_simin = NULL, *o_simout = NULL;
static double o_ppm = 0.0, o_secs = 0.0, o_tone = 0.0;
static volatile sig_atomic_t g_stop = 0;
static volatile uint32_t *g_reg = NULL;
static atomic_uint st_underflow, st_silence, st_audio_ovf, st_resets, st_resync;
static double g_ppm_now = 0.0;
#define AUD_TARGET    (SF_SAMPLES * 2)     // audio buffer fill the rate control aims for (about 240 ms)

static uint32_t rd(unsigned off){ return g_reg[off / 4]; }
static void     wr(unsigned off, uint32_t v){ g_reg[off / 4] = v; }

// ---- polyphase resampler (Kaiser windowed sinc, 16 taps, 256 phases, linear between phases) ------------------------------------
#define RS_TAPS   16
#define RS_PH     256
static float rs_h[RS_PH + 1][RS_TAPS];
static double rs_pos = 0.0;                // read position relative to aud_r, in input samples

static double bessel_i0(double x){ double s = 1, t = 1; for (int k = 1; k < 30; k++){ t *= (x / (2 * k)) * (x / (2 * k)); s += t; } return s; }
static void rs_init(void){
    const double beta = 8.0, fc = 0.92;    // cutoff 0.92 x Nyquist (22 kHz): above the AAC bandwidth
    for (int p = 0; p <= RS_PH; p++){
        double sum = 0, frac = (double)p / RS_PH;
        for (int k = 0; k < RS_TAPS; k++){
            double t = (k - (RS_TAPS / 2 - 1)) - frac;        // tap k sits at input index floor(pos) - 7 + k
            double w = fabs(t) >= RS_TAPS / 2 ? 0 : bessel_i0(beta * sqrt(1 - (t / (RS_TAPS / 2)) * (t / (RS_TAPS / 2)))) / bessel_i0(beta);
            double s = t == 0 ? fc : sin(M_PI * fc * t) / (M_PI * t);
            rs_h[p][k] = (float)(s * w); sum += s * w;
        }
        for (int k = 0; k < RS_TAPS; k++) rs_h[p][k] /= (float)sum;
    }
}
// n output samples with input step 'ratio'; consumes the input it passes. Needs aud_fill() >= n * ratio + RS_TAPS.
static void rs_run(float *out, int n, double ratio){
    unsigned base = atomic_load(&aud_r);
    for (int i = 0; i < n; i++){
        int ip = (int)rs_pos; double fr = (rs_pos - ip) * RS_PH; int ph = (int)fr; float a = (float)(fr - ph);
        float l = 0, r = 0;
        for (int k = 0; k < RS_TAPS; k++){
            float h = rs_h[ph][k] + a * (rs_h[ph + 1][k] - rs_h[ph][k]);
            const float *x = aud_ring[(base + ip + k) % AUD_RING];   // index floor(pos) - 7 + k, with 7 samples of history kept
            l += h * x[0]; r += h * x[1];
        }
        out[2 * i] = l; out[2 * i + 1] = r;
        rs_pos += ratio;
    }
    int adv = (int)rs_pos; rs_pos -= adv;
    atomic_store(&aud_r, base + adv);
}

// ---- DLS from a file (re-read when it changes) ---------------------------------------------------------------------------------
static void dls_poll(dabplus_enc *enc){
    static struct timespec last; static int have = 0;
    struct stat st;
    if (!o_dlsfile || stat(o_dlsfile, &st) != 0) return;
    if (have && st.st_mtim.tv_sec == last.tv_sec && st.st_mtim.tv_nsec == last.tv_nsec) return;
    FILE *f = fopen(o_dlsfile, "rb"); if (!f) return;
    char buf[129]; size_t n = fread(buf, 1, 128, f); fclose(f); buf[n] = 0;
    while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = 0;
    dabplus_enc_set_dls(enc, buf, 15);
    last = st.st_mtim; have = 1;
    if (o_verbose) fprintf(stderr, "DLS: %s\n", buf);
}

// ---- producer: audio -> super frame -> CIFs -> phase words ---------------------------------------------------------------------
static dabplus_enc *g_enc; static dab_mod *g_mod; static dabplus_cfg g_cfg;
static float   *g_pcm; static uint8_t *g_sf; static int g_lfb;
static double   g_integ = 0.0, g_lat_target = 0.0;
static double   g_seed_ppm = 0.0; static atomic_int g_seed_ready;   // measured input clock offset, handed to the rate control once
static int      g_primed = 0, g_cifs = 0;
static unsigned g_sim_fifo = 0;            // simulation: the OFDM FIFO level
#define RC_KP   0.35                       // ppm per sample of latency error
#define RC_KI   1.8e-4                     // ppm per sample per super frame
#define RC_MAX  2000.0                     // ppm (the PicoAudio clocks its 48 kHz from the MPX clock: +977 ppm)
#define RESYNC  4800                       // samples (100 ms) of excess audio that are skipped at once

static unsigned ofdm_fifo_level(void){ return g_reg ? (rd(R_PHASE) & 0x3FFF) : g_sim_fifo; }
// the audio latency in samples: audio waiting + audio already encoded but not yet transmitted (CIFs in the modulator, phase words
// in the ring and in the FPGA's FIFO). The audio buffer alone jumps by thousands of samples with the beat of super frames (120 ms)
// and transmission frames (96 ms); this sum only moves with the clock difference.
static double latency(void){
    return (double)aud_fill() + (ph_fill() + ofdm_fifo_level()) * (FS / WORDS_PER_S) + (g_cifs % 4) * (FS * 0.024);
}

// one super frame (5 CIFs = 1.25 transmission frames). Returns the transmission frames completed.
// ---- live configuration (-c): labels at once, bitrate / service id through an announced multiplex reconfiguration ----------
static dab_ensemble g_ens;                 // the ensemble as it is on the air (or will be after the pending change)
static int g_chg_cif = -1, g_chg_kbps = 0; // the CIF of a pending reconfiguration and the bitrate it brings
#define CHG_LEAD 125                       // CIFs between the announcement and the change (3 s; FIG 0/0 allows up to 6 s)
static void cfg_poll(void){
    if (!o_cfgfile) return;
    FILE *f = fopen(o_cfgfile, "r"); if (!f) return;
    char ln[96], svc[17] = "", ens[17] = ""; int br = 0; unsigned sid = 0;
    while (fgets(ln, sizeof ln, f)){
        ln[strcspn(ln, "\r\n")] = 0;
        if (!strncmp(ln, "br=", 3)) br = atoi(ln + 3);
        else if (!strncmp(ln, "sid=", 4)) sid = (unsigned)strtoul(ln + 4, NULL, 16) & 0xFFFF;
        else if (!strncmp(ln, "svc=", 4)) snprintf(svc, sizeof svc, "%.16s", ln + 4);
        else if (!strncmp(ln, "ens=", 4)) snprintf(ens, sizeof ens, "%.16s", ln + 4);
    }
    fclose(f);
    if ((svc[0] && strcmp(svc, g_ens.svc[0].label)) || (ens[0] && strcmp(ens, g_ens.label))){
        if (svc[0]) snprintf(g_ens.svc[0].label, sizeof g_ens.svc[0].label, "%s", svc);
        if (ens[0]) snprintf(g_ens.label, sizeof g_ens.label, "%s", ens);
        dab_mod_set_labels(g_mod, ens[0] ? ens : NULL, svc[0] ? svc : NULL);
        fprintf(stderr, "labels: \"%s\" / \"%s\"\n", g_ens.svc[0].label, g_ens.label);
    }
    if (g_chg_cif >= 0 || dab_mod_change_pending(g_mod)) return;         // one change at a time
    int want_br = (br >= 8 && br <= 192 && br % 8 == 0) ? br : g_ens.svc[0].bitrate;
    unsigned want_sid = sid ? sid : g_ens.svc[0].sid;
    if (want_br == g_ens.svc[0].bitrate && want_sid == g_ens.svc[0].sid) return;
    dab_ensemble next = g_ens; next.svc[0].bitrate = want_br; next.svc[0].sid = (uint16_t)want_sid;
    int at = (dab_mod_cif_count(g_mod) + CHG_LEAD + 4) / 5 * 5 % 5000;    // a super frame boundary (super frames start at counts 0 mod 5)
    if (dab_mod_reconfigure(g_mod, &next, at) == 0){
        g_chg_cif = at; g_chg_kbps = want_br; g_ens = next;
        fprintf(stderr, "reconfiguration announced: %d kbit/s, SId %04X at CIF %d\n", want_br, want_sid, at);
    }
}

static double now_s(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static double g_t_enc = 0.0, g_t_mod = 0.0;   // seconds spent in the AAC encoder and in the DAB multiplex/modulation (-v)

static int produce_superframe(void){
    double ratio = 1.0 + g_ppm_now * 1e-6;
    int was_primed = g_primed;
    if (o_tone > 0){
        // test tone: generated here on the transmitter's clock, so no resampling and no rate control
        static double ph_l = 0, ph_r = 0;
        for (int i = 0; i < SF_SAMPLES; i++){
            g_pcm[2 * i] = 0.25f * (float)sin(ph_l); g_pcm[2 * i + 1] = 0.25f * (float)sin(ph_r);
            ph_l = fmod(ph_l + 2 * M_PI * o_tone / FS, 2 * M_PI); ph_r = fmod(ph_r + 2 * M_PI * 1.5 * o_tone / FS, 2 * M_PI);
        }
    } else {
        unsigned need = (unsigned)ceil(SF_SAMPLES * ratio + rs_pos) + RS_TAPS;
        // start (and restart after a gap) only with AUD_TARGET in the buffer
        if (!g_primed && aud_fill() >= AUD_TARGET + need) g_primed = 1;
        if (g_primed && aud_fill() < need){ g_primed = 0; g_integ = 0; g_ppm_now = 0; }
        if (g_primed){
            // far too much audio (the start, or a stall of this program): skip the excess instead of slewing it away at the rate limit.
            // Once running this is judged on the latency: the audio buffer alone swings by thousands of samples with the 120/96 ms
            // beat and would trigger it without reason. The rate control keeps its frequency estimate (g_integ).
            double excess = was_primed ? latency() - g_lat_target : (double)aud_fill() - (double)(AUD_TARGET + need);
            int nearly_full = aud_fill() > AUD_RING - RS_TAPS - 2 * SF_SAMPLES;
            if (excess > RESYNC || nearly_full){
                unsigned room = aud_fill() - need, skip = excess > 0 ? (unsigned)excess : 0;
                if (nearly_full && skip < aud_fill() - (AUD_TARGET + need)) skip = aud_fill() - (AUD_TARGET + need);
                if (skip > room) skip = room;
                atomic_fetch_add(&aud_r, skip); atomic_fetch_add(&st_resync, 1); was_primed = 0;
            }
            rs_run(g_pcm, SF_SAMPLES, ratio);
        } else { memset(g_pcm, 0, sizeof(float) * SF_SAMPLES * 2); atomic_fetch_add(&st_silence, 1); }
    }
    double t0 = now_s();
    cfg_poll();
    if (g_chg_cif >= 0 && dab_mod_cif_count(g_mod) == g_chg_cif){
        // this super frame is the first one of the new configuration: the encoder changes bitrate with it
        if (dabplus_enc_set_bitrate(g_enc, g_chg_kbps) == 0){ g_cfg.bitrate_kbps = g_chg_kbps; g_lfb = 3 * g_chg_kbps; }
        fprintf(stderr, "reconfiguration: now %d kbit/s\n", g_cfg.bitrate_kbps);
        g_chg_cif = -1;
    }
    dls_poll(g_enc);
    if (dabplus_enc_superframe(g_enc, g_pcm, g_sf)) return -1;
    double t1 = now_s(); g_t_enc += t1 - t0;
    int frames = 0;
    for (int k = 0; k < 5; k++){
        const uint8_t *lf[1] = { g_sf + k * g_lfb };
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
    if (o_tone <= 0 && g_primed){
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
        if (g_ppm_now >  RC_MAX){ g_ppm_now =  RC_MAX; g_integ -= e; }
        if (g_ppm_now < -RC_MAX){ g_ppm_now = -RC_MAX; g_integ -= e; }
    }
    return frames;
}
static int producer_wants(void){ return ph_fill() + FRAME_WORDS * 5 / 4 <= PH_RING - FRAME_WORDS; }   // room for the next super frame

static void *producer_thread(void *arg){
    (void)arg;
    while (!g_stop){
        if (producer_wants()){ if (produce_superframe() < 0){ fprintf(stderr, "encoder failed\n"); g_stop = 1; } }
        else usleep(5000);
    }
    return NULL;
}

// ---- hardware I/O: audio FIFO -> ring, phase ring -> OFDM FIFO -----------------------------------------------------------------
static unsigned g_words_out = 0;           // phase words written since the last (re)start, for the frame alignment
static void hw_restart(void){
    // after an underflow the OFDM block must restart at a frame boundary: reset it and drop the rest of the frame in progress
    wr(R_DAB, (uint32_t)(0x3 | (o_gain << 2)));
    usleep(20);                                                      // hold the reset: both sides of the CDC FIFO, synchronised to l_clk
    unsigned rest = (FRAME_WORDS - g_words_out % FRAME_WORDS) % FRAME_WORDS;
    if (rest > ph_fill()) rest = ph_fill();
    atomic_fetch_add(&ph_r, rest);
    g_words_out = 0;
    wr(R_DAB, (uint32_t)(0x1 | (o_gain << 2)));
    atomic_fetch_add(&st_resets, 1);
}
static unsigned long long g_in_total = 0; static unsigned g_in_nmax = 0;   // audio words read from the FPGA, the largest batch (-v)
static void hw_io_step(void){
    unsigned n = o_tone > 0 ? 0 : rd(R_AUDCNT);                       // with the test tone the I2S audio is left in the FIFO
    // Bitstream B1D0001E (fixed in B1D0001F) reports 1024 while the FIFO's write pointer has wrapped and its read pointer not yet (23 times per second
    // at 48 kHz, for a few ms): never read more than can have arrived since the last read (98 % of 48 kHz; the rest stays for the
    // next round). Only a stall of this thread can then still overflow the FIFO.
    static double t_prev = 0.0;
    double t_now = now_s();
    if (n >= 1023 && t_prev > 0){
        unsigned est = (unsigned)((t_now - t_prev) * FS * 0.98);
        if (est < n) n = est; else atomic_fetch_add(&st_audio_ovf, 1);
    }
    t_prev = t_now;
    g_in_total += n; if (n > g_in_nmax) g_in_nmax = n;
    unsigned w = atomic_load(&aud_w), room = AUD_RING - RS_TAPS - aud_fill(), kept = 0;
    for (unsigned i = 0; i < n; i++){
        uint32_t v = rd(R_AUDIO);                                    // every read pops; keep what fits (the rate control prevents a full ring)
        if (kept == room) continue;
        aud_ring[(w + kept) % AUD_RING][0] = (float)(int16_t)(v >> 16) * (1.0f / 32768.0f);
        aud_ring[(w + kept) % AUD_RING][1] = (float)(int16_t)(v & 0xFFFF) * (1.0f / 32768.0f);
        kept++;
    }
    atomic_store(&aud_w, w + kept);
    uint32_t s = rd(R_PHASE);
    if (s & (1u << 16)){ atomic_fetch_add(&st_underflow, 1); hw_restart(); return; }
    unsigned lvl = s & 0x3FFF, space = lvl < OFDM_FIFO_MAX ? OFDM_FIFO_MAX - lvl : 0, avail = ph_fill();
    if (space > avail) space = avail;
    unsigned r = atomic_load(&ph_r);
    for (unsigned i = 0; i < space; i++) wr(R_PHASE, ph_ring[(r + i) % PH_RING]);
    atomic_store(&ph_r, r + space);
    g_words_out += space;
}

static void on_signal(int sig){ (void)sig; g_stop = 1; }

static void usage(void){
    fprintf(stderr,
        "skypluto-dab [-b kbit/s] [-s service] [-e ensemble] [-i service id hex] [-d DLS text] [-t DLS file] [-g gain 0..3] [-L level] [-T tone Hz]\n"
        "             [-S status file] [-c config file] [-v]\n"
        "skypluto-dab -f in.f32 -o phases.bin [-p audio clock ppm] [-n seconds] ...   (simulation, no hardware)\n");
}

int main(int argc, char **argv){
    int c;
    while ((c = getopt(argc, argv, "b:s:e:i:d:t:c:g:L:T:S:vf:o:p:n:h")) != -1){
        switch (c){
        case 'b': o_kbps = atoi(optarg); break;
        case 's': o_svc = optarg; break;
        case 'e': o_ens = optarg; break;
        case 'd': o_dls = optarg; break;
        case 't': o_dlsfile = optarg; break;
        case 'g': o_gain = atoi(optarg) & 3; break;
        case 'L': o_level = atoi(optarg); break;
        case 'T': o_tone = atof(optarg); break;
        case 'i': o_sid = (unsigned)strtoul(optarg, NULL, 16) & 0xFFFF; break;
        case 'S': o_status = optarg; break;
        case 'c': o_cfgfile = optarg; break;
        case 'v': o_verbose = 1; break;
        case 'f': o_simin = optarg; break;
        case 'o': o_simout = optarg; break;
        case 'p': o_ppm = atof(optarg); break;
        case 'n': o_secs = atof(optarg); break;
        default: usage(); return 2;
        }
    }
    if (o_kbps < 8 || o_kbps > 192 || o_kbps % 8){ fprintf(stderr, "bitrate: 8..192 kbit/s, multiple of 8\n"); return 2; }
    g_cfg = (dabplus_cfg){ o_kbps, 1, 0, 0, 1 };
    g_enc = dabplus_enc_open(&g_cfg, 0);
    dab_ensemble ens; memset(&ens, 0, sizeof ens);
    ens.eid = 0x8FED; snprintf(ens.label, sizeof ens.label, "%s", o_ens);
    ens.ecc = 0xE3; ens.lto_halfhours = 2; ens.nsvc = 1;
    ens.svc[0].sid = (uint16_t)o_sid; snprintf(ens.svc[0].label, sizeof ens.svc[0].label, "%s", o_svc);
    ens.svc[0].bitrate = o_kbps; ens.svc[0].prot_level = 3;
    g_mod = dab_mod_open(&ens); g_ens = ens;
    if (!g_enc || !g_mod){ fprintf(stderr, "bad configuration\n"); return 2; }
    dab_mod_set_phases_only(g_mod, 1);                               // the FPGA does the OFDM
    if (o_dls) dabplus_enc_set_dls(g_enc, o_dls, 15);
    g_pcm = calloc(SF_SAMPLES * 2, sizeof(float)); g_sf = malloc(120 * 24);   /* the largest super frame (192 kbit/s): the bitrate can change */ g_lfb = dab_mod_lf_bytes(g_mod, 0);
    rs_init();
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);

    if (o_simin){
        // ---- simulation: audio arrives at 48 kHz x (1 + ppm), the OFDM block takes 76000 words/s; steps of 4 ms ----------------
        FILE *fi = fopen(o_simin, "rb"), *fo = o_simout ? fopen(o_simout, "wb") : NULL;
        if (!fi || !fo){ perror("file"); return 2; }
        double acc = 0, wacc = 0, t = 0; long words = 0; int eof = 0;
        unsigned fifo = 0;                                           // simulated OFDM FIFO level (g_sim_fifo for latency())
        while (!eof && (o_secs <= 0 || t < o_secs)){
            acc += FS * (1 + o_ppm * 1e-6) * 0.004;
            int n = (int)acc; acc -= n;
            for (int i = 0; i < n; i++){
                float s[2];
                if (fread(s, sizeof s, 1, fi) != 1){ eof = 1; break; }
                if (aud_fill() < AUD_RING - RS_TAPS){ unsigned w = atomic_load(&aud_w); aud_ring[w % AUD_RING][0] = s[0]; aud_ring[w % AUD_RING][1] = s[1]; atomic_store(&aud_w, w + 1); }
                else atomic_fetch_add(&st_audio_ovf, 1);
            }
            while (producer_wants()) if (produce_superframe() < 0) return 1;
            wacc += WORDS_PER_S * 0.004; int take = (int)wacc; wacc -= take;   // the FPGA consumes
            if ((unsigned)take > fifo){ if (words > FRAME_WORDS) atomic_fetch_add(&st_underflow, 1); fifo = 0; } else fifo -= take;
            unsigned space = OFDM_FIFO_MAX - fifo, avail = ph_fill(); if (space > avail) space = avail;
            unsigned r = atomic_load(&ph_r);
            for (unsigned i = 0; i < space; i++){ uint32_t v = ph_ring[(r + i) % PH_RING]; fwrite(&v, 4, 1, fo); }
            atomic_store(&ph_r, r + space); fifo += space; words += space; g_sim_fifo = fifo;
            t += 0.004;
            if (o_verbose && fmod(t, 5.0) < 0.004)
                fprintf(stderr, "t=%.0f s audio %u (target %d) latency %.0f (target %.0f) ratio %+.1f ppm silence %u resync %u underflow %u\n", t, aud_fill(), AUD_TARGET, latency(), g_lat_target,
                        g_ppm_now, atomic_load(&st_silence), atomic_load(&st_resync), atomic_load(&st_underflow));
        }
        fclose(fi); fclose(fo);
        printf("simulation: %.1f s, %ld phase words (%ld frames), final ratio %+.1f ppm, silent super frames %u, underflows %u, audio overflows %u\n",
               t, words, words / FRAME_WORDS, g_ppm_now, atomic_load(&st_silence), atomic_load(&st_underflow), atomic_load(&st_audio_ovf));
        printf("time per second of audio: AAC encoder %.0f ms, DAB multiplex %.0f ms\n", 1000 * g_t_enc / t, 1000 * g_t_mod / t);
        return 0;
    }

    // ---- hardware ------------------------------------------------------------------------------------------------------------
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0){ perror("open /dev/mem"); return 1; }
    void *m = mmap(NULL, MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, WFM_BASE);
    if (m == MAP_FAILED){ perror("mmap"); return 1; }
    g_reg = (volatile uint32_t *)m;
    if (o_level >= 0) wr(R_LEVEL, (uint32_t)o_level);
    wr(R_DAB, (uint32_t)(0x3 | (o_gain << 2)));                     // DAB mode with the OFDM block in reset: no output until the first frame is ready
    for (int i = 0; i < 2; i++) if (produce_superframe() < 0) return 1;
    wr(R_DAB, (uint32_t)(0x1 | (o_gain << 2)));                     // DAB mode
    while (rd(R_AUDCNT)) (void)rd(R_AUDIO);                           // stale audio
    if (o_tone > 0) fprintf(stderr, "test tone %.0f Hz left, %.0f Hz right (-12 dBFS)\n", o_tone, 1.5 * o_tone);
    pthread_t th; pthread_create(&th, NULL, producer_thread, NULL);
    struct timespec t0; clock_gettime(CLOCK_MONOTONIC, &t0); double last = 0;
    while (!g_stop){
        hw_io_step();
        usleep(3000);
        struct timespec t1; clock_gettime(CLOCK_MONOTONIC, &t1);
        double t = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;
        // the input clock against this CPU's clock over 10 s (from 2 s on): the start value of the rate control
        { static int seed_state = 0; static double seed_t0; static unsigned long long seed_n0;
          if (o_tone <= 0 && seed_state == 0 && t >= 2.0){ seed_t0 = t; seed_n0 = g_in_total; seed_state = 1; }
          else if (seed_state == 1 && t - seed_t0 >= 10.0){
              double ppm = ((double)(g_in_total - seed_n0) / (t - seed_t0) / FS - 1.0) * 1e6;
              if (fabs(ppm) < RC_MAX){ g_seed_ppm = ppm; atomic_store(&g_seed_ready, 1); }
              fprintf(stderr, "    input clock %+.0f ppm (measured over 10 s)\n", ppm);
              seed_state = 2;
          } }
        static double last_st = 0;
        if (o_status && t - last_st >= 1.0){
            // one line for skypluto-ctl (?DAB), replaced as a whole (rename) so a reader never sees half a line
            last_st = t;
            char tmp[256]; snprintf(tmp, sizeof tmp, "%s.new", o_status);
            FILE *sf = fopen(tmp, "w");
            if (sf){
                fprintf(sf, "fifo=%u uf=%u rst=%u aud=%u ppm=%+.0f sil=%u cpu=%.0f\n", rd(R_PHASE) & 0x3FFF, atomic_load(&st_underflow), atomic_load(&st_resets),
                        aud_fill(), g_ppm_now, atomic_load(&st_silence), 100 * (g_t_enc + g_t_mod) / t);
                fclose(sf); rename(tmp, o_status);
            }
        }
        if (o_verbose && t - last >= 5.0){
            static unsigned long long in_last = 0;
            fprintf(stderr, "    audio in %.0f samples/s, largest batch %u\n", (g_in_total - in_last) / (t - last), g_in_nmax);
            in_last = g_in_total; g_in_nmax = 0;
            last = t;
            fprintf(stderr, "t=%.0f s audio %u ratio %+.1f ppm phase queue %u OFDM FIFO %u silence %u resync %u underflow %u restarts %u audio-ovf %u cpu enc %.0f%% mux %.0f%%\n",
                    t, aud_fill(), g_ppm_now, ph_fill(), rd(R_PHASE) & 0x3FFF, atomic_load(&st_silence), atomic_load(&st_resync), atomic_load(&st_underflow),
                    atomic_load(&st_resets), atomic_load(&st_audio_ovf), 100 * g_t_enc / t, 100 * g_t_mod / t);
        }
    }
    pthread_join(th, NULL);
    wr(R_DAB, 0x2); usleep(20); wr(R_DAB, 0x0);                      // back to FM
    munmap(m, MAP_LEN); close(fd);
    return 0;
}
