// SPDX-License-Identifier: Apache-2.0
#include "ext_stream.h"
#include "dabplus_enc.h"
#include "zmtp_sub.h"
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RING     32                        // super frames (3.8 s)
#define TARGET   5                         // start (again) with this many buffered: 600 ms against network jitter
#define HIGH     12                        // above this many the oldest is dropped (the remote clock runs fast, or a burst)
#define HDR      12                        // ODR-AudioEnc's ZMQ frame header
#define SFMAX    (120 * 192 / 8)           // the largest super frame (192 kbit/s): the buffers hold any bitrate
#define DETECT   3                         // super frames in a row of one other size before that bitrate counts as detected

struct ext_stream {
    int port, kbps, sfbytes, sls; unsigned sid; char label[17]; char file[128];
    zmtp_sub *z; FILE *sim;
    uint8_t *ring; atomic_uint w, r; int running;
    uint8_t *silence, *msg;
    atomic_ulong received, used, silent, dropped, bad;
    atomic_int lvl_l, lvl_r;
    atomic_int det_kbps, det_count;         // the bitrate the sender uses (from the super frame size) and how often in a row
};

// a silent super frame of the current bitrate, encoded once: sent whenever the stream has nothing
static int make_silence(ext_stream *e){
    dabplus_cfg c = { e->kbps, 1, 0, 0, 1 };
    dabplus_enc *enc = dabplus_enc_open(&c, 0);
    float *z = calloc(6 * 960 * 2, sizeof(float));
    int ok = enc && z;
    for (int i = 0; ok && i < 3; i++) ok = dabplus_enc_superframe(enc, z, e->silence) == 0;   // a few: past the encoder's start-up
    dabplus_enc_close(enc); free(z);
    return ok ? 0 : -1;
}

static ext_stream *start(ext_stream *e){
    if (e->kbps < 8 || e->kbps > 192 || e->kbps % 8 || !e->sid){ free(e); return NULL; }
    e->sfbytes = 120 * e->kbps / 8; atomic_store(&e->det_kbps, e->kbps);
    e->ring = malloc((size_t)RING * SFMAX); e->silence = malloc(SFMAX); e->msg = malloc(SFMAX + HDR + 64);
    int ok = e->ring && e->silence && e->msg && make_silence(e) == 0;
    if (ok && e->file[0]) ok = (e->sim = fopen(e->file, "rb")) != NULL;
    else if (ok) ok = (e->z = zmtp_sub_listen(e->port)) != NULL;
    if (!ok){ ext_close(e); return NULL; }
    return e;
}

ext_stream *ext_open_params(int port, int kbps, unsigned sid, const char *label, int sls, const char *simfile){
    ext_stream *e = calloc(1, sizeof *e); if (!e) return NULL;
    e->port = port; e->kbps = kbps; e->sid = sid; e->sls = sls;
    snprintf(e->label, sizeof e->label, "%.16s", label ? label : "Extern");
    if (simfile) snprintf(e->file, sizeof e->file, "%.127s", simfile);
    return start(e);
}

ext_stream *ext_open(const char *conf_file){
    FILE *f = fopen(conf_file, "r"); if (!f) return NULL;
    ext_stream *e = calloc(1, sizeof *e); if (!e){ fclose(f); return NULL; }
    e->port = 2491; e->kbps = 128; e->sls = 1; e->sid = 0x8F02; snprintf(e->label, sizeof e->label, "Extern");
    char ln[160];
    while (fgets(ln, sizeof ln, f)){
        ln[strcspn(ln, "\r\n")] = 0;
        if (!strncmp(ln, "port=", 5)) e->port = atoi(ln + 5);
        else if (!strncmp(ln, "br=", 3)) e->kbps = atoi(ln + 3);
        else if (!strncmp(ln, "sls=", 4)) e->sls = atoi(ln + 4) ? 1 : 0;
        else if (!strncmp(ln, "sid=", 4)) e->sid = (unsigned)strtoul(ln + 4, NULL, 16) & 0xFFFF;
        else if (!strncmp(ln, "label=", 6)) snprintf(e->label, sizeof e->label, "%.16s", ln + 6);
        else if (!strncmp(ln, "file=", 5)) snprintf(e->file, sizeof e->file, "%.127s", ln + 5);
    }
    fclose(f);
    return start(e);
}

int ext_port(const ext_stream *e){ return e->port; }

void ext_close(ext_stream *e){ if (!e) return; zmtp_sub_close(e->z); if (e->sim) fclose(e->sim); free(e->ring); free(e->silence); free(e->msg); free(e); }
int ext_kbps(const ext_stream *e){ return e->kbps; }
int ext_sls(const ext_stream *e){ return e->sls; }
unsigned ext_sid(const ext_stream *e){ return e->sid; }
const char *ext_label(const ext_stream *e){ return e->label; }

static void take(ext_stream *e, const uint8_t *m, int n){
    // header: version (16 bit LE, 1), encoder, data size (32 bit LE), audio level left/right (16 bit)
    if (n < HDR || (m[0] | m[1] << 8) != 1){ atomic_fetch_add(&e->bad, 1); return; }
    uint32_t sz = (uint32_t)m[4] | (uint32_t)m[5] << 8 | (uint32_t)m[6] << 16 | (uint32_t)m[7] << 24;
    if (n < HDR + (int)sz){ atomic_fetch_add(&e->bad, 1); return; }
    if (sz != (uint32_t)e->sfbytes){
        // another bitrate than the sub-channel has: not usable, but remembered (the layout follows the sender, see ext_stats.kbps_in)
        int k = (sz * 8) % 120 ? 0 : (int)(sz * 8 / 120);
        if (k >= 8 && k <= 192 && k % 8 == 0){
            if (atomic_load(&e->det_kbps) == k) atomic_fetch_add(&e->det_count, 1); else { atomic_store(&e->det_kbps, k); atomic_store(&e->det_count, 1); }
        }
        atomic_fetch_add(&e->bad, 1); return;
    }
    atomic_store(&e->det_kbps, e->kbps); atomic_store(&e->det_count, 0);
    atomic_store(&e->lvl_l, m[8] | m[9] << 8); atomic_store(&e->lvl_r, m[10] | m[11] << 8);
    atomic_fetch_add(&e->received, 1);
    unsigned w = atomic_load(&e->w);
    if (w - atomic_load(&e->r) >= RING){ atomic_fetch_add(&e->dropped, 1); return; }
    memcpy(e->ring + (size_t)(w % RING) * e->sfbytes, m + HDR, sz);
    atomic_store(&e->w, w + 1);
}

void ext_poll(ext_stream *e){
    if (e->sim){                                               // simulation: keep the buffer at its target from the recording
        while (atomic_load(&e->w) - atomic_load(&e->r) < TARGET){
            uint32_t len; if (fread(&len, 4, 1, e->sim) != 1 || len > (uint32_t)SFMAX + HDR + 64){ rewind(e->sim); if (fread(&len, 4, 1, e->sim) != 1) return; }
            if (fread(e->msg, 1, len, e->sim) != len) return;
            take(e, e->msg, (int)len);
        }
        return;
    }
    int n;
    while ((n = zmtp_sub_poll(e->z, e->msg, (size_t)SFMAX + HDR + 64)) > 0) take(e, e->msg, n);
}

const uint8_t *ext_next(ext_stream *e){
    unsigned r = atomic_load(&e->r), fill = atomic_load(&e->w) - r;
    if (!e->running && fill >= TARGET) e->running = 1;
    if (!e->running || fill == 0){ e->running = 0; atomic_fetch_add(&e->silent, 1); return e->silence; }   // nothing (yet): silence, then fill up again
    if (fill > HIGH){ r++; atomic_fetch_add(&e->dropped, 1); }  // too much waiting: skip one (120 ms)
    const uint8_t *p = e->ring + (size_t)(r % RING) * e->sfbytes;
    atomic_store(&e->r, r + 1); atomic_fetch_add(&e->used, 1);
    return p;
}

void ext_get_stats(const ext_stream *e, ext_stats *s){
    s->received = atomic_load(&e->received); s->used = atomic_load(&e->used); s->silent = atomic_load(&e->silent);
    s->dropped = atomic_load(&e->dropped); s->bad = atomic_load(&e->bad);
    s->fill = (int)(atomic_load(&e->w) - atomic_load(&e->r));
    s->level_l = atomic_load(&e->lvl_l); s->level_r = atomic_load(&e->lvl_r);
    s->peer = e->z ? zmtp_sub_peer(e->z) : "file";
    s->kbps_in = atomic_load(&e->det_count) >= DETECT ? atomic_load(&e->det_kbps) : e->kbps;
}

int ext_set_kbps(ext_stream *e, int kbps){
    if (kbps < 8 || kbps > 192 || kbps % 8) return -1;
    if (kbps == e->kbps) return 0;
    int old = e->kbps; e->kbps = kbps;
    if (make_silence(e)){ e->kbps = old; return -1; }
    e->sfbytes = 120 * kbps / 8; e->running = 0;
    atomic_store(&e->r, atomic_load(&e->w));                   // what is buffered has the old size
    atomic_store(&e->det_kbps, kbps); atomic_store(&e->det_count, 0);
    return 0;
}
