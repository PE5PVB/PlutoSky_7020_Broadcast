// SPDX-License-Identifier: Apache-2.0
#include "dabplus_enc.h"
#include "aac_enc.h"
#include "dab_pad.h"
#include "dab_mot.h"
#include <stdlib.h>
#include <string.h>

#define MIN_AU 8                                   // bytes the encoder can always fit (silent frame)

struct dabplus_enc {
    dabplus_cfg cfg;
    aacenc *aac;
    int n;
    uint8_t au[6][2048];
    int len[6];
    dab_dls dls;
    dab_mot mot;
};

static aacenc *open_aac(const dabplus_cfg *cfg, int bandwidth_hz){
    const int sr = cfg->dac_rate ? 48000 : 32000, ch = cfg->stereo ? 2 : 1;
    if (bandwidth_hz <= 0){                        // rough default: audio bandwidth grows with the bitrate per channel
        int per = cfg->bitrate_kbps / ch;
        bandwidth_hz = per >= 64 ? 16000 : per >= 48 ? 15000 : per >= 40 ? 13000 : per >= 32 ? 11000 : 8000;
    }
    if (bandwidth_hz > sr / 2 - 1000) bandwidth_hz = sr / 2 - 1000;
    return aacenc_open(sr, ch, bandwidth_hz);
}

dabplus_enc *dabplus_enc_open(const dabplus_cfg *cfg, int bandwidth_hz){
    if (cfg->sbr || cfg->ps || cfg->bitrate_kbps < 8 || cfg->bitrate_kbps > 192 || (cfg->bitrate_kbps % 8)) return NULL;
    dabplus_enc *e = calloc(1, sizeof *e);
    if (!e) return NULL;
    e->cfg = *cfg;
    e->n = dabplus_num_aus(cfg);
    e->aac = open_aac(cfg, bandwidth_hz);
    if (!e->aac){ free(e); return NULL; }
    dab_mot_init(&e->mot);
    return e;
}

int dabplus_enc_set_bitrate(dabplus_enc *e, int kbps){
    if (kbps < 8 || kbps > 192 || kbps % 8) return -1;
    dabplus_cfg c = e->cfg; c.bitrate_kbps = kbps;
    aacenc *a = open_aac(&c, 0);                   // the audio bandwidth follows the bitrate; the DLS state stays
    if (!a) return -1;
    aacenc_close(e->aac); e->aac = a; e->cfg = c; e->n = dabplus_num_aus(&c);
    return 0;
}

void dabplus_enc_close(dabplus_enc *e){ if (!e) return; aacenc_close(e->aac); dab_mot_free(&e->mot); free(e); }

// slideshow bytes per access unit: about a tenth of the bitrate at 128 kbit/s and more, less below (the PAD is taken from the audio)
static int mot_chunk(int kbps){ return kbps >= 128 ? 48 : kbps >= 80 ? 32 : kbps >= 48 ? 24 : 8; }

int dabplus_enc_frames(const dabplus_enc *e){ return e->n; }

int dabplus_enc_superframe(dabplus_enc *e, const float *pcm, uint8_t *out){
    const int ch = e->cfg.stereo ? 2 : 1;
    // the largest AU: what an AAC decoder must accept (6144 bits per channel), within our buffer
    const int cap = 768 * ch < (int)sizeof e->au[0] ? 768 * ch : (int)sizeof e->au[0];
    int remaining = dabplus_payload_budget(&e->cfg);
    // what every later AU must keep: a silent frame plus its slideshow piece (CI list, length indicator, the piece)
    const int resv = MIN_AU + (dab_mot_busy(&e->mot) ? mot_chunk(e->cfg.bitrate_kbps) + 12 : 0);
    for (int i = 0; i < e->n; i++){
        const int left = e->n - 1 - i;
        int maxb = remaining - left * resv, target = remaining / (e->n - i), pad = 0;
        if (maxb > cap) maxb = cap;
        if (target > maxb) target = maxb;
        // fill: what the AUs after this one cannot hold has to go into this one (after quiet AUs, at high bitrates, the
        // last AU alone could not take the rest of the super frame); the last AU takes all that remains
        const int need = remaining - left * cap;
        if (need > 0){ pad = need > target ? need : target; if (pad > cap) return -1; maxb = pad; if (target > pad) target = pad; }
        // aim a little below the hard maximum: the encoder's bit count can overshoot its target by a few bits, and the fill elements
        // (pad) make up the difference where the size must be exact (the last AU, whose target would otherwise equal its maximum)
        if (target > maxb - 8 && maxb - 8 >= MIN_AU) target = maxb - 8;
        {                                                          // PAD: one DLS segment per super frame (first AU), slideshow pieces in every AU
            dab_xpad_sub s[4]; int ns = 0; uint8_t dlg[20];
            if (i == 0){ int n = dab_dls_next_dg(&e->dls, dlg); if (n > 0) s[ns++] = (dab_xpad_sub){ 2, dlg, n }; }
            ns += dab_mot_next(&e->mot, mot_chunk(e->cfg.bitrate_kbps), s + ns);
            uint8_t padb[260]; int pl = ns ? dab_xpad_build(s, ns, padb, sizeof padb) : 0;
            if (pl > 0) aacenc_set_dse(e->aac, padb, pl);
        }
        int r = aacenc_frame(e->aac, pcm + (size_t)i * 960 * ch, target, maxb, pad, e->au[i]);
        if (r < 1) return -1;
        e->len[i] = r;
        remaining -= r;
    }
    const uint8_t *p[6]; for (int i = 0; i < e->n; i++) p[i] = e->au[i];
    return dabplus_build(&e->cfg, p, e->len, out);
}

int dabplus_enc_last_aus(const dabplus_enc *e, const uint8_t **au, int *len){
    for (int i = 0; i < e->n; i++){ au[i] = e->au[i]; len[i] = e->len[i]; }
    return e->n;
}

void dabplus_enc_set_dls(dabplus_enc *e, const char *text, int charset){ dab_dls_set(&e->dls, text, charset); }

int dabplus_enc_set_slide(dabplus_enc *e, const uint8_t *img, int len, int subtype, const char *name, unsigned tid){
    return dab_mot_load(&e->mot, img, len, subtype, name, tid);
}
int dabplus_enc_slide_busy(const dabplus_enc *e){ return dab_mot_busy(&e->mot); }
void dabplus_enc_slide_abort(dabplus_enc *e){ dab_mot_abort(&e->mot); }
int dabplus_enc_slide_rate(const dabplus_enc *e){ return mot_chunk(e->cfg.bitrate_kbps) * e->n * 1000 / 120; }

int dabplus_enc_asc(const dabplus_enc *e, uint8_t asc[2]){ return aacenc_asc(e->aac, asc); }
