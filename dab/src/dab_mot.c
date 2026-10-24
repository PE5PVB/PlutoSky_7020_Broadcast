// SPDX-License-Identifier: Apache-2.0
#include "dab_mot.h"
#include "dab_crc.h"
#include <stdlib.h>
#include <string.h>

#define SEG_MAX 8189                       // MOT segment size (13 bits), and a data group stays below the 14-bit DGLI

void dab_mot_init(dab_mot *m){ memset(m, 0, sizeof *m); }
void dab_mot_free(dab_mot *m){ free(m->buf); dab_mot_init(m); }
int  dab_mot_busy(const dab_mot *m){ return m->cur < m->ndg; }
void dab_mot_abort(dab_mot *m){ m->cur = m->ndg; }

// one MSC data group (EN 300 401 5.3.3) carrying a MOT segment: header with CRC, segment and user access flags, session
// header (segment number, last flag, transport id), MOT segmentation header (EN 301 234 5.1.1), the data, CRC
static int put_dg(dab_mot *m, int type, unsigned ci, int segno, int last, unsigned tid, const uint8_t *d, int n){
    if (m->ndg >= 16) return -1;
    int o = m->ndg ? m->dg_off[m->ndg - 1] + m->dg_len[m->ndg - 1] : 0, p = o;
    if (o + 13 + n > m->cap) return -1;
    uint8_t *b = m->buf;
    b[p++] = (uint8_t)(0x70 | type);                                   // extension 0, CRC 1, segment 1, user access 1
    b[p++] = (uint8_t)((ci & 15) << 4);                                // continuity index, repetition index 0
    b[p++] = (uint8_t)((last << 7) | ((segno >> 8) & 0x7F)); b[p++] = (uint8_t)segno;
    b[p++] = 0x12;                                                     // transport id flag, length indicator 2
    b[p++] = (uint8_t)(tid >> 8); b[p++] = (uint8_t)tid;
    b[p++] = (uint8_t)((n >> 8) & 0x1F); b[p++] = (uint8_t)n;          // repetition count 0, segment size
    memcpy(b + p, d, (size_t)n); p += n;
    uint16_t crc = dab_crc16(b + o, (size_t)(p - o));
    b[p++] = (uint8_t)(crc >> 8); b[p++] = (uint8_t)crc;
    m->dg_off[m->ndg] = o; m->dg_len[m->ndg] = p - o; m->ndg++;
    return 0;
}

int dab_mot_load(dab_mot *m, const uint8_t *img, int len, int subtype, const char *name, unsigned tid){
    if (len < 1 || len > DAB_MOT_MAX_BODY) return -1;
    const int nbody = (len + SEG_MAX - 1) / SEG_MAX, cap = len + (nbody + 1) * 13 + 128;
    if (cap > m->cap){ uint8_t *nb = realloc(m->buf, (size_t)cap); if (!nb) return -1; m->buf = nb; m->cap = cap; }
    m->ndg = 0; m->cur = 0; m->pos = 0;
    // the MOT header: core (body size 28 bits, header size 13, content type 6, subtype 9), then the extension:
    // TriggerTime = now (PLI 10, id 0x05, 4 zero bytes) and ContentName (PLI 11, id 0x0C, length, charset 0, the name)
    uint8_t h[96]; int hp = 7, nl = (int)strlen(name); if (nl > 64) nl = 64;
    h[hp++] = 0x85; memset(h + hp, 0, 4); hp += 4;
    h[hp++] = 0xCC; h[hp++] = (uint8_t)(nl + 1); h[hp++] = 0x00; memcpy(h + hp, name, (size_t)nl); hp += nl;
    const unsigned bs = (unsigned)len, hs = (unsigned)hp, ct = 2, cs = (unsigned)subtype;
    h[0] = (uint8_t)(bs >> 20); h[1] = (uint8_t)(bs >> 12); h[2] = (uint8_t)(bs >> 4);
    h[3] = (uint8_t)(((bs & 15) << 4) | (hs >> 9)); h[4] = (uint8_t)(hs >> 1);
    h[5] = (uint8_t)(((hs & 1) << 7) | (ct << 1) | (cs >> 8)); h[6] = (uint8_t)cs;
    if (put_dg(m, 3, m->ci_hdr++, 0, 1, tid, h, hp)) return -1;
    for (int s = 0; s < nbody; s++){
        const int off = s * SEG_MAX, n = len - off < SEG_MAX ? len - off : SEG_MAX;
        if (put_dg(m, 4, m->ci_body++, s, s == nbody - 1, tid, img + off, n)) return -1;
    }
    return 0;
}

int dab_mot_next(dab_mot *m, int chunk, dab_xpad_sub *s){
    if (!dab_mot_busy(m) || chunk < 4) return 0;
    if (chunk > 48) chunk = 48;
    const uint8_t *dg = m->buf + m->dg_off[m->cur]; const int dl = m->dg_len[m->cur];
    int k = 0;
    if (m->pos == 0){                                                  // a new data group: its length indicator first
        m->dgli[0] = (uint8_t)((dl >> 8) & 0x3F); m->dgli[1] = (uint8_t)dl;
        uint16_t crc = dab_crc16(m->dgli, 2); m->dgli[2] = (uint8_t)(crc >> 8); m->dgli[3] = (uint8_t)crc;
        s[k++] = (dab_xpad_sub){ 1, m->dgli, 4 };
    }
    const int n = dl - m->pos < chunk ? dl - m->pos : chunk;
    s[k++] = (dab_xpad_sub){ m->pos == 0 ? 12 : 13, dg + m->pos, n };
    m->pos += n;
    if (m->pos >= dl){ m->cur++; m->pos = 0; }
    return k;
}
