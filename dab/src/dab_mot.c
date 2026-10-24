// SPDX-License-Identifier: Apache-2.0
#include "dab_mot.h"
#include "dab_crc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SEG_MAX \
    1013 // MOT body segment: data groups of 1 kB as ODR-PadEnc sends them (receivers with small
         // buffers take those; the format would allow 8189)

// one MSC data group (EN 300 401 5.3.3) carrying a MOT segment: header with CRC, segment and user access flags, session
// header (segment number, last flag, transport id), MOT segmentation header (EN 301 234 5.1.1: repetition count, segment
// size), the data, CRC. b must hold n + 13 bytes; returns the length.
static int mk_dg(uint8_t *b, int type, unsigned ci, int segno, int last, unsigned tid, int rep, const uint8_t *d, int n){
    int p = 0;
    b[p++] = (uint8_t)(0x70 | type);    // extension 0, CRC 1, segment 1, user access 1
    b[p++] = (uint8_t)((ci & 15) << 4); // continuity index, repetition index 0
    b[p++] = (uint8_t)((last << 7) | ((segno >> 8) & 0x7F));
    b[p++] = (uint8_t)segno;
    b[p++] = 0x12; // transport id flag, length indicator 2
    b[p++] = (uint8_t)(tid >> 8);
    b[p++] = (uint8_t)tid;
    b[p++] = (uint8_t)(((rep & 7) << 5) | ((n >> 8) & 0x1F));
    b[p++] = (uint8_t)n; // repetition count, segment size
    memcpy(b + p, d, (size_t)n);
    p += n;
    uint16_t crc = dab_crc16(b, (size_t)p);
    b[p++] = (uint8_t)(crc >> 8);
    b[p++] = (uint8_t)crc;
    return p;
}

// the X-PAD subfields for the next piece of a data group (dg, dl bytes, *pos already sent): at the start its length indicator
// (application type 1, with CRC, into dgli) and the first piece (app), else a continuation (app + 1)
static int xpad_piece(const uint8_t *dg, int dl, int *pos, uint8_t dgli[4], int app, int chunk, dab_xpad_sub *s){
    int k = 0;
    if (*pos == 0){
        dgli[0] = (uint8_t)((dl >> 8) & 0x3F);
        dgli[1] = (uint8_t)dl;
        uint16_t crc = dab_crc16(dgli, 2);
        dgli[2] = (uint8_t)(crc >> 8);
        dgli[3] = (uint8_t)crc;
        s[k++] = (dab_xpad_sub){ 1, dgli, 4 };
    }
    const int n = dl - *pos < chunk ? dl - *pos : chunk;
    s[k++] = (dab_xpad_sub){ *pos == 0 ? app : app + 1, dg + *pos, n };
    *pos += n;
    return k;
}

// ==== header mode (slideshow) ===================================================================================================
void dab_mot_init(dab_mot *m) { memset(m, 0, sizeof *m); }
void dab_mot_free(dab_mot *m){
    free(m->buf);
    dab_mot_init(m);
}
int dab_mot_busy(const dab_mot *m) { return m->cur < m->ndg; }
void dab_mot_abort(dab_mot *m) { m->cur = m->ndg; }

static int put_dg(dab_mot *m, int type, unsigned ci, int segno, int last, unsigned tid, const uint8_t *d, int n){
    if (m->ndg >= DAB_MOT_MAX_DG) return -1;
    int o = m->ndg ? m->dg_off[m->ndg - 1] + m->dg_len[m->ndg - 1] : 0;
    if (o + 13 + n > m->cap) return -1;
    m->dg_off[m->ndg] = o;
    m->dg_len[m->ndg] = mk_dg(m->buf + o, type, ci, segno, last, tid, 0, d, n);
    m->ndg++;
    return 0;
}

int dab_mot_load(dab_mot *m, const uint8_t *img, int len, int subtype, const char *name, unsigned tid){
    if (len < 1 || len > DAB_MOT_MAX_BODY) return -1;
    const int nbody = (len + SEG_MAX - 1) / SEG_MAX, cap = len + (nbody + 1) * 13 + 128;
    if (cap > m->cap){
        uint8_t *nb = realloc(m->buf, (size_t)cap);
        if (!nb) return -1;
        m->buf = nb;
        m->cap = cap;
    }
    m->ndg = 0;
    m->cur = 0;
    m->pos = 0;
    // the MOT header: core (body size 28 bits, header size 13, content type 6, subtype 9), then the extension:
    // TriggerTime = now (PLI 10, id 0x05, 4 zero bytes) and ContentName (PLI 11, id 0x0C, length, charset 0, the name)
    uint8_t h[96];
    int hp = 7, nl = (int)strlen(name);
    if (nl > 64) nl = 64;
    h[hp++] = 0x85;
    memset(h + hp, 0, 4);
    hp += 4;
    h[hp++] = 0xCC;
    h[hp++] = (uint8_t)(nl + 1);
    h[hp++] = 0x00;
    memcpy(h + hp, name, (size_t)nl);
    hp += nl;
    const unsigned bs = (unsigned)len, hs = (unsigned)hp, ct = 2, cs = (unsigned)subtype;
    h[0] = (uint8_t)(bs >> 20);
    h[1] = (uint8_t)(bs >> 12);
    h[2] = (uint8_t)(bs >> 4);
    h[3] = (uint8_t)(((bs & 15) << 4) | (hs >> 9));
    h[4] = (uint8_t)(hs >> 1);
    h[5] = (uint8_t)(((hs & 1) << 7) | (ct << 1) | (cs >> 8));
    h[6] = (uint8_t)cs;
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
    const int k = xpad_piece(m->buf + m->dg_off[m->cur], m->dg_len[m->cur], &m->pos, m->dgli, 12, chunk, s);
    if (m->pos >= m->dg_len[m->cur]){
        m->cur++;
        m->pos = 0;
    }
    return k;
}

// ==== directory mode (carousel) =================================================================================================
// MOT header information of an object: core (BodySize 28, HeaderSize 13, ContentType 6, ContentSubType 9), ContentName (PLI 11,
// id 0x0C, charset 15 = UTF-8), then the object's own parameters. Returns the size, -1 if it does not fit.
static int mk_header(uint8_t *h, int cap, const dab_motdir_obj *o){
    int nl = (int)strlen(o->name), hp = 7;
    if (nl < 1 || nl > DAB_MOTDIR_MAX_NAME) return -1;
    if (hp + 3 + nl + o->nparam > cap) return -1;
    h[hp++] = 0xCC;
    h[hp++] = (uint8_t)(nl + 1);
    h[hp++] = 0xF0;
    memcpy(h + hp, o->name, (size_t)nl);
    hp += nl;
    if (o->nparam > 0) memcpy(h + hp, o->param, (size_t)o->nparam);
    hp += o->nparam;
    const unsigned bs = (unsigned)o->len, hs = (unsigned)hp, ct = (unsigned)o->ct & 63, cs = (unsigned)o->cst & 511;
    h[0] = (uint8_t)(bs >> 20);
    h[1] = (uint8_t)(bs >> 12);
    h[2] = (uint8_t)(bs >> 4);
    h[3] = (uint8_t)(((bs & 15) << 4) | (hs >> 9));
    h[4] = (uint8_t)(hs >> 1);
    h[5] = (uint8_t)(((hs & 1) << 7) | (ct << 1) | (cs >> 8));
    h[6] = (uint8_t)cs;
    return hp;
}

static uint32_t fnv(uint32_t h, const uint8_t *d, int n){
    for (int i = 0; i < n; i++) h = (h ^ d[i]) * 16777619u;
    return h;
}

void dab_motdir_init(dab_motdir *c, int app, unsigned tid_seed){
    memset(c, 0, sizeof *c);
    c->app = app;
    c->next_tid = tid_seed & 0xFFFF;
}
void dab_motdir_free(dab_motdir *c){
    free(c->buf);
    free(c->dg_off);
    free(c->dg_len);
    const int app = c->app;
    const unsigned t = c->next_tid;
    dab_motdir_init(c, app, t);
}
static unsigned new_tid(dab_motdir *c){
    c->next_tid = (c->next_tid + 1) & 0xFFFF;
    if (!c->next_tid) c->next_tid = 1;
    return c->next_tid;
}
static int cmp_name(const void *a, const void *b){
    const dab_motdir_obj *x = *(const dab_motdir_obj *const *)a, *y = *(const dab_motdir_obj *const *)b;
    return strcmp(x->name, y->name); // byte by byte (EN 301 234 annex A); names are unique
}
// the data groups of one entity (directory or body), segments of SEG_MAX
static int add_entity(dab_motdir *c, int e, int type, unsigned tid, const uint8_t *d, int n){
    const int nseg = n > 0 ? (n + SEG_MAX - 1) / SEG_MAX : 1;
    c->ent[e].dg0 = c->ndg;
    c->ent[e].ndg = nseg;
    c->ent[e].type = type;
    c->ent[e].tid = tid;
    for (int s = 0; s < nseg; s++){
        const int off = s * SEG_MAX, k = n - off < SEG_MAX ? n - off : SEG_MAX;
        if (c->ndg >= c->max_dg || c->used + k + 13 > c->cap) return -1;
        c->dg_off[c->ndg] = c->used;
        c->dg_len[c->ndg] = mk_dg(c->buf + c->used, type, 0, s, s == nseg - 1, tid, 0, d + off, k);
        c->used += c->dg_len[c->ndg];
        c->ndg++;
    }
    return 0;
}

int dab_motdir_load(dab_motdir *c, const dab_motdir_obj *objs, int n){
    if (n < 0 || n > DAB_MOTDIR_MAX_OBJ) return -1;
    if (n == 0){
        dab_motdir_free(c);
        return 0;
    }
    // directory: the core (13 bytes), the extension (SortedHeaderInformation) and per object TransportId + header information
    const dab_motdir_obj *ord[DAB_MOTDIR_MAX_OBJ];
    int total = 0;
    for (int i = 0; i < n; i++){
        ord[i] = &objs[i];
        if (objs[i].len < 0 || objs[i].nparam < 0 || objs[i].nparam > 256) return -1;
        total += objs[i].len;
        for (int j = 0; j < i; j++)
            if (!strcmp(objs[i].name, objs[j].name)) return -1; // ContentNames must be unique
    }
    qsort(ord, (size_t)n, sizeof ord[0], cmp_name);
    uint8_t *dir = malloc(DAB_MOTDIR_MAX_DIR);
    if (!dir) return -1;
    // TransportIds: an object that is the same as before (name, header and body) keeps its id, a new or changed one gets a
    // new id; the directory always gets a new one (EN 301 234 7.2.7.3/7.2.7.4)
    unsigned tid[DAB_MOTDIR_MAX_OBJ];
    uint32_t hash[DAB_MOTDIR_MAX_OBJ];
    int dp = 13;
    dir[dp++] = 0x00; // SortedHeaderInformation (PLI 00, id 0)
    const int ext_len = dp - 13;
    for (int i = 0; i < n; i++){
        uint8_t h[512];
        const int hl = mk_header(h, sizeof h, ord[i]);
        if (hl < 0 || dp + 2 + hl > DAB_MOTDIR_MAX_DIR){
            free(dir);
            return -1;
        }
        hash[i] = fnv(fnv(2166136261u, h, hl), ord[i]->body, ord[i]->len);
        tid[i] = 0;
        for (int j = 0; j < c->nobj; j++)
            if (c->obj_hash[j] == hash[i] && !strcmp(c->obj_name[j], ord[i]->name)) tid[i] = c->obj_tid[j];
        if (!tid[i]) tid[i] = new_tid(c);
        dir[dp++] = (uint8_t)(tid[i] >> 8);
        dir[dp++] = (uint8_t)tid[i];
        memcpy(dir + dp, h, (size_t)hl);
        dp += hl;
    }
    // core: CF 0, Rfu 0, DirectorySize (30), NumberOfObjects (16), DataCarouselPeriod (24, 0 = undefined),
    // Rfu (1) Rfa (2) SegmentSize (13), DirectoryExtensionLength (16)
    dir[0] = (uint8_t)((dp >> 24) & 0x3F);
    dir[1] = (uint8_t)(dp >> 16);
    dir[2] = (uint8_t)(dp >> 8);
    dir[3] = (uint8_t)dp;
    dir[4] = (uint8_t)(n >> 8);
    dir[5] = (uint8_t)n;
    dir[6] = dir[7] = dir[8] = 0;
    dir[9] = (uint8_t)(SEG_MAX >> 8);
    dir[10] = (uint8_t)SEG_MAX;
    dir[11] = (uint8_t)(ext_len >> 8);
    dir[12] = (uint8_t)ext_len;
    // the data groups of everything
    const int maxdg = (dp + total) / SEG_MAX + n + 2, cap = dp + total + 13 * maxdg + 16;
    uint8_t *buf = malloc((size_t)cap);
    int *off = malloc(sizeof(int) * (size_t)maxdg), *len = malloc(sizeof(int) * (size_t)maxdg);
    if (!buf || !off || !len){
        free(dir);
        free(buf);
        free(off);
        free(len);
        return -1;
    }
    free(c->buf);
    free(c->dg_off);
    free(c->dg_len);
    c->buf = buf;
    c->dg_off = off;
    c->dg_len = len;
    c->cap = cap;
    c->max_dg = maxdg;
    c->used = 0;
    c->ndg = 0;
    int bad = add_entity(c, 0, 6, new_tid(c), dir, dp);
    for (int i = 0; i < n && !bad; i++) bad = add_entity(c, i + 1, 4, tid[i], ord[i]->body, ord[i]->len);
    free(dir);
    if (bad){
        dab_motdir_free(c);
        return -1;
    }
    // the carousel order: the directory, then the SI object(s) (ContentType 7/0) right after it (TS 102 371 6.6), then the
    // rest in name order
    c->nent = n + 1;
    c->nfirst = 0;
    for (int i = 0; i < n; i++)
        if (ord[i]->ct == 7 && ord[i]->cst == 0) c->first[c->nfirst++] = i + 1;
    c->nobj = n;
    c->dir_size = dp;
    c->bytes = c->used;
    for (int i = 0; i < n; i++){
        c->obj_tid[i] = tid[i];
        c->obj_hash[i] = hash[i];
        snprintf(c->obj_name[i], sizeof c->obj_name[i], "%s", ord[i]->name);
        c->obj_si[i + 1] = ord[i]->ct == 7 && ord[i]->cst == 0;
    }
    // a change starts a new turn at once, with the directory (7.2.7.6)
    c->cur_dg = -1;
    c->pos = 0;
    memset(c->last_dg, 0, sizeof c->last_dg);
    c->nplan = 0;
    c->plan_i = 0;
    c->next_cycle_ms = c->t_ms;
    c->last_dir_ms = -1;
    return 0;
}

void dab_motdir_tick(dab_motdir *c, int ms) { c->t_ms += ms; }

// plan a turn: directory, SI, the other objects
static void plan_cycle(dab_motdir *c){
    int k = 0;
    c->plan[k++] = 0;
    for (int i = 0; i < c->nfirst; i++) c->plan[k++] = c->first[i];
    for (int e = 1; e < c->nent; e++)
        if (!c->obj_si[e]) c->plan[k++] = e;
    c->nplan = k;
    c->plan_i = 0;
}

// the next entity to send, or -1: a new turn not before DAB_MOTDIR_MIN_CYCLE_MS after the start of the last; within a long
// turn the directory and the SI again once DAB_MOTDIR_DIR_MS have passed since the last directory
static int next_entity(dab_motdir *c){
    if (c->nent <= 0) return -1;
    if (c->plan_i >= c->nplan){
        if (c->t_ms < c->next_cycle_ms) return -1;
        plan_cycle(c);
        c->next_cycle_ms = c->t_ms + DAB_MOTDIR_MIN_CYCLE_MS;
        c->cycles++;
    } else if (c->last_dir_ms >= 0 && c->t_ms - c->last_dir_ms >= DAB_MOTDIR_DIR_MS && c->plan[c->plan_i] != 0){
        // insert directory + SI before the rest of this turn
        int ins[1 + DAB_MOTDIR_MAX_OBJ], k = 0;
        ins[k++] = 0;
        for (int i = 0; i < c->nfirst; i++) ins[k++] = c->first[i];
        const int rest = c->nplan - c->plan_i;
        if (k + rest <= (int)(sizeof c->plan / sizeof c->plan[0])){
            memmove(c->plan + k, c->plan + c->plan_i, sizeof(int) * (size_t)rest);
            memcpy(c->plan, ins, sizeof(int) * (size_t)k);
            c->nplan = k + rest;
            c->plan_i = 0;
        }
    }
    const int e = c->plan[c->plan_i++];
    if (e == 0) c->last_dir_ms = c->t_ms;
    return e;
}

int dab_motdir_pending(dab_motdir *c){
    if (c->nent <= 0) return 0;
    if (c->cur_dg >= 0) return 1;
    if (c->plan_i < c->nplan) return 1;
    return c->t_ms >= c->next_cycle_ms;
}

int dab_motdir_next(dab_motdir *c, int chunk, dab_xpad_sub *s){
    if (c->nent <= 0 || chunk < 4) return 0;
    if (chunk > 48) chunk = 48;
    if (c->cur_dg < 0){ // start the next entity
        const int e = next_entity(c);
        if (e < 0) return 0;
        c->cur_ent = e;
        c->cur_dg = c->ent[e].dg0;
        c->pos = 0;
    }
    if (c->pos == 0){
        // the continuity index of this data group type (EN 300 401 5.3.3.1): +1 for every data group that differs from the
        // previous one of its type; the header CRC is redone
        const int t = c->ent[c->cur_ent].type & 7;
        if (c->last_dg[t] != c->cur_dg + 1) c->ci[t] = (c->ci[t] + 1) & 15;
        c->last_dg[t] = c->cur_dg + 1;
        uint8_t *dg = c->buf + c->dg_off[c->cur_dg];
        const int dl = c->dg_len[c->cur_dg];
        dg[1] = (uint8_t)(c->ci[t] << 4);
        uint16_t crc = dab_crc16(dg, (size_t)(dl - 2));
        dg[dl - 2] = (uint8_t)(crc >> 8);
        dg[dl - 1] = (uint8_t)crc;
    }
    const int k = xpad_piece(c->buf + c->dg_off[c->cur_dg], c->dg_len[c->cur_dg], &c->pos, c->dgli, c->app, chunk, s);
    if (c->pos >= c->dg_len[c->cur_dg]){
        c->sent += c->dg_len[c->cur_dg];
        c->pos = 0;
        c->cur_dg++;
        if (c->cur_dg >= c->ent[c->cur_ent].dg0 + c->ent[c->cur_ent].ndg) c->cur_dg = -1;
    }
    return k;
}
