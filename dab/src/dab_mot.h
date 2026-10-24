// SPDX-License-Identifier: Apache-2.0
// MOT slideshow in X-PAD (ETSI EN 301 234 MOT, ETSI TS 101 499 SlideShow, ETSI EN 300 401 5.3.3 and 7.4.5): one image at a
// time as a MOT object (header and body in MSC data groups of type 3 and 4), handed out in pieces as X-PAD data subfields:
// a data group length indicator (application type 1) and the first piece (type 12) together, then continuation pieces
// (type 13). The service has to announce the slideshow in FIG 0/13 (user application 0x002, X-PAD application type 12).
#ifndef DAB_MOT_H
#define DAB_MOT_H

#include "dab_pad.h"
#include <stdint.h>

#define DAB_MOT_MAX_BODY  (60 * 1024)      // the SlideShow profile allows 50 kB per image; a little room above that
#define DAB_MOT_JFIF      1                // MOT content subtypes of content type 2 (image)
#define DAB_MOT_PNG       3
#define DAB_MOT_MAX_DG    72               // the header and up to 71 body segments of 1013 bytes

typedef struct {
    uint8_t *buf; int cap;                 // the data groups of the object, one after the other
    int dg_off[DAB_MOT_MAX_DG], dg_len[DAB_MOT_MAX_DG], ndg;   // where each data group starts and its length
    int cur, pos;                          // data group being sent and the bytes of it already sent
    uint8_t dgli[4];                       // the length indicator for the data group being started
    unsigned ci_hdr, ci_body;              // continuity index per data group type (4 bits)
} dab_mot;

void dab_mot_init(dab_mot *m);
void dab_mot_free(dab_mot *m);
// queue an image (JPEG: DAB_MOT_JFIF, PNG: DAB_MOT_PNG) under a name and transport id; it is shown at once ("trigger now").
// Returns 0, or -1 when it is too large or out of memory.
int  dab_mot_load(dab_mot *m, const uint8_t *img, int len, int subtype, const char *name, unsigned tid);
// 1 while an object is still being sent
int  dab_mot_busy(const dab_mot *m);
// stop sending the current object
void dab_mot_abort(dab_mot *m);
// the subfields for one access unit, data pieces of at most 'chunk' bytes (a subfield length: 4..48): the length indicator
// and the start of a data group, or a continuation. Returns the number of subfields put into s (0, 1 or 2).
int  dab_mot_next(dab_mot *m, int chunk, dab_xpad_sub *s);

// ---- MOT directory mode (EN 301 234 7.2): a data carousel -----------------------------------------------------------------
// A set of objects described by one MOT directory (data group type 6, uncompressed, header information sorted by
// ContentName and signalled with SortedHeaderInformation), the bodies in data groups of type 4. Used for the SPI (EPG,
// ETSI TS 102 371) on its own X-PAD application type pair (app = start of a data group, app + 1 = continuation), next to
// the slideshow on 12/13. A turn of the carousel: the directory, the SI object(s) (ContentType 7/0), then the other objects;
// a new turn starts at most every DAB_MOTDIR_MIN_CYCLE_MS (time counted with dab_motdir_tick), and inside a longer turn the
// directory and the SI are repeated every DAB_MOTDIR_DIR_MS. A new set (dab_motdir_load) starts a new turn at once; an
// object whose name, header and body are unchanged keeps its TransportId, the directory always gets a new one.
#define DAB_MOTDIR_MAX_OBJ      24
#define DAB_MOTDIR_MAX_NAME     64
#define DAB_MOTDIR_MAX_DIR      8192       // TS 102 371 6.2: the MOT directory at most 8 kbytes
#define DAB_MOTDIR_MIN_CYCLE_MS 30000
#define DAB_MOTDIR_DIR_MS       60000

typedef struct {
    const char *name;                      // ContentName (UTF-8, unique, 1..64 bytes)
    int ct, cst;                           // ContentType / ContentSubType
    const uint8_t *param; int nparam;      // further header extension parameters, already encoded (e.g. ScopeStart)
    const uint8_t *body; int len;
} dab_motdir_obj;

typedef struct {
    int app;                               // X-PAD application type of a data group start
    uint8_t *buf; int cap, used;           // the data groups of the directory and all bodies
    int *dg_off, *dg_len, ndg, max_dg;
    struct { int dg0, ndg, type; unsigned tid; } ent[DAB_MOTDIR_MAX_OBJ + 1];   // 0 = directory, 1.. = bodies (name order)
    int nent, nobj, dir_size, bytes;
    int first[DAB_MOTDIR_MAX_OBJ], nfirst; // SI objects (sent right after the directory)
    int obj_si[DAB_MOTDIR_MAX_OBJ + 1];
    unsigned obj_tid[DAB_MOTDIR_MAX_OBJ];  // per object (name order): TransportId, hash of header + body, name
    uint32_t obj_hash[DAB_MOTDIR_MAX_OBJ];
    char obj_name[DAB_MOTDIR_MAX_OBJ][DAB_MOTDIR_MAX_NAME + 1];
    unsigned next_tid;
    int plan[2 * DAB_MOTDIR_MAX_OBJ + 4], nplan, plan_i;   // the entities of the current turn
    int cur_ent, cur_dg, pos;              // data group being sent (-1: none) and bytes of it sent
    uint8_t dgli[4];
    unsigned ci[8]; int last_dg[8];        // continuity index per data group type
    long long t_ms, next_cycle_ms, last_dir_ms;
    unsigned long cycles, sent;            // turns started, data group bytes sent
} dab_motdir;

void dab_motdir_init(dab_motdir *c, int app, unsigned tid_seed);
void dab_motdir_free(dab_motdir *c);       // empty again (keeps app and the TransportId counter)
// a new set of objects (copied); n = 0 empties the carousel. 0 = ok, -1: too many, names not unique, directory too large,
// out of memory (the carousel is then empty)
int  dab_motdir_load(dab_motdir *c, const dab_motdir_obj *objs, int n);
void dab_motdir_tick(dab_motdir *c, int ms);
// 1 when there is something to send now
int  dab_motdir_pending(dab_motdir *c);
// as dab_mot_next(), on the carousel's application types
int  dab_motdir_next(dab_motdir *c, int chunk, dab_xpad_sub *s);

#endif
