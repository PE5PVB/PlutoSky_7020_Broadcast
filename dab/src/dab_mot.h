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

#endif
