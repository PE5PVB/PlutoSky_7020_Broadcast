// SPDX-License-Identifier: Apache-2.0
// Programme-associated data for DAB+ (ETSI EN 300 401, 7.4; ETSI TS 102 563, 5.4): Dynamic Label (DLS) segments and other
// X-PAD data subfields in variable-size X-PAD. The PAD bytes go into a data stream element at the start of an access unit.
#ifndef DAB_PAD_H
#define DAB_PAD_H

#include <stdint.h>

#define DAB_DLS_MAX 128

typedef struct {
    char    text[DAB_DLS_MAX + 1];
    int     len, charset;                  // charset: 0 = EBU Latin, 15 = UTF-8
    int     toggle, seg;                   // toggle bit (flips on a new label), next segment to send
} dab_dls;

// one X-PAD data subfield: application type (1 = data group length indicator, 2 = DL start, 12/13 = MOT start/continuation)
// and its data (up to 48 bytes; it is padded with zeros to the next subfield length 4, 6, 8, 12, 16, 24, 32 or 48)
typedef struct { int app; const uint8_t *data; int len; } dab_xpad_sub;

// PAD bytes for one access unit from 1..4 subfields: X-PAD (CI list and subfields, byte-reversed) followed by the 2 F-PAD
// bytes. Returns the number of bytes, or 0 when nothing to send or it does not fit in max.
int  dab_xpad_build(const dab_xpad_sub *s, int n, uint8_t *pad, int max);

// set the label (text up to 128 bytes in the given charset); a changed text flips the toggle bit and restarts
void dab_dls_set(dab_dls *d, const char *text, int charset);
// the next DL data group (a segment of up to 16 characters, at most 20 bytes) into dg; returns its length, 0 when no label
int  dab_dls_next_dg(dab_dls *d, uint8_t *dg);
// PAD bytes for the next access unit carrying a DLS segment only. Returns the number of bytes (0 when no label is set).
int  dab_dls_next_pad(dab_dls *d, uint8_t *pad, int max);

#endif
