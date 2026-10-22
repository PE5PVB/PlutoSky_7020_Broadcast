// SPDX-License-Identifier: Apache-2.0
// Programme-associated data for DAB+ (ETSI EN 300 401, 7.4; ETSI TS 102 563, 5.4): Dynamic Label (DLS) segments carried
// in variable-size X-PAD. The PAD bytes go into a data stream element at the start of an access unit.
#ifndef DAB_PAD_H
#define DAB_PAD_H

#include <stdint.h>

#define DAB_DLS_MAX 128

typedef struct {
    char    text[DAB_DLS_MAX + 1];
    int     len, charset;                  // charset: 0 = EBU Latin, 15 = UTF-8
    int     toggle, seg;                   // toggle bit (flips on a new label), next segment to send
} dab_dls;

// set the label (text up to 128 bytes in the given charset); a changed text flips the toggle bit and restarts
void dab_dls_set(dab_dls *d, const char *text, int charset);
// PAD bytes for the next access unit carrying a DLS segment: X-PAD (byte-reversed) followed by the 2 F-PAD bytes.
// Returns the number of bytes (0 when no label is set).
int  dab_dls_next_pad(dab_dls *d, uint8_t *pad, int max);

#endif
