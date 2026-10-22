// SPDX-License-Identifier: Apache-2.0
#include "dab_pad.h"
#include "dab_crc.h"
#include <string.h>

static const int XPAD_LEN[8] = { 4, 6, 8, 12, 16, 24, 32, 48 };   // variable-size X-PAD subfield lengths

void dab_dls_set(dab_dls *d, const char *text, int charset){
    int n = (int)strlen(text); if (n > DAB_DLS_MAX) n = DAB_DLS_MAX;
    if (n == d->len && charset == d->charset && memcmp(text, d->text, (size_t)n) == 0) return;
    memcpy(d->text, text, (size_t)n); d->text[n] = 0; d->len = n; d->charset = charset;
    d->toggle ^= 1; d->seg = 0;
}

int dab_dls_next_pad(dab_dls *d, uint8_t *pad, int max){
    if (d->len <= 0) return 0;
    const int nseg = (d->len + 15) / 16, s = d->seg, off = s * 16;
    const int nch = d->len - off < 16 ? d->len - off : 16;
    // the DL data group: 2 prefix bytes, the characters, CRC
    uint8_t dg[20]; int p = 0;
    dg[p++] = (uint8_t)((d->toggle << 7) | ((s == 0) << 6) | ((s == nseg - 1) << 5) | (nch - 1));   // C flag 0: a segment
    dg[p++] = s == 0 ? (uint8_t)(d->charset << 4) : (uint8_t)(s << 4);
    memcpy(dg + p, d->text + off, (size_t)nch); p += nch;
    uint16_t crc = dab_crc16(dg, (size_t)p);
    dg[p++] = (uint8_t)(crc >> 8); dg[p++] = (uint8_t)crc;
    // one subfield (application type 2 = DL segment start) of the smallest size that holds the data group
    int li = 0; while (XPAD_LEN[li] < p) li++;
    const int sub = XPAD_LEN[li];
    // X-PAD in transmission order: CI, end marker CI, subfield (padded with zeros)
    uint8_t x[2 + 48]; int xl = 0;
    x[xl++] = (uint8_t)((li << 5) | 2);
    x[xl++] = 0x00;                                                  // end marker: fewer than 4 CIs
    memcpy(x + xl, dg, (size_t)p); memset(x + xl + p, 0, (size_t)(sub - p)); xl += sub;
    if (xl + 2 > max) return 0;
    for (int i = 0; i < xl; i++) pad[i] = x[xl - 1 - i];            // DAB+: X-PAD bytes in reverse order
    pad[xl] = 0x20;                                                  // F-PAD: type 00, X-PAD ind 10 (variable size)
    pad[xl + 1] = 0x02;                                              // CI flag: the X-PAD starts with a CI list
    d->seg = (s + 1) % nseg;
    return xl + 2;
}
