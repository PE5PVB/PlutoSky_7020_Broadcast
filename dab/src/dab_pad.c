// SPDX-License-Identifier: Apache-2.0
#include "dab_pad.h"
#include "dab_crc.h"
#include <string.h>

static const int XPAD_LEN[8] = { 4, 6, 8, 12, 16, 24, 32, 48 };   // variable-size X-PAD subfield lengths

int dab_xpad_build(const dab_xpad_sub *s, int n, uint8_t *pad, int max){
    if (n < 1 || n > 4) return 0;
    uint8_t x[4 + 4 * 48]; int xl = 0, li[4];
    for (int i = 0; i < n; i++){
        if (s[i].len < 1 || s[i].len > 48) return 0;
        li[i] = 0; while (XPAD_LEN[li[i]] < s[i].len) li[i]++;
        x[xl++] = (uint8_t)((li[i] << 5) | (s[i].app & 0x1F));
    }
    if (n < 4) x[xl++] = 0x00;                                       // end marker: fewer than 4 CIs
    // the subfields in the order of the CIs, each padded with zeros
    for (int i = 0; i < n; i++){
        const int sub = XPAD_LEN[li[i]];
        memcpy(x + xl, s[i].data, (size_t)s[i].len); memset(x + xl + s[i].len, 0, (size_t)(sub - s[i].len)); xl += sub;
    }
    if (xl + 2 > max) return 0;
    for (int i = 0; i < xl; i++) pad[i] = x[xl - 1 - i];            // DAB+: X-PAD bytes in reverse order
    pad[xl] = 0x20;                                                  // F-PAD: type 00, X-PAD ind 10 (variable size)
    pad[xl + 1] = 0x02;                                              // CI flag: the X-PAD starts with a CI list
    return xl + 2;
}

void dab_dls_set(dab_dls *d, const char *text, int charset){
    int n = (int)strlen(text); if (n > DAB_DLS_MAX) n = DAB_DLS_MAX;
    if (n == d->len && charset == d->charset && memcmp(text, d->text, (size_t)n) == 0) return;
    memcpy(d->text, text, (size_t)n); d->text[n] = 0; d->len = n; d->charset = charset;
    d->toggle ^= 1; d->seg = 0;
}

int dab_dls_next_dg(dab_dls *d, uint8_t *dg){
    if (d->len <= 0) return 0;
    const int nseg = (d->len + 15) / 16, s = d->seg, off = s * 16;
    const int nch = d->len - off < 16 ? d->len - off : 16;
    // the DL data group: 2 prefix bytes, the characters, CRC
    int p = 0;
    dg[p++] = (uint8_t)((d->toggle << 7) | ((s == 0) << 6) | ((s == nseg - 1) << 5) | (nch - 1));   // C flag 0: a segment
    dg[p++] = s == 0 ? (uint8_t)(d->charset << 4) : (uint8_t)(s << 4);
    memcpy(dg + p, d->text + off, (size_t)nch); p += nch;
    uint16_t crc = dab_crc16(dg, (size_t)p);
    dg[p++] = (uint8_t)(crc >> 8); dg[p++] = (uint8_t)crc;
    d->seg = (s + 1) % nseg;
    return p;
}

int dab_dls_next_pad(dab_dls *d, uint8_t *pad, int max){
    uint8_t dg[20]; int p = dab_dls_next_dg(d, dg);
    if (p <= 0) return 0;
    dab_xpad_sub s = { 2, dg, p };                                   // application type 2 = DL segment start
    return dab_xpad_build(&s, 1, pad, max);
}
