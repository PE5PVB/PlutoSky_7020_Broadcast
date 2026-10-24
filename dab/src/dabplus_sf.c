// SPDX-License-Identifier: Apache-2.0
#include "dabplus_sf.h"
#include "dab_crc.h"
#include "dab_rs.h"
#include "bitwriter.h"
#include <string.h>

int dabplus_num_aus(const dabplus_cfg *c){
    if (c->sbr) return c->dac_rate ? 3 : 2;
    return c->dac_rate ? 6 : 4;
}

int dabplus_header_bytes(const dabplus_cfg *c){
    int n = dabplus_num_aus(c), bits = 16 + 8 + 12 * (n - 1);
    if (bits & 7) bits += 4; // padding nibble when the AU start list ends mid-byte
    return bits / 8;
}

int dabplus_audio_bytes(const dabplus_cfg *c) { return 110 * (c->bitrate_kbps / 8); }
int dabplus_total_bytes(const dabplus_cfg *c) { return 120 * (c->bitrate_kbps / 8); }
int dabplus_payload_budget(const dabplus_cfg *c){
    return dabplus_audio_bytes(c) - dabplus_header_bytes(c) - 2 * dabplus_num_aus(c);
}

int dabplus_build(const dabplus_cfg *c, const uint8_t *const au[], const int len[], uint8_t *out){
    if (c->bitrate_kbps < 8 || c->bitrate_kbps > 192 || (c->bitrate_kbps % 8)) return -1;
    const int n = dabplus_num_aus(c), s = c->bitrate_kbps / 8, hdr = dabplus_header_bytes(c), abytes = dabplus_audio_bytes(c);
    int sum = 0;
    for (int i = 0; i < n; i++){
        if (len[i] < 1) return -1;
        sum += len[i];
    }
    if (sum != dabplus_payload_budget(c)) return -1;

    memset(out, 0, (size_t)dabplus_total_bytes(c));
    // AU positions: AU i starts at start[i] and ends with its CRC
    int start[6], pos = hdr;
    for (int i = 0; i < n; i++){
        start[i] = pos;
        pos += len[i] + 2;
    }
    if (pos != abytes || start[n - 1] > 0xFFF) return -1;

    bitwriter w;
    bw_init(&w, out, (size_t)hdr);
    bw_put(&w, 0, 16); // fire code, filled in below
    bw_put(&w, 0, 1);  // rfa
    bw_put(&w, (uint32_t)c->dac_rate, 1);
    bw_put(&w, (uint32_t)c->sbr, 1);
    bw_put(&w, (uint32_t)c->stereo, 1);
    bw_put(&w, (uint32_t)c->ps, 1);
    bw_put(&w, 0, 3); // mpeg_surround_config: none
    for (int i = 1; i < n; i++) bw_put(&w, (uint32_t)start[i], 12);
    bw_align(&w);
    if (w.overflow || (int)bw_bytes(&w) != hdr) return -1;

    for (int i = 0; i < n; i++){
        memcpy(out + start[i], au[i], (size_t)len[i]);
        uint16_t crc = dab_crc16(out + start[i], (size_t)len[i]);
        out[start[i] + len[i]] = (uint8_t)(crc >> 8);
        out[start[i] + len[i] + 1] = (uint8_t)crc;
    }
    uint16_t fc = dab_firecode(out + 2);
    out[0] = (uint8_t)(fc >> 8);
    out[1] = (uint8_t)fc;

    // RS(120,110) with virtual interleaving: code word r takes bytes r, r+s, r+2s, ... (s rows, 110 data columns)
    uint8_t col[110], par[10];
    for (int r = 0; r < s; r++){
        for (int k = 0; k < 110; k++) col[k] = out[s * k + r];
        dab_rs_encode(col, par);
        for (int k = 0; k < 10; k++) out[s * (110 + k) + r] = par[k];
    }
    return 0;
}
