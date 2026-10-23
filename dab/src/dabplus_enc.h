// SPDX-License-Identifier: Apache-2.0
// DAB+ audio encoder: PCM in, complete audio super frames (120 ms, with RS parity) out.
#ifndef DABPLUS_ENC_H
#define DABPLUS_ENC_H

#include "dabplus_sf.h"
#include <stdint.h>

typedef struct dabplus_enc dabplus_enc;

// cfg->sbr and cfg->ps must be 0 (AAC-LC); bandwidth_hz <= 0 picks a default for the bitrate
dabplus_enc *dabplus_enc_open(const dabplus_cfg *cfg, int bandwidth_hz);
void         dabplus_enc_close(dabplus_enc *e);
// another bitrate from the next super frame on (a fresh AAC encoder: the first frame after it starts from silence). 0 = ok.
int          dabplus_enc_set_bitrate(dabplus_enc *e, int kbps);
int          dabplus_enc_frames(const dabplus_enc *e);           // PCM frames of 960 samples per super frame (4 or 6)
// pcm: dabplus_enc_frames() * 960 samples per channel, interleaved, +-1.0. out: dabplus_total_bytes() bytes. 0 = ok.
int          dabplus_enc_superframe(dabplus_enc *e, const float *pcm, uint8_t *out);
// the access units of the last super frame (for tests)
int          dabplus_enc_last_aus(const dabplus_enc *e, const uint8_t **au, int *len);
int          dabplus_enc_asc(const dabplus_enc *e, uint8_t asc[2]);
// dynamic label text (DLS), up to 128 bytes; charset 0 = EBU Latin, 15 = UTF-8
void         dabplus_enc_set_dls(dabplus_enc *e, const char *text, int charset);

#endif
