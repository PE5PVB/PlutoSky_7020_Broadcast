// SPDX-License-Identifier: Apache-2.0
// DAB+ audio encoder: PCM in, complete audio super frames (120 ms, with RS parity) out.
#ifndef DABPLUS_ENC_H
#define DABPLUS_ENC_H

#include "dabplus_sf.h"
#include "dab_mot.h"
#include <stdint.h>
#include <stdio.h>

typedef struct dabplus_enc dabplus_enc;

// cfg->sbr and cfg->ps must be 0 (AAC-LC); bandwidth_hz <= 0 picks a default for the bitrate
dabplus_enc *dabplus_enc_open(const dabplus_cfg *cfg, int bandwidth_hz);
// the same encoder interface for DAB (MPEG-1 Layer II, 48 kHz stereo, 112..192 kbit/s): a "super frame" is then 5 MP2 frames
// (120 ms, 15 x kbit/s bytes), with the same PAD (DLS, slideshow). dabplus_enc_last_aus() and dabplus_enc_asc() do not apply.
dabplus_enc *dabplus_enc_open_mp2(int kbps);
int          dabplus_enc_is_mp2(const dabplus_enc *e);
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
// slideshow: send one image (subtype DAB_MOT_JFIF or DAB_MOT_PNG) in the PAD, shown at once; busy while it is being sent;
// rate = slideshow bytes per second (data groups including their overhead)
int          dabplus_enc_set_slide(dabplus_enc *e, const uint8_t *img, int len, int subtype, const char *name, unsigned tid);
int          dabplus_enc_slide_busy(const dabplus_enc *e);
void         dabplus_enc_slide_abort(dabplus_enc *e);
int          dabplus_enc_slide_rate(const dabplus_enc *e);
// SPI (EPG) carousel in the PAD (owned by the caller, NULL = none): it gets every third MOT piece while it has something to
// send (see frame_pad() in dabplus_enc.c); its clock advances by 120 ms per super frame
void         dabplus_enc_set_spi(dabplus_enc *e, dab_motdir *c);
// test: write the PAD of every access unit (or MP2 frame) to f: a 2-byte length, big endian, then the PAD bytes
void         dabplus_enc_pad_dump(dabplus_enc *e, FILE *f);

#endif
