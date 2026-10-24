// SPDX-License-Identifier: Apache-2.0
// MPEG-1 Audio Layer II encoder (ISO/IEC 11172-3) for DAB (ETSI EN 300 401, 7.1 and 7.4): 48 kHz, stereo, 64..192 kbit/s,
// with the ISO CRC and the DAB end of frame: X-PAD, ScF-CRC and F-PAD. Written for this project; the analysis window is our own
// prototype design (a Kaiser-windowed sinc), matched to the standard synthesis filterbank by measurement.
#ifndef MP2_ENC_H
#define MP2_ENC_H

#include <stdint.h>

#define MP2_FRAME 1152                     // samples per channel per frame (24 ms at 48 kHz)

typedef struct mp2enc mp2enc;

// kbps: 64, 96, 112, 128, 160 or 192 (stereo at 48 kHz). NULL for anything else.
mp2enc *mp2enc_open(int kbps);
void    mp2enc_close(mp2enc *e);
int     mp2enc_valid_kbps(int kbps);
// Encode one frame: pcm = MP2_FRAME stereo samples, interleaved, full scale +-1.0. pad = the PAD as dab_xpad_build() makes it
// (X-PAD in transmission order followed by the 2 F-PAD bytes), padlen its length; 0 = no X-PAD (F-PAD 00 00).
// Writes exactly 3 * kbps bytes to out; returns that number, or -1.
int     mp2enc_frame(mp2enc *e, const float *pcm, const uint8_t *pad, int padlen, uint8_t *out);
// bytes at the end of a frame that are not audio (ScF-CRC + F-PAD), so that the caller can size the X-PAD
int     mp2enc_tail_bytes(const mp2enc *e);

#endif
