// SPDX-License-Identifier: Apache-2.0
// AAC-LC encoder for DAB+: frame length 960, block switching (long and eight short windows), mono (SCE) or stereo (CPE,
// common window, M/S per band), perceptual noise substitution.
// Output: raw_data_block()s (no ADTS/LATM), as carried in DAB+ access units.
#ifndef AAC_ENC_H
#define AAC_ENC_H

#include <stdint.h>

#define AAC_FRAME 960

typedef struct aacenc aacenc;

// sample_rate 32000 or 48000; channels 1 or 2; bandwidth_hz = audio bandwidth (lines above it are not coded)
aacenc *aacenc_open(int sample_rate, int channels, int bandwidth_hz);
void    aacenc_close(aacenc *e);

// Encode one frame: pcm = AAC_FRAME samples per channel, interleaved, full scale +-1.0.
// The rate control aims at target_bytes and never exceeds max_bytes. With pad_to > 0 the block is padded with fill
// elements to exactly pad_to bytes (pad_to <= max_bytes). Returns the number of bytes written to out, or -1.
// The output lags the input by two frames (MDCT overlap plus one frame of look-ahead for the block switching).
int aacenc_frame(aacenc *e, const float *pcm, int target_bytes, int max_bytes, int pad_to, uint8_t *out);

// data stream element (DAB+ PAD) to put at the start of the next frame's raw_data_block; 0..510 bytes
int aacenc_set_dse(aacenc *e, const uint8_t *data, int len);

// AudioSpecificConfig (AAC-LC, frameLengthFlag = 1) for decoders that need it (tests); returns its length (2)
int aacenc_asc(const aacenc *e, uint8_t asc[2]);

#endif
