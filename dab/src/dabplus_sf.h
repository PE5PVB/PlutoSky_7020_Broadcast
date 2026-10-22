// SPDX-License-Identifier: Apache-2.0
// DAB+ audio super frame (ETSI TS 102 563): header with fire code, access units with CRC, Reed-Solomon outer code.
#ifndef DABPLUS_SF_H
#define DABPLUS_SF_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    int bitrate_kbps;     // sub-channel bitrate, multiple of 8 (8..192)
    int dac_rate;         // 1 = 48 kHz output rate, 0 = 32 kHz
    int sbr;              // SBR used (HE-AAC)
    int ps;               // parametric stereo (HE-AAC v2)
    int stereo;           // aac_channel_mode: 1 = stereo
} dabplus_cfg;

// number of access units per super frame (120 ms): 6 (48 kHz), 4 (32 kHz), 3 (48 kHz SBR), 2 (32 kHz SBR)
int dabplus_num_aus(const dabplus_cfg *c);
// header bytes before the first AU (fire code, flags, AU start addresses, padding)
int dabplus_header_bytes(const dabplus_cfg *c);
// bytes of the audio super frame without RS parity (110 * bitrate/8)
int dabplus_audio_bytes(const dabplus_cfg *c);
// total bytes of the super frame including RS parity (120 * bitrate/8) = 5 logical frames of 24 * bitrate/8 bytes
int dabplus_total_bytes(const dabplus_cfg *c);
// bytes available for the AU payloads together (excluding each AU's 2-byte CRC)
int dabplus_payload_budget(const dabplus_cfg *c);

// Build one super frame. au[i] are raw_data_block payloads (without CRC), len[i] their sizes. The payloads must fill the
// budget exactly: sum(len) == dabplus_payload_budget() (the last AU is padded by the AAC encoder with a fill element).
// out must hold dabplus_total_bytes(). Returns 0, or -1 on a size mismatch / bad configuration.
int dabplus_build(const dabplus_cfg *c, const uint8_t *const au[], const int len[], uint8_t *out);

#endif
