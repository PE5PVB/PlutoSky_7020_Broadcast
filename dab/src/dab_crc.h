// SPDX-License-Identifier: Apache-2.0
// CRCs used by DAB / DAB+ (ETSI EN 300 401, ETSI TS 102 563).
#ifndef DAB_CRC_H
#define DAB_CRC_H

#include <stdint.h>
#include <stddef.h>

// CRC-16-CCITT x^16+x^12+x^5+1, preset 0xFFFF, result inverted (DAB+ access-unit CRC, FIB CRC).
uint16_t dab_crc16(const uint8_t *data, size_t len);

// DAB+ fire code x^16+x^14+x^13+x^12+x^11+x^5+x^3+x^2+x+1 = (x^11+1)(x^5+x^3+x^2+x+1), preset 0, not inverted, over the 9 bytes that
// follow the fire code in the audio super frame.
uint16_t dab_firecode(const uint8_t *data9);

#endif
