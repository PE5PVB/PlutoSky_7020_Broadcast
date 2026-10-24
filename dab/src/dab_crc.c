// SPDX-License-Identifier: Apache-2.0
#include "dab_crc.h"

// bitwise MSB-first CRC-16 with a given polynomial (without the x^16 term) and preset
static uint16_t crc16_msb(const uint8_t *data, size_t len, uint16_t poly, uint16_t crc){
    for (size_t i = 0; i < len; i++){
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ poly) : (uint16_t)(crc << 1);
    }
    return crc;
}

uint16_t dab_crc16(const uint8_t *data, size_t len) { return (uint16_t)~crc16_msb(data, len, 0x1021u, 0xFFFFu); }

uint16_t dab_firecode(const uint8_t *data9) { return crc16_msb(data9, 9, 0x782Fu, 0x0000u); }
