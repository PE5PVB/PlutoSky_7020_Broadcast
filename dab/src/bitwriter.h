// SPDX-License-Identifier: Apache-2.0
// MSB-first bit writer into a caller-owned byte buffer.
#ifndef DAB_BITWRITER_H
#define DAB_BITWRITER_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    uint8_t *buf;
    size_t   cap;      // bytes
    size_t   pos;      // bits written
    int      overflow; // set when a write did not fit
} bitwriter;

static inline void bw_init(bitwriter *w, uint8_t *buf, size_t cap){ w->buf = buf; w->cap = cap; w->pos = 0; w->overflow = 0; memset(buf, 0, cap); }

// write the n (0..32) least significant bits of v, most significant first
static inline void bw_put(bitwriter *w, uint32_t v, int n){
    for (int i = n - 1; i >= 0; i--){
        size_t byte = w->pos >> 3;
        if (byte >= w->cap){ w->overflow = 1; return; }
        if ((v >> i) & 1u) w->buf[byte] |= (uint8_t)(0x80u >> (w->pos & 7));
        w->pos++;
    }
}

static inline void bw_align(bitwriter *w){ while (w->pos & 7) bw_put(w, 0, 1); }
static inline size_t bw_bits(const bitwriter *w){ return w->pos; }
static inline size_t bw_bytes(const bitwriter *w){ return (w->pos + 7) >> 3; }

#endif
