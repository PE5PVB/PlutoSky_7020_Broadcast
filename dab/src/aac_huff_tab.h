// SPDX-License-Identifier: Apache-2.0
// AAC Huffman code books (ISO/IEC 14496-3 Annex 4.A). Data in aac_huff_tab.c (generated, see tools/gen_aac_huff.py).
//
// Index of a code word in book n:
//   books 1, 2   (signed,   4-tuples, |v| <= 1):  27(w+1) + 9(x+1) + 3(y+1) + (z+1)
//   books 3, 4   (unsigned, 4-tuples, |v| <= 2):  27|w| + 9|x| + 3|y| + |z|          + sign bits
//   books 5, 6   (signed,   pairs,    |v| <= 4):  9(y+4) + (z+4)
//   books 7, 8   (unsigned, pairs,    |v| <= 7):  8|y| + |z|                         + sign bits
//   books 9, 10  (unsigned, pairs,    |v| <= 12): 13|y| + |z|                        + sign bits
//   book 11      (unsigned, pairs,    |v| <= 16, 16 = escape): 17|y| + |z|           + sign bits + escapes
// Scale factor book: index = delta + 60 (delta -60..60).
#ifndef AAC_HUFF_TAB_H
#define AAC_HUFF_TAB_H

#include <stdint.h>

extern const uint8_t  *const aac_hcb_len[12];    // [1..11]
extern const uint16_t *const aac_hcb_code[12];   // [1..11]
extern const uint8_t  aac_scf_len[121];
extern const uint32_t aac_scf_code[121];

#endif
