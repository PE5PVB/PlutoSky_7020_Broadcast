// SPDX-License-Identifier: Apache-2.0
// Reed-Solomon RS(120,110), shortened from RS(255,245) over GF(2^8), field polynomial x^8+x^4+x^3+x^2+1,
// generator roots alpha^0 .. alpha^9 (ETSI TS 102 563, 6.1).
#ifndef DAB_RS_H
#define DAB_RS_H

#include <stdint.h>

// parity[10] for data[110] (systematic: the code word is data followed by parity)
void dab_rs_encode(const uint8_t data[110], uint8_t parity[10]);

#endif
