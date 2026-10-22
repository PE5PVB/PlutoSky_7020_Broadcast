// SPDX-License-Identifier: Apache-2.0
#include "dab_rs.h"

#define NPAR 10

static uint8_t gf_exp[512], gf_log[256], gen[NPAR + 1];   // gen[0] = x^10 coefficient (1) ... gen[10] = constant term
static int rs_ready = 0;

static uint8_t gf_mul(uint8_t a, uint8_t b){ return (a && b) ? gf_exp[gf_log[a] + gf_log[b]] : 0; }

static void rs_init(void){
    int x = 1;
    for (int i = 0; i < 255; i++){ gf_exp[i] = (uint8_t)x; gf_log[x] = (uint8_t)i; x <<= 1; if (x & 0x100) x ^= 0x11D; }
    for (int i = 255; i < 512; i++) gf_exp[i] = gf_exp[i - 255];
    // g(x) = prod_{i=0}^{9} (x - alpha^i), highest power first
    uint8_t g[NPAR + 1] = {1};
    int deg = 0;
    for (int i = 0; i < NPAR; i++){
        uint8_t r = gf_exp[i];
        g[deg + 1] = 0;
        for (int j = deg + 1; j > 0; j--) g[j] ^= gf_mul(g[j - 1], r);   // multiply by (x + r): new[j] = old[j] + r*old[j-1]
        deg++;
    }
    for (int i = 0; i <= NPAR; i++) gen[i] = g[i];
    rs_ready = 1;
}

void dab_rs_encode(const uint8_t data[110], uint8_t parity[10]){
    if (!rs_ready) rs_init();
    uint8_t reg[NPAR] = {0};                         // remainder of data(x) * x^10 mod g(x), reg[0] = highest
    for (int i = 0; i < 110; i++){
        uint8_t fb = data[i] ^ reg[0];
        for (int j = 0; j < NPAR - 1; j++) reg[j] = reg[j + 1] ^ gf_mul(fb, gen[j + 1]);
        reg[NPAR - 1] = gf_mul(fb, gen[NPAR]);
    }
    for (int j = 0; j < NPAR; j++) parity[j] = reg[j];
}
