// SPDX-License-Identifier: Apache-2.0
// Builds DAB+ super frames from pseudo-random access units for every configuration and writes them to
// <outdir>/sf_<rate>_<dac>_<sbr>.bin together with the AUs (<...>.aus: [u16 len][bytes]...). tests/check_sf.py verifies them.
#include "../src/dabplus_sf.h"
#include <stdio.h>
#include <stdlib.h>

static uint32_t rng = 12345;
static uint8_t rnd8(void){ rng = rng * 1103515245u + 12345u; return (uint8_t)(rng >> 16); }

int main(int argc, char **argv){
    const char *dir = argc > 1 ? argv[1] : ".";
    const int rates[] = { 8, 48, 72, 96, 128, 192 };
    int fails = 0;
    for (unsigned ri = 0; ri < sizeof rates / sizeof rates[0]; ri++)
        for (int dac = 0; dac <= 1; dac++)
            for (int sbr = 0; sbr <= 1; sbr++){
                dabplus_cfg c = { rates[ri], dac, sbr, 0, 1 };
                int n = dabplus_num_aus(&c), budget = dabplus_payload_budget(&c);
                if (budget < n){ continue; }
                static uint8_t aubuf[6][4096]; const uint8_t *au[6]; int len[6], left = budget;
                for (int i = 0; i < n; i++){
                    len[i] = (i == n - 1) ? left : 1 + (int)(rnd8() % (unsigned)(left / (n - i)));
                    left -= len[i];
                    for (int k = 0; k < len[i]; k++) aubuf[i][k] = rnd8();
                    au[i] = aubuf[i];
                }
                static uint8_t sf[120 * 24];
                if (dabplus_build(&c, au, len, sf) != 0){ printf("build failed: %d kbit/s dac=%d sbr=%d\n", rates[ri], dac, sbr); fails++; continue; }
                char fn[512]; snprintf(fn, sizeof fn, "%s/sf_%d_%d_%d.bin", dir, rates[ri], dac, sbr);
                FILE *f = fopen(fn, "wb"); fwrite(sf, 1, (size_t)dabplus_total_bytes(&c), f); fclose(f);
                snprintf(fn, sizeof fn, "%s/sf_%d_%d_%d.aus", dir, rates[ri], dac, sbr);
                f = fopen(fn, "wb");
                for (int i = 0; i < n; i++){ uint8_t l[2] = { (uint8_t)(len[i] >> 8), (uint8_t)len[i] }; fwrite(l, 1, 2, f); fwrite(au[i], 1, (size_t)len[i], f); }
                fclose(f);
            }
    // a size mismatch must be refused
    { dabplus_cfg c = { 96, 1, 0, 0, 1 }; static uint8_t d[4096]; const uint8_t *au[6] = { d, d, d, d, d, d }; int len[6] = { 10, 10, 10, 10, 10, 10 }; static uint8_t sf[120 * 24];
      if (dabplus_build(&c, au, len, sf) == 0){ printf("size mismatch not refused\n"); fails++; } }
    printf("%s\n", fails ? "FAIL" : "built");
    return fails ? 1 : 0;
}
