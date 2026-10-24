// SPDX-License-Identifier: Apache-2.0
// The fast MDCT (DCT-IV over a mixed-radix FFT) against the direct definition, for M = 960 and 120.
#include "../src/aac_mdct.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int check(int M){
    mdct_plan p;
    if (mdct_init(&p, M)){
        printf("init %d failed\n", M);
        return 1;
    }
    float *x = malloc(sizeof(float) * 2 * M), *X = malloc(sizeof(float) * M);
    srand(7);
    for (int n = 0; n < 2 * M; n++) x[n] = (float)rand() / RAND_MAX * 2.0f - 1.0f;
    mdct_run(&p, x, X);
    double maxerr = 0, maxv = 0;
    for (int k = 0; k < M; k++){
        double s = 0;
        for (int n = 0; n < 2 * M; n++) s += x[n] * cos(M_PI / M * (n + 0.5 + M / 2.0) * (k + 0.5));
        double e = fabs(s - X[k]);
        if (e > maxerr) maxerr = e;
        if (fabs(s) > maxv) maxv = fabs(s);
    }
    printf("MDCT M=%d: max error %.2e (relative %.2e)\n", M, maxerr, maxerr / maxv);
    mdct_free(&p);
    free(x);
    free(X);
    return maxerr / maxv > 1e-5;
}

int main(void){
    int f = check(960) + check(120);
    printf("%s\n", f ? "FAIL" : "PASS");
    return f;
}
