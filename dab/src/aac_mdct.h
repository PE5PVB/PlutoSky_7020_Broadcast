// SPDX-License-Identifier: Apache-2.0
// MDCT for AAC with frame length 960 (and 120 for short windows), via a DCT-IV computed with a mixed-radix complex FFT.
#ifndef AAC_MDCT_H
#define AAC_MDCT_H

typedef struct { float re, im; } cpx;

typedef struct {
    int n;              // FFT size
    int factors[16];    // radix list, product == n
    int nfactors;
    cpx *tw;            // e^{-2 pi i k / n}, k = 0..n-1
} fft_plan;

int  fft_init(fft_plan *p, int n);       // radices 2, 3, 4, 5; returns 0 on success
void fft_free(fft_plan *p);
void fft_run(const fft_plan *p, const cpx *in, cpx *out);   // forward, unnormalised

typedef struct {
    int M;              // number of output coefficients (960 or 120)
    fft_plan fft;       // size M/2
    cpx *pre, *post;    // twiddles
    cpx *buf, *buf2;
    float *fold;
} mdct_plan;

int  mdct_init(mdct_plan *p, int M);
void mdct_free(mdct_plan *p);
// X[k] = sum_{n=0}^{2M-1} x[n] cos(pi/M (n + 1/2 + M/2)(k + 1/2)), k = 0..M-1 (x already windowed)
void mdct_run(const mdct_plan *p, const float *x, float *X);

#endif
