// SPDX-License-Identifier: Apache-2.0
#include "aac_mdct.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static cpx cmul(cpx a, cpx b){ cpx r = { a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re }; return r; }

int fft_init(fft_plan *p, int n){
    memset(p, 0, sizeof *p);
    p->n = n;
    int m = n, k = 0;
    while (m > 1){                                   // prefer radix 4, then 2, 3, 5
        int r = (m % 4 == 0) ? 4 : (m % 2 == 0) ? 2 : (m % 3 == 0) ? 3 : (m % 5 == 0) ? 5 : 0;
        if (!r || k >= 16) return -1;
        p->factors[k++] = r; m /= r;
    }
    p->nfactors = k;
    p->tw = malloc(sizeof(cpx) * (size_t)n);
    if (!p->tw) return -1;
    for (int i = 0; i < n; i++){ double a = -2.0 * M_PI * i / n; p->tw[i].re = (float)cos(a); p->tw[i].im = (float)sin(a); }
    return 0;
}

void fft_free(fft_plan *p){ free(p->tw); p->tw = NULL; }

// decimation in time, generic radix butterflies with the global twiddle table
static void fft_work(const fft_plan *p, cpx *out, const cpx *in, int fstride, const int *f){
    const int r = f[0], n = p->n / fstride, m = n / r;
    if (m == 1){
        for (int q = 0; q < r; q++) out[q] = in[q * fstride];
    } else {
        for (int q = 0; q < r; q++) fft_work(p, out + q * m, in + q * fstride, fstride * r, f + 1);
    }
    cpx s[5];
    for (int k = 0; k < m; k++){
        for (int q = 0; q < r; q++) s[q] = q ? cmul(out[k + q * m], p->tw[(q * k * fstride) % p->n]) : out[k];
        for (int q1 = 0; q1 < r; q1++){
            cpx acc = s[0];
            for (int q = 1; q < r; q++){
                cpx t = cmul(s[q], p->tw[((q * q1) % r) * (p->n / r)]);
                acc.re += t.re; acc.im += t.im;
            }
            out[k + q1 * m] = acc;
        }
    }
}

void fft_run(const fft_plan *p, const cpx *in, cpx *out){ fft_work(p, out, in, 1, p->factors); }

int mdct_init(mdct_plan *p, int M){
    memset(p, 0, sizeof *p);
    p->M = M;
    const int L = M / 2;
    if (fft_init(&p->fft, L)) return -1;
    p->pre = malloc(sizeof(cpx) * (size_t)L); p->post = malloc(sizeof(cpx) * (size_t)L);
    p->buf = malloc(sizeof(cpx) * (size_t)L); p->buf2 = malloc(sizeof(cpx) * (size_t)L);
    p->fold = malloc(sizeof(float) * (size_t)M);
    if (!p->pre || !p->post || !p->buf || !p->buf2 || !p->fold) return -1;
    for (int n = 0; n < L; n++){
        double a = -M_PI * (n + 0.25) / M, b = -M_PI * n / M;
        p->pre[n].re = (float)cos(a); p->pre[n].im = (float)sin(a);
        p->post[n].re = (float)cos(b); p->post[n].im = (float)sin(b);
    }
    return 0;
}

void mdct_free(mdct_plan *p){ fft_free(&p->fft); free(p->pre); free(p->post); free(p->buf); free(p->buf2); free(p->fold); }

void mdct_run(const mdct_plan *p, const float *x, float *X){
    const int M = p->M, q = M / 2, L = M / 2;
    const float *a = x, *b = x + q, *c = x + 2 * q, *d = x + 3 * q;
    float *v = p->fold;
    for (int n = 0; n < q; n++){
        v[n]     = -c[q - 1 - n] - d[n];                 // (-c_r - d)
        v[q + n] =  a[n] - b[q - 1 - n];                 // ( a - b_r)
    }
    // DCT-IV of v via an L-point complex FFT
    for (int n = 0; n < L; n++){ cpx z = { v[2 * n], v[M - 1 - 2 * n] }; p->buf[n] = cmul(z, p->pre[n]); }
    fft_run(&p->fft, p->buf, p->buf2);
    for (int k = 0; k < L; k++){ cpx y = cmul(p->buf2[k], p->post[k]); X[2 * k] = y.re; X[M - 1 - 2 * k] = -y.im; }
}
