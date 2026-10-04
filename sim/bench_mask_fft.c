// Radix-4 NEON DIF FFT (bit-reversed output) for the mask tool's max-hold, checked against the existing scalar FFT and timed.
// usage: bench_fft4 <hop> <reps> <variant: 0 = current scalar, 1 = fast>
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>
#include <arm_neon.h>

#define N 512
#define CBIN 84
#define K0 (CBIN - 28)
#define K1 (CBIN + 28)
static float win[N] __attribute__((aligned(16)));
static float twr[N], twi[N];
static int rev[N];
// tables of the fast FFT
static float t1r[256] __attribute__((aligned(16))), t1i[256] __attribute__((aligned(16)));     // radix-2 first stage: w512^j, j < 256
static float t4[3][3][2][64] __attribute__((aligned(16)));                                      // [stage][r-1][re/im][j]: w_L^(r j), L = 256, 64, 16 (q = 64, 16, 4)
static int rvk[K1 - K0 + 1];                                                                    // bit-reversed positions of the bins K0..K1
static void init(void){
    for (int i = 0; i < N; i++) win[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (N - 1));
    for (int i = 0; i < N; i++){ int r = 0; for (int b = 0; b < 9; b++) if (i & (1<<b)) r |= 1 << (8-b); rev[i] = r; }
    for (int half = 4; half < N; half <<= 1)
        for (int j = 0; j < half; j++){ float a = 2.0f * (float)M_PI * j / (2 * half); twr[half + j] = cosf(a); twi[half + j] = -sinf(a); }
    for (int j = 0; j < 256; j++){ double a = 2.0 * M_PI * j / 512.0; t1r[j] = (float)cos(a); t1i[j] = (float)-sin(a); }
    for (int st = 0, L = 256; st < 3; st++, L >>= 2){
        int q = L / 4;
        for (int r = 1; r <= 3; r++)
            for (int j = 0; j < q; j++){ double a = 2.0 * M_PI * r * j / L; t4[st][r-1][0][j] = (float)cos(a); t4[st][r-1][1][j] = (float)-sin(a); }
    }
    for (int k = K0; k <= K1; k++) rvk[k - K0] = rev[k];
}
// ---- the existing scalar FFT (reference) ----
static void fft(float * restrict re, float * restrict im){
    for (int i = 0; i < N; i++){ int j = rev[i]; if (j > i){ float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; } }
    for (int i = 0; i < N; i += 2){ float ar = re[i], ai = im[i], br = re[i+1], bi = im[i+1]; re[i] = ar + br; im[i] = ai + bi; re[i+1] = ar - br; im[i+1] = ai - bi; }
    for (int i = 0; i < N; i += 4){
        float a0r = re[i], a0i = im[i], a1r = re[i+1], a1i = im[i+1], b0r = re[i+2], b0i = im[i+2], b1r = re[i+3], b1i = im[i+3];
        float x1r = b1i, x1i = -b1r;
        re[i] = a0r + b0r; im[i] = a0i + b0i; re[i+2] = a0r - b0r; im[i+2] = a0i - b0i;
        re[i+1] = a1r + x1r; im[i+1] = a1i + x1i; re[i+3] = a1r - x1r; im[i+3] = a1i - x1i;
    }
    for (int len = 8; len <= N; len <<= 1){
        int half = len >> 1;
        const float * restrict wr = &twr[half], * restrict wi = &twi[half];
        for (int i = 0; i < N; i += len){
            float * restrict ra = re + i, * restrict ia = im + i; float * restrict rb = ra + half, * restrict ib = ia + half;
            for (int j = 0; j < half; j++){
                float xr = rb[j]*wr[j] - ib[j]*wi[j], xi = rb[j]*wi[j] + ib[j]*wr[j];
                rb[j] = ra[j] - xr; ib[j] = ia[j] - xi; ra[j] += xr; ia[j] += xi;
            }
        }
    }
}
// ---- fast FFT: DIF, one radix-2 stage then radix-4 stages (L = 256, 64, 16) and a twiddle-free radix-4 last stage; output in bit-reversed order ----
static inline void cmulq(float32x4_t ur, float32x4_t ui, float32x4_t wr, float32x4_t wi, float32x4_t *yr, float32x4_t *yi){
    *yr = vmlsq_f32(vmulq_f32(ur, wr), ui, wi);
    *yi = vmlaq_f32(vmulq_f32(ur, wi), ui, wr);
}
static inline __attribute__((always_inline)) void r4(float *p0r, float *p0i, int j, int q, const float *w1r, const float *w1i, const float *w2r, const float *w2i, const float *w3r, const float *w3i){
    float32x4_t x0r = vld1q_f32(p0r + j), x0i = vld1q_f32(p0i + j);
    float32x4_t x1r = vld1q_f32(p0r + j + q), x1i = vld1q_f32(p0i + j + q);
    float32x4_t x2r = vld1q_f32(p0r + j + 2*q), x2i = vld1q_f32(p0i + j + 2*q);
    float32x4_t x3r = vld1q_f32(p0r + j + 3*q), x3i = vld1q_f32(p0i + j + 3*q);
    float32x4_t t0r = vaddq_f32(x0r, x2r), t0i = vaddq_f32(x0i, x2i), t1r_ = vaddq_f32(x1r, x3r), t1i_ = vaddq_f32(x1i, x3i);
    float32x4_t t2r = vsubq_f32(x0r, x2r), t2i = vsubq_f32(x0i, x2i), dr = vsubq_f32(x1r, x3r), di = vsubq_f32(x1i, x3i);
    float32x4_t t3r = di, t3i = vnegq_f32(dr);                          // -j * (x1 - x3)
    float32x4_t y1r, y1i, y2r, y2i, y3r, y3i;
    cmulq(vsubq_f32(t0r, t1r_), vsubq_f32(t0i, t1i_), vld1q_f32(w2r + j), vld1q_f32(w2i + j), &y1r, &y1i);
    cmulq(vaddq_f32(t2r, t3r), vaddq_f32(t2i, t3i), vld1q_f32(w1r + j), vld1q_f32(w1i + j), &y2r, &y2i);
    cmulq(vsubq_f32(t2r, t3r), vsubq_f32(t2i, t3i), vld1q_f32(w3r + j), vld1q_f32(w3i + j), &y3r, &y3i);
    vst1q_f32(p0r + j, vaddq_f32(t0r, t1r_)); vst1q_f32(p0i + j, vaddq_f32(t0i, t1i_));
    vst1q_f32(p0r + j + q, y1r); vst1q_f32(p0i + j + q, y1i);
    vst1q_f32(p0r + j + 2*q, y2r); vst1q_f32(p0i + j + 2*q, y2i);
    vst1q_f32(p0r + j + 3*q, y3r); vst1q_f32(p0i + j + 3*q, y3i);
}
static void fft_fast(float * restrict re, float * restrict im){
    for (int j = 0; j < 256; j += 4){                                  // L = 512, radix-2
        float32x4_t ar = vld1q_f32(re + j), ai = vld1q_f32(im + j), br = vld1q_f32(re + j + 256), bi = vld1q_f32(im + j + 256);
        float32x4_t dr = vsubq_f32(ar, br), di = vsubq_f32(ai, bi), yr, yi;
        cmulq(dr, di, vld1q_f32(t1r + j), vld1q_f32(t1i + j), &yr, &yi);
        vst1q_f32(re + j, vaddq_f32(ar, br)); vst1q_f32(im + j, vaddq_f32(ai, bi));
        vst1q_f32(re + j + 256, yr); vst1q_f32(im + j + 256, yi);
    }
    for (int st = 0, L = 256; st < 3; st++, L >>= 2){                   // radix-4 stages
        const int q = L >> 2;
        const float *w1r = t4[st][0][0], *w1i = t4[st][0][1], *w2r = t4[st][1][0], *w2i = t4[st][1][1], *w3r = t4[st][2][0], *w3i = t4[st][2][1];
        if (q >= 8){
            for (int blk = 0; blk < N; blk += L)
                for (int j = 0; j < q; j += 8){ r4(re + blk, im + blk, j, q, w1r, w1i, w2r, w2i, w3r, w3i); r4(re + blk, im + blk, j + 4, q, w1r, w1i, w2r, w2i, w3r, w3i); }
        } else {
            for (int blk = 0; blk < N; blk += 2 * L){                   // q = 4: two blocks per pass
                r4(re + blk, im + blk, 0, q, w1r, w1i, w2r, w2i, w3r, w3i);
                r4(re + blk + L, im + blk + L, 0, q, w1r, w1i, w2r, w2i, w3r, w3i);
            }
        }
    }
    for (int g = 0; g < N; g += 16){                                    // last stage: radix-4 without twiddles, four blocks of four at once (vld4 de-interleaves them)
        float32x4x4_t R = vld4q_f32(re + g), I = vld4q_f32(im + g), Y_R, Y_I;
        float32x4_t t0r = vaddq_f32(R.val[0], R.val[2]), t0i = vaddq_f32(I.val[0], I.val[2]), t1r_ = vaddq_f32(R.val[1], R.val[3]), t1i_ = vaddq_f32(I.val[1], I.val[3]);
        float32x4_t t2r = vsubq_f32(R.val[0], R.val[2]), t2i = vsubq_f32(I.val[0], I.val[2]), dr = vsubq_f32(R.val[1], R.val[3]), di = vsubq_f32(I.val[1], I.val[3]);
        float32x4_t t3r = di, t3i = vnegq_f32(dr);
        Y_R.val[0] = vaddq_f32(t0r, t1r_); Y_I.val[0] = vaddq_f32(t0i, t1i_);
        Y_R.val[1] = vsubq_f32(t0r, t1r_); Y_I.val[1] = vsubq_f32(t0i, t1i_);
        Y_R.val[2] = vaddq_f32(t2r, t3r);  Y_I.val[2] = vaddq_f32(t2i, t3i);
        Y_R.val[3] = vsubq_f32(t2r, t3r);  Y_I.val[3] = vsubq_f32(t2i, t3i);
        vst4q_f32(re + g, Y_R); vst4q_f32(im + g, Y_I);
    }
}
typedef struct { const int16_t *x; long s0, s1; int hop; float *mh; long nf; } Job;
static void *worker(void *arg){
    Job *j = arg; float re[N] __attribute__((aligned(16))), im[N] __attribute__((aligned(16)));
    for (int k = 0; k < N; k++) j->mh[k] = 0;
    for (long s = j->s0; s < j->s1; s += j->hop, j->nf++){
        const int16_t *xp = j->x + 2 * s;
        for (int i = 0; i < N; i++){ re[i] = xp[2*i] * win[i]; im[i] = xp[2*i+1] * win[i]; }
        fft(re, im);
        for (int k = K0; k <= K1; k++){ float p = re[k]*re[k] + im[k]*im[k]; if (p > j->mh[k]) j->mh[k] = p; }
    }
    return NULL;
}
static void *worker_fast(void *arg){
    Job *j = arg; float re[N] __attribute__((aligned(16))), im[N] __attribute__((aligned(16)));
    for (int k = 0; k < N; k++) j->mh[k] = 0;
    for (long s = j->s0; s < j->s1; s += j->hop, j->nf++){
        const int16_t *xp = j->x + 2 * s;
        for (int i = 0; i < N; i += 8){
            int16x8x2_t v = vld2q_s16(xp + 2 * i);
            float32x4_t i0 = vcvtq_f32_s32(vmovl_s16(vget_low_s16(v.val[0]))), i1 = vcvtq_f32_s32(vmovl_s16(vget_high_s16(v.val[0])));
            float32x4_t q0 = vcvtq_f32_s32(vmovl_s16(vget_low_s16(v.val[1]))), q1 = vcvtq_f32_s32(vmovl_s16(vget_high_s16(v.val[1])));
            float32x4_t w0 = vld1q_f32(win + i), w1 = vld1q_f32(win + i + 4);
            vst1q_f32(re + i, vmulq_f32(i0, w0)); vst1q_f32(re + i + 4, vmulq_f32(i1, w1));
            vst1q_f32(im + i, vmulq_f32(q0, w0)); vst1q_f32(im + i + 4, vmulq_f32(q1, w1));
        }
        fft_fast(re, im);
        for (int k = K0; k <= K1; k++){ int r = rvk[k - K0]; float p = re[r]*re[r] + im[r]*im[r]; if (p > j->mh[k]) j->mh[k] = p; }
    }
    return NULL;
}
static double now(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
int main(int argc, char **argv){
    int hop = argc > 1 ? atoi(argv[1]) : 256, reps = argc > 2 ? atoi(argv[2]) : 5, var = argc > 3 ? atoi(argv[3]) : 0;
    long frames = 768000; init();
    int16_t *x = malloc(frames * 4);
    // test signal: a carrier at bin 84 with noise, so that the bins K0..K1 carry a realistic dynamic range
    for (long i = 0; i < frames; i++){
        double ph = 2.0 * M_PI * 84.0 / 512.0 * i;
        x[2*i] = (int16_t)(12000.0 * cos(ph) + (rand() & 0x3f) - 32); x[2*i+1] = (int16_t)(12000.0 * sin(ph) + (rand() & 0x3f) - 32);
    }
    static float mh1[N] __attribute__((aligned(16))), mh2[N] __attribute__((aligned(16)));
    if (var == 2){                                                       // correctness: both workers on the same data, compare the bins K0..K1
        Job a = { x, 0, 50000, 256, mh1, 0 }, b = { x, 0, 50000, 256, mh2, 0 };
        worker(&a); worker_fast(&b);
        double maxrel = 0; for (int k = K0; k <= K1; k++){ double d = fabs(10.0 * log10((mh1[k] + 1e-30) / (mh2[k] + 1e-30))); if (d > maxrel) maxrel = d; }
        printf("max difference over bins %d..%d: %.5f dB (%ld windows)\n", K0, K1, maxrel, a.nf);
        for (int k = K0; k <= K1; k += 7) printf("  bin %d: %.4f dB vs %.4f dB\n", k, 10.0 * log10(mh1[k] + 1e-30), 10.0 * log10(mh2[k] + 1e-30));
        return 0;
    }
    for (int r = 0; r < reps; r++){
        long nv = (frames - N) / hop + 1, half = nv / 2;
        Job a = { x, 0, half * hop, hop, mh1, 0 }, b = { x, half * hop, nv * hop, hop, mh2, 0 };
        void *(*wk)(void *) = var ? worker_fast : worker;
        double t0 = now(); pthread_t th; pthread_create(&th, NULL, wk, &b); wk(&a); pthread_join(th, NULL);
        double dt = now() - t0;
        printf("var %d hop %d: %ld FFTs in %.1f ms (%.1f us per FFT per core)\n", var, hop, a.nf + b.nf, dt * 1e3, dt * 1e6 * 2 / (a.nf + b.nf));
    }
    return 0;
}
