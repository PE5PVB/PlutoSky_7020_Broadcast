// =============================================================================
// skypluto-mask.c - SM.1268-5 mask monitoring on the Pluto itself
// -----------------------------------------------------------------------------
// Takes a short IQ capture from an RX channel, computes the max-hold spectrum
// (Hann STFT, 512 points @3.072 MSPS = 6 kHz/bin, ENBW ~9 kHz ~ RBW 10 kHz),
// measures its own noise floor with the RX LO 30 MHz away from the carrier, and reports the margin
// relative to the mask (0 dB @74, -15 @107.5, -30 @124, -40 @152.5 kHz).
//
//   skypluto-mask <tx_lo_Hz> [rx=1|2] [port=A_BALANCED] [gain=0(auto)] [seconds=1.0]
//
// rx=1: TX1 -> (attenuator) -> RX1     rx=2: TX2 (twin) -> RX2
// Output (1 line):
//   MASK conclusive=<0|1> margin=<dB> at=<kHz> shoulder=<dB> floor=<dB> dev=<kHz> g=<gain> [reason]
//     margin      smallest margin relative to the mask over +-80..170 kHz (positive = inside the mask)
//     shoulder    mean max-hold relative to the peak over +-130..170 kHz
//     floor       the same measure for the own noise floor (same RX gain)
//     conclusive  1 if the shoulder is >= 6 dB above the floor; 0 = the margin is then a lower bound
//                 (the measurement includes the own noise: if that is already inside the mask, the signal certainly is)
//   Two lines follow:           SPEC <57 chars>  (max-hold relative to the peak, -168..+168 kHz in 6 kHz steps)
//                               FLOOR <57 chars> (own noise floor, same scale); char = 1.25 dB/step (A..Z a..z 0..9)
// Build: arm-linux-gnueabihf-gcc -O2 -Wall -o skypluto-mask skypluto-mask.c -lm
// =============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <sys/stat.h>

#define FS      3072000.0
#define N       512
#define HOP     384                     // hop 384 = 25 % overlap (Hann): ~1 dB less peak-sensitive than 256, 1.5x faster on the ARM
#define OFFSET  504000.0               // carrier in the capture (RX LO = TX LO - 504 kHz) = bin 84: bins fall on k x 6 kHz
#define CBIN    84
#define GRID    57                      // output grid: k = -28..+28 -> -168..+168 kHz in 6 kHz steps
#define BINHZ   (FS / N)

static float win[N], twc[N/2], tws[N/2];
static int   rev[N];
static void fft_tw_init(void);

static void fft_init(void){
    for (int i = 0; i < N; i++) win[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (N - 1));
    for (int k = 0; k < N/2; k++){ twc[k] = cosf(2.0f*(float)M_PI*k/N); tws[k] = -sinf(2.0f*(float)M_PI*k/N); }
    for (int i = 0; i < N; i++){ int r = 0; for (int b = 0; b < 9; b++) if (i & (1<<b)) r |= 1 << (8-b); rev[i] = r; }
    fft_tw_init();
}
// twiddles per stage, contiguous: the stage with half = len/2 uses twr[half + j], twi[half + j] (j < half)
static float twr[N], twi[N];
static void fft_tw_init(void){
    for (int half = 4; half < N; half <<= 1)
        for (int j = 0; j < half; j++){ float a = 2.0f * (float)M_PI * j / (2 * half); twr[half + j] = cosf(a); twi[half + j] = -sinf(a); }
}
static void fft(float * restrict re, float * restrict im){
    for (int i = 0; i < N; i++){ int j = rev[i]; if (j > i){ float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; } }
    for (int i = 0; i < N; i += 2){                                   // stage len=2: w = 1
        float ar = re[i], ai = im[i], br = re[i+1], bi = im[i+1];
        re[i] = ar + br; im[i] = ai + bi; re[i+1] = ar - br; im[i+1] = ai - bi;
    }
    for (int i = 0; i < N; i += 4){                                   // stage len=4: w = 1 and -j (no multiplications)
        float a0r = re[i], a0i = im[i], a1r = re[i+1], a1i = im[i+1], b0r = re[i+2], b0i = im[i+2], b1r = re[i+3], b1i = im[i+3];
        float x1r = b1i, x1i = -b1r;
        re[i]   = a0r + b0r; im[i]   = a0i + b0i; re[i+2] = a0r - b0r; im[i+2] = a0i - b0i;
        re[i+1] = a1r + x1r; im[i+1] = a1i + x1i; re[i+3] = a1r - x1r; im[i+3] = a1i - x1i;
    }
    for (int len = 8; len <= N; len <<= 1){
        int half = len >> 1;
        const float * restrict wr = &twr[half], * restrict wi = &twi[half];
        for (int i = 0; i < N; i += len){
            float * restrict ra = re + i, * restrict ia = im + i;
            float * restrict rb = ra + half, * restrict ib = ia + half;
            for (int j = 0; j < half; j++){
                float xr = rb[j]*wr[j] - ib[j]*wi[j], xi = rb[j]*wi[j] + ib[j]*wr[j];
                rb[j] = ra[j] - xr; ib[j] = ia[j] - xi; ra[j] += xr; ia[j] += xi;
            }
        }
    }
}
static double mask_db(double d){               // d in kHz (absolute)
    d = fabs(d);
    if (d <= 74.0)  return 0.0;
    if (d <= 107.5) return -15.0 * (d - 74.0) / 33.5;
    if (d <= 124.0) return -15.0 - 15.0 * (d - 107.5) / 16.5;
    if (d <= 152.5) return -30.0 - 10.0 * (d - 124.0) / 28.5;
    return -40.0;
}

static void sh(const char *fmt, ...) __attribute__((format(printf,1,2)));
#include <stdarg.h>
static void sh(const char *fmt, ...){
    char c[256]; va_list ap; va_start(ap, fmt); vsnprintf(c, sizeof c, fmt, ap); va_end(ap);
    if (system(c)) { /* errors show up via the capture */ }
}

// fetch n frames from the selected RX; 0 = success
static int capture(int rx, long frames, int16_t *out){
    char c[200];
    snprintf(c, sizeof c, "iio_readdev -b 65536 -s %ld cf-ad9361-lpc voltage%d voltage%d 2>/dev/null", frames, rx==2?2:0, rx==2?3:1);
    FILE *p = popen(c, "r"); if (!p) return -1;
    size_t want = (size_t)frames * 2, got = fread(out, sizeof(int16_t), want, p);
    pclose(p);
    return got == want ? 0 : -1;
}
static double meanpower(const int16_t *x, long frames){
    double s = 0; for (long i = 0; i < frames; i++) s += (double)x[2*i]*x[2*i] + (double)x[2*i+1]*x[2*i+1];
    return s / frames;
}
static void maxhold(const int16_t *x, long frames, double *mh){
    static float re[N], im[N];
    for (int k = 0; k < N; k++) mh[k] = 0;
    for (long s = 0; s + N <= frames; s += HOP){
        for (int i = 0; i < N; i++){ re[i] = x[2*(s+i)] * win[i]; im[i] = x[2*(s+i)+1] * win[i]; }
        fft(re, im);
        for (int k = 0; k < N; k++){ double p = (double)re[k]*re[k] + (double)im[k]*im[k]; if (p > mh[k]) mh[k] = p; }
    }
}
static double bin_freq(int k){ return (k < N/2 ? k : k - N) * BINHZ; }

// Continuity check of the capture: the carrier sits OFFSET above the RX LO, so the phase rotates by
// 2*pi*OFFSET/FS rad per sample; the FM (max ~+-90 kHz) adds at most ~0.2 rad. A step of more than ~35 degrees away from that
// rotation can NOT originate from the transmitter (the interpolator makes jumps impossible) and means dropped or
// corrupted samples in the capture itself. Counts such jumps on samples with enough signal.
static long count_jumps(const int16_t *x, long frames){
    const double phi0 = 2.0 * M_PI * OFFSET / FS, c0 = cos(phi0), s0 = sin(phi0);
    long n = 0;
    for (long i = 0; i + 1 < frames; i++){
        double ar = x[2*i], ai = x[2*i+1], br = x[2*i+2], bi = x[2*i+3];
        if (ar*ar + ai*ai < 1.0e4) continue;                        // too little signal: phase meaningless
        double re = br*ar + bi*ai, im = bi*ar - br*ai;              // b * conj(a)
        double r2 = re*c0 + im*s0, i2 = im*c0 - re*s0;              // derotation by the expected step
        if (r2 <= 0.0 || i2*i2 > 0.5 * r2*r2) n++;                  // angle > ~35 degrees
    }
    return n;
}

// 1 char per point: index v = round(-dB / 1.25) clamped to 0..61; 'A'..'Z' = 0..25, 'a'..'z' = 26..51, '0'..'9' = 52..61
static char encdb(double db){
    int v = (int)floor(-db / 1.25 + 0.5); if (v < 0) v = 0; if (v > 61) v = 61;
    return v < 26 ? 'A' + v : v < 52 ? 'a' + (v - 26) : '0' + (v - 52);
}

// ============================ CONTINUOUS MODE (stream) ==================================================
// skypluto-mask <tx_lo> <rx> <poort> <gain> <secs_floor/gain> <interval_s> <duration_s>
// Sets up gain + noise floor ONCE, then reads <duration_s> seconds of uninterrupted IQ (reader thread + ring of buffers, so gap-free
// as long as processing is faster than real time) and delivers a MASK/SPEC/FLOOR block every <interval_s> seconds with the max-hold of
// EXACTLY that interval. Processing adapts the FFT hop (384..1024) so the ARM keeps up in real time; 'cov' = fraction of the samples
// that fall inside an FFT window (1.00 = gap-free).  The phase-jump check (jumps) also counts across the window boundary.
#include <pthread.h>
// Ring of NB window buffers. The reader thread keeps writing without interruption (never waits for processing); if processing falls behind,
// the OLDEST ready windows are skipped (coverage drops, but no gaps appear INSIDE a window and hence no
// false phase jumps).  seq[] = sequence number of the window; consecutive sequence numbers = uninterrupted stream.
#define NB 8
enum { B_FREE = 0, B_FILL, B_READY, B_BUSY };
typedef struct { int16_t *buf[NB]; int st[NB]; long seq[NB]; long frames; int eof; FILE *p; pthread_mutex_t m; pthread_cond_t c;
                 int ch4; double pw_s2[NB], pw_si[NB], pw_sq[NB]; long pw_pk[NB]; } Rd;     // ch4: 4 channels (RX1 I/Q + RX2 I/Q); RX1 statistics per window for the power meter
static Rd rdc;

static void *reader_thread(void *arg){
    (void)arg;
    long k = 0;
    for (;;){
        int b = (int)(k % NB);
        pthread_mutex_lock(&rdc.m);
        while (rdc.st[b] == B_BUSY) pthread_cond_wait(&rdc.c, &rdc.m);     // only if processing is still using exactly this (older) window
        rdc.st[b] = B_FILL;
        pthread_mutex_unlock(&rdc.m);
        size_t want = (size_t)rdc.frames * 2, got = 0;
        if (!rdc.ch4) got = fread(rdc.buf[b], sizeof(int16_t), want, rdc.p);
        else {                                                           // 4 channels: RX2 into the window, RX1 only as power statistics (sum of squares, DC, peak)
            static int16_t t4[16384 * 4];
            double s2 = 0, si = 0, sq = 0; long pk = 0; long left = rdc.frames; int16_t *dst = rdc.buf[b];
            while (left > 0){
                size_t nf = left > 16384 ? 16384 : (size_t)left, gf = fread(t4, 4 * sizeof(int16_t), nf, rdc.p);
                for (size_t i = 0; i < gf; i++){
                    int x = t4[4*i], y = t4[4*i+1];
                    s2 += (double)x * x + (double)y * y; si += x; sq += y;
                    long ax = x < 0 ? -x : x, ay = y < 0 ? -y : y; if (ax > pk) pk = ax; if (ay > pk) pk = ay;
                    dst[2*i] = t4[4*i+2]; dst[2*i+1] = t4[4*i+3];
                }
                got += gf * 2; dst += 2 * gf; left -= (long)gf;
                if (gf != nf) break;
            }
            rdc.pw_s2[b] = s2; rdc.pw_si[b] = si; rdc.pw_sq[b] = sq; rdc.pw_pk[b] = pk;
        }
        pthread_mutex_lock(&rdc.m);
        if (got != want){ rdc.st[b] = B_FREE; rdc.eof = 1; pthread_cond_broadcast(&rdc.c); pthread_mutex_unlock(&rdc.m); break; }
        rdc.st[b] = B_READY; rdc.seq[b] = k; pthread_cond_broadcast(&rdc.c); pthread_mutex_unlock(&rdc.m);
        k++;
    }
    return NULL;
}
// same check as count_jumps, in float (faster) and with a settable starting value (window boundary)
static long count_jumps_f(const int16_t *x, long frames, float pr, float pi_, int have_prev){
    const float phi0 = (float)(2.0 * M_PI * OFFSET / FS), c0 = cosf(phi0), s0 = sinf(phi0);
    long n = 0; float ar = pr, ai = pi_; long i0 = have_prev ? 0 : 1;
    if (!have_prev){ ar = x[0]; ai = x[1]; }
    for (long i = i0; i < frames; i++){
        float br = x[2*i], bi = x[2*i+1];
        if (ar*ar + ai*ai >= 1.0e4f){
            float re = br*ar + bi*ai, im = bi*ar - br*ai;
            float r2 = re*c0 + im*s0, i2 = im*c0 - re*s0;
            if (r2 <= 0.0f || i2*i2 > 0.5f * r2*r2) n++;
        }
        ar = br; ai = bi;
    }
    return n;
}
// max-hold with adjustable hop; returns the number of FFT windows
typedef struct { const int16_t *x; long s0, s1; int hop; double *mh; long nf; } MhJob;
static void *mh_worker(void *arg){                          // FFT max-hold over the window positions s0, s0+hop, ... < s1
    MhJob *j = (MhJob *)arg;
    float re[N], im[N];
    for (int k = 0; k < N; k++) j->mh[k] = 0;
    j->nf = 0;
    for (long s = j->s0; s < j->s1; s += j->hop, j->nf++){
        const int16_t *xp = j->x + 2 * s;
        for (int i = 0; i < N; i++){ re[i] = xp[2*i] * win[i]; im[i] = xp[2*i+1] * win[i]; }
        fft(re, im);
        for (int k = 0; k < N; k++){ float p = re[k]*re[k] + im[k]*im[k]; if (p > j->mh[k]) j->mh[k] = p; }
    }
    return NULL;
}
// max-hold with adjustable hop, split over 2 threads (the Zynq has 2 A9 cores); returns the number of FFT windows
static long maxhold_h(const int16_t *x, long frames, double *mh, int hop){
    long nvenster = (frames >= N) ? (frames - N) / hop + 1 : 0;       // number of window positions
    long half = nvenster / 2;
    static double mh2[N];
    MhJob a = { x, 0, half * hop, hop, mh, 0 }, b = { x, half * hop, nvenster * hop, hop, mh2, 0 };
    pthread_t th; pthread_create(&th, NULL, mh_worker, &b);
    mh_worker(&a);
    pthread_join(th, NULL);
    for (int k = 0; k < N; k++) if (mh2[k] > mh[k]) mh[k] = mh2[k];
    return a.nf + b.nf;
}
static double est_dev(const int16_t *buf, long nd){        // peak deviation (kHz) over the first nd samples (LP 16), as in the single-shot mode
    double lp[16] = {0}, sum = 0, mean = 0, pk = 0; long cnt = 0;
    float *fl = malloc((size_t)nd * sizeof(float)); if (!fl) return 0.0;
    for (long i = 0; i + 1 < nd; i++){
        double ar = buf[2*i], ai = buf[2*i+1], br = buf[2*i+2], bi = buf[2*i+3];
        double f = atan2(bi*ar - br*ai, br*ar + bi*ai) * FS / (2.0 * M_PI);
        sum += f - lp[i & 15]; lp[i & 15] = f;
        if (i >= 15){ fl[cnt++] = (float)(sum / 16.0); mean += fl[cnt-1]; }
    }
    mean /= cnt ? cnt : 1;
    for (long i = 0; i < cnt; i++){ double d = fabs(fl[i] - mean); if (d > pk) pk = d; }
    free(fl); return pk / 1e3;
}
// ---- tone measurement (audio linearity): FM demodulation of the capture, FFT, peak -> frequency + deviation -----------------------------------------------
// Only active if /tmp/skypluto.tone was touched within the last 10 s (by '?A' of the daemon). 16384 points at 192 kHz (box-16 decimation of the instantaneous
// frequency), Hann + parabolic interpolation; the box-16 droop is corrected. Yields the tone frequency (Hz) and the deviation amplitude (kHz).
static void fft_inplace(double *re, double *im, int n){
    for (int i = 1, j = 0; i < n; i++){
        int bit = n >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit;
        if (i < j){ double t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (int len = 2; len <= n; len <<= 1){
        double ang = -2.0 * M_PI / len, wr = cos(ang), wi = sin(ang);
        for (int i = 0; i < n; i += len){
            double cr = 1.0, ci = 0.0;
            for (int k = 0; k < len / 2; k++){
                double ur = re[i+k], ui = im[i+k];
                double vr = re[i+k+len/2] * cr - im[i+k+len/2] * ci, vi = re[i+k+len/2] * ci + im[i+k+len/2] * cr;
                re[i+k] = ur + vr; im[i+k] = ui + vi; re[i+k+len/2] = ur - vr; im[i+k+len/2] = ui - vi;
                double nr = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = nr;
            }
        }
    }
}
static int tone_enabled(void){
    struct stat st;
    return stat("/tmp/skypluto.tone", &st) == 0 && time(NULL) - st.st_mtime < 10;
}
static int tone_analyze(const int16_t *buf, long nfr, double *fo, double *dev_khz){
    enum { M = 16384, D = 16 };
    if (nfr < (long)M * D + 2) return 0;
    static double re[M], im[M];
    double mean = 0.0;
    for (int k = 0; k < M; k++){
        double s = 0.0;
        for (int j = 0; j < D; j++){
            long i = (long)k * D + j;
            double ar = buf[2*i], ai = buf[2*i+1], br = buf[2*i+2], bi = buf[2*i+3];
            s += atan2(bi*ar - br*ai, br*ar + bi*ai);
        }
        re[k] = s * FS / (2.0 * M_PI * D); mean += re[k];
    }
    mean /= M;
    for (int k = 0; k < M; k++){ double w = 0.5 - 0.5 * cos(2.0 * M_PI * k / M); re[k] = (re[k] - mean) * w; im[k] = 0.0; }
    fft_inplace(re, im, M);
    double binw = (FS / D) / M;
    int k0 = (int)ceil(100.0 / binw), k1 = (int)floor(25000.0 / binw), kp = k0; double best = 0.0;
    for (int k = k0; k <= k1; k++){ double m2 = re[k]*re[k] + im[k]*im[k]; if (m2 > best){ best = m2; kp = k; } }
    if (best <= 0.0) return 0;
    double a = log(sqrt(re[kp-1]*re[kp-1] + im[kp-1]*im[kp-1]) + 1e-30), b = 0.5 * log(best + 1e-60), c = log(sqrt(re[kp+1]*re[kp+1] + im[kp+1]*im[kp+1]) + 1e-30);
    double den = a - 2.0 * b + c, d = (fabs(den) > 1e-12) ? 0.5 * (a - c) / den : 0.0;
    double f = (kp + d) * binw;
    // amplitude from the POWER in the main lobe (kp +-3 bins; Parseval, Hann: sum(w^2) = 3M/8): independent of where the tone falls between two bins
    double slobe = 0.0; for (int k = kp - 3; k <= kp + 3; k++) slobe += re[k]*re[k] + im[k]*im[k];
    double amp = 2.0 * sqrt(slobe / ((double)M * 3.0 * M / 8.0));
    double H = sin(M_PI * f * D / FS) / (D * sin(M_PI * f / FS));               // box-16 droop
    *fo = f; *dev_khz = amp / H / 1e3;
    return 1;
}
static double now_s(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
// margin/shoulder/spectrum from mh and fmh -> MASK/SPEC/FLOOR block (same format as the single-shot mode + jumps + cov)
static void report_block(const double *mh, const double *fmh, int g, double dev, long jumps, double cov, double *margin_out){
    double ref = 0;
    for (int k = 0; k < N; k++){ double d = fabs(bin_freq(k) - OFFSET) / 1e3; if (d <= 170.0 && mh[k] > ref) ref = mh[k]; }
    if (ref <= 0){ printf("MASK error=geen_signaal g=%d\n", g); fflush(stdout); return; }
    double sh_sum = 0, fl_sum = 0; int nfl = 0; double margin = 1e9, at = 0;
    for (int k = 0; k < N; k++){
        double d = (bin_freq(k) - OFFSET) / 1e3, ad = fabs(d);
        if (ad < 80.0 || ad > 170.0) continue;
        double rel = 10.0 * log10(mh[k] / ref + 1e-30), m = mask_db(d) - rel;
        if (m < margin){ margin = m; at = d; }
        if (ad >= 130.0){ sh_sum += rel; fl_sum += 10.0 * log10(fmh[k] / ref + 1e-30); nfl++; }
    }
    double shoulder = nfl ? sh_sum / nfl : 0, floorv = nfl ? fl_sum / nfl : 0;
    printf("MASK conclusive=%d margin=%+.1f at=%+.0f shoulder=%.1f floor=%.1f dev=%.1f g=%d jumps=%ld cov=%.2f\n",
           (shoulder - floorv) >= 6.0, margin, at, shoulder, floorv, dev, g, jumps, cov);
    char sp[GRID + 1], fp[GRID + 1];
    for (int i = 0; i < GRID; i++){
        int k = (CBIN + i - GRID / 2) & (N - 1);
        sp[i] = encdb(10.0 * log10(mh[k]  / ref + 1e-30));
        fp[i] = encdb(10.0 * log10(fmh[k] / ref + 1e-30));
    }
    sp[GRID] = fp[GRID] = 0;
    printf("SPEC %s\nFLOOR %s\n", sp, fp);
    fflush(stdout);
    if (margin_out) *margin_out = margin;
}
static int stream_main(long long txlo, int rx, const char *port, int gain, double secs, double interval, double dur, int pwr){
    fft_init();
    long capfr = (long)(FS * secs); if (capfr < 16384) capfr = 16384;
    long win_fr = (long)(FS * interval); if (win_fr < 65536) win_fr = 65536;
    int nwin = (int)(dur / interval + 0.5); if (nwin < 1) nwin = 1;
    int16_t *cap = malloc((size_t)capfr * 2 * sizeof(int16_t));
    int alloc_ok = (cap != NULL);
    for (int i = 0; i < NB; i++){ rdc.buf[i] = malloc((size_t)win_fr * 2 * sizeof(int16_t)); if (!rdc.buf[i]) alloc_ok = 0; }
    if (!alloc_ok){ printf("MASK error=geheugen\n"); return 1; }
    int ch = rx - 1;
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select %s >/dev/null 2>&1", ch, port);
    sh("iio_attr -i -c ad9361-phy voltage%d gain_control_mode manual >/dev/null 2>&1", ch);
    sh("iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", txlo - (long long)OFFSET); usleep(60000);
    int g = gain > 0 ? gain : 20;
    if (gain <= 0){
        for (int it = 0; it < 3; it++){
            sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
            if (capture(rx, 8192, cap)){ printf("MASK error=opname\n"); goto fail; }
            double p = meanpower(cap, 8192);
            if (p < 5.0){ printf("MASK error=geen_signaal g=%d pwr=%.1f\n", g, p); goto fail; }
            double dg = 10.0 * log10(3.0e5 / p);
            int ng = (int)floor(g + dg + 0.5); if (ng < 0) ng = 0; if (ng > 60) ng = 60;
            if (ng == g) break;
            g = ng;
        }
    }
    sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
    // noise floor once: RX LO 30 MHz away from the carrier
    static double fmh[N];
    sh("iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", txlo + 30000000LL - (long long)OFFSET); usleep(60000);
    if (capture(rx, capfr, cap)){ printf("MASK error=vloer_opname\n"); goto fail; }
    maxhold_h(cap, capfr, fmh, HOP);
    sh("iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", txlo - (long long)OFFSET); usleep(250000);   // let the PLL lock
    // continuous capture
    char c[200];
    if (pwr && rx == 2) snprintf(c, sizeof c, "iio_readdev -b 65536 -s %ld cf-ad9361-lpc voltage0 voltage1 voltage2 voltage3 2>/dev/null", (long)(win_fr * (long)(nwin + 1)));
    else snprintf(c, sizeof c, "iio_readdev -b 65536 -s %ld cf-ad9361-lpc voltage%d voltage%d 2>/dev/null", (long)(win_fr * (long)(nwin + 1)), rx==2?2:0, rx==2?3:1);
    rdc.ch4 = (pwr && rx == 2); rdc.frames = win_fr; rdc.eof = 0;
    for (int i = 0; i < NB; i++){ rdc.st[i] = B_FREE; rdc.seq[i] = -1; }
    pthread_mutex_init(&rdc.m, NULL); pthread_cond_init(&rdc.c, NULL);
    rdc.p = popen(c, "r"); if (!rdc.p){ printf("MASK error=popen\n"); goto fail; }
    pthread_t th; pthread_create(&th, NULL, reader_thread, NULL);
    static double mh[N];
    int hop = HOP; float pr = 0, pi_ = 0; double last_bad = 0;
    long lastseq = -1, nproc = 0, nskip = 0;
    for (;;){
        int b = -1; long seqb = -1;
        pthread_mutex_lock(&rdc.m);
        for (;;){                                                  // pick the NEWEST ready window
            for (int i = 0; i < NB; i++) if (rdc.st[i] == B_READY && rdc.seq[i] > seqb){ seqb = rdc.seq[i]; b = i; }
            if (b >= 0 || rdc.eof) break;
            pthread_cond_wait(&rdc.c, &rdc.m);
        }
        if (b >= 0){
            for (int i = 0; i < NB; i++) if (rdc.st[i] == B_READY && i != b){ rdc.st[i] = B_FREE; nskip++; }   // skip the backlog
            rdc.st[b] = B_BUSY;
        }
        pthread_mutex_unlock(&rdc.m);
        if (b < 0) break;                                          // stream ended and nothing left ready
        const int16_t *x = rdc.buf[b];
        if (seqb == 0){                                             // first window = PLL/LO settling after the floor measurement: do NOT measure, but keep the phase chain
            pr = x[2*(win_fr-1)]; pi_ = x[2*(win_fr-1)+1];
            pthread_mutex_lock(&rdc.m); rdc.st[b] = B_FREE; pthread_cond_broadcast(&rdc.c); pthread_mutex_unlock(&rdc.m);
            lastseq = 0; continue;
        }
        double t0 = now_s();
        int consecutive = (lastseq >= 0 && seqb == lastseq + 1);   // only then is the stream uninterrupted across the window boundary
        long jumps = count_jumps_f(x, win_fr, pr, pi_, consecutive);
        pr = x[2*(win_fr-1)]; pi_ = x[2*(win_fr-1)+1];
        long nf = maxhold_h(x, win_fr, mh, hop);
        double cov = (double)N / hop; if (cov > 1.0) cov = 1.0;
        double dev = est_dev(x, 12288);                             // 4 ms is enough for a deviation indication (expensive atan2 per sample, so keep it short)
        if (dev > 150.0) jumps += 1000;                             // impossible deviation (max ~+-75 kHz): capture disturbed -> the daemon ignores this window
        double margin = 99;
        if (rdc.ch4){ double n = (double)win_fr, mi = rdc.pw_si[b] / n, mq = rdc.pw_sq[b] / n, p = rdc.pw_s2[b] / n - mi * mi - mq * mq; printf("PWR p=%.1f pk=%ld\n", p, rdc.pw_pk[b]); fflush(stdout); }
        if (tone_enabled()){ double tf, td; if (tone_analyze(x, win_fr, &tf, &td)){ printf("TONE f=%.1f dev=%.3f\n", tf, td); fflush(stdout); } }
        report_block(mh, fmh, g, dev, jumps, cov, &margin);
        if (margin < 3.0 && jumps == 0 && t0 - last_bad > 5.0){    // keep a bad, undisturbed capture (two alternating files)
            char fn[64]; snprintf(fn, sizeof fn, "/tmp/mask_bad_%d.iq", (int)(time(NULL) & 1));
            FILE *bf = fopen(fn, "wb");
            if (bf){ fwrite(x, 2 * sizeof(int16_t), (size_t)win_fr, bf); fclose(bf); fprintf(stderr, "bewaard: %s (marge %+.1f dB)\n", fn, margin); last_bad = t0; }
        }
        pthread_mutex_lock(&rdc.m); rdc.st[b] = B_FREE; pthread_cond_broadcast(&rdc.c); pthread_mutex_unlock(&rdc.m);
        lastseq = seqb; nproc++;
        double dt = now_s() - t0;                                   // adapt to processing time relative to real time
        if (dt > 0.85 * interval && hop < 1024) hop += 128;
        else if (dt < 0.55 * interval && hop > HOP) hop -= 128;
        (void)nf;
        if (seqb >= nwin) break;                                    // enough windows processed (window 0 was skipped; the rest of the stream is still drained)
    }
    fprintf(stderr, "stream: %ld vensters verwerkt, %ld overgeslagen (te traag), hop %d\n", nproc, nskip, hop);
    pthread_join(th, NULL);       // the reader thread runs until the end of the stream (iio_readdev stops after its -s samples) and then exits
    pclose(rdc.p);
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select A_BALANCED >/dev/null 2>&1", ch);
    if (rx == 2) sh("iio_attr -i -c ad9361-phy voltage1 gain_control_mode slow_attack >/dev/null 2>&1");
    return 0;
fail:
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select A_BALANCED >/dev/null 2>&1", ch);
    if (rx == 2) sh("iio_attr -i -c ad9361-phy voltage1 gain_control_mode slow_attack >/dev/null 2>&1");
    return 1;
}

// ============================ END-TO-END IMPULSE RESPONSE (--ir) ==========================================
// skypluto-mask --ir <tx_lo_Hz> <rx=2> <port> <seconds> <output_file>
// Precondition: IMP_CTRL[0] = 1 in the FPGA (the daemon sets it with 'J'): the RX1 channels (voltage0/1) then carry timestamps:
//   voltage0 (I) = l_clk counter (low 16 bits) at the moment of each RX sample, voltage1 (Q) = l_clk counter of the last detected I2S pulse.
// Every time voltage1 changes there is a new pulse; the RX2 samples (voltage2/3) from that moment on are FM-demodulated, aligned
// to the pulse time (Fourier shift for the l_clk resolution) and averaged. The averaged spectrum yields magnitude (relative to 1 kHz),
// phase and group delay from I2S frame -> RX2 demodulation. Constants in the result: AD9361 RX filters (~10-20 us) and the frame duration (IR_FRAME_TICKS).
#define IR_W        16384            // RX samples per window (5.33 ms = the wrap of the 16-bit counter)
#define IR_NF       65536            // FFT length (zero padding)
#define IR_SEG_S    100.0            // maximum duration of a single iio_readdev capture (s), see ir_main
#define IR_GATE_US  250.0            // half width of the time window around the impulse-response peak (us)
#define IR_FRAME_TICKS 27            // I2S word complete ~25 BCLK + 2 after the frame start (BCLK = l_clk = 12.288 MHz): used to translate back to the frame start
#define IR_LCLK     12288000.0
static void ir_fft(double *re, double *im, int n){ fft_inplace(re, im, n); }
static int ir_main(long long txlo, int rx, const char *port, double secs, const char *outf){
    if (rx != 2){ fprintf(stderr, "ir: alleen rx=2\n"); return 2; }
    int ch = rx - 1;
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select %s >/dev/null 2>&1", ch, port);
    sh("iio_attr -i -c ad9361-phy voltage%d gain_control_mode manual >/dev/null 2>&1", ch);
    sh("iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", txlo - (long long)OFFSET); usleep(60000);
    int16_t *tmp = malloc(8192 * 2 * sizeof(int16_t)); if (!tmp) return 1;
    int g = 20;
    for (int it = 0; it < 3; it++){
        sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
        if (capture(rx, 8192, tmp)) break;
        double p = meanpower(tmp, 8192);
        if (p < 5.0) break;
        int ng = (int)floor(g + 10.0 * log10(3.0e5 / p) + 0.5); if (ng < 0) ng = 0; if (ng > 60) ng = 60;
        if (ng == g) break;
        g = ng;
    }
    sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
    free(tmp);

    char c[200];
    static double Are[IR_NF], Aim[IR_NF], re[IR_W], im[IR_W], y[IR_W];
    static int16_t wi[IR_W], wq[IR_W];
    enum { CH = 8192 };
    int16_t *buf = malloc((size_t)CH * 4 * sizeof(int16_t)); if (!buf) return 1;
    int collecting = 0, pos = 0, have_prev = 0, nwin = 0, nbad = 0, nimp = 0; uint16_t prevq = 0, tau0 = 0, expect = 0;
    long total = 0;
    // The capture is done in segments of at most IR_SEG_S seconds: iio_readdev counts the number of bytes in 32 bits (4 channels x 2 bytes x 3.072 MSPS = 24.6 MB/s)
    // and hangs after ~4 GiB (~170 s). Each segment is its own iio_readdev; pulses that straddle a segment boundary are skipped.
    long remaining = (long)(FS * secs);
    while (remaining > 0){
    long frames = remaining > (long)(FS * IR_SEG_S) ? (long)(FS * IR_SEG_S) : remaining; remaining -= frames;
    snprintf(c, sizeof c, "iio_readdev -b 65536 -s %ld cf-ad9361-lpc voltage0 voltage1 voltage2 voltage3 2>/dev/null", frames);
    FILE *p = popen(c, "r");
    if (!p){ fprintf(stderr, "ir: iio_readdev mislukt\n"); break; }
    have_prev = 0; collecting = 0;
    for (;;){
        size_t got = fread(buf, 4 * sizeof(int16_t), CH, p);
        if (got == 0) break;
        for (size_t f = 0; f < got; f++){
            uint16_t ti = (uint16_t)buf[4*f], tq = (uint16_t)buf[4*f+1];
            if (!collecting){
                if (have_prev && tq != prevq){
                    nimp++;
                    tau0 = (uint16_t)(ti - tq);
                    if (tau0 < 64){ collecting = 1; pos = 0; expect = ti; }
                    else nbad++;
                }
                prevq = tq; have_prev = 1;
            }
            if (collecting){
                if (ti != expect){ nbad++; collecting = 0; continue; }       // counter does not advance exactly per sample (RX sample dropped)
                wi[pos] = buf[4*f+2]; wq[pos] = buf[4*f+3]; pos++; expect = (uint16_t)(expect + 4);
                if (pos == IR_W){
                    collecting = 0;
                    // FM demodulation: instantaneous frequency between sample k and k+1
                    double mean0 = 0.0; int nm = 1500;
                    for (int k = 0; k < IR_W - 1; k++){
                        double ar = wi[k], ai = wq[k], br = wi[k+1], bi = wq[k+1];
                        y[k] = atan2(bi*ar - br*ai, br*ar + bi*ai) * FS / (2.0 * M_PI);
                        if (k < nm) mean0 += y[k];
                    }
                    mean0 /= nm; y[IR_W-1] = mean0;
                    for (int k = 0; k < IR_W; k++){ re[k] = y[k] - mean0; im[k] = 0.0; }
                    ir_fft(re, im, IR_W);                                    // 16384 points per window (fast); the fine frequency grid follows after the gating
                    // align to the frame start of the pulse: y[k] belongs to t = (tau0 + IR_FRAME_TICKS + 4k + 2) / l_clk after the pulse
                    double t0 = (tau0 + IR_FRAME_TICKS + 2) / IR_LCLK;
                    {   // phase rotation e^{-j 2 pi f t0} via a recurrence (no trig per bin): step per bin = e^{-j 2 pi (FS/IR_W) t0}
                        double da = -2.0 * M_PI * (FS / IR_W) * t0, sr = cos(da), si = sin(da), cr = 1.0, ci = 0.0;
                        for (int m = 0; m <= IR_W / 2; m++){
                            Are[m] += re[m] * cr - im[m] * ci; Aim[m] += re[m] * ci + im[m] * cr;
                            double nr = cr * sr - ci * si; ci = cr * si + ci * sr; cr = nr;
                            if ((m & 1023) == 1023){ double nn = hypot(cr, ci); cr /= nn; ci /= nn; }    // restore the amplitude
                        }
                    }
                    nwin++;
                }
            }
        }
        total += (long)got;
    }
    pclose(p);
    }
    free(buf);
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select A_BALANCED >/dev/null 2>&1", ch);
    sh("iio_attr -i -c ad9361-phy voltage1 gain_control_mode slow_attack >/dev/null 2>&1");
    FILE *fo = fopen(outf, "w");
    if (!fo) return 1;
    if (nwin < 3){ fprintf(fo, "# fout: te weinig impulsen (venster %d, impulsen gezien %d, slecht %d, frames %ld)\n", nwin, nimp, nbad, total); fclose(fo); return 3; }
    // Time gating against noise: back to the time domain (average impulse response, aligned to the frame start), find the peak (1..3 ms), a window of
    // +-IR_GATE_US around the peak (Tukey) and back to the spectrum. The impulse response of the path is much shorter than the 5.3 ms window; the
    // remainder is noise (and the pilot residue). The time reference is preserved (no shift).
    {
        static double gre[IR_W], gim[IR_W];
        for (int m = 0; m <= IR_W / 2; m++){ gre[m] = Are[m]; gim[m] = -Aim[m]; }                        // conj(X), half spectrum
        for (int m = 1; m < IR_W / 2; m++){ gre[IR_W - m] = Are[m]; gim[IR_W - m] = Aim[m]; }            // Hermitian symmetry (conj of conj)
        ir_fft(gre, gim, IR_W);                                                                         // forward FFT of conj(X) = conj(IFFT sum)
        int n0 = (int)(0.5e-3 * FS), n1 = (int)(4.5e-3 * FS), npk = n0; double best = 0.0;
        for (int n = n0; n < n1; n++){ double v = fabs(gre[n]); if (v > best){ best = v; npk = n; } }
        int hw = (int)(IR_GATE_US * 1e-6 * FS), tap = hw / 4;
        static double hre[IR_NF], him[IR_NF];
        for (int n = 0; n < IR_NF; n++){
            int d = abs(n - npk); double w = 0.0;
            if (n < IR_W && d <= hw){ w = 1.0; if (d > hw - tap) w = 0.5 * (1.0 + cos(M_PI * (double)(d - (hw - tap)) / tap)); }
            hre[n] = n < IR_W ? gre[n] * w : 0.0; him[n] = 0.0;
        }
        ir_fft(hre, him, IR_NF);                                                                         // spectrum of the gated impulse response (gre = x*NF, up to the factor)
        for (int m = 0; m <= IR_NF / 2; m++){ Are[m] = hre[m]; Aim[m] = him[m]; }                        // FFT of the gated real time series = gated spectrum (same convention as Are/Aim)
        fprintf(stderr, "ir: piek bij %.1f us, venster +-%d us\n", npk / FS * 1e6, (int)IR_GATE_US);
    }
    // magnitude relative to the mean in 900..1100 Hz; unwrap phase; group delay = -dphi/domega
    double ref = 0.0; int nref = 0;
    for (int m = 0; m <= IR_NF / 2; m++){ double fm = m * FS / IR_NF; if (fm >= 900.0 && fm <= 1100.0){ ref += hypot(Are[m], Aim[m]); nref++; } }
    ref = nref ? ref / nref : 1.0;
    static double ph[IR_NF/2 + 1];
    double prev = 0.0, acc = 0.0;
    for (int m = 0; m <= IR_NF / 2; m++){
        double a = atan2(Aim[m], Are[m]);
        if (m > 0){ double d = a - prev; while (d > M_PI) d -= 2*M_PI; while (d < -M_PI) d += 2*M_PI; acc += d; } else acc = a;
        ph[m] = acc; prev = a;
    }
    fprintf(fo, "# impulsrespons eind-tot-eind: I2S-frame (impuls) -> RX2-demodulatie; venster %d gemiddeld, impulsen %d, slecht %d, frames %ld, referentie 1 kHz\n", nwin, nimp, nbad, total);
    fprintf(fo, "# frame-start-correctie %d l_clk-tikken (%.2f us); bevat de AD9361-RX-filtervertraging (niet te scheiden)\n", IR_FRAME_TICKS, IR_FRAME_TICKS / IR_LCLK * 1e6);
    fprintf(fo, "f_Hz,mag_dB,fase_deg,groepsvertraging_us\n");
    double df = FS / IR_NF;
    for (int m = 2; m < IR_NF / 2 - 2; m++){
        double fm = m * df; if (fm < 100.0) continue; if (fm > 25000.0) break;
        double mag = 20.0 * log10(hypot(Are[m], Aim[m]) / ref + 1e-30);
        double gd = -(ph[m+1] - ph[m-1]) / (2.0 * M_PI * 2.0 * df) * 1e6;
        fprintf(fo, "%.1f,%.3f,%.2f,%.2f\n", fm, mag, ph[m] * 180.0 / M_PI, gd);
    }
    fclose(fo);
    printf("IR klaar: %d vensters, %d impulsen, %d slecht\n", nwin, nimp, nbad);
    return 0;
}

// ============================ LO LEAKAGE / FINE SPECTRUM (--lo) ===============================================================
// skypluto-mask --lo <tx_lo_Hz> <rx=2> <port> <seconds> <prefix> [rx_gain_dB]   (with rx_gain: fixed RX gain instead of automatic)
// Captures <seconds> of RX2 IQ (the TX2 twin) and delivers per 0.5 s window: the strongest audio tone (frequency, deviation), the level of the carrier bin
// (dBc = relative to the total power = the unmodulated carrier power of the constant-envelope FM signal) and absolute levels. An FM carrier is zero at a
// Bessel null (tone f_m, deviation 2.405 x f_m): what remains in the carrier bin is then LO leakage (+ DC of the RX itself).
// Files: <prefix>_sweep.csv (per window), <prefix>_null.csv (fine spectrum +-150 kHz of the window with the deepest carrier), <prefix>_ref.csv (same, first window).
#define LO_N        32768            // FFT length: RBW = FS/LO_N = 93.75 Hz (OFFSET = bin 5376 exactly)
#define LO_WIN_S    0.5              // window (s)
#define LO_SEGS     12               // FFT segments per window (Welch, non-overlapping)
#define LO_SPAN     1600             // bins on either side of the carrier in the spectrum files (+-150 kHz)
#define LO_SEG_S    150.0            // maximum duration per iio_readdev (2 channels: 12.3 MB/s -> 4 GiB after ~350 s)
static int lo_main(long long txlo, int rx, const char *port, double secs, const char *prefix, int fixgain){
    if (rx != 2){ fprintf(stderr, "lo: alleen rx=2\n"); return 2; }
    int ch = rx - 1;
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select %s >/dev/null 2>&1", ch, port);
    sh("iio_attr -i -c ad9361-phy voltage%d gain_control_mode manual >/dev/null 2>&1", ch);
    sh("iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", txlo - (long long)OFFSET); usleep(60000);
    int16_t *tmp = malloc(8192 * 2 * sizeof(int16_t)); if (!tmp) return 1;
    int g = fixgain > 0 ? fixgain : 20;
    for (int it = 0; it < (fixgain > 0 ? 0 : 3); it++){
        sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
        if (capture(rx, 8192, tmp)) break;
        double p = meanpower(tmp, 8192); if (p < 5.0) break;
        int ng = (int)floor(g + 10.0 * log10(3.0e5 / p) + 0.5); if (ng < 0) ng = 0; if (ng > 60) ng = 60;
        if (ng == g) break;
        g = ng;
    }
    sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
    free(tmp);

    const long wf = (long)(FS * LO_WIN_S);
    int16_t *w = malloc((size_t)wf * 2 * sizeof(int16_t)); if (!w) return 1;
    static double re[LO_N], im[LO_N], pw[LO_N], hw[LO_N], P[LO_N], Pnull[LO_N], Pref[LO_N];
    for (int i = 0; i < LO_N; i++) hw[i] = 0.5 - 0.5 * cos(2.0 * M_PI * i / LO_N);
    const double binw = FS / LO_N; const int kc = (int)floor(OFFSET / binw + 0.5);
    char fn[300]; snprintf(fn, sizeof fn, "%s_sweep.csv", prefix);
    FILE *fs = fopen(fn, "w"); if (!fs){ free(w); return 1; }
    fprintf(fs, "t_s,tone_Hz,tone_dev_kHz,carrier_dBc,ptot_dB,pcarrier_dB\n");
    double best = 1e9; int nwin = 0; double t0 = now_s(); int have_ref = 0;
    long remaining = (long)(FS * secs);
    while (remaining > 0){
        long frames = remaining > (long)(FS * LO_SEG_S) ? (long)(FS * LO_SEG_S) : remaining; remaining -= frames;
        char c[200]; snprintf(c, sizeof c, "iio_readdev -b 65536 -s %ld cf-ad9361-lpc voltage2 voltage3 2>/dev/null", frames);
        FILE *p = popen(c, "r"); if (!p) break;
        for (;;){
            size_t got = fread(w, 2 * sizeof(int16_t), (size_t)wf, p);
            if (got < (size_t)wf) break;
            double tf = 0, td = 0; int tok = tone_analyze(w, wf, &tf, &td);
            for (int k = 0; k < LO_N; k++) P[k] = 0.0;
            for (int sgi = 0; sgi < LO_SEGS; sgi++){
                long o = (long)sgi * (wf / LO_SEGS - LO_N / LO_SEGS);                  // segments spread over the window
                if (o + LO_N > wf) o = wf - LO_N;
                for (int i = 0; i < LO_N; i++){ re[i] = w[2*(o+i)] * hw[i]; im[i] = w[2*(o+i)+1] * hw[i]; }
                fft_inplace(re, im, LO_N);
                for (int k = 0; k < LO_N; k++) P[k] += re[k]*re[k] + im[k]*im[k];
            }
            // total power (+-250 kHz around the carrier) and carrier bin (3 bins: the Hann main lobe of a tone on/near the bin)
            double tot = 0.0, car = 0.0; int span = (int)(250000.0 / binw);
            for (int d = -span; d <= span; d++){ int k = (kc + d) & (LO_N - 1); tot += P[k]; }
            for (int d = -1; d <= 1; d++) car += P[(kc + d) & (LO_N - 1)];
            double dbc = 10.0 * log10(car / tot + 1e-30);
            double norm = (double)LO_SEGS * LO_N * (3.0 * LO_N / 8.0);                // Parseval normalisation to mean power (ADC counts^2)
            fprintf(fs, "%.2f,%.1f,%.3f,%.2f,%.2f,%.2f\n", now_s() - t0, tok ? tf : 0.0, tok ? td : 0.0, dbc, 10.0 * log10(tot / norm + 1e-30), 10.0 * log10(car / norm + 1e-30));
            if (dbc < best){ best = dbc; for (int k = 0; k < LO_N; k++) Pnull[k] = P[k] / tot; }
            if (!have_ref){ have_ref = 1; for (int k = 0; k < LO_N; k++) Pref[k] = P[k] / tot; }
            nwin++;
        }
        pclose(p);
    }
    fclose(fs); free(w);
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select A_BALANCED >/dev/null 2>&1", ch);
    sh("iio_attr -i -c ad9361-phy voltage1 gain_control_mode slow_attack >/dev/null 2>&1");
    const char *nm[2] = {"null", "ref"}; double *src[2] = {Pnull, Pref};
    for (int v = 0; v < 2; v++){
        snprintf(fn, sizeof fn, "%s_%s.csv", prefix, nm[v]);
        FILE *fo = fopen(fn, "w"); if (!fo) continue;
        fprintf(fo, "f_rel_Hz,dBc\n");
        for (int d = -LO_SPAN; d <= LO_SPAN; d++){ int k = (kc + d) & (LO_N - 1); fprintf(fo, "%.2f,%.2f\n", d * binw, 10.0 * log10(src[v][k] + 1e-30)); }
        fclose(fo);
    }
    printf("LO klaar: %d vensters, diepste draaggolf %.1f dBc\n", nwin, best);
    return nwin >= 1 ? 0 : 3;
}

// ============================ WRITE OUT DEMODULATED MPX (--mpx) =========================================================
// skypluto-mask --mpx <tx_lo_Hz> <rx=2> <port> <seconds> <prefix>
// Repeatedly captures a 0.5 s window of RX2 IQ (the TX2 twin), FM-demodulates (instantaneous frequency from the phase step between consecutive samples, 3.072 MSPS),
// subtracts the mean carrier offset and decimates with a linear-phase FIR (351 taps, flat to ~60 kHz, >70 dB rejection from 100 kHz) to EXACTLY
// 192 kHz. Output: <prefix>.f32 = float32, deviation in kHz (frequency deviation of the carrier), windows of 96000 samples back to back;
// <prefix>_index.csv = per window: index, start time (s since start), UTC epoch (ms), strongest tone (Hz) and its deviation (kHz).
// The windows follow each other with gaps (iio_readdev start + processing, ~1.5 s per window); the tone frequency per window tells which step it belongs to.
#define MPX_WF      1536000L         // 0.5 s
#define MPX_DEC     16
#define MPX_TAPS    351
static int mpx_main(long long txlo, int rx, const char *port, double secs, const char *prefix){
    if (rx != 2){ fprintf(stderr, "mpx: alleen rx=2\n"); return 2; }
    int ch = rx - 1;
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select %s >/dev/null 2>&1", ch, port);
    sh("iio_attr -i -c ad9361-phy voltage%d gain_control_mode manual >/dev/null 2>&1", ch);
    sh("iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", txlo - (long long)OFFSET); usleep(60000);
    int16_t *tmp = malloc(8192 * 2 * sizeof(int16_t)); if (!tmp) return 1;
    int g = 20;
    for (int it = 0; it < 3; it++){
        sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
        if (capture(rx, 8192, tmp)) break;
        double p = meanpower(tmp, 8192); if (p < 5.0) break;
        int ng = (int)floor(g + 10.0 * log10(3.0e5 / p) + 0.5); if (ng < 0) ng = 0; if (ng > 60) ng = 60;
        if (ng == g) break;
        g = ng;
    }
    sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
    free(tmp);
    // FIR: Kaiser window (beta 8), cutoff frequency 80 kHz (transition 60..100 kHz), normalised to DC gain 1
    static double h[MPX_TAPS];
    {
        double fc = 80000.0 / FS, beta = 8.0, sum = 0.0;
        double i0b = 0.0; { double x = beta / 2.0, t = 1.0; i0b = 1.0; for (int k = 1; k < 40; k++){ t *= (x / k) * (x / k); i0b += t; } }
        for (int n = 0; n < MPX_TAPS; n++){
            double m = n - (MPX_TAPS - 1) / 2.0, sinc = (m == 0.0) ? 2.0 * fc : sin(2.0 * M_PI * fc * m) / (M_PI * m);
            double r = 2.0 * n / (MPX_TAPS - 1) - 1.0, a = beta * sqrt(1.0 - r * r), t = 1.0, i0 = 1.0, x = a / 2.0;
            for (int k = 1; k < 40; k++){ t *= (x / k) * (x / k); i0 += t; }
            h[n] = sinc * i0 / i0b; sum += h[n];
        }
        for (int n = 0; n < MPX_TAPS; n++) h[n] /= sum;
    }
    int16_t *w = malloc((size_t)MPX_WF * 2 * sizeof(int16_t)); float *f = malloc((size_t)MPX_WF * sizeof(float));
    if (!w || !f) return 1;
    char fn[300]; snprintf(fn, sizeof fn, "%s.f32", prefix); FILE *fo = fopen(fn, "wb");
    snprintf(fn, sizeof fn, "%s_index.csv", prefix); FILE *fi = fopen(fn, "w");
    if (!fo || !fi) return 1;
    fprintf(fi, "window,t_s,utc_ms,tone_Hz,tone_dev_kHz\n");
    double t0 = now_s(); int nwin = 0;
    while (now_s() - t0 < secs){
        double tw = now_s() - t0; struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
        long long utc = (long long)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
        if (capture(rx, MPX_WF, w)){ usleep(200000); continue; }
        double tf = 0, td = 0; int tok = tone_analyze(w, MPX_WF, &tf, &td);
        double mean = 0.0;
        for (long i = 0; i + 1 < MPX_WF; i++){
            double ar = w[2*i], ai = w[2*i+1], br = w[2*i+2], bi = w[2*i+3];
            f[i] = (float)(atan2(bi*ar - br*ai, br*ar + bi*ai) * FS / (2.0 * M_PI)); mean += f[i];
        }
        f[MPX_WF - 1] = f[MPX_WF - 2]; mean /= (MPX_WF - 1);
        long nout = (MPX_WF - MPX_TAPS) / MPX_DEC; if (nout > 96000) nout = 96000;
        for (long o = 0; o < nout; o++){
            const float *x = f + o * MPX_DEC; double acc = 0.0;
            for (int n = 0; n < MPX_TAPS; n++) acc += h[n] * (x[n] - mean);
            float v = (float)(acc / 1000.0); fwrite(&v, sizeof v, 1, fo);
        }
        for (long o = nout; o < 96000; o++){ float v = 0.0f; fwrite(&v, sizeof v, 1, fo); }
        fprintf(fi, "%d,%.2f,%lld,%.1f,%.3f\n", nwin, tw, utc, tok ? tf : 0.0, tok ? td : 0.0);
        fflush(fo); fflush(fi); nwin++;
    }
    fclose(fo); fclose(fi); free(w); free(f);
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select A_BALANCED >/dev/null 2>&1", ch);
    sh("iio_attr -i -c ad9361-phy voltage1 gain_control_mode slow_attack >/dev/null 2>&1");
    printf("MPX klaar: %d vensters\n", nwin);
    return nwin >= 1 ? 0 : 3;
}

// One-shot RX1 power measurement (when the mask stream is not running): skypluto-mask --pwr <tx_lo_Hz> <seconds>  -> 'PWR p=<counts^2, AC power> pk=<peak>'
// The RX LO is at tx_lo - OFFSET (the carrier falls at +504 kHz); the daemon sets the RX1 port and gain.
static int pwr_main(long long txlo, double secs){
    long frames = (long)(FS * secs); if (frames < 16384) frames = 16384; if (frames > 1536000L) frames = 1536000L;
    int16_t *w = malloc((size_t)frames * 2 * sizeof(int16_t)); if (!w) return 1;
    sh("iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", txlo - (long long)OFFSET);
    usleep(100000);
    if (capture(1, frames, w)){ printf("PWR error=opname\n"); free(w); return 1; }
    double s2 = 0, si = 0, sq = 0; long pk = 0;
    for (long i = 0; i < frames; i++){
        int x = w[2*i], y = w[2*i+1]; s2 += (double)x * x + (double)y * y; si += x; sq += y;
        long ax = x < 0 ? -x : x, ay = y < 0 ? -y : y; if (ax > pk) pk = ax; if (ay > pk) pk = ay;
    }
    double n = (double)frames, mi = si / n, mq = sq / n;
    printf("PWR p=%.1f pk=%ld\n", s2 / n - mi * mi - mq * mq, pk);
    free(w);
    return 0;
}

int main(int argc, char **argv){
    if (argc >= 4 && !strcmp(argv[1], "--pwr")){
        struct sched_param spx; spx.sched_priority = 0; sched_setscheduler(0, SCHED_OTHER, &spx);
        return pwr_main(atoll(argv[2]), atof(argv[3]));
    }
    if (argc >= 6 && !strcmp(argv[1], "--mpx")){
        struct sched_param spx; spx.sched_priority = 0; sched_setscheduler(0, SCHED_OTHER, &spx);
        return mpx_main(atoll(argv[2]), atoi(argv[3]), argv[4], atof(argv[5]), argc > 6 ? argv[6] : "/tmp/pluto_mpx");
    }
    if (argc >= 6 && !strcmp(argv[1], "--lo")){
        struct sched_param spx; spx.sched_priority = 0; sched_setscheduler(0, SCHED_OTHER, &spx);
        return lo_main(atoll(argv[2]), atoi(argv[3]), argv[4], atof(argv[5]), argc > 6 ? argv[6] : "/tmp/pluto_lo", argc > 7 ? atoi(argv[7]) : 0);
    }
    if (argc >= 6 && !strcmp(argv[1], "--ir")){
        struct sched_param spx; spx.sched_priority = 0; sched_setscheduler(0, SCHED_OTHER, &spx);
        return ir_main(atoll(argv[2]), atoi(argv[3]), argv[4], atof(argv[5]), argc > 6 ? argv[6] : "/tmp/pluto_ir.csv");
    }
    if (argc < 2){ fprintf(stderr, "gebruik: %s <tx_lo_Hz> [rx=1|2] [poort] [gain=0] [seconden=1.0] [stream_interval] [stream_duur]\n", argv[0]); return 2; }
    if (argc > 7 && atof(argv[6]) > 0.0){
        int rxs = atoi(argv[2]); if (rxs != 2) rxs = 1;
        struct sched_param spx; spx.sched_priority = 0; sched_setscheduler(0, SCHED_OTHER, &spx);
        return stream_main(atoll(argv[1]), rxs, argv[3], atoi(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]), argc > 8 ? atoi(argv[8]) : 0);
    }
    long long txlo = atoll(argv[1]);
    int rx = argc > 2 ? atoi(argv[2]) : 1; if (rx != 2) rx = 1;
    const char *port = argc > 3 ? argv[3] : "A_BALANCED";
    int gain = argc > 4 ? atoi(argv[4]) : 0;
    double secs = argc > 5 ? atof(argv[5]) : 1.0;
    long frames = (long)(FS * secs); if (frames < 16384) frames = 16384; if (frames > 4L*3072000) frames = 4L*3072000;

    struct sched_param sp; sp.sched_priority = 0; sched_setscheduler(0, SCHED_OTHER, &sp);
    fft_init();
    int16_t *buf = malloc((size_t)frames * 2 * sizeof(int16_t)); if (!buf){ printf("MASK error=geheugen\n"); return 1; }
    int ch = rx - 1;
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select %s >/dev/null 2>&1", ch, port);
    sh("iio_attr -i -c ad9361-phy voltage%d gain_control_mode manual >/dev/null 2>&1", ch);
    sh("iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", txlo - (long long)OFFSET); usleep(60000);

    // RX gain: automatic so that the mean power is around 3e5 (rms ~550 of 2048)
    int g = gain > 0 ? gain : 20;
    if (gain <= 0){
        for (int it = 0; it < 3; it++){
            sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
            if (capture(rx, 8192, buf)){ printf("MASK error=opname\n"); free(buf); return 1; }
            double p = meanpower(buf, 8192);
            if (p < 5.0){ printf("MASK error=geen_signaal g=%d pwr=%.1f\n", g, p); goto restore_fail; }
            double dg = 10.0 * log10(3.0e5 / p);
            int ng = (int)floor(g + dg + 0.5); if (ng < 0) ng = 0; if (ng > 60) ng = 60;
            if (ng == g) break;
            g = ng;
        }
    }
    sh("iio_attr -i -c ad9361-phy voltage%d hardwaregain %d >/dev/null 2>&1", ch, g); usleep(100000);
    if (capture(rx, frames, buf)){ printf("MASK error=opname\n"); goto restore_fail; }

    long jumps = count_jumps(buf, frames);
    { FILE *cf = fopen("/tmp/mask_cur.iq", "wb");                 // keep the main capture temporarily (deleted or renamed at the end)
      if (cf){ fwrite(buf, 2 * sizeof(int16_t), (size_t)frames, cf); fclose(cf); } }
    // deviation (LP 16 points) and signal spectrum
    double pk = 0;
    { long nd = frames < 768000 ? frames : 768000;      // deviation over the first quarter second (atan2 per sample is expensive on the ARM)
      double lp[16] = {0}; double sum = 0, mean = 0; long cnt = 0; static double *fl; fl = malloc((size_t)nd * sizeof(double));
      if (fl){
        for (long i = 0; i + 1 < nd; i++){
            double ar = buf[2*i], ai = buf[2*i+1], br = buf[2*i+2], bi = buf[2*i+3];
            double re = br*ar + bi*ai, im = bi*ar - br*ai;
            double f = atan2(im, re) * FS / (2.0 * M_PI);
            sum += f - lp[i & 15]; lp[i & 15] = f;
            if (i >= 15){ fl[cnt++] = sum / 16.0; mean += fl[cnt-1]; }
        }
        mean /= cnt ? cnt : 1;
        for (long i = 0; i < cnt; i++){ double d = fabs(fl[i] - mean); if (d > pk) pk = d; }
        free(fl);
      } }
    static double mh[N], fmh[N];
    maxhold(buf, frames, mh);

    // noise floor: carrier outside the RX filter
    sh("iio_attr -o -c ad9361-phy altvoltage0 frequency %lld >/dev/null 2>&1", txlo + 30000000LL - (long long)OFFSET); usleep(60000);
    if (capture(rx, frames, buf)){ printf("MASK error=vloer_opname\n"); goto restore_fail; }
    maxhold(buf, frames, fmh);

    // reference = highest bin within +-170 kHz of the carrier
    double ref = 0;
    for (int k = 0; k < N; k++){ double d = fabs(bin_freq(k) - OFFSET) / 1e3; if (d <= 170.0 && mh[k] > ref) ref = mh[k]; }
    if (ref <= 0){ printf("MASK error=geen_signaal g=%d\n", g); goto restore_fail; }
    double sh_sum = 0, fl_sum = 0; int nfl = 0; double margin = 1e9, at = 0;
    for (int k = 0; k < N; k++){
        double d = (bin_freq(k) - OFFSET) / 1e3, ad = fabs(d);
        if (ad < 80.0 || ad > 170.0) continue;
        double rel = 10.0 * log10(mh[k] / ref + 1e-30), m = mask_db(d) - rel;
        if (m < margin){ margin = m; at = d; }
        if (ad >= 130.0){ sh_sum += rel; fl_sum += 10.0 * log10(fmh[k] / ref + 1e-30); nfl++; }
    }
    double shoulder = nfl ? sh_sum / nfl : 0, floorv = nfl ? fl_sum / nfl : 0;
    int conclusive = (shoulder - floorv) >= 6.0;
    // Diagnostics: keep a capture with a poor margin (raw IQ, int16 I,Q,I,Q; tmpfs) so the burst can be analysed afterwards.
    // Two alternating files (rounded to the second), so the RAM does not fill up.
    // (the main capture was written to /tmp/mask_cur.iq right after capturing: 'buf' here already holds the noise-floor capture)
    if (margin < 3.0){
        char fn[64]; snprintf(fn, sizeof fn, "/tmp/mask_bad_%d.iq", (int)(time(NULL) & 1));
        if (rename("/tmp/mask_cur.iq", fn) == 0) fprintf(stderr, "bewaard: %s (marge %+.1f dB)\n", fn, margin);
    } else unlink("/tmp/mask_cur.iq");
    printf("MASK conclusive=%d margin=%+.1f at=%+.0f shoulder=%.1f floor=%.1f dev=%.1f g=%d jumps=%ld\n",
           conclusive, margin, at, shoulder, floorv, pk / 1e3, g, jumps);
    {   // spectrum and floor on the fixed grid (k = -28..+28 -> -168..+168 kHz), relative to the same peak
        char sp[GRID + 1], fp[GRID + 1];
        for (int i = 0; i < GRID; i++){
            int k = (CBIN + i - GRID / 2) & (N - 1);
            sp[i] = encdb(10.0 * log10(mh[k]  / ref + 1e-30));
            fp[i] = encdb(10.0 * log10(fmh[k] / ref + 1e-30));
        }
        sp[GRID] = fp[GRID] = 0;
        printf("SPEC %s\nFLOOR %s\n", sp, fp);
    }
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select A_BALANCED >/dev/null 2>&1", ch);
    if (rx == 2) sh("iio_attr -i -c ad9361-phy voltage1 gain_control_mode slow_attack >/dev/null 2>&1");
    free(buf); return 0;
restore_fail:
    sh("iio_attr -i -c ad9361-phy voltage%d rf_port_select A_BALANCED >/dev/null 2>&1", ch);
    if (rx == 2) sh("iio_attr -i -c ad9361-phy voltage1 gain_control_mode slow_attack >/dev/null 2>&1");
    free(buf); return 1;
}
