#!/usr/bin/env python3
"""Bit-exact model of the FPGA mask controller (hdl/library/skypluto_wfm/skypluto_maskctl.v) and of the way the conditioner
(skypluto_sigcond.v) applies its result.

Per block b of 64 composite samples (192 kHz, 24-bit counts, full scale 2^23) and a trial gain step q:
  x2     the composite at 384 kHz: x[n], then the halfband midpoint sum_k HB[k] (x[n-k] + x[n+1+k]) >> 15 (zeros before x[0])
  frames 8 frames of 64 samples starting at 128 b + 16 i
  FM     xs = (x2 * GTAB[q]) >> 16, inc = xs * kdev (mod 2^32), phase[t] = sum of inc before t (0 at the frame start),
         s = SIN[phase >> 20], c = SIN[(phase >> 20) + 1024], windowed (WIN * c) >> 13, FFT64 radix-2 DIT (Q15 twiddles, floor)
  test   P = re^2 + im^2 on the bins |k| = 13..28; pass when (P >> 10) <= T[|k| - 13] for every bin of every frame
  search step 0 if it passes, else binary search over 1..31 (the first passing step, 31 if none)
The conditioner: g_req per sample = gain of its block's step, in the first 24 samples of a block at most the previous block's
(those samples lie in the previous block's frames); then its look-ahead smoothing (minimum over 16-sample blocks, window of
16 blocks, average of 16 windows), applied 256 samples later.

  python3 sim/maskctl_model.py vectors <out-dir> [seconds]   test vectors for sim/tb_maskctl.v from a synthetic multiplex
"""
import os, sys
import numpy as np

N = 64; HOP = 16; FR_PER_BLK = 8; BLK = 64
NQ = 32; QSTEP_DB = 0.375
OVL = (FR_PER_BLK - 1) * HOP // 2 + N // 2 - BLK      # samples of the next block covered by a block's frames (24)
GTAB = np.array([int(round(65536 * 10 ** (-QSTEP_DB * q / 20))) for q in range(NQ)], np.int64)
SIN = np.array([int(round(8191 * np.sin(2 * np.pi * p / 4096))) for p in range(4096)], np.int64)
WIN = np.array([int(round(32767 * 0.5 * (1 - np.cos(2 * np.pi * t / N)))) for t in range(N)], np.int64)
TW_R = np.array([int(round(32767 * np.cos(2 * np.pi * k / N))) for k in range(N // 2)], np.int64)
TW_I = np.array([int(round(-32767 * np.sin(2 * np.pi * k / N))) for k in range(N // 2)], np.int64)
BITREV = np.array([int('{:06b}'.format(i)[::-1], 2) for i in range(N)])
KS = np.arange(13, 29)                       # skirt bins (6 kHz each): 78 .. 168 kHz
BINS = np.concatenate([KS, N - KS])
MASK_F = np.array([0, 74, 107.5, 124, 152.5, 170]) * 1e3
MASK_D = np.array([0, 0, -15, -30, -40, -40])

HB_L = 8
def _hb():
    # 31-tap halfband (Kaiser 7): the taps at odd offsets from the centre form the midpoint filter, sum = 0.5 in Q15
    k = np.arange(-15, 16); h = np.sinc(k / 2.0) * np.kaiser(31, 7.0); h /= h.sum()
    c = h[0::2] * 2                          # offsets -15, -13, ..., 13, 15
    c = c[:HB_L][::-1]                       # c[k] weights x[n-k] and x[n+1+k]
    return np.round(c / c.sum() * 0.5 * 2 ** 15).astype(np.int64)
HB = _hb()

def thresholds(margin_db):
    """the 16 thresholds the ARM writes for a margin (dB below the mask)"""
    pref = float(np.sum(WIN * 8191 >> 13)) ** 2          # all carrier power in one bin
    m = np.interp(KS * 6000.0, MASK_F, MASK_D) - margin_db
    return np.array([min(int(pref * 10 ** (d / 10)) >> 10, (1 << 28) - 1) for d in m], np.int64)

def x384(x):
    n = len(x)
    xp = np.concatenate([np.zeros(HB_L, np.int64), x, np.zeros(HB_L + 1, np.int64)])
    acc = np.zeros(n, np.int64)
    for k in range(HB_L):
        acc += HB[k] * (xp[HB_L - k: HB_L - k + n] + xp[HB_L + 1 + k: HB_L + 1 + k + n])
    out = np.empty(2 * n, np.int64); out[0::2] = x; out[1::2] = acc >> 15
    return out

def fft64(re, im):
    re = re[:, BITREV].copy(); im = im[:, BITREV].copy()
    m = 2
    while m <= N:
        h = m // 2; step = N // m
        for s in range(0, N, m):
            for k in range(h):
                wr = TW_R[k * step]; wi = TW_I[k * step]
                ar = re[:, s + k]; ai = im[:, s + k]; br = re[:, s + k + h]; bi = im[:, s + k + h]
                tr = (wr * br - wi * bi) >> 15; ti = (wr * bi + wi * br) >> 15
                re[:, s + k], re[:, s + k + h] = ar + tr, ar - tr
                im[:, s + k], im[:, s + k + h] = ai + ti, ai - ti
        m *= 2
    return re, im

def n_blocks(nsamp):
    return (2 * nsamp - (2 * BLK + N)) // (2 * BLK)

def pass_matrix(x, kdev, T):
    """pass[block, step] for every step"""
    x2 = x384(x)
    nblk = n_blocks(len(x))
    idx = (np.arange(nblk)[:, None, None] * (2 * BLK) + np.arange(FR_PER_BLK)[None, :, None] * HOP + np.arange(N)[None, None, :])
    F = x2[idx].reshape(-1, N)
    res = np.zeros((nblk, NQ), bool)
    TT = np.concatenate([T, T])[None, :]
    for q in range(NQ):
        xs = (F * GTAB[q]) >> 16
        inc = (xs * kdev) & 0xFFFFFFFF
        ph = np.concatenate([np.zeros((len(F), 1), np.int64), np.cumsum(inc, axis=1)[:, :-1]], axis=1) & 0xFFFFFFFF
        p = ph >> 20
        re, im = fft64((WIN * SIN[(p + 1024) & 4095]) >> 13, (WIN * SIN[p]) >> 13)
        P = re[:, BINS] ** 2 + im[:, BINS] ** 2
        res[:, q] = ((P >> 10) <= TT).all(axis=1).reshape(nblk, FR_PER_BLK).all(axis=1)
    return res

def search(pm):
    qs = np.zeros(len(pm), np.int64)
    for j, row in enumerate(pm):
        if row[0]:
            continue
        lo, hi = 1, NQ - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if row[mid]: hi = mid
            else: lo = mid + 1
        qs[j] = lo
    return qs

def block_greq(q):
    g = GTAB[q]
    gm = np.repeat(g, BLK).reshape(len(q), BLK)
    prev = np.concatenate([[65536], g[:-1]])
    gm[:, :OVL] = np.minimum(gm[:, :OVL], prev[:, None])
    return gm.reshape(-1)

def sigcond_gain(greq):
    n = len(greq) // 16
    bm = greq[:n * 16].reshape(n, 16).min(axis=1)
    W = np.array([bm[max(0, j - 15): j + 1].min() for j in range(n)])
    gsb = np.array([(W[max(0, j - 15): j + 1].sum() + 65536 * max(0, 15 - j)) >> 4 for j in range(n)])
    gout = np.full(n, 65536, np.int64); gout[:n - 15] = gsb[15:]
    return np.repeat(gout, 16)

def run(x_int, kdev, margin_db, pm=None):
    T = thresholds(margin_db)
    if pm is None:
        pm = pass_matrix(x_int, kdev, T)
    q = search(pm)
    greq = np.full(len(x_int), 65536, np.int64)
    gm = block_greq(q); greq[:len(gm)] = gm
    gs = sigcond_gain(greq)
    return (x_int[:len(gs)] * gs) >> 16, q, pm

def synth_mpx(secs, seed=1):
    """a loud synthetic multiplex (noise music-like L/R with bursts, stereo, pilot) in counts"""
    rng = np.random.default_rng(seed)
    fs = 192000; n = int(secs * fs); t = np.arange(n) / fs
    from scipy import signal
    b, a = signal.butter(4, 15000, fs=fs)
    env = np.repeat(rng.uniform(0.2, 1.0, n // 4800 + 1), 4800)[:n]
    l = signal.lfilter(b, a, rng.standard_normal(n)) * env; r = signal.lfilter(b, a, rng.standard_normal(n)) * env[::-1]
    l /= np.max(np.abs(l)); r /= np.max(np.abs(r))
    mpx = 0.9 * ((l + r) / 2 + (l - r) / 2 * np.sin(2 * np.pi * 38000 * t)) + 0.09 * np.sin(2 * np.pi * 19000 * t)
    mpx /= np.max(np.abs(mpx))
    return np.round(mpx * 0.999 * 2 ** 23).astype(np.int64)

if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == 'vectors':
        out = sys.argv[2]; secs = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
        os.makedirs(out, exist_ok=True)
        x = synth_mpx(secs)
        kdev, margin = 133, 3.0
        T = thresholds(margin)
        q = search(pass_matrix(x, kdev, T))
        np.savetxt(os.path.join(out, 'mc_x.hex'), x & 0xFFFFFF, fmt='%06x')
        np.savetxt(os.path.join(out, 'mc_thr.hex'), T, fmt='%07x')
        np.savetxt(os.path.join(out, 'mc_q.txt'), q, fmt='%d')
        with open(os.path.join(out, 'mc_cfg.txt'), 'w') as f:
            f.write('%d %d %d\n' % (kdev, len(x), len(q)))
        print('%d samples, %d blocks, steps > 0 in %d blocks, max %d' % (len(x), len(q), np.sum(q > 0), q.max()))
    else:
        print(__doc__)
