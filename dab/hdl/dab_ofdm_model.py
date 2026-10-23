# SPDX-License-Identifier: Apache-2.0
"""Bit-exact model of the FPGA part of the DAB transmitter (skypluto_dab_ofdm.v):

  phase words (76 symbols x 96 words of 16 x 2-bit p per transmission frame)
  -> constellation: carrier a of symbol l at 45 degrees x ((2p + l) mod 8), amplitude C0 = 16384 (C1 = 11585 on the diagonals)
  -> bins: carriers +1..+768 at bins 1..768, -768..-1 at bins 1280..2047
  -> 2048-point inverse FFT, radix-2 decimation in frequency, 18-bit data, twiddles round(2^16 (cos, sin)), products rounded
     >> 16, stages 0..5 halve (rounded), stages 6..10 do not, saturation at 18 bits, output in bit-reversed order
  -> 16-bit samples (x + 2) >> 2, saturated
  -> per frame: null symbol (2656 zeros), then 76 x (504 cyclic prefix + 2048) samples   = 2.048 MS/s
  -> 3/2 polyphase interpolator: 72 taps at 6.144 MS/s (24 per phase), 18-bit coefficients (2^17, DC gain 3), output
     (acc + 2^16) >> 17 saturated to 16 bit                                                  = 3.072 MS/s

The numbers here are the specification the HDL must reproduce exactly (sim/ compares them sample by sample)."""
import numpy as np

N, K, TG, TNULL, NSYM = 2048, 1536, 504, 2656, 76
C0, C1 = 16384, 11585
DW, TWS = 18, 16
SCALED = [s < 6 for s in range(11)]
NTAP, CB = 144, 17

CONST = np.array([(C0, 0), (C1, C1), (0, C0), (-C1, C1), (-C0, 0), (-C1, -C1), (0, -C0), (C1, -C1)], np.int64)
BIN = np.array([a + 1 if a < K // 2 else a + 512 for a in range(K)])
TW = np.array([(int(round(np.cos(2 * np.pi * m / N) * 2 ** TWS)), int(round(np.sin(2 * np.pi * m / N) * 2 ** TWS))) for m in range(N // 2)], np.int64)
BITREV = np.array([int(format(n, '011b')[::-1], 2) for n in range(N)])

def sat(v, bits):
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return np.clip(v, lo, hi)

def interp_coefs():
    from scipy.signal import remez
    # passband to the outermost carriers (768 kHz, ripple 0.1 dB), stopband from 900 kHz: besides the images this also takes the
    # sidelobes of the rectangular OFDM symbols (the 'shoulders': -47 dB at 850 kHz, -73 dB at 970 kHz instead of -32 / -38 dB)
    h = remez(NTAP, [0, 768e3, 900e3, 3.072e6], [1, 0], weight=[1, 20], fs=6.144e6)
    return np.round(h * 3 * 2 ** CB).astype(np.int64)
HI = interp_coefs()

_STAGE = []
for _s in range(11):
    _h = N >> (_s + 1)
    _i = np.array([x for x in range(N) if not (x & _h)])
    _STAGE.append((_i, _i + _h, (_i & (_h - 1)) << _s))

def ifft_fixed(re, im):
    """re, im: int64 arrays of 2048 (natural bin order). Returns the time samples in natural order (18-bit)."""
    re = re.copy(); im = im.copy()
    for s in range(11):
        i, j, m = _STAGE[s]
        ar, ai, br, bi = re[i], im[i], re[j], im[j]
        sr, si = ar + br, ai + bi
        dr, di = ar - br, ai - bi
        wr, wi = TW[m, 0], TW[m, 1]
        tr = (dr * wr - di * wi + (1 << (TWS - 1))) >> TWS
        ti = (dr * wi + di * wr + (1 << (TWS - 1))) >> TWS
        if SCALED[s]:
            sr, si, tr, ti = (sr + 1) >> 1, (si + 1) >> 1, (tr + 1) >> 1, (ti + 1) >> 1
        re[i], im[i], re[j], im[j] = sat(sr, DW), sat(si, DW), sat(tr, DW), sat(ti, DW)
    return re[BITREV], im[BITREV]

def symbol_samples(words, l):
    """words: 96 uint32 of one symbol, l: symbol index 0..75 -> 2552 complex int16 samples (CP + symbol)"""
    p = np.array([(int(words[a // 16]) >> (2 * (a % 16))) & 3 for a in range(K)])
    q = (2 * p + l) & 7
    re = np.zeros(N, np.int64); im = np.zeros(N, np.int64)
    re[BIN] = CONST[q, 0]; im[BIN] = CONST[q, 1]
    tr, ti = ifft_fixed(re, im)
    xr, xi = sat((tr + 2) >> 2, 16), sat((ti + 2) >> 2, 16)
    xr = np.concatenate([xr[N - TG:], xr]); xi = np.concatenate([xi[N - TG:], xi])
    return xr, xi

def frames_2048(phase_words):
    """all transmission frames: int16 I/Q at 2.048 MS/s"""
    nfr = len(phase_words) // (NSYM * 96)
    out_r, out_i = [], []
    for f in range(nfr):
        w = phase_words[f * NSYM * 96:(f + 1) * NSYM * 96]
        out_r.append(np.zeros(TNULL, np.int64)); out_i.append(np.zeros(TNULL, np.int64))
        for l in range(NSYM):
            r, i = symbol_samples(w[l * 96:(l + 1) * 96], l)
            out_r.append(r); out_i.append(i)
    return np.concatenate(out_r), np.concatenate(out_i)

def interp_3_2(xr, xi):
    """2.048 -> 3.072 MS/s: y[m] = sum_t h[ph + 3t] x[(2m - ph)/3 - t], ph = 2m mod 3 (x[n] = 0 for n < 0)"""
    n_out = len(xr) * 3 // 2
    m = np.arange(n_out); ph = (2 * m) % 3; idx = (2 * m - ph) // 3
    yr = np.zeros(n_out, np.int64); yi = np.zeros(n_out, np.int64)
    for p in range(3):
        taps = HI[p::3]
        cr = np.convolve(xr, taps)[:len(xr)]; ci = np.convolve(xi, taps)[:len(xi)]
        sel = ph == p
        yr[sel] = (cr[idx[sel]] + (1 << (CB - 1))) >> CB
        yi[sel] = (ci[idx[sel]] + (1 << (CB - 1))) >> CB
    return sat(yr, 16), sat(yi, 16)
