#!/usr/bin/env python3
"""Generate the coefficients of the x4 polyphase interpolation FIR (hdl/library/skypluto_wfm/data/interp_coefs.mem).

The FIR runs at 768.75 kHz (4 x 192.1875 kHz) and is followed by a fractional LINEAR interpolation to l_clk, whose response is sinc^2(f/768.75 kHz) (-0.28 dB at 76 kHz).
The prototype is designed so that the TWO STAGES TOGETHER are flat: the passband target is the inverse of that droop (1/sinc^2), the stopband (116 kHz ... 384 kHz, the images
of the 192 kHz stream) is minimised in a weighted least-squares / iteratively reweighted sense. 96 taps, symmetric, DC gain 4 (each of the 4 branches sums to exactly 2^15).

    python gen_interp_coefs.py [output.mem]        needs numpy only
The file holds 96 words of 5 hex digits (18-bit two's complement, branch-major: word b*24+k is tap 4k+b of the prototype).
"""
import sys
import numpy as np

FS = 768750.0                  # rate of the FIR output
NT = 96                        # taps (4 branches x 24)
NB = 4
SCALE = 1 << 15                # each branch sums to 2^15 (COEF_SHIFT = 15 in skypluto_interp.v)
FPASS, FSTOP = 76e3, 116e3     # the multiplex ends at 76 kHz (RDS2); the first image of the 192 kHz stream starts at 192.1875 - 76 = 116 kHz


def droop_inverse(f):
    return 1.0 / np.sinc(f / FS) ** 2


def basis(f):
    """Amplitude response of an even-length symmetric FIR (type II) for the 48 independent taps."""
    n2 = NT // 2
    k = np.arange(n2)
    return 2.0 * np.cos(np.outer(2 * np.pi * f / FS, (n2 - 0.5 - k)))


def design(wp=1.0, iterations=40, nodes=6000):
    fp = np.linspace(0, FPASS, nodes // 3)
    fs = np.linspace(FSTOP, FS / 2, 2 * nodes // 3)
    f = np.concatenate([fp, fs])
    target = np.concatenate([droop_inverse(fp), np.zeros_like(fs)])
    a = basis(f)
    w = np.concatenate([np.full_like(fp, wp), np.ones_like(fs)])
    tol = np.concatenate([np.full_like(fp, 3e-4), np.full_like(fs, 1e-4)])
    for _ in range(iterations):
        sol, *_ = np.linalg.lstsq(a * np.sqrt(w)[:, None], target * np.sqrt(w), rcond=None)
        err = np.abs(a @ sol - target)
        w = np.maximum(w * (1.0 + 0.5 * err / tol) ** 0.5, 1e-9)       # towards equal ripple (minimax)
        w /= w.max()
    return np.concatenate([sol, sol[::-1]])


def quantise(h):
    """Integer coefficients: symmetric, every branch sums to exactly SCALE."""
    c = np.rint(h / h.sum() * NB * SCALE).astype(int)
    for b in range(NB // 2):                   # branch b and its mirror image NB-1-b carry the same taps in reverse order
        for bb in (b, NB - 1 - b):
            d = SCALE - c[bb::NB].sum()
            idx = np.argmax(np.abs(c[bb::NB]))  # put the rounding remainder on the largest tap
            c[bb + NB * idx] += d
    # restore exact symmetry (the remainder went to mirrored taps in mirrored branches: check)
    assert np.array_equal(c, c[::-1]), "prototype is not symmetric after quantisation"
    return c


def response(c, f):
    n = np.arange(len(c))
    h = c / float(NB * SCALE)
    return np.array([abs(np.sum(h * np.exp(-2j * np.pi * ff / FS * n))) for ff in f])


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "interp_coefs.mem"
    c = quantise(design())
    assert abs(c).max() < (1 << 17), "coefficient does not fit 18 bit"
    db = lambda x: 20 * np.log10(np.maximum(x, 1e-12))
    fp = np.linspace(1e3, FPASS, 400)
    tot = response(c, fp) * np.sinc(fp / FS) ** 2
    fs = np.linspace(FSTOP, 380e3, 4000)
    print("passband 0-76 kHz, FIR x linear stage: %.4f .. %.4f dB" % (db(tot).min(), db(tot).max()))
    print("stopband 116-380 kHz worst: %.1f dB; max |coef| %d; branch sums %s" % (db(response(c, fs)).max(), abs(c).max(), [int(c[b::NB].sum()) for b in range(NB)]))
    with open(out, "w", newline="\n") as f:
        for b in range(NB):
            for k in range(NT // NB):
                f.write("%05X\n" % (int(c[NB * k + b]) & 0xFFFFF))
    print("written", out)


if __name__ == "__main__":
    main()
