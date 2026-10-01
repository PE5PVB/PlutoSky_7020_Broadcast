#!/usr/bin/env python3
"""Computes the linearity of the DIGITAL audio path of the Pluto (I2S composite -> modulator input) from the real FIR coefficients.

Path (see hdl/library/skypluto_wfm):
  1. FIFO + conditioner: pure delay (256 samples @ 192.1875 kHz) and gain 1.0 as long as the limiter does not act (bit-exact)
  2. interpolator stage 1: x4 polyphase FIR, 96 taps (linear phase, symmetric) at 768.75 kHz
  3. interpolator stage 2: linear interpolation 768.75 kHz -> l_clk (kernel = triangle of 2 samples)
  4. FM modulator: phase accumulator (ideal: frequency ~ composite; integrator + ideal demodulator cancel each other)
  5. AD9361 TX (rf_bandwidth 18 MHz, FIR off, HB interpolators): flat within +-100 kHz (not computed; see docs)
Output: CSV with f, magnitude (dB), group delay (us) per stage and total.
Usage: python scripts/pluto_audio_response.py [out.csv]
"""
import sys
import numpy as np

COEF = 'hdl/library/skypluto_wfm/data/interp_coefs.mem'
FS_IN = 192187.5                 # I2S sample rate
FS1 = 4 * FS_IN                  # 768.75 kHz (stage 1 output)
COND_DELAY = 256 / FS_IN         # conditioner delay line (s)
FIFO_DELAY = 0.5 * 32 / FS_IN    # CDC FIFO, on average half full (s); max 32 words = 0.167 ms


def load_prototype():
    v = [int(l.strip(), 16) & 0x3FFFF for l in open(COEF) if l.strip()]
    v = np.array([x - (1 << 18) if x >= (1 << 17) else x for x in v], float)
    return v.reshape(4, 24).T.reshape(-1)          # n = 4*k + b (symmetric around 47.5)


def resp(f, h, fs):
    n = np.arange(len(h))
    H = np.array([np.sum(h * np.exp(-2j * np.pi * fi * n / fs)) for fi in f])
    return H / np.sum(h)


def main(out):
    h = load_prototype()
    f = np.unique(np.concatenate([np.geomspace(20, 100e3, 300), np.arange(1000, 20001, 1000.0), [19000, 38000, 57000, 76000]]))
    H1 = resp(f, h, FS1)
    x = f / FS1
    H2 = (np.sinc(x) ** 2) * np.exp(-2j * np.pi * f * (1.0 / FS1))       # linear interpolation: |sinc^2|, on average 1 sample delay
    H = H1 * H2
    ph = np.unwrap(np.angle(H))
    gd = -np.gradient(ph, 2 * np.pi * f)                                  # s
    gd_fir = -np.gradient(np.unwrap(np.angle(H1)), 2 * np.pi * f)
    tot_delay = gd + COND_DELAY + FIFO_DELAY
    with open(out, 'w') as fo:
        fo.write('f_Hz,mag_dB_fir,mag_dB_interp,mag_dB_totaal,fase_deg_totaal,groepsvertraging_us_filters,groepsvertraging_us_totaal_incl_cond_fifo\n')
        for i, fi in enumerate(f):
            fo.write('%.2f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f\n' % (
                fi, 20 * np.log10(abs(H1[i])), 20 * np.log10(abs(H2[i])), 20 * np.log10(abs(H[i])),
                np.degrees(ph[i]), gd[i] * 1e6, tot_delay[i] * 1e6))
    for fi in (200, 1000, 5000, 10000, 15000, 19000, 20000, 38000, 57000, 76000):
        i = int(np.argmin(abs(f - fi)))
        print('%6d Hz  fir %+.4f dB  interp %+.4f dB  totaal %+.4f dB  gd %.2f us' % (fi, 20 * np.log10(abs(H1[i])), 20 * np.log10(abs(H2[i])), 20 * np.log10(abs(H[i])), gd[i] * 1e6))
    print('vertraging: conditioner %.1f us, FIFO ~%.1f us (max %.1f), FIR %.1f us' % (COND_DELAY * 1e6, FIFO_DELAY * 1e6, 32 / FS_IN * 1e6, 47.5 / FS1 * 1e6))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'docs/measurements/pluto_audio_path.csv')
