#!/usr/bin/env python3
"""Convert the MPX recording from the Pluto (`Z <secs>`, /tmp/pluto_mpx.f32 + _index.csv) to a NumPy file.

Usage:  python scripts/mpx_to_npz.py pluto_mpx.f32 pluto_mpx_index.csv out.npz
Output (np.load): x[window, 96000] float32 = demodulated MPX as frequency deviation in kHz (sampled at 192 kHz), fs=192000,
t_s, utc_ms, tone_Hz, tone_dev_kHz per window. Each window is 0.5 s contiguous; there are gaps (~1 s) between the windows.
"""
import sys
import numpy as np

f32, idx, out = sys.argv[1:4]
x = np.fromfile(f32, dtype='<f4')
n = len(x) // 96000
x = x[:n * 96000].reshape(n, 96000)
rows = np.genfromtxt(idx, delimiter=',', skip_header=1, dtype=float)
rows = np.atleast_2d(rows)[:n]
np.savez(out, x=x, fs=192000.0, t_s=rows[:, 1], utc_ms=rows[:, 2].astype(np.int64), tone_Hz=rows[:, 3], tone_dev_kHz=rows[:, 4])
print('%d vensters -> %s' % (n, out))
