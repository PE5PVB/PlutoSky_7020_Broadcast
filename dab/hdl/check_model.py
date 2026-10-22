# SPDX-License-Identifier: Apache-2.0
"""Checks the FPGA model (dab_ofdm_model.py) against the floating-point C modulator and prepares IQ files for welle-cli.
  python check_model.py <phase words file> <C reference .cf32.iq> <out dir>
Writes <out>/model2048.cf32.iq (the model at 2.048 MS/s) and <out>/model3072to2048.cf32.iq (the 3.072 MS/s output brought
back to 2.048 MS/s by an independent resampler), so both can be fed to welle-cli."""
import sys, os
import numpy as np
from scipy.signal import resample_poly
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dab_ofdm_model as M

ph = np.fromfile(sys.argv[1], np.uint32)
ref = np.fromfile(sys.argv[2], np.complex64)
out = sys.argv[3]
nfr = min(len(ph) // (M.NSYM * 96), len(ref) // 196608, 20)
ph = ph[:nfr * M.NSYM * 96]; ref = ref[:nfr * 196608].astype(np.complex128)
xr, xi = M.frames_2048(ph)
x = xr + 1j * xi
assert len(x) == len(ref), (len(x), len(ref))
g = np.vdot(x, ref) / np.vdot(x, x)
err = ref - g * x
mer = 10 * np.log10(np.sum(np.abs(ref) ** 2) / np.sum(np.abs(err) ** 2))
rms = np.sqrt(np.mean(np.abs(x[x != 0]) ** 2))
print('%d frames; model vs floating point: MER %.1f dB, phase %.2f deg; model level: rms %.0f (%.1f dBFS), peak %d (%.1f dBFS), clipped %d'
      % (nfr, mer, np.degrees(np.angle(g)), rms, 20 * np.log10(rms / 32768), max(np.abs(xr).max(), np.abs(xi).max()),
         20 * np.log10(max(np.abs(xr).max(), np.abs(xi).max()) / 32768), int(np.sum((np.abs(xr) == 32767) | (np.abs(xi) == 32767)))))
(x / 32768).astype(np.complex64).tofile(os.path.join(out, 'model2048.cf32.iq'))
yr, yi = M.interp_3_2(xr, xi)
y = (yr + 1j * yi) / 32768
f = np.fft.fftshift(np.fft.fftfreq(8192, 1 / 3.072e6)); P = np.zeros(8192)
for s in range(0, len(y) - 8192, 8192): P += np.abs(np.fft.fftshift(np.fft.fft(y[s:s + 8192] * np.hanning(8192)))) ** 2
P = 10 * np.log10(P / P.max())
for lo, hi in ((0, 700e3), (800e3, 900e3), (970e3, 1100e3), (1280e3, 1536e3)):
    sel = (np.abs(f) >= lo) & (np.abs(f) < hi); print('  3.072 MS/s spectrum %4.0f..%4.0f kHz: max %.1f dB' % (lo / 1e3, hi / 1e3, P[sel].max()))
z = resample_poly(y, 2, 3)
z.astype(np.complex64).tofile(os.path.join(out, 'model3072to2048.cf32.iq'))
np.save(os.path.join(out, 'model_out3072.npy'), np.stack([yr, yi]).astype(np.int16))
print('written', out)
