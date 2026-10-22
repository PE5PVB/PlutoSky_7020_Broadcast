# SPDX-License-Identifier: Apache-2.0
"""Runs tb_dab_ofdm (iverilog) on the first frames of a phase word file and compares with dab_ofdm_model.py, sample by sample:
the 2.048 MS/s stream (after the leading zeros the HDL plays before the first frame) and the 3.072 MS/s output (the model's
interpolator run on the HDL's own 2.048 MS/s stream, any delay).
  python3 check_hdl.py <phase words .bin> [frames]"""
import os, sys, subprocess, tempfile
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dab_ofdm_model as M

ph = np.fromfile(sys.argv[1], np.uint32)
nfr = int(sys.argv[2]) if len(sys.argv) > 2 else 1
w = ph[:nfr * M.NSYM * 96]
d = tempfile.mkdtemp()
open(os.path.join(d, 'w.hex'), 'w').write('\n'.join('%08x' % v for v in w) + '\n')
nclk = int((nfr * 196608 + 4000) * 6)
subprocess.run(['iverilog', '-g2012', '-I', HERE, '-o', os.path.join(d, 'tb.vvp'), os.path.join(HERE, 'tb_dab_ofdm.v'),
                os.path.join(HERE, 'skypluto_dab_ofdm.v')], check=True)
subprocess.run(['vvp', '-n', os.path.join(d, 'tb.vvp'), '+WORDS=' + os.path.join(d, 'w.hex'), '+X=' + os.path.join(d, 'x.txt'),
                '+Y=' + os.path.join(d, 'y.txt'), '+NCLK=%d' % nclk], check=True)
hx = np.loadtxt(os.path.join(d, 'x.txt'), dtype=np.int64)
hy = np.loadtxt(os.path.join(d, 'y.txt'), dtype=np.int64)
mr, mi = M.frames_2048(w)
# the HDL plays zeros until the first symbol is ready, then the frame (which starts with its null symbol)
nz = 0
while nz < len(hx) and hx[nz, 0] == 0 and hx[nz, 1] == 0: nz += 1
start = nz - M.TNULL                                  # the frame's first sample (the null symbol is zeros as well)
seg = hx[start:start + len(mr)]
n = min(len(seg), len(mr))
err_x = int(np.abs(seg[:n, 0] - mr[:n]).max()) if n else -1
err_x = max(err_x, int(np.abs(seg[:n, 1] - mi[:n]).max())) if n else -1
print('2.048 MS/s: %d HDL samples, frame from sample %d, %d compared, max error %d LSB' % (len(hx), start, n, err_x))
# 3.072 MS/s: the model interpolator on the HDL's own 2.048 MS/s stream
yr, yi = M.interp_3_2(hx[:, 0], hx[:, 1])
best = None
for lag in range(0, 200):
    a = hy[lag:lag + 20000]; b_r = yr[:len(a)]; b_i = yi[:len(a)]
    e = int(max(np.abs(a[:, 0] - b_r).max(), np.abs(a[:, 1] - b_i).max()))
    if best is None or e < best[0]: best = (e, lag)
    if e == 0: break
lag = best[1]; a = hy[lag:]; m = min(len(a), len(yr))
err_y = int(max(np.abs(a[:m, 0] - yr[:m]).max(), np.abs(a[:m, 1] - yi[:m]).max()))
print('3.072 MS/s: lag %d, %d compared, max error %d LSB' % (lag, m, err_y))
print('PASS' if err_x == 0 and err_y == 0 and n == len(mr) else 'FAIL')
