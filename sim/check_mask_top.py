#!/usr/bin/env python3
"""Check of sim/tb_mask_top.v against the model: run in the directory with mt_xf.txt, mt_res.txt, mt_greq.txt, mc_thr.hex, mc_cfg.txt.
  1. the samples the conditioner sent are numbered without gaps from 0
  2. the mask controller's step per block equals the model's on those samples
  3. g_req at the limiter equals the model's (the block's step, in the first 24 samples at least the previous block's)
Prints PASS or FAIL."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import maskctl_model as M

kdev = int(open('mc_cfg.txt').read().split()[0])
T = np.loadtxt('mc_thr.hex', dtype=str)
T = np.array([int(v, 16) for v in T], np.int64)
xf = np.loadtxt('mt_xf.txt', dtype=np.int64)
res = np.loadtxt('mt_res.txt', dtype=np.int64)
gq = np.loadtxt('mt_greq.txt', dtype=np.int64)
ok = True

tags = xf[:, 0]; x = xf[:, 1]
if not np.array_equal(tags, np.arange(len(tags)) % 16384):
    print('sample numbering has gaps'); ok = False
qm = M.search(M.pass_matrix(x, kdev, T))
nb = min(len(qm), len(res))
qd = res[:nb, 1]
if not np.array_equal(res[:nb, 0], np.arange(nb) % 256):
    print('result blocks out of order'); ok = False
d = np.nonzero(qd != qm[:nb])[0]
print('%d samples, %d blocks compared, %d differ; steps > 0 in %d blocks, max %d' % (len(x), nb, len(d), np.sum(qd > 0), qd.max()))
if len(d):
    ok = False; print('first differing blocks:', [(int(j), int(qd[j]), int(qm[j])) for j in d[:8]])

gm = M.block_greq(qm[:nb])
cidx = gq[:, 0]; greq = gq[:, 1]
# cidx counts mod 16384 from -258; the limiter sees sample number cidx (the first 258 pulls are the zeros in the delay lines)
n = np.arange(len(cidx)) - 258
sel = (n >= 0) & (n < len(gm))
if not np.array_equal(cidx[sel], n[sel] % 16384):
    print('cidx does not follow the samples'); ok = False
dg = np.nonzero(greq[sel] != gm[n[sel]])[0]
print('%d samples of g_req compared, %d differ' % (sel.sum(), len(dg)))
if len(dg):
    ok = False; print('first:', [(int(n[sel][j]), int(greq[sel][j]), int(gm[n[sel][j]])) for j in dg[:8]])
print('PASS' if ok else 'FAIL')
