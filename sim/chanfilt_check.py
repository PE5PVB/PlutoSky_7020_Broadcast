#!/usr/bin/env python3
"""Runs sim/tb_chanfilt.v (iverilog) on an FM-modulated test signal and compares it with the bit-exact model of scripts/gen_chanfilt.py.
Usage (from the repository root, in an environment with iverilog):  python sim/chanfilt_check.py [mode ...]"""
import os, sys, subprocess, tempfile
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import gen_chanfilt as g

def test_signal(n=24000, seed=1):
    """FM at 3.072 MS/s: a noise-like composite (band-limited to 60 kHz, 75 kHz peak) + pilot, carrier level as in operation (-9 dBFS)"""
    rng = np.random.default_rng(seed)
    from scipy.signal import firwin, lfilter
    m = lfilter(firwin(201, 60e3, fs=g.FS), 1, rng.standard_normal(n + 400))[400:]
    m = 0.9 * m / np.abs(m).max() + 0.09 * np.sin(2 * np.pi * 19000 * np.arange(n) / g.FS)
    ph = 2 * np.pi * np.cumsum(75e3 * m) / g.FS
    a = 32767 * 10 ** (-9 / 20)
    return np.round(a * np.cos(ph)).astype(np.int64), np.round(a * np.sin(ph)).astype(np.int64)

def run(mode, xi, xq):
    d = tempfile.mkdtemp()
    fin, fout, vvp = os.path.join(d, 'in.txt'), os.path.join(d, 'out.txt'), os.path.join(d, 'tb.vvp')
    with open(fin, 'w') as f:
        for a, b in zip(xi, xq): f.write('%04x %04x\n' % (a & 0xFFFF, b & 0xFFFF))
    subprocess.run(['iverilog', '-g2012', '-o', vvp, os.path.join(ROOT, 'sim', 'tb_chanfilt.v'),
                    os.path.join(ROOT, 'hdl', 'library', 'skypluto_wfm', 'skypluto_chanfilt.v')], check=True)
    subprocess.run(['vvp', '-n', vvp, '+IN=' + fin, '+OUT=' + fout, '+MODE=%d' % mode], check=True, stdout=subprocess.DEVNULL)
    o = np.loadtxt(fout, dtype=np.int64)
    return o[:, 0], o[:, 1]

def best_match(hw, x, mode):
    """the model at every decimation phase; the hardware output must equal it exactly at one phase and delay"""
    best = None
    for ph in range(4):
        ref = g.model(x, mode, ph)
        for dly in range(0, 400):
            n = min(len(ref), len(hw) - dly) - 600
            if n < 1000: break
            a, b = hw[dly + 600:dly + 600 + n], ref[600:600 + n]
            err = int(np.abs(a - b).max())
            if best is None or err < best[0]: best = (err, ph, dly)
            if err == 0: return best
    return best

if __name__ == '__main__':
    modes = [int(m) for m in sys.argv[1:]] or [0, 1, 2, 3]
    xi, xq = test_signal()
    ok = True
    for mode in modes:
        yi, yq = run(mode, xi, xq)
        if mode == 0:
            # bypass: the input unchanged (one register, inside the 4-clock hold)
            e = max(int(np.abs(yi - xi[:len(yi)]).max()), int(np.abs(yq - xq[:len(yq)]).max()))
            print('mode 0 (bypass): max error %d LSB' % e); ok &= (e == 0)
            continue
        ei = best_match(yi, xi, mode); eq = best_match(yq, xq, mode)
        print('mode %d (+-%d kHz): I max error %d LSB (phase %d, delay %d)   Q max error %d LSB (phase %d, delay %d)'
              % (mode, g.BWS[mode - 1], ei[0], ei[1], ei[2], eq[0], eq[1], eq[2]))
        ok &= (ei[0] == 0 and eq[0] == 0 and ei[1:] == eq[1:])
    print('PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)
