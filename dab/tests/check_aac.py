# SPDX-License-Identifier: Apache-2.0
"""Encodes test signals with build/enc_file and decodes every access unit with FFmpeg's AAC decoder (PyAV), checks the
super frames (RS, fire code, AU CRCs) and compares the decoded audio with the input (gain, delay, SNR).
Usage: python check_aac.py <build dir> <encoder command prefix, e.g. "wsl -d Ubuntu-24.04 -- ">"""
import os, sys, subprocess, shlex
import numpy as np
import av
import check_sf

FS = {1: 48000, 0: 32000}

def signal(kind, fs, sec=4.0):
    t = np.arange(int(sec * fs)) / fs
    if kind == 'sines':                        # L: 1 kHz -6 dBFS, R: 3.15 kHz -12 dBFS
        return np.stack([0.5 * np.sin(2 * np.pi * 1000 * t), 0.25 * np.sin(2 * np.pi * 3150 * t)], 1)
    if kind == 'multitone':                    # a chord with harmonics, slow level changes, partly correlated channels
        x = sum(0.08 / (k + 1) * np.sin(2 * np.pi * f0 * (k + 1) * t + k) for f0 in (220, 277.2, 329.6) for k in range(12))
        env = 0.6 + 0.4 * np.sin(2 * np.pi * 0.5 * t)
        return np.stack([x * env, 0.8 * x * env + 0.05 * np.sin(2 * np.pi * 5000 * t)], 1)
    if kind == 'clicks':                       # castanet-like bursts on a near-silent background (pre-echo test)
        rng = np.random.default_rng(2); x = np.zeros((len(t), 2)) + 1e-4 * np.sin(2 * np.pi * 440 * t)[:, None]
        for t0 in onsets(fs, sec):
            n = np.arange(int(0.06 * fs)); b = rng.standard_normal((len(n), 2)) * np.exp(-n / (0.008 * fs))[:, None]
            x[t0:t0 + len(n)] += 0.5 * b / np.abs(b).max()
        return x
    if kind == 'noise':                        # pink-ish noise, uncorrelated
        rng = np.random.default_rng(1)
        w = rng.standard_normal((len(t), 2))
        X = np.fft.rfft(w, axis=0); f = np.fft.rfftfreq(len(t), 1 / fs); X[1:] /= np.sqrt(f[1:, None] / 100.0); X[0] = 0
        y = np.fft.irfft(X, n=len(t), axis=0); return 0.25 * y / np.abs(y).max()
    raise ValueError(kind)

def onsets(fs, sec):
    return [int((0.3 + 0.37 * i) * fs) for i in range(int((sec - 0.5) / 0.37))]

def pre_echo(x, y, d, fs, sec):
    """decoded energy in the 2..20 ms before each onset, relative to the burst energy (dB, median over the onsets)"""
    r = []
    for t0 in onsets(fs, sec):
        a, b = t0 - int(0.020 * fs), t0 - int(0.002 * fs)
        if a < 0 or t0 + d + int(0.03 * fs) > len(y): continue
        pre = np.sum(y[a + d:b + d] ** 2); burst = np.sum(x[t0:t0 + int(0.03 * fs)] ** 2)
        r.append(10 * np.log10(pre / burst + 1e-20))
    return float(np.median(r)) if r else float('nan')

def decode(prefix, ch):
    asc = open(prefix + '.asc', 'rb').read()
    ctx = av.CodecContext.create('aac', 'r'); ctx.extradata = asc
    raw = open(prefix + '.aus', 'rb').read(); p = 0; out = []; errors = 0
    while p < len(raw):
        l = raw[p] << 8 | raw[p + 1]; au = raw[p + 2:p + 2 + l]; p += 2 + l
        try:
            for fr in ctx.decode(av.Packet(au)):
                a = fr.to_ndarray()                # planar float: (channels, samples)
                out.append(a.T if a.shape[0] == ch else a.reshape(-1, ch))
        except av.error.FFmpegError as e:
            errors += 1
    return (np.concatenate(out) if out else np.zeros((0, ch))), errors

def compare(x, y):
    # delay by cross-correlation of the first channel, then the best gain and the SNR
    n = min(len(x), len(y)); a, b = x[:n, 0], y[:n, 0]
    c = np.fft.irfft(np.fft.rfft(b, 2 * n) * np.conj(np.fft.rfft(a, 2 * n)))
    d = 1920                                   # the encoder's fixed delay: two frames (correlation is ambiguous for tones)
    yy = y[d:d + n]; xx = x[:len(yy)]
    s = slice(len(xx) // 8, len(xx) - len(xx) // 8)
    g = np.sum(xx[s] * yy[s]) / np.sum(xx[s] ** 2)
    snr = [10 * np.log10(np.sum(xx[s, k] ** 2) / np.sum((yy[s, k] / g - xx[s, k]) ** 2)) for k in range(x.shape[1])]
    return d, g, snr

def main():
    build, run = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else ''
    ok = True
    for dac, kbps, kind, env in ((1, 96, 'sines', ''), (1, 96, 'multitone', ''), (1, 96, 'noise', ''), (0, 80, 'multitone', ''),
                                 (1, 128, 'noise', ''), (1, 96, 'sines', 'AACENC_FORCESHORT=1 '), (1, 96, 'multitone', 'AACENC_FORCESHORT=1 '), (1, 96, 'clicks', 'AACENC_NOSHORT=1 '), (1, 96, 'clicks', ''), (0, 80, 'clicks', '')):
        fs = FS[dac]; x = signal(kind, fs).astype(np.float32)
        base = os.path.join(build, 'aac_%s_%d_%d%s' % (kind, dac, kbps, ('_long' if 'NOSHORT' in env else '_short') if env else ''))
        x.tofile(base + '.f32')
        cmd = run + ('env ' + env if env else '') + 'build/enc_file %s 2 %d %d %s' % (base.replace('\\', '/') + '.f32', dac, kbps, base.replace('\\', '/'))
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if r.returncode != 0: print('ENCODER FAILED', kind, r.stdout, r.stderr); ok = False; continue
        # super frame structure
        sf = open(base + '.sf', 'rb').read(); s = kbps // 8; nsf = len(sf) // (120 * s)
        tmp = os.path.join(build, 'sf_%d_%d_0.bin' % (kbps, dac))
        bad = 0
        for i in range(nsf):
            open(tmp, 'wb').write(sf[i * 120 * s:(i + 1) * 120 * s])
            try:
                cw_ok = all(check_sf.RS.check(bytes(sf[i * 120 * s + s * k + rr] for k in range(120)))[0] for rr in range(s))
                assert cw_ok
            except AssertionError:
                bad += 1
        y, errors = decode(base, 2)
        d, g, snr = compare(x.astype(float), y)
        extra = ''
        if kind == 'clicks': extra = ', pre-echo %.1f dB%s' % (pre_echo(x.astype(float), y, d, fs, 4.0), ' (long windows only)' if 'NOSHORT' in env else '')
        print('%-9s %2d kHz %3d kbit/s: %d super frames (RS %s), decoder errors %d, delay %d, gain %.4f, SNR L %.1f dB R %.1f dB%s'
              % (kind + ('*' if 'FORCE' in env else ''), fs // 1000, kbps, nsf, 'ok' if not bad else '%d bad' % bad, errors, d, g, snr[0], snr[1], extra))
        ok &= (errors == 0 and bad == 0)
        (y / max(1.0, np.abs(y).max())).astype(np.float32).tofile(base + '.dec.f32')
    print('PASS (decodes without errors)' if ok else 'FAIL')

if __name__ == '__main__':
    main()
