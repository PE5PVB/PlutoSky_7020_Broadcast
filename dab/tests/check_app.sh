#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# skypluto-dab without hardware: its simulation mode takes the audio at 48 kHz x (1 + ppm) against the transmitter clock and
# writes the phase words the FPGA would get. The phase words go through the bit-exact FPGA model (hdl/dab_ofdm_model.py) to
# IQ at 2.048 MS/s, then welle-cli decodes them. Checks: the rate control locks, no underflow, and the audio comes out.
#   tests/check_app.sh [ppm] [seconds]
#   TONE=1000 tests/check_app.sh    the built-in test tone (-T) instead of the audio file: left TONE Hz, right 1.5 x TONE Hz
# Needs welle-cli (WELLE=<path>, LD_LIBRARY_PATH for its libraries) and numpy.
set -e
PPM=${1:-80}; SECS=${2:-30}; TONE=${TONE:-0}
WELLE=${WELLE:-welle-cli}
HERE=$(cd "$(dirname "$0")/.." && pwd)
W=$(mktemp -d)
[ -n "$KEEP" ] && echo "work dir $W" || trap 'rm -rf "$W"' EXIT
python3 - "$W/in.f32" "$SECS" <<'EOF'
import sys, numpy as np
fs = 48000; t = np.arange(int(float(sys.argv[2]) * fs)) / fs
l = sum(0.08 * np.sin(2 * np.pi * f * t) for f in (310, 1020, 3150, 7030))
r = sum(0.08 * np.sin(2 * np.pi * f * t) for f in (440, 1500, 5000, 11000))
np.stack([l, r], 1).astype(np.float32).tofile(sys.argv[1])
EOF
"$HERE/build/skypluto-dab" -f "$W/in.f32" -o "$W/ph.bin" -p "$PPM" -T "$TONE" -b 96 -s "PlutoSky Test" -e "PlutoSky DAB" -d "skypluto-dab simulatie" -v
python3 - "$W/ph.bin" "$W/tx.cf32.iq" <<EOF
import sys, numpy as np
sys.path.insert(0, "$HERE/hdl")
import dab_ofdm_model as M
w = np.fromfile(sys.argv[1], np.uint32)
r, i = M.frames_2048(w)
np.stack([r, i], 1).astype(np.float32).__truediv__(8192.0).astype(np.float32).tofile(sys.argv[2])
print('IQ: %d frames' % (len(w) // (76 * 96)))
EOF
( cd "$W" && timeout 40 stdbuf -oL -eL "$WELLE" -f tx.cf32.iq -D 2>&1 | tr -s '\r' '\n' \
    | sed -e "s/\*\*\*\* Enter '.' to quit.//g" | grep -v '^[[:space:]]*$' | head -400 > welle.log ) || true
grep -E "Superframe sync succeeded|AAC|DLS" "$W/welle.log" | sort -u
python3 - "$W/PlutoSky Test.wav" "$TONE" <<'EOF'
import sys, numpy as np
fs = 48000
tone = float(sys.argv[2])
FL, FR = ((tone,), (1.5 * tone,)) if tone > 0 else ((310, 1020, 3150, 7030), (440, 1500, 5000, 11000))
raw = open(sys.argv[1], 'rb').read()[44:]
y = np.frombuffer(raw[:len(raw) // 4 * 4], np.int16).reshape(-1, 2).astype(float) / 32768
def tone_snr(seg, freqs):
    # power in the four tones (+-3 bins) against the rest, Hann window
    n = len(seg); X = np.abs(np.fft.rfft(seg * np.hanning(n))) ** 2; f = np.fft.rfftfreq(n, 1 / fs)
    m = np.zeros(len(X), bool)
    for t in freqs: m |= np.abs(f - t) < 3 * fs / n
    m2 = f > 50
    return 10 * np.log10(X[m].sum() / X[~m & m2].sum())
res = [(tone_snr(y[s:s + fs, 0], FL), tone_snr(y[s:s + fs, 1], FR))
       for s in range(2 * fs, len(y) - fs, fs)]
print('decoded %.1f s; tone SNR per second L/R (dB): %s' % (len(y) / fs, ' '.join('%.0f/%.0f' % r for r in res)))
print('PASS' if len(res) >= 5 and min(min(r) for r in res[2:]) > 30 else 'FAIL')
EOF
