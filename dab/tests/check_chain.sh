#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# End-to-end test with an independent receiver: PCM -> build/dab_tx_file -> IQ (complex float, 2.048 MS/s) ->
# welle-cli (welle.io: decodes the FIC, the DAB+ super frames, the DLS and the AAC with FAAD2) -> WAV, compared with
# the input.
#   tests/check_chain.sh <input.f32 (stereo 48 kHz float, a few seconds)> [kbit/s]
# Needs welle-cli: set WELLE=<path to welle-cli> (and LD_LIBRARY_PATH for its libraries) if it is not on the PATH.
set -e
IN=$1; KBPS=${2:-96}
WELLE=${WELLE:-welle-cli}
HERE=$(cd "$(dirname "$0")/.." && pwd)
W=$(mktemp -d)
trap 'rm -rf "$W"' EXIT
DLS="PlutoSky DAB+ ketentest: dynamic label over meerdere segmenten"
"$HERE/build/dab_tx_file" "$IN" 48000 "$KBPS" "$W/tx.cf32.iq" "PlutoSky Test" "PlutoSky DAB" "$DLS"
# welle-cli prints its prompt without a newline in front of other messages: cut the prompt out and keep the lines;
# stdout must be line buffered, or the DLS line is lost when the timeout stops welle-cli
( cd "$W" && timeout 30 stdbuf -oL -eL "$WELLE" -f tx.cf32.iq -D 2>&1 | tr -s '\r' '\n' \
    | sed -e "s/\*\*\*\* Enter '.' to quit.//g" | grep -v '^[[:space:]]*$' | head -400 > welle.log ) || true
grep -E "Superframe sync succeeded|AAC|ASCTy|AU #|DLS|X-PAD" "$W/welle.log" | sort -u
if grep -qF "DLS: $DLS" "$W/welle.log"; then echo "DLS ok"; else echo "DLS MISSING"; fi
python3 - "$IN" "$W/PlutoSky Test.wav" <<'EOF'
import sys, numpy as np
fs = 48000
x = np.fromfile(sys.argv[1], np.float32).reshape(-1, 2).astype(float)
raw = open(sys.argv[2], 'rb').read()[44:]                 # welle-cli is stopped by the timeout: the header sizes stay 0
y = np.frombuffer(raw[:len(raw) // 4 * 4], np.int16).reshape(-1, 2).astype(float) / 32768
res = []
for s in range(fs, len(y) - fs, fs):                      # every second: best position in the input, then the SNR
    seg = y[s:s + fs // 2, 0]
    c = np.fft.irfft(np.fft.rfft(x[:, 0]) * np.conj(np.fft.rfft(seg, len(x))), len(x))
    k = int(np.argmax(c[:len(x) - len(seg)]))
    ref = x[k:k + fs // 2]; d = y[s:s + fs // 2]
    res.append(10 * np.log10(np.sum(ref ** 2) / np.sum((d - ref) ** 2)))
print('decoded %.1f s; SNR per second (dB): %s' % (len(y) / fs, ' '.join('%.1f' % r for r in res)))
print('PASS' if len(y) > 2 * fs else 'FAIL: no audio')
EOF
