#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# The EPG (SPI) next to the slideshow and the DLS in the PAD of the service encoded here, end to end without hardware:
#   1. build/test_spi generates an EPG (SI + PI objects, index) for a fixed time, as skypluto-ctl does on the Pluto;
#   2. skypluto-dab in simulation mode with that EPG (-E), two slides (-P) and a DLS, recording the PAD of every audio frame (-D);
#   3. tests/check_spi.py demultiplexes the X-PAD, reassembles the MOT directory carousel (types 16/17) and the slideshow
#      (12/13) and compares every object with its file, decodes the SPI objects, reports the timing;
#   4. the phase words -> FPGA model -> IQ -> welle-cli: the audio (tone S/N), the DLS and the slides (web server /slide/<sid>).
#   tests/check_spi.sh [seconds]       (default 40: two turns of the carousel)
#   SLS=<dir with 1.jpg, 2.jpg>      real slides instead of generated test files; BR=<kbit/s> (default 96)
# Needs welle-cli (WELLE=<path>), numpy, curl. Run `make` first.
set -e
SECS=${1:-40}; BR=${BR:-96}
WELLE=${WELLE:-welle-cli}
HERE=$(cd "$(dirname "$0")/.." && pwd)
B=${BUILD:-$HERE/build}
W=$(mktemp -d)
[ -n "$KEEP" ] && echo "work dir $W" || trap 'rm -rf "$W"; pkill -f "welle-cli -f $W" 2>/dev/null || true' EXIT
mkdir -p "$W/epg" "$W/sls"
printf 'eid=8FED\nlabel=PlutoSky DAB\necc=E3\nlto=2\nsvc=i2s;8F01;PlutoSky Test;%s;3A;1;aac;10\n' "$BR" > "$W/mux.conf"
"$B/test_spi" "$W/epg" 1792836000 - "$W/mux.conf" > "$W/test_spi.log" || { cat "$W/test_spi.log"; exit 1; }
tail -2 "$W/test_spi.log"
if [ -n "$SLS" ]; then cp "$SLS"/1.jpg "$SLS"/2.jpg "$W/sls/"; else
python3 - "$W/sls" <<'EOF'
# two valid baseline JPEGs, 320 x 240 in one flat grey: every 8x8 block codes DC difference 0 and end of block (one-symbol
# Huffman tables, code '0' each); a comment segment of random bytes brings them to 9 and 14 kB (several MOT segments)
import os, sys, random, struct
random.seed(1)
def seg(m, d): return b'\xff' + bytes([m]) + struct.pack('>H', len(d) + 2) + d
def jpeg(w, h, pad):
    out = b'\xff\xd8' + seg(0xFE, bytes(random.getrandbits(8) & 0x7F for _ in range(pad)))
    out += seg(0xDB, b'\x00' + bytes([1] * 64))                                   # quantisation table 0: all 1
    out += seg(0xC0, b'\x08' + struct.pack('>HH', h, w) + b'\x01\x01\x11\x00')     # baseline, 1 component
    out += seg(0xC4, b'\x00' + b'\x01' + b'\x00' * 15 + b'\x00')                  # DC table 0: one code (len 1) -> 0
    out += seg(0xC4, b'\x10' + b'\x01' + b'\x00' * 15 + b'\x00')                  # AC table 0: one code (len 1) -> EOB
    out += seg(0xDA, b'\x01\x01\x00\x00\x3f\x00')
    blocks = (w // 8) * (h // 8)
    bits = '00' * blocks
    bits += '1' * (-len(bits) % 8)
    out += bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits), 8))
    return out + b'\xff\xd9'
for n, pad in ((1, 9000), (2, 14000)):
    open(os.path.join(sys.argv[1], '%d.jpg' % n), 'wb').write(jpeg(320, 240, pad))
EOF
fi
python3 - "$W/in.f32" "$SECS" <<'EOF'
import sys, numpy as np
fs = 48000; t = np.arange(int(float(sys.argv[2]) * fs)) / fs
l = sum(0.08 * np.sin(2 * np.pi * f * t) for f in (310, 1020, 3150, 7030))
r = sum(0.08 * np.sin(2 * np.pi * f * t) for f in (440, 1500, 5000, 11000))
np.stack([l, r], 1).astype(np.float32).tofile(sys.argv[1])
EOF
echo "EPG test: DLS with the EPG and slides" > "$W/dls.txt"
"$B/skypluto-dab" -m "$W/mux.conf" -t "$W/dls.txt" -P "$W/sls" -E "$W/epg" -D "$W/pad.bin" -f "$W/in.f32" -o "$W/ph.bin" \
    -n "$SECS" -v 2> "$W/dab.log" | tail -2
grep -E "EPG|slide" "$W/dab.log" | head -8
python3 "$HERE/tests/check_spi.py" --pad "$W/pad.bin" "$W/epg" "$W/sls" 50 | tee "$W/pad.log" | grep -v "^ok   PI_\|^ok   SI"
python3 - "$W/ph.bin" "$W/tx.cf32.iq" <<EOF
import sys, numpy as np
sys.path.insert(0, "$HERE/hdl")
import dab_ofdm_model as M
w = np.fromfile(sys.argv[1], np.uint32)
r, i = M.frames_2048(w)
np.stack([r, i], 1).astype(np.float32).__truediv__(8192.0).astype(np.float32).tofile(sys.argv[2])
print('IQ: %d frames' % (len(w) // (76 * 96)))
EOF
# audio and DLS (welle-cli dump mode)
( cd "$W" && timeout $((SECS * 2 + 20)) stdbuf -oL -eL "$WELLE" -f tx.cf32.iq -D 2>&1 | tr -s '\r' '\n' \
    | sed -e "s/\*\*\*\* Enter '.' to quit.//g" | grep -v '^[[:space:]]*$' > welle.log ) || true
grep -E "Superframe sync succeeded|DLS" "$W/welle.log" | sort -u | head -5
python3 - "$W/PlutoSky Test.wav" <<'EOF' | tee "$W/audio.log"
import sys, numpy as np
fs = 48000
FL, FR = (310, 1020, 3150, 7030), (440, 1500, 5000, 11000)
raw = open(sys.argv[1], 'rb').read()[44:]
y = np.frombuffer(raw[:len(raw) // 4 * 4], np.int16).reshape(-1, 2).astype(float) / 32768
def tone_snr(seg, freqs):
    n = len(seg); X = np.abs(np.fft.rfft(seg * np.hanning(n))) ** 2; f = np.fft.rfftfreq(n, 1 / fs)
    m = np.zeros(len(X), bool)
    for t in freqs: m |= np.abs(f - t) < 3 * fs / n
    return 10 * np.log10(X[m].sum() / X[~m & (f > 50)].sum())
res = [(tone_snr(y[s:s + fs, 0], FL), tone_snr(y[s:s + fs, 1], FR)) for s in range(2 * fs, len(y) - fs, fs)]
print('audio: decoded %.1f s; worst tone S/N %.0f dB' % (len(y) / fs, min(min(r) for r in res[2:]) if len(res) > 2 else -99))
print('audio PASS' if len(res) >= 5 and min(min(r) for r in res[2:]) > 30 else 'audio FAIL')
EOF

# the slides through welle-cli's web server
PORT=$((7900 + RANDOM % 90))
( cd "$W" && timeout $((SECS * 3 + 30)) "$WELLE" -f "$W/tx.cf32.iq" -w $PORT -C 1 -P > welle_w.log 2>&1 & )
got=""
for t in $(seq 1 $((SECS * 3 / 2 + 15))); do
    sleep 2
    curl -s -o "$W/s.bin" "http://127.0.0.1:$PORT/slide/0x8f01" || continue
    for n in 1 2; do cmp -s "$W/s.bin" "$W/sls/$n.jpg" && case " $got " in *" $n "*) ;; *) got="$got $n"; echo "welle-cli: slide $n (byte-exact) after $((t * 2)) s";; esac; done
    [ "$got" = " 1 2" ] || [ "$got" = " 2 1" ] && break
done
curl -s "http://127.0.0.1:$PORT/mux.json" | python3 -c "
import json, sys
j = json.load(sys.stdin)
for s in j.get('services', []):
    print('welle-cli service', s.get('sid'), repr(s.get('label', {}).get('label') if isinstance(s.get('label'), dict) else s.get('label')),
          'DLS', repr((s.get('dls') or {}).get('label') if isinstance(s.get('dls'), dict) else s.get('dls')))" 2>/dev/null || true
pkill -f "welle-cli -f $W" 2>/dev/null || true
grep -q "^PASS" "$W/pad.log" && grep -q "audio PASS" "$W/audio.log" && [ -n "$got" ] && echo "PASS" || { echo "FAIL"; exit 1; }
