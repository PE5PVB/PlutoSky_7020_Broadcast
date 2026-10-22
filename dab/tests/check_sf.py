# SPDX-License-Identifier: Apache-2.0
"""Independent check of the super frames written by test_sf: Reed-Solomon with the reedsolo library (and that it corrects
errors), the header fields, the fire code and every AU's CRC, and that the AUs come back unchanged.
Usage: python check_sf.py <dir>"""
import os, sys, glob, random
import reedsolo

RS = reedsolo.RSCodec(nsym=10, nsize=255, fcr=0, prim=0x11D, generator=2, c_exp=8)

def crc16(data, poly, init):
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def check(fn):
    rate, dac, sbr = map(int, os.path.basename(fn)[3:-4].split('_'))
    sf = bytearray(open(fn, 'rb').read()); s = rate // 8
    assert len(sf) == 120 * s, 'length'
    # RS: every code word r = bytes r, r+s, ... (110 data + 10 parity) must be a valid code word
    for r in range(s):
        cw = bytes(sf[s * k + r] for k in range(120))
        assert RS.check(cw)[0], 'RS code word %d invalid' % r
    # and must correct 5 byte errors
    cw = bytearray(sf[s * k] for k in range(120)); orig = bytes(cw)
    for p in random.sample(range(120), 5): cw[p] ^= random.randint(1, 255)
    dec = RS.decode(bytes(cw))[0]
    assert bytes(dec) == orig[:110], 'RS correction'
    # header
    assert crc16(sf[2:11], 0x782F, 0) == (sf[0] << 8 | sf[1]), 'fire code'
    hb = sf[2]
    assert (hb >> 7) == 0 and ((hb >> 6) & 1) == dac and ((hb >> 5) & 1) == sbr and ((hb >> 4) & 1) == 1 and ((hb >> 3) & 1) == 0 and (hb & 7) == 0, 'flags'
    n = (3 if dac else 2) if sbr else (6 if dac else 4)
    bits = int.from_bytes(sf[3:3 + 8], 'big'); starts = []
    for i in range(n - 1): starts.append((bits >> (64 - 12 * (i + 1))) & 0xFFF)
    hdr =(24 + 12 * (n - 1) + (4 if (12 * (n - 1)) % 8 else 0)) // 8
    starts = [hdr] + starts + [110 * s]
    aus = []
    for i in range(n):
        a, e = starts[i], starts[i + 1]
        body, crc = sf[a:e - 2], sf[e - 2] << 8 | sf[e - 1]
        assert (crc16(body, 0x1021, 0xFFFF) ^ 0xFFFF) == crc, 'AU %d CRC' % i
        aus.append(bytes(body))
    # the AUs must be the ones that went in
    raw = open(fn[:-4] + '.aus', 'rb').read(); p = 0; ref = []
    while p < len(raw):
        l = raw[p] << 8 | raw[p + 1]; ref.append(raw[p + 2:p + 2 + l]); p += 2 + l
    assert ref == aus, 'AU contents'
    return rate, dac, sbr, n

if __name__ == '__main__':
    files = sorted(glob.glob(os.path.join(sys.argv[1], 'sf_*.bin')))
    for fn in files:
        r = check(fn); print('ok  %3d kbit/s dac_rate=%d sbr=%d  %d AUs' % r)
    print('PASS (%d super frames)' % len(files) if files else 'no files')
