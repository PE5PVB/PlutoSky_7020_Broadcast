#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Independent decoder for the SPI (EPG) output, written from the standards (not from the C encoder):

  check_spi.py <epg dir>                     decode every object listed in <epg dir>/index (ETSI TS 102 371 binary encoding)
                                             to XML-like text and check it against the index (ScopeStart/End/ID) and the
                                             basic-profile limits
  check_spi.py --pad <pad dump> <epg dir> [<slide dir>] [<au per second>]
                                             parse the PAD of every audio frame (skypluto-dab -D), demultiplex the X-PAD
                                             (EN 300 401 7.4), reassemble the MSC data groups (5.3.3, CRC) per application,
                                             the MOT directory carousel (EN 301 234 7.2) on X-PAD application types 16/17 and
                                             the slideshow on 12/13 (header mode), the DLS on 2/3; compare the received
                                             objects with the files, decode them, and report the timing

Exit status 0 when everything checks out.
"""
import os
import sys

FAILS = []


def fail(msg):
    FAILS.append(msg)
    print('FAIL', msg)


def ok(msg):
    print('ok  ', msg)


# ---- ETSI TS 102 371 annex D (element tags) and annex E (attribute tags per element) -----------------------------------------
ELEM = {0x02: 'epg', 0x03: 'serviceInformation', 0x04: 'tokenTable', 0x06: 'defaultLanguage', 0x10: 'shortName',
        0x11: 'mediumName', 0x12: 'longName', 0x13: 'mediaDescription', 0x14: 'genre', 0x16: 'keywords', 0x17: 'memberOf',
        0x18: 'link', 0x19: 'location', 0x1A: 'shortDescription', 0x1B: 'longDescription', 0x1C: 'programme',
        0x20: 'programmeGroups', 0x21: 'schedule', 0x23: 'programmeGroup', 0x24: 'scope', 0x25: 'serviceScope',
        0x26: 'ensemble', 0x28: 'service', 0x29: 'bearer', 0x2A: 'presentationLanguage', 0x2B: 'multimedia', 0x2C: 'time',
        0x2D: 'bearer', 0x2E: 'programmeEvent', 0x2F: 'relativeTime'}
# (element, attribute tag) -> (name, type)
ATTR = {
    ('serviceInformation', 0x80): ('version', 'int'), ('serviceInformation', 0x81): ('creationTime', 'time'),
    ('serviceInformation', 0x82): ('originator', 'str'), ('serviceInformation', 0x83): ('serviceProvider', 'str'),
    ('ensemble', 0x80): ('id', 'eid'),
    ('service', 0x80): ('version', 'int'),
    ('bearer', 0x80): ('id', 'bearer'), ('bearer', 0x82): ('url', 'str'),
    ('shortName', 0x80): ('xml:lang', 'str'), ('mediumName', 0x80): ('xml:lang', 'str'),
    ('longName', 0x80): ('xml:lang', 'str'), ('shortDescription', 0x80): ('xml:lang', 'str'),
    ('multimedia', 0x80): ('mimeValue', 'str'), ('multimedia', 0x81): ('xml:lang', 'str'),
    ('multimedia', 0x82): ('url', 'str'), ('multimedia', 0x83): ('type', 'enum'), ('multimedia', 0x84): ('width', 'int'),
    ('multimedia', 0x85): ('height', 'int'),
    ('schedule', 0x80): ('version', 'int'), ('schedule', 0x81): ('creationTime', 'time'),
    ('schedule', 0x82): ('originator', 'str'),
    ('scope', 0x80): ('startTime', 'time'), ('scope', 0x81): ('stopTime', 'time'),
    ('serviceScope', 0x80): ('id', 'bearer'),
    ('programme', 0x80): ('id', 'str'), ('programme', 0x81): ('shortId', 'int'), ('programme', 0x82): ('version', 'int'),
    ('programme', 0x83): ('recommendation', 'enum'), ('programme', 0x84): ('broadcast', 'enum'),
    ('programme', 0x86): ('xml:lang', 'str'),
    ('time', 0x80): ('time', 'time'), ('time', 0x81): ('duration', 'dur'), ('time', 0x82): ('actualTime', 'time'),
    ('time', 0x83): ('actualDuration', 'dur'),
}


def mjd_to_ymd(mjd):
    # EN 300 401 / ETS 300 707 annex: MJD -> Y, M, D
    yp = int((mjd - 15078.2) / 365.25)
    mp = int((mjd - 14956.1 - int(yp * 365.25)) / 30.6001)
    d = mjd - 14956 - int(yp * 365.25) - int(mp * 30.6001)
    k = 1 if mp in (14, 15) else 0
    return 1900 + yp + k, mp - 1 - k * 12, d


def dec_time(b):
    """timePoint (5.4.5.2) -> (iso text, utc seconds, lto half hours or None)"""
    v = int.from_bytes(b[0:4], 'big')
    mjd = (v >> 14) & 0x1FFFF
    lto_f, utc_f = (v >> 12) & 1, (v >> 11) & 1
    h, mi = (v >> 6) & 31, v & 63
    p, s = 4, 0
    if utc_f:
        s = b[4] >> 2
        p = 6
    lto = None
    if lto_f:
        lb = b[p]
        lto = -(lb & 31) if lb & 0x20 else lb & 31
        p += 1
    if p != len(b):
        raise ValueError('timePoint length %d, expected %d' % (len(b), p))
    y, mo, d = mjd_to_ymd(mjd)
    utc = (mjd - 40587) * 86400 + h * 3600 + mi * 60 + s
    txt = '%04d-%02d-%02dT%02d:%02d:%02dZ' % (y, mo, d, h, mi, s)
    if lto is not None:
        txt += ' (LTO %+.1f h)' % (lto / 2)
    return txt, utc, lto


def dec_bearer(b):
    if len(b) < 6:
        raise ValueError('bearerURI too short')
    f = b[0]
    if f & 0x80 or not f & 0x40 or f & 0x20:
        raise ValueError('bearerURI flags %02X (Rfa 0, Ens 1, X-PAD 0 expected)' % f)
    scids = f & 15
    if f & 0x10:
        sid = int.from_bytes(b[4:8], 'big')
        n = 8
    else:
        sid = int.from_bytes(b[4:6], 'big')
        n = 6
    if n != len(b):
        raise ValueError('bearerURI length')
    ecc, eid = b[1], int.from_bytes(b[2:4], 'big')
    gcc = '%x%02x' % ((sid >> 12) & 15, ecc)
    return 'dab:%s.%04x.%04x.%d' % (gcc, eid, sid, scids)


def read_tlv(buf, p, end):
    tag, ln = buf[p], buf[p + 1]
    p += 2
    if ln == 0xFE:
        ln = int.from_bytes(buf[p:p + 2], 'big')
        p += 2
    elif ln == 0xFF:
        ln = int.from_bytes(buf[p:p + 3], 'big')
        p += 3
    if p + ln > end:
        raise ValueError('TLV at %d: length %d runs past the end' % (p, ln))
    return tag, p, p + ln


class Node:
    def __init__(self, name):
        self.name, self.attrs, self.kids, self.text = name, {}, [], None


def parse_elem(buf, p, end, name, depth=0):
    """the value of element 'name' (attributes, then child elements, CDATA) -> Node"""
    n = Node(name)
    seen_child = False
    while p < end:
        tag, vs, ve = read_tlv(buf, p, end)
        val = buf[vs:ve]
        if tag == 0x01:  # CDATA
            n.text = val.decode('utf-8')
        elif tag >= 0x80:
            if seen_child:
                raise ValueError('%s: attribute after a child element' % name)
            an, at = ATTR.get((name, tag), ('attr%02X' % tag, 'hex'))
            if at == 'int':
                n.attrs[an] = int.from_bytes(val, 'big')
            elif at == 'time':
                n.attrs[an] = dec_time(val)
            elif at == 'dur':
                if len(val) != 2:
                    raise ValueError('duration length')
                n.attrs[an] = int.from_bytes(val, 'big')
            elif at == 'bearer':
                n.attrs[an] = dec_bearer(val)
            elif at == 'eid':
                n.attrs[an] = '%02x.%04x' % (val[0], int.from_bytes(val[1:3], 'big'))
            elif at == 'str':
                n.attrs[an] = val.decode('utf-8')
            else:
                n.attrs[an] = val.hex()
        elif tag in (0x04, 0x06):
            raise ValueError('token table / default language not expected')
        else:
            seen_child = True
            cn = ELEM.get(tag)
            if cn is None:
                raise ValueError('unknown element tag %02X in %s' % (tag, name))
            n.kids.append(parse_elem(buf, vs, ve, cn, depth + 1))
        p = ve
    return n


def decode_object(buf):
    tag, vs, ve = read_tlv(buf, 0, len(buf))
    if ve != len(buf):
        raise ValueError('top-level element does not span the object (%d of %d bytes)' % (ve, len(buf)))
    if tag not in (0x02, 0x03):
        raise ValueError('top-level tag %02X' % tag)
    return parse_elem(buf, vs, ve, ELEM[tag])


def xml(n, ind=0):
    def av(v):
        return v[0] if isinstance(v, tuple) else str(v)
    a = ''.join(' %s="%s"' % (k, av(v)) for k, v in n.attrs.items())
    pad = '  ' * ind
    if not n.kids and n.text is None:
        return '%s<%s%s/>\n' % (pad, n.name, a)
    if not n.kids:
        return '%s<%s%s>%s</%s>\n' % (pad, n.name, a, n.text, n.name)
    return '%s<%s%s>\n%s%s</%s>\n' % (pad, n.name, a, ''.join(xml(k, ind + 1) for k in n.kids), pad, n.name)


def kid(n, name):
    return [k for k in n.kids if k.name == name]


def nchars(s):
    return len(s or '')


# ---- the objects in a directory ------------------------------------------------------------------------------------------------
def read_index(d):
    objs = []
    for ln in open(os.path.join(d, 'index'), encoding='utf-8'):
        f = ln.split()
        if not f or f[0] != 'o':
            continue
        o = {'file': f[1], 'ct': int(f[2]), 'cst': int(f[3]), 'name': f[4]}
        for t in f[5:]:
            k, v = t.split('=', 1)
            o[k] = v
        objs.append(o)
    return objs


def check_objects(d, objs, quiet=False):
    short_ids = {}
    for o in objs:
        buf = open(os.path.join(d, o['file']), 'rb').read()
        o['body'] = buf
        what = o['name']
        try:
            root = decode_object(buf)
        except Exception as e:  # noqa: BLE001
            fail('%s: %s' % (what, e))
            continue
        if not quiet:
            print('---- %s (%s, ContentType %d/%d, %d bytes)' % (what, o['file'], o['ct'], o['cst'], len(buf)))
            print(xml(root), end='')
        if len(buf) > 16384:
            fail('%s: %d bytes, more than 16 kB' % (what, len(buf)))
        if o['ct'] == 7 and o['cst'] == 0:
            ens = kid(root, 'ensemble')
            okk = root.name == 'serviceInformation' and 'version' in root.attrs and len(ens) == 1
            okk = okk and len(kid(ens[0], 'shortName')) == 1 and len(kid(ens[0], 'mediumName')) == 1
            svcs = kid(ens[0], 'service') if ens else []
            for s in svcs:
                okk = okk and len(kid(s, 'shortName')) == 1 and len(kid(s, 'mediumName')) == 1 and len(kid(s, 'bearer')) >= 1
                okk = okk and nchars(kid(s, 'shortName')[0].text) <= 8 and nchars(kid(s, 'mediumName')[0].text) <= 16
            eid = ens[0].attrs.get('id') if ens else None
            scope = bytes.fromhex(o.get('scope', ''))
            okk = okk and len(scope) == 3 and eid == '%02x.%04x' % (scope[0], int.from_bytes(scope[1:], 'big'))
            (ok if okk else fail)('%s: SI object (ensemble %s, %d services, ScopeID = ensemble id)' % (what, eid, len(svcs)))
        elif o['ct'] == 7 and o['cst'] == 1:
            sch = kid(root, 'schedule')
            okk = root.name == 'epg' and len(sch) == 1 and 'version' in sch[0].attrs
            sc = kid(sch[0], 'scope') if sch else []
            progs = kid(sch[0], 'programme') if sch else []
            okk = okk and len(sc) == 1 and len(kid(sc[0], 'serviceScope')) == 1
            if not okk:
                fail('%s: PI structure' % what)
                continue
            st, sp = sc[0].attrs['startTime'], sc[0].attrs['stopTime']
            sid_uri = kid(sc[0], 'serviceScope')[0].attrs['id']
            scope = bytes.fromhex(o.get('scope', ''))
            okk = dec_bearer(scope) == sid_uri
            okk = okk and int(o['ss']) // 60 * 60 == st[1] and int(o['se']) // 60 * 60 == sp[1]
            last_end = None
            times = []
            for pg in progs:
                sid = pg.attrs.get('shortId')
                if sid in short_ids:
                    fail('%s: shortId %d also in %s' % (what, sid, short_ids[sid]))
                short_ids[sid] = what
                mn, lnm = kid(pg, 'mediumName'), kid(pg, 'longName')
                loc = kid(pg, 'location')
                tm = kid(loc[0], 'time') if loc else []
                if len(mn) != 1 or len(lnm) != 1 or len(tm) != 1:
                    fail('%s: programme %s: mediumName/longName/location.time missing' % (what, sid))
                    okk = False
                    continue
                if nchars(mn[0].text) > 16 or nchars(lnm[0].text) > 128:
                    fail('%s: programme %s: name too long' % (what, sid))
                md = kid(pg, 'mediaDescription')
                for m in md:
                    for sd in kid(m, 'shortDescription'):
                        if nchars(sd.text) > 180:
                            fail('%s: description longer than 180 characters' % what)
                t0, dur = tm[0].attrs['time'][1], tm[0].attrs['duration']
                if last_end is not None and t0 < last_end:
                    fail('%s: programme %s starts before the previous one ends' % (what, sid))
                last_end = t0 + dur
                times.append((t0, dur))
            okk = okk and bool(progs) and times[0][0] // 60 * 60 == st[1] and (times[-1][0] + times[-1][1]) // 60 * 60 == sp[1]
            (ok if okk else fail)('%s: PI object for %s, %d programmes, scope %s .. %s matches the index'
                                  % (what, sid_uri, len(progs), st[0], sp[0]))
        else:
            fail('%s: ContentType %d/%d' % (what, o['ct'], o['cst']))


# ---- PAD / X-PAD / data groups / MOT -----------------------------------------------------------------------------------------
def crc16(d):
    """EN 300 401 5.3.3.4: CRC-16 CCITT (x^16 + x^12 + x^5 + 1), preset all ones, the result inverted"""
    c = 0xFFFF
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c ^ 0xFFFF


XPAD_LEN = [4, 6, 8, 12, 16, 24, 32, 48]


class DGAssembler:
    """X-PAD data groups of one application (start type, continuation type) after a data group length indicator"""

    def __init__(self, name):
        self.name, self.want, self.buf, self.groups, self.bad = name, None, None, [], 0

    def start(self, data, dgli_len):
        if dgli_len is None:
            self.bad += 1
            self.buf = None
            return
        self.want, self.buf = dgli_len, bytearray()
        self.more(data)

    def more(self, data):
        if self.buf is None:
            return
        need = self.want - len(self.buf)
        self.buf += data[:need]
        if len(self.buf) == self.want:
            self.groups.append((self.au, bytes(self.buf)))
            self.buf = None


def parse_pad(path):
    """per audio frame: the X-PAD subfields [(apptype, bytes)] (variable size X-PAD with contents indicators)"""
    raw = open(path, 'rb').read()
    frames, p = [], 0
    while p + 2 <= len(raw):
        n = int.from_bytes(raw[p:p + 2], 'big')
        pad = raw[p + 2:p + 2 + n]
        p += 2 + n
        subs = []
        if n >= 2:
            fpad = pad[-2:]
            xpad_ind = (fpad[0] >> 4) & 3
            ci_flag = (fpad[1] >> 1) & 1
            if xpad_ind != 2 or not ci_flag:
                raise ValueError('frame %d: F-PAD %s (variable size X-PAD with CI expected)' % (len(frames), fpad.hex()))
            x = pad[:-2][::-1]  # DAB+ / here: the X-PAD bytes are stored in reverse order
            cis, q = [], 0
            while q < len(x) and len(cis) < 4:
                ci = x[q]
                q += 1
                if ci & 31 == 0:
                    break
                cis.append((XPAD_LEN[ci >> 5], ci & 31))
            for ln, app in cis:
                subs.append((app, x[q:q + ln]))
                q += ln
            if q != len(x):
                raise ValueError('frame %d: X-PAD of %d bytes, subfields end at %d' % (len(frames), len(x), q))
        frames.append(subs)
    return frames


def demux(frames, pairs):
    """the MSC data groups of each application pair; DLS (2/3) too"""
    asm = {s: DGAssembler(name) for s, name in pairs.items()}
    dgli = None
    dls = []
    for i, subs in enumerate(frames):
        for app, data in subs:
            if app == 1:
                if crc16(data[:2]) == int.from_bytes(data[2:4], 'big'):
                    dgli = ((data[0] & 0x3F) << 8) | data[1]
                else:
                    dgli = None
                continue
            if app == 2:
                dls.append((i, data, True))
                continue
            if app == 3:
                dls.append((i, data, False))
                continue
            for s, a in asm.items():
                a.au = i
                if app == s:
                    a.start(data, dgli)
                    dgli = None
                elif app == s + 1:
                    a.more(data)
    return asm, dls


def parse_dg(dg):
    """MSC data group (EN 300 401 5.3.3) -> dict, or raises"""
    if crc16(dg[:-2]) != int.from_bytes(dg[-2:], 'big'):
        raise ValueError('data group CRC')
    h0, h1 = dg[0], dg[1]
    ext, crcf, segf, uaf, typ = h0 >> 7, (h0 >> 6) & 1, (h0 >> 5) & 1, (h0 >> 4) & 1, h0 & 15
    if not crcf:
        raise ValueError('no CRC flag')
    p = 2 + (2 if ext else 0)
    last, seg = 0, 0
    if segf:
        last, seg = dg[p] >> 7, ((dg[p] & 0x7F) << 8) | dg[p + 1]
        p += 2
    tid = None
    if uaf:
        li = dg[p] & 15
        tidf = (dg[p] >> 4) & 1
        p += 1
        if tidf:
            tid = int.from_bytes(dg[p:p + 2], 'big')
        p += li
    data = dg[p:-2]
    rep, size = data[0] >> 5, ((data[0] & 31) << 8) | data[1]
    seg_data = data[2:]
    if size != len(seg_data):
        raise ValueError('segment size %d, data %d' % (size, len(seg_data)))
    return {'type': typ, 'ci': h1 >> 4, 'last': last, 'seg': seg, 'tid': tid, 'rep': rep, 'data': seg_data}


def parse_params(b):
    """MOT header extension parameters (EN 301 234 6.2) -> [(id, data)]"""
    out, p = [], 0
    while p < len(b):
        pli, pid = b[p] >> 6, b[p] & 63
        p += 1
        if pli == 0:
            n = 0
        elif pli == 1:
            n = 1
        elif pli == 2:
            n = 4
        else:
            if b[p] & 0x80:
                n = ((b[p] & 0x7F) << 8) | b[p + 1]
                p += 2
            else:
                n = b[p]
                p += 1
        out.append((pid, b[p:p + n]))
        p += n
    if p != len(b):
        raise ValueError('header extension overruns')
    return out


def parse_header(b):
    bs = (b[0] << 20) | (b[1] << 12) | (b[2] << 4) | (b[3] >> 4)
    hs = ((b[3] & 15) << 9) | (b[4] << 1) | (b[5] >> 7)
    ct, cst = (b[5] >> 1) & 63, ((b[5] & 1) << 8) | b[6]
    return {'bodysize': bs, 'headersize': hs, 'ct': ct, 'cst': cst, 'params': parse_params(b[7:hs])}, hs


def parse_directory(d):
    if d[0] & 0xC0:
        raise ValueError('CF/Rfu set')
    size = int.from_bytes(d[0:4], 'big') & 0x3FFFFFFF
    nobj = int.from_bytes(d[4:6], 'big')
    period = int.from_bytes(d[6:9], 'big')
    segsize = int.from_bytes(d[9:11], 'big') & 0x1FFF
    extlen = int.from_bytes(d[11:13], 'big')
    if size != len(d):
        raise ValueError('DirectorySize %d, received %d' % (size, len(d)))
    ext = parse_params(d[13:13 + extlen])
    p, ents = 13 + extlen, []
    for _ in range(nobj):
        tid = int.from_bytes(d[p:p + 2], 'big')
        h, hs = parse_header(d[p + 2:])
        ents.append((tid, h))
        p += 2 + hs
    if p != len(d):
        raise ValueError('directory entries end at %d of %d' % (p, len(d)))
    return {'size': size, 'n': nobj, 'period': period, 'segsize': segsize, 'ext': ext, 'entries': ents}


def content_name(h):
    for pid, v in h['params']:
        if pid == 0x0C:
            return v[0] >> 4, v[1:].decode('utf-8')
    return None, None


def reassemble(groups, types):
    """MOT entities from data groups: {(type, tid): (bytes, first au, last au)} for the complete ones"""
    segs, done = {}, {}
    for au, dg in groups:
        try:
            g = parse_dg(dg)
        except ValueError as e:
            fail('data group at frame %d: %s' % (au, e))
            continue
        if g['type'] not in types:
            continue
        k = (g['type'], g['tid'])
        s = segs.setdefault(k, {'parts': {}, 'last': None, 'au0': au})
        s['parts'][g['seg']] = g['data']
        if g['last']:
            s['last'] = g['seg']
        if s['last'] is not None and all(i in s['parts'] for i in range(s['last'] + 1)):
            if k not in done:
                done[k] = (b''.join(s['parts'][i] for i in range(s['last'] + 1)), s['au0'], au)
            segs.pop(k)
    return done


def check_pad(pad_path, epg_dir, sls_dir=None, au_per_s=50.0):
    frames = parse_pad(pad_path)
    secs = len(frames) / au_per_s
    print('PAD: %d audio frames (%.1f s)' % (len(frames), secs))
    asm, dls = demux(frames, {12: 'slideshow', 16: 'SPI'})
    # MOT piece share per application
    share = {}
    for subs in frames:
        for app, data in subs:
            share[app] = share.get(app, 0) + len(data)
    print('X-PAD bytes per application type: ' + ', '.join('%d: %d (%.0f B/s)' % (k, v, v / secs) for k, v in sorted(share.items())))
    # DLS: segments (EN 300 401 7.4.5.2)
    txt, label = {}, None
    for i, d, st in dls:
        if st:
            pass
        hdr0 = d[0]
        nch = (hdr0 & 15) + 1
        segno = 0 if hdr0 & 0x40 else (d[1] >> 4) & 7
        seg_len = 2 + nch + 2
        if crc16(d[:2 + nch]) == int.from_bytes(d[2 + nch:seg_len], 'big'):
            txt[segno] = d[2:2 + nch]
            if hdr0 & 0x20:
                label = b''.join(txt.get(k, b'?') for k in range(segno + 1)).decode('utf-8', 'replace')
    print('DLS: %r' % label)
    if not label:
        fail('no DLS label')
    # ---- SPI carousel
    spi = asm[16]
    if spi.bad:
        fail('SPI: %d data group starts without a valid length indicator' % spi.bad)
    ents = reassemble(spi.groups, (4, 6))
    dirs = sorted(((au0, au1, tid, b) for (t, tid), (b, au0, au1) in ents.items() if t == 6))
    bodies = {tid: (b, au0, au1) for (t, tid), (b, au0, au1) in ents.items() if t == 4}
    print('SPI: %d data groups, %d directories (TransportIds %s), %d bodies'
          % (len(spi.groups), len(dirs), sorted(set(x[2] for x in dirs)), len(bodies)))
    # the directory is repeated: count its transmissions (every type-6 data group with segment 0)
    dir_starts = []
    for au, dg in spi.groups:
        try:
            g = parse_dg(dg)
        except ValueError:
            continue
        if g['type'] == 6 and g['seg'] == 0:
            dir_starts.append(au)
    if len(dir_starts) > 1:
        gaps = [(b - a) / au_per_s for a, b in zip(dir_starts, dir_starts[1:])]
        print('SPI: directory sent at %s s; intervals %s s' % (', '.join('%.1f' % (a / au_per_s) for a in dir_starts),
                                                                 ', '.join('%.1f' % g for g in gaps)))
    if not dirs:
        fail('SPI: no complete MOT directory')
        return
    objs = read_index(epg_dir) if epg_dir else []
    want = {o['name']: o for o in objs}
    for o in objs:
        o['body'] = open(os.path.join(epg_dir, o['file']), 'rb').read()
    au0, au1, dtid, dbytes = dirs[-1]
    try:
        dr = parse_directory(dbytes)
    except ValueError as e:
        fail('SPI directory: %s' % e)
        return
    names = []
    sorted_flag = any(pid == 0x00 for pid, _ in dr['ext'])
    print('SPI directory (TransportId %d, %d bytes, %d objects, segment size %d, SortedHeaderInformation %s):'
          % (dtid, dr['size'], dr['n'], dr['segsize'], 'yes' if sorted_flag else 'no'))
    if dr['size'] > 8192:
        fail('SPI directory larger than 8 kB')
    all_ok = True
    for tid, h in dr['entries']:
        cs, name = content_name(h)
        names.append(name.encode('utf-8'))
        params = {pid: v for pid, v in h['params']}
        extra = []
        if 0x25 in params:
            extra.append('ScopeStart %s' % dec_time(params[0x25])[0])
        if 0x26 in params:
            extra.append('ScopeEnd %s' % dec_time(params[0x26])[0])
        if 0x27 in params:
            v = params[0x27]
            extra.append('ScopeID %s' % (dec_bearer(v) if len(v) >= 6 else v.hex()))
        got = bodies.get(tid)
        print('  tid %5d  %-22s %d/%d  %5d bytes  charset %d  %s  %s' % (tid, name, h['ct'], h['cst'], h['bodysize'], cs,
                                                                     '; '.join(extra), 'received' if got else 'NOT RECEIVED'))
        o = want.get(name)
        if not o:
            fail('SPI: %s in the directory but not in the index' % name)
            all_ok = False
            continue
        if (h['ct'], h['cst']) != (o['ct'], o['cst']) or cs != 15:
            fail('SPI: %s ContentType/charset' % name)
            all_ok = False
        if o['cst'] == 1:
            if dec_time(params[0x25])[1] != int(o['ss']) // 60 * 60 or dec_time(params[0x26])[1] != int(o['se']) // 60 * 60:
                fail('SPI: %s ScopeStart/End differ from the index' % name)
                all_ok = False
        if 0x27 not in params or params[0x27].hex().upper() != o['scope'].upper():
            fail('SPI: %s ScopeID' % name)
            all_ok = False
        if not got:
            fail('SPI: %s (tid %d) not received completely' % (name, tid))
            all_ok = False
        elif got[0] != o['body'] or len(got[0]) != h['bodysize']:
            fail('SPI: %s received body differs from the file (%d / %d / %d bytes)' % (name, len(got[0]), len(o['body']),
                                                                                      h['bodysize']))
            all_ok = False
    if names != sorted(names):
        fail('SPI: directory not sorted by ContentName')
        all_ok = False
    if set(want) != set(n.decode() for n in names):
        fail('SPI: index and directory list different objects')
        all_ok = False
    (ok if all_ok else fail)('SPI: directory and all %d objects received, CRCs, sizes and bodies match the files' % dr['n'])
    if all_ok:
        # decode the received objects (the bodies equal the files: check_objects decodes those)
        check_objects(epg_dir, objs, quiet=True)
    # first complete reception of everything (time to the full EPG for a receiver switched on at t = 0)
    t_full = max([au1] + [bodies[t][2] for t, _ in dr['entries'] if t in bodies]) / au_per_s
    print('SPI: everything received after %.1f s' % t_full)
    # ---- slideshow (header mode)
    sl = reassemble(asm[12].groups, (3, 4))
    hdrs = {tid: b for (t, tid), (b, a, z) in sl.items() if t == 3}
    sbodies = {tid: (b, z) for (t, tid), (b, a, z) in sl.items() if t == 4}
    shown = []
    for tid, hb in hdrs.items():
        h, _ = parse_header(hb)
        cs, name = content_name(h)
        if tid in sbodies and len(sbodies[tid][0]) == h['bodysize']:
            shown.append((sbodies[tid][1], name, sbodies[tid][0]))
    shown.sort()
    print('slideshow: %d slides complete: %s' % (len(shown), ', '.join('%s at %.1f s' % (n, z / au_per_s) for z, n, _ in shown)))
    if sls_dir:
        for z, name, body in shown:
            n = name.replace('slide', '').split('.')[0]
            ext = name.split('.')[-1]
            ref = open(os.path.join(sls_dir, '%s.%s' % (n, ext)), 'rb').read()
            (ok if ref == body else fail)('slide %s equals %s/%s.%s' % (name, sls_dir, n, ext))
        if not shown:
            fail('no slide received')


def main():
    a = sys.argv[1:]
    if a and a[0] == '--pad':
        check_pad(a[1], a[2] if len(a) > 2 else None, a[3] if len(a) > 3 else None, float(a[4]) if len(a) > 4 else 50.0)
    elif a:
        objs = read_index(a[0])
        if not objs:
            fail('no objects in the index')
        check_objects(a[0], objs)
    else:
        print(__doc__)
        sys.exit(2)
    print('FAIL (%d)' % len(FAILS) if FAILS else 'PASS')
    sys.exit(1 if FAILS else 0)


if __name__ == '__main__':
    main()
