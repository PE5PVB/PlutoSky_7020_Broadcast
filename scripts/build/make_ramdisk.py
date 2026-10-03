#!/usr/bin/env python3
"""Bundle the PlutoSky 7020 Broadcast control software into the Pluto's boot ramdisk (uramdisk.image.gz).

    make_ramdisk.py <uramdisk.image.gz in> <folder with the control software> <uramdisk.image.gz out>

The ramdisk is a u-boot legacy image around a gzip-compressed 'newc' cpio archive. This tool appends
  /usr/share/skypluto/{FILES, VERSION, skypluto-ctl, skypluto-mask, autorun.sh, skypluto-*.sh}
and a replacement /etc/init.d/S98autostart (the kernel's initramfs unpacker lets a later entry replace an earlier one).
At every boot the new S98autostart compares VERSION with /mnt/jffs2/skypluto.version and, when they differ, copies the bundled files into the
persistent flash (/mnt/jffs2) before it runs /mnt/jffs2/autorun.sh as before. Copying the SD-card files is then the complete installation, and the
settings kept in /mnt/jffs2 (skypluto-*.conf) are left alone.
"""
import gzip, hashlib, os, struct, sys, time, zlib

FILES = ["skypluto-ctl", "skypluto-mask", "autorun.sh", "skypluto-supervise.sh", "skypluto-wfm.sh", "skypluto-cmd.sh"]
PRODUCT = "PlutoSky_7020_Broadcast-1.02"

S98 = b"""#!/bin/sh
#
# Runs /mnt/jffs2/autorun.sh at boot. Before that, the PlutoSky 7020 Broadcast control software that is bundled in this ramdisk
# (/usr/share/skypluto) is installed into the persistent flash when it is missing there or is a different version.
#

SRC=/usr/share/skypluto
JD=/mnt/jffs2

install_skypluto() {
	[ -f $SRC/VERSION ] || return
	grep -q " $JD " /proc/mounts || { echo "skypluto: $JD is not mounted, nothing installed" > /tmp/skypluto-install.log; return; }
	[ "`cat $SRC/VERSION`" = "`cat $JD/skypluto.version 2> /dev/null`" ] && return
	echo "Installing the PlutoSky 7020 Broadcast control software: `cat $SRC/VERSION`"
	for f in `cat $SRC/FILES`; do
		cp $SRC/$f $JD/$f.new && chmod 700 $JD/$f.new && mv $JD/$f.new $JD/$f || { echo "skypluto: cannot install $f" > /tmp/skypluto-install.log; return; }
	done
	cp $SRC/VERSION $JD/skypluto.version
	sync
}

case "$1" in
  start)
	install_skypluto
	if test -f /mnt/jffs2/autorun.sh; then
		source /mnt/jffs2/autorun.sh
	fi
	;;
  stop)
	;;
  restart|reload)
	;;
  *)
	echo "Usage: $0 {start|stop|restart}"
	exit 1
esac

exit $?
"""


def read_uimage(path):
    b = open(path, "rb").read()
    magic, hcrc, tm, size, load, ep, dcrc, os_, arch, typ, comp = struct.unpack(">IIIIIIIBBBB", b[:32])
    assert magic == 0x27051956, "not a u-boot legacy image"
    data = b[64:64 + size]
    assert len(data) == size and (zlib.crc32(data) & 0xffffffff) == dcrc, "data CRC mismatch"
    return dict(load=load, ep=ep, os=os_, arch=arch, type=typ, comp=comp, name=b[32:64]), data


def write_uimage(path, meta, data):
    h = struct.pack(">IIIIIIIBBBB", 0x27051956, 0, int(time.time()), len(data), meta["load"], meta["ep"], zlib.crc32(data) & 0xffffffff,
                    meta["os"], meta["arch"], meta["type"], meta["comp"]) + meta["name"]
    h = h[:4] + struct.pack(">I", zlib.crc32(h) & 0xffffffff) + h[8:]
    open(path, "wb").write(h + data)


def parse_cpio(raw):
    ents, p = [], 0
    while True:
        assert raw[p:p + 6] == b"070701", "bad cpio magic at %d" % p
        f = [int(raw[p + 6 + 8 * i:p + 14 + 8 * i], 16) for i in range(13)]
        ino, mode, uid, gid, nlink, mtime, fsize, dmaj, dmin, rmaj, rmin, nsize, _ = f
        name = raw[p + 110:p + 110 + nsize - 1].decode()
        ds = (p + 110 + nsize + 3) & ~3
        ents.append(dict(ino=ino, mode=mode, uid=uid, gid=gid, nlink=nlink, mtime=mtime, dmaj=dmaj, dmin=dmin, rmaj=rmaj, rmin=rmin, name=name, data=raw[ds:ds + fsize]))
        p = (ds + fsize + 3) & ~3
        if name == "TRAILER!!!":
            return ents


def pack_entry(e):
    name = e["name"].encode() + b"\0"
    out = b"070701" + b"".join(b"%08X" % v for v in (e["ino"], e["mode"], e["uid"], e["gid"], e["nlink"], e["mtime"], len(e["data"]),
                                                     e["dmaj"], e["dmin"], e["rmaj"], e["rmin"], len(name), 0)) + name
    out += b"\0" * ((4 - len(out) % 4) % 4) + e["data"]
    return out + b"\0" * ((4 - len(out) % 4) % 4)


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    src, folder, dst = sys.argv[1:]
    meta, data = read_uimage(src)
    ents = parse_cpio(gzip.decompress(data))
    trailer = ents.pop()
    names = {e["name"] for e in ents}
    ino = max(e["ino"] for e in ents) + 1
    now = int(time.time())

    def entry(name, mode, content=b"", nlink=1):
        nonlocal ino
        e = dict(ino=ino, mode=mode, uid=0, gid=0, nlink=nlink, mtime=now, dmaj=0, dmin=0, rmaj=0, rmin=0, name=name, data=content)
        ino += 1
        return e

    files = {}
    for f in FILES:
        b = open(os.path.join(folder, f), "rb").read()
        if f.endswith(".sh"):
            b = b.replace(b"\r\n", b"\n")                           # the scripts run on BusyBox: Unix line endings only
        files[f] = b
    digest = hashlib.md5(b"".join(files[f] for f in FILES)).hexdigest()[:12]
    version = ("%s %s\n" % (PRODUCT, digest)).encode()

    add = []
    if "usr/share/skypluto" not in names:
        add.append(entry("usr/share/skypluto", 0o040755, nlink=2))
    for f in FILES:
        add.append(entry("usr/share/skypluto/" + f, 0o100755, files[f]))
    add.append(entry("usr/share/skypluto/FILES", 0o100644, ("\n".join(FILES) + "\n").encode()))
    add.append(entry("usr/share/skypluto/VERSION", 0o100644, version))
    add.append(entry("etc/init.d/S98autostart", 0o100755, S98))     # replaces the original of the same name
    raw = b"".join(pack_entry(e) for e in ents + add) + pack_entry(trailer)
    raw += b"\0" * ((512 - len(raw) % 512) % 512)
    out = gzip.compress(raw, 9, mtime=0)
    write_uimage(dst, meta, out)
    print("written %s: %d entries (%d added), %d bytes, version '%s'" % (dst, len(ents) + len(add), len(add), os.path.getsize(dst), version.decode().strip()))


if __name__ == "__main__":
    main()
