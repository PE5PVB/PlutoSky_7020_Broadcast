#!/bin/sh
# =============================================================================
# activate.sh - runs ON THE PLUTO (started by install_on_pluto.bat) from /tmp/pkg:
# installs the control software into the persistent flash (/mnt/jffs2) and restarts it.
# An open transmitter stays open (the daemon adopts it); a closed one stays closed.
# =============================================================================
cd /tmp/pkg || exit 1
JD=/mnt/jffs2
FILES="skypluto-ctl skypluto-mask autorun.sh skypluto-supervise.sh skypluto-wfm.sh skypluto-cmd.sh"

for f in $FILES; do
    [ -f "$f" ] || { echo "missing file: $f"; exit 1; }
done

# the scripts must have Unix line endings (a Windows checkout can add CR characters)
for f in autorun.sh skypluto-supervise.sh skypluto-wfm.sh skypluto-cmd.sh; do
    tr -d '\r' < "$f" > "$f.n" && mv "$f.n" "$f"
done
chmod +x $FILES

echo "stopping the running control software ..."
for d in /proc/[0-9]*; do
    c=$(tr '\0' ' ' < "$d/cmdline" 2>/dev/null)
    case "$c" in *skypluto-supervise*|*skypluto-autocal*) kill -9 "$(basename $d)" 2>/dev/null;; esac    # (an older install also ran skypluto-autocal.sh)
done
for d in /proc/[0-9]*; do
    l=$(readlink "$d/exe" 2>/dev/null)
    case "$l" in *skypluto-ctl*|*skypluto-mask*|*iio_readdev*) kill -9 "$(basename $d)" 2>/dev/null;; esac
done
sleep 1

echo "installing into $JD ..."
for f in $FILES; do
    cp "$f" "$JD/$f.new" && chmod +x "$JD/$f.new" && mv "$JD/$f.new" "$JD/$f" || { echo "cannot write $JD/$f"; exit 1; }
done
sync

# the board's own web server would occupy port 80 (our web interface); autorun.sh stops it at every boot
for d in /proc/[0-9]*; do
    l=$(readlink "$d/exe" 2>/dev/null)
    case "$l" in *busybox*) case "$(tr '\0' ' ' < "$d/cmdline" 2>/dev/null)" in 'httpd -h /www'*) kill "$(basename $d)" 2>/dev/null;; esac;; esac
done

echo "starting ..."
nohup sh "$JD/skypluto-supervise.sh" > /dev/null 2>&1 < /dev/null &
sleep 6
"$JD/skypluto-ctl" -c "?V" || echo "the daemon did not answer yet (give it a few more seconds)"
rm -rf /tmp/pkg
