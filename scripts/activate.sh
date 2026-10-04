#!/bin/sh
# =============================================================================
# activate.sh - runs ON THE PLUTO (started by install_on_pluto.bat) from /tmp/pkg:
# installs the control software into the persistent flash (/mnt/jffs2) and restarts it.
# An open transmitter stays open (the daemon adopts it); a closed one stays closed.
# The new files are staged next to the old ones FIRST; the running software is only stopped when every file is in place, and it is
# started again in every case, so a failure never leaves the Pluto without its control software.
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

echo "staging the new files in $JD ..."
for f in $FILES; do
    if ! { cp "$f" "$JD/$f.new" && chmod +x "$JD/$f.new"; }; then
        echo "cannot write $JD/$f.new (flash full?): nothing was changed"
        for g in $FILES; do rm -f "$JD/$g.new"; done
        rm -rf /tmp/pkg
        exit 1
    fi
done
sync

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
rc=0
for f in $FILES; do
    mv "$JD/$f.new" "$JD/$f" || { echo "cannot replace $JD/$f"; rc=1; }
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
if ! "$JD/skypluto-ctl" -c "?V"; then
    echo "the daemon did not answer: check /tmp/ctl.log on the Pluto"
    rc=1
fi
rm -rf /tmp/pkg
exit $rc
