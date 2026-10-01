#!/usr/bin/env bash
# =============================================================================
# install_on_pluto.sh - install the SkyPluto_WFM control software on a running Pluto
# -----------------------------------------------------------------------------
# The FPGA bitstream (BOOT.bin) goes on the SD card (see README, "Installing a release").
# The control software (daemon, mask tool and start-up scripts) lives in the board's
# persistent flash (/mnt/jffs2) and is copied there over ssh:
#
#   scripts/install_on_pluto.sh <pluto-ip> <release-dir> [password]
#
# <release-dir> must contain the compiled binaries skypluto-ctl and skypluto-mask (from the
# GitHub release or from scripts/build/build_daemon.sh). The shell scripts are taken from the
# directory of this script. Needs ssh and sshpass on the host (default root password: analog).
# The daemon is restarted; an open transmitter stays open (the daemon adopts it).
# =============================================================================
set -euo pipefail

IP="${1:?usage: $0 <pluto-ip> <release-dir> [password]}"
REL="${2:?usage: $0 <pluto-ip> <release-dir> [password]}"
PW="${3:-analog}"
HERE="$(cd "$(dirname "$0")" && pwd)"
O="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=8"
SSH() { sshpass -p "$PW" ssh $O root@"$IP" "$@"; }

for f in skypluto-ctl skypluto-mask; do
    [ -f "$REL/$f" ] || { echo "missing $REL/$f"; exit 1; }
done
for f in autorun.sh skypluto-supervise.sh skypluto-autocal.sh skypluto-wfm.sh skypluto-cmd.sh; do
    [ -f "$HERE/$f" ] || { echo "missing $HERE/$f"; exit 1; }
done

push() {   # push <local-file> <name-on-board> : copy to a temporary name, then move into place
    tr -d '\r' < "$1" | SSH "cat > /mnt/jffs2/$2.new && chmod +x /mnt/jffs2/$2.new"
}
pushbin() {
    SSH "cat > /mnt/jffs2/$2.new && chmod +x /mnt/jffs2/$2.new" < "$1"
}

echo "==> copying to /mnt/jffs2 on $IP"
pushbin "$REL/skypluto-ctl"  skypluto-ctl
pushbin "$REL/skypluto-mask" skypluto-mask
for f in autorun.sh skypluto-supervise.sh skypluto-autocal.sh skypluto-wfm.sh skypluto-cmd.sh; do
    push "$HERE/$f" "$f"
done

echo "==> activating (supervisor and daemon are restarted)"
SSH 'sh -s' <<'REMOTE'
for d in /proc/[0-9]*; do
    c=$(tr '\0' ' ' < "$d/cmdline" 2>/dev/null)
    case "$c" in *skypluto-supervise*) kill -9 "$(basename $d)" 2>/dev/null;; esac
done
for d in /proc/[0-9]*; do
    l=$(readlink "$d/exe" 2>/dev/null)
    case "$l" in *skypluto-ctl*|*skypluto-mask*|*iio_readdev*) kill -9 "$(basename $d)" 2>/dev/null;; esac
done
sleep 1
for f in skypluto-ctl skypluto-mask autorun.sh skypluto-supervise.sh skypluto-autocal.sh skypluto-wfm.sh skypluto-cmd.sh; do
    mv /mnt/jffs2/$f.new /mnt/jffs2/$f
done
sync
# the stock web server would occupy port 80 (our web interface); autorun.sh stops it at every boot
for d in /proc/[0-9]*; do
    l=$(readlink "$d/exe" 2>/dev/null)
    case "$l" in *busybox*) case "$(tr '\0' ' ' < "$d/cmdline" 2>/dev/null)" in 'httpd -h /www'*) kill "$(basename $d)" 2>/dev/null;; esac;; esac
done
nohup sh /mnt/jffs2/skypluto-supervise.sh > /dev/null 2>&1 &
sleep 5
/mnt/jffs2/skypluto-ctl -c "?V"
REMOTE
echo "==> done. Web interface: http://$IP/   (reboot once to test the complete start-up chain)"
