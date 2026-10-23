#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# install_dab_test.sh - put everything for a DAB test on a running Pluto, WITHOUT changing what it boots.
#   dab/tools/install_dab_test.sh <pluto-ip> <BOOT.bin of the DAB build> [password]
# Copies to the SD card: BOOT.bin.fm (a copy of the BOOT.bin it boots now, only if not there yet) and BOOT.bin.dab.
# Copies to /mnt/jffs2: skypluto-dab (from build-arm/), dab-test.sh, dab-boot.sh.
# Switching happens on the board: /mnt/jffs2/dab-boot.sh dab (or fm), which reboots.
set -euo pipefail
IP="${1:?usage: $0 <pluto-ip> <DAB BOOT.bin> [password]}"
BOOT="${2:?usage: $0 <pluto-ip> <DAB BOOT.bin> [password]}"
PW="${3:-analog}"
HERE="$(cd "$(dirname "$0")" && pwd)"
APP="$HERE/../build-arm/skypluto-dab"
O="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=8"
SSH() { sshpass -p "$PW" ssh $O root@"$IP" "$@"; }
[ -f "$BOOT" ] || { echo "missing $BOOT"; exit 1; }
[ -f "$APP" ] || { echo "missing $APP (tools/build_arm.sh)"; exit 1; }

SSH 'mkdir -p /mnt/sd; grep -q " /mnt/sd " /proc/mounts || mount /dev/mmcblk0p1 /mnt/sd; [ -f /mnt/sd/BOOT.bin.fm ] || cp /mnt/sd/BOOT.bin /mnt/sd/BOOT.bin.fm'
SSH 'cat > /mnt/sd/BOOT.bin.dab.new && mv /mnt/sd/BOOT.bin.dab.new /mnt/sd/BOOT.bin.dab && sync' < "$BOOT"
SSH 'cat > /mnt/jffs2/skypluto-dab.new && chmod +x /mnt/jffs2/skypluto-dab.new && mv /mnt/jffs2/skypluto-dab.new /mnt/jffs2/skypluto-dab' < "$APP"
for f in dab-test.sh dab-boot.sh; do
    tr -d '\r' < "$HERE/$f" | SSH "cat > /mnt/jffs2/$f && chmod +x /mnt/jffs2/$f"
done
SSH 'sync; cd /mnt/sd; md5sum BOOT.bin BOOT.bin.fm BOOT.bin.dab; ls -la /mnt/jffs2/skypluto-dab /mnt/jffs2/dab-*.sh; df /mnt/jffs2 | tail -1'
echo "staged. To switch: ssh root@$IP /mnt/jffs2/dab-boot.sh dab   (back: dab-boot.sh fm)"
