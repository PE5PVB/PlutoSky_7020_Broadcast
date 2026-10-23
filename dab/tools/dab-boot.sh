#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# dab-boot.sh - choose the bitstream the Pluto boots (runs on the board, from /mnt/jffs2), then reboot.
#   dab-boot.sh dab     BOOT.bin.dab (FM + DAB, build id B1D0001E)
#   dab-boot.sh fm      BOOT.bin.fm  (the FM release the board ran before, kept by install_dab_test.sh)
# Kernel, device tree and ramdisk stay the same for both.
SD=/mnt/sd
case "$1" in dab|fm) ;; *) echo "usage: dab-boot.sh dab|fm"; exit 2;; esac
mkdir -p $SD; grep -q " $SD " /proc/mounts || mount /dev/mmcblk0p1 $SD || exit 1
[ -f $SD/BOOT.bin.$1 ] || { echo "$SD/BOOT.bin.$1 is missing"; exit 1; }
cp $SD/BOOT.bin.$1 $SD/BOOT.bin.new && mv $SD/BOOT.bin.new $SD/BOOT.bin && sync || exit 1
echo "BOOT.bin = BOOT.bin.$1 ($(md5sum $SD/BOOT.bin | cut -d' ' -f1)), reboot"
umount $SD; reboot
