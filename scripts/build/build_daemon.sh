#!/bin/bash
# Builds the control daemon (skypluto-ctl) and the mask tool (skypluto-mask) for the Pluto's ARM CPU with the devkit's cross compiler.
# The mask tool is built with -O3 -ffast-math (Cortex-A9, NEON/VFP).
#
#   DK    the fishball7020-fpga-devkit checkout (default: ~/fishball7020-fpga-devkit)
#   OUT   where the sources are staged and the binaries are written (default: <repository>/out/build)
set -e
HERE="$(cd "$(dirname "$0")/../.." && pwd)"
DK="${DK:-$HOME/fishball7020-fpga-devkit}"
OUT="${OUT:-$HERE/out/build}"
GCC="$DK/firmware/src/buildroot/output/host/bin/arm-linux-gnueabihf-gcc"
[ -x "$GCC" ] || { echo "cross compiler not found: $GCC (set DK to your devkit checkout)"; exit 1; }
mkdir -p "$OUT"
# stage the sources with Unix line endings (a Windows checkout may have CRLF)
for f in skypluto-ctl.c skypluto-mask.c web_index.h; do tr -d '\r' < "$HERE/src/$f" > "$OUT/$f"; done
$GCC -O2 -Wall -Wno-unused-variable -o "$OUT/skypluto-ctl" "$OUT/skypluto-ctl.c" -lm
$GCC -O3 -mcpu=cortex-a9 -mfpu=neon -ffast-math -Wall -Wno-unused-variable -o "$OUT/skypluto-mask" "$OUT/skypluto-mask.c" -lm -lpthread
ls -l "$OUT/skypluto-ctl" "$OUT/skypluto-mask"
md5sum "$OUT/skypluto-ctl" "$OUT/skypluto-mask"
