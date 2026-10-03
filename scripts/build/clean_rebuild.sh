#!/bin/bash
# Clean FPGA build (--hdl-only) in the devkit: wipes the IP cache and the generated Vivado project first, so every change in the RTL and in the
# project files is used. Run scripts/build/sync_hdl.sh and copy the project files (hdl/projects/skypluto) into the devkit before this.
#   DK    the fishball7020-fpga-devkit checkout (default: ~/fishball7020-fpga-devkit)
#   LOG   the build log (default: /tmp/hdl_build.log)
set -e
DK="${DK:-$HOME/fishball7020-fpga-devkit}"
LOG="${LOG:-/tmp/hdl_build.log}"
P=$DK/firmware/src/hdl/projects/pluto
IPC=$DK/firmware/src/hdl/ipcache

echo "=== wiping the IP cache (forces re-synthesis of the exciter from the current sources) ==="
rm -rf "$IPC"/* 2>/dev/null || true

echo "=== wiping the generated project (the sources stay: *.tcl, *.xdc, *.v) ==="
cd "$P"
rm -rf pluto.xpr pluto.cache pluto.gen pluto.hw pluto.ip_user_files pluto.runs pluto.sim pluto.srcs pluto.sdk .Xil 2>/dev/null || true

echo "=== clean build ==="
cd "$DK"
./devkit container build --hdl-only 2>&1 | tee "$LOG" | tail -3
echo "BUILD-EXIT=${PIPESTATUS[0]}"

echo "=== BOOT.bin ==="
ls -l "$DK/firmware/output/BOOT.bin"; md5sum "$DK/firmware/output/BOOT.bin"
echo "timing: $(grep -aE 'timing constraints are met|not met' "$P/timing.rpt" | head -1)"
