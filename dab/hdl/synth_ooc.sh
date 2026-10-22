#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Out-of-context synthesis + implementation of skypluto_dab_ofdm alone (resources and timing at 12.288 MHz), in the devkit's
# Vivado container. Nothing of the FM project is touched; the files are copied to $DK/firmware/dab_ooc.
set -e
DK="${DK:-$HOME/fishball7020-fpga-devkit}"
HERE=$(cd "$(dirname "$0")" && pwd)
W=$DK/firmware/dab_ooc
rm -rf "$W"; mkdir -p "$W"
cp "$HERE"/skypluto_dab_ofdm.v "$HERE"/skypluto_dab_interp_coefs.vh "$HERE"/skypluto_dab_twiddle.vh "$W"/
cat > "$W/ooc.tcl" <<'EOF'
read_verilog skypluto_dab_ofdm.v
synth_design -top skypluto_dab_ofdm -part xc7z020clg400-2 -mode out_of_context -include_dirs .
create_clock -period 81.380 -name l_clk [get_ports clk]
opt_design
place_design
route_design
report_utilization -file util.rpt
report_timing_summary -file timing.rpt
EOF
# the devkit container with its Vivado environment (tools/env-vivado.sh supplies the libraries Vivado 2022.2 misses)
cat > "$W/run.sh" <<'EOF'
source tools/env-vivado.sh
cd firmware/dab_ooc
vivado -mode batch -nojournal -nolog -source ooc.tcl > ooc.log 2>&1
echo "vivado-exit=$?"
EOF
cd "$DK" && ./devkit container shell firmware/dab_ooc/run.sh
grep -E "Slice LUTs|Slice Registers|Block RAM Tile|DSPs" "$W/util.rpt" | head -6
grep -E "WNS\(ns\)" -A2 "$W/timing.rpt" | head -3
grep -iE "^ERROR|CRITICAL WARNING" "$W/ooc.log" | head -10
tail -3 "$W/ooc.log"
