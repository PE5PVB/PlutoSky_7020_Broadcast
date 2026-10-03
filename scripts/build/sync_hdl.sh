#!/bin/bash
# Copies the RTL library (hdl/library/skypluto_wfm) into the devkit's source tree, with Unix line endings.
#   DK   the fishball7020-fpga-devkit checkout (default: ~/fishball7020-fpga-devkit)
set -e
HERE="$(cd "$(dirname "$0")/../.." && pwd)"
DK="${DK:-$HOME/fishball7020-fpga-devkit}"
SRC="$HERE/hdl/library/skypluto_wfm"
DST="$DK/firmware/src/hdl/library/skypluto_wfm"
mkdir -p "$DST/data"
for f in skypluto_wfm_exciter skypluto_interp skypluto_uart_hd skypluto_async_fifo \
         skypluto_fm_modulator skypluto_sincos skypluto_axi_regs skypluto_i2s_rx skypluto_sigcond; do
    tr -d '\r' < "$SRC/$f.v" > "$DST/$f.v"
done
tr -d '\r' < "$SRC/data/interp_coefs.mem" > "$DST/data/interp_coefs.mem"
tr -d '\r' < "$SRC/data/sine_lut.mem" > "$DST/data/sine_lut.mem"
echo "synced to the devkit:"
ls -1 "$DST"/*.v; echo "data:"; ls -1 "$DST/data"
