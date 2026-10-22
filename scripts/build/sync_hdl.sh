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
         skypluto_fm_modulator skypluto_sincos skypluto_axi_regs skypluto_i2s_rx skypluto_sigcond skypluto_chanfilt skypluto_dab_ofdm skypluto_async_fifo_bram; do
    tr -d '\r' < "$SRC/$f.v" > "$DST/$f.v"
done
for f in skypluto_dab_twiddle skypluto_dab_interp_coefs; do
    tr -d '\r' < "$SRC/$f.vh" > "$DST/$f.vh"
done
tr -d '\r' < "$SRC/data/interp_coefs.mem" > "$DST/data/interp_coefs.mem"
tr -d '\r' < "$SRC/data/sine_lut.mem" > "$DST/data/sine_lut.mem"
echo "synced to the devkit:"
ls -1 "$DST"/*.v "$DST"/*.vh; echo "data:"; ls -1 "$DST/data"
