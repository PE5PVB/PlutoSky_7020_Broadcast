#!/bin/sh
# =============================================================================
# skypluto-autocal.sh - autonomous, fully INTERNAL TX calibration on the Pluto
# -----------------------------------------------------------------------------
# Runs on the Pluto itself. Triggers the chip's own calibration (internal cal loopback
# of the AD9361 - no cable, no TX_MONITOR) whenever the die temperature
# changes appreciably -> tracks temperature drift with minimal TX interruption.
#
#   image -> ~-64 dBc (tx_quad),  LO leakage -> ~-44 dBc (rf_dc_offs, chip floor)
#
# Usage:  skypluto-autocal.sh [dELTA_mdegC] [interval_s] [debounce_n]
#   dELTA_mdegC : recal threshold in millidegrees (default 3000 = 3.0 C)
#   interval_s  : temp-check interval (default 15 s)
#   debounce_n  : number of consecutive checks the deviation must PERSIST
#                 before a recal (default 4 -> 60 s at 15 s). Prevents thrashing
#                 (and thus carrier dropouts) with a steady-state temp oscillation
#                 around the threshold: only real, sustained drift calibrates.
# =============================================================================
PHY=ad9361-phy
THRESH=${1:-3000}
PERIOD=${2:-15}
NEED=${3:-4}

temp() { iio_attr -c $PHY temp0 input 2>/dev/null; }
cal() {
    iio_attr -d -q $PHY calib_mode rf_dc_offs >/dev/null 2>&1   # LO leakage
    iio_attr -d -q $PHY calib_mode tx_quad     >/dev/null 2>&1   # image
}

cal
last=$(temp)
pend=0
echo "autocal: start, init-cal @ $((last/1000)).$(( (last%1000)/100 )) C (drempel $((THRESH/1000))C, elke ${PERIOD}s, debounce ${NEED}x)"

while :; do
    sleep "$PERIOD"
    cur=$(temp); [ -z "$cur" ] && continue
    d=$((cur - last)); [ "$d" -lt 0 ] && d=$((-d))
    if [ "$d" -ge "$THRESH" ]; then
        pend=$((pend + 1))
        if [ "$pend" -ge "$NEED" ]; then
            cal
            echo "autocal: hercal @ $((cur/1000)) C (aanhoudende drift $((d/1000)) C, ${pend}x)"
            last=$cur
            pend=0
        fi
    else
        pend=0                    # deviation disappeared again -> no recal (anti-thrash)
    fi
done
