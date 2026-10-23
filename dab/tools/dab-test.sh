#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# dab-test.sh - first DAB+ test on the Pluto (runs on the board, from /mnt/jffs2). Needs the DAB bitstream (build id B1D0001E).
#
#   dab-test.sh <frequency Hz> [tone Hz]    e.g. dab-test.sh 174928000         (channel 5A, test tone 1 kHz / 1.5 kHz)
#   dab-test.sh <frequency Hz> i2s          the stereo I2S input instead of the tone (needs a source at 48 kHz)
#
# Through the daemon: TX bandwidth 3 MHz (DAB is 1.536 MHz wide; 1.6 MHz would cut its edges), tune + calibration at the DAB
# frequency. Then skypluto-dab runs until Ctrl-C; afterwards the exciter is back in FM and the TX bandwidth is restored. The FM
# frequency is not restored: the Pico's next tune (F) sets it, or F by hand.
# The DAB signal is about 16 dB below the FM carrier at the same setting (crest factor), so P -15 gives roughly -31 dBm rms.
# The daemon's ALC (referenced on the FM carrier) sees the lower power and trims at most 3 dB up; the mask monitor judges
# against the FM mask and is meaningless during the test.
C=/mnt/jffs2/skypluto-ctl
D=/mnt/jffs2/skypluto-dab
F=${1:?usage: dab-test.sh <frequency Hz> [tone Hz | i2s]}
T=${2:-1000}

[ -x $D ] || { echo "$D is missing"; exit 1; }
$C -c "?V" | grep -q "bit=B1D0001E" || { echo "the DAB bitstream (B1D0001E) is not loaded: $($C -c '?V')"; exit 1; }

tb_old=$($C -c "?TB" | sed -n 's/^tb=\([0-9]*\).*/\1/p'); tb_old=${tb_old:-1600}
# a TB change and a tune both close the transmitter, calibrate and open it again; a tune sent while the TB calibration
# still runs is overtaken by it (the daemon then re-applies the old frequency), so one after the other
settle() {
    sleep 2; i=0
    while [ $i -lt 30 ]; do $C -c "?E" | grep -q "cal=ok" && $C -c "?S" | grep -q "tx=on" && break; sleep 1; i=$((i + 1)); done
}
echo "TX bandwidth 3000 kHz (was $tb_old)"
$C -c "TB 3000"; settle
echo "tune to $F Hz"
$C -c "F $F"; settle
$C -c "?S" | grep -q "f=$F " || { echo "the tune did not take: $($C -c '?S')"; $C -c "TB $tb_old"; exit 1; }
$C -c "?S"

restore() {
    trap - INT TERM
    echo "back to FM, TX bandwidth $tb_old kHz"
    $C -c "TB $tb_old"
}
trap restore INT TERM
if [ "$T" = "i2s" ]; then
    $D -v -d "PlutoSky DAB test" -s "PlutoSky" -e "PlutoSky DAB"
else
    $D -v -T "$T" -d "PlutoSky DAB test" -s "PlutoSky" -e "PlutoSky DAB"
fi
restore
