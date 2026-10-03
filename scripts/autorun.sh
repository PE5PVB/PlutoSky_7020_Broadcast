#!/bin/sh
# =============================================================================
# autorun.sh - SkyPluto_WFM boot bring-up (sourced by /etc/init.d/S98autostart)
# -----------------------------------------------------------------------------
# Comes up automatically after power-on with:
#   - transmitter CLOSED (TX LO off, atten 89): RF only after a tune (F) from the Pico (skypluto-ctl then opens the transmitter)
#   - full digital level, zero-IF (offset=0)
#   - I2S input active in the fabric (T9/U10/V10 = JP5-13/11/7, 3.3V):
#     as soon as your encoder delivers MPX, it modulates along automatically.
#   - the TX calibration (LO leakage, image) is done by the daemon at every tune, with the output muted
# Everything lives in the persistent jffs2 flash; nothing needs to be started manually.
# =============================================================================
CARRIER=108000000       # for bring-up only; the transmitter stays CLOSED (TX LO off, atten 89) until the Pico sends a tune (F)
ATTEN=89                # dB TX attenuation during boot: maximum (no RF)
JD=/mnt/jffs2

# Boot status to the Pico (status bar of the splash screen): '#B <pct> <text>' over the serial line (docs/serial-protocol.md). The
# daemon then repeats the last step until the Pico sends something itself.
bootmsg() { [ -x $JD/skypluto-ctl ] && $JD/skypluto-ctl -b "$1" >/dev/null 2>&1; }
bootmsg "10 Linux gestart"

# 1) wait until the AD9361 (iio) is ready (max ~20 s) and IMMEDIATELY switch the TX LO off + set the attenuation to maximum
i=0
while [ $i -lt 40 ]; do
    iio_attr -o -c ad9361-phy altvoltage1 frequency >/dev/null 2>&1 && break
    i=$((i+1)); sleep 0.5
done
bootmsg "30 Radio-chip klaar"
# direct sysfs writes (a few ms, checked) with the iio_attr tool as fallback; the '--' keeps a negative value from being read as an option
PHY=""
for d in /sys/bus/iio/devices/iio:device*; do [ "$(cat $d/name 2>/dev/null)" = "ad9361-phy" ] && PHY=$d; done
tx_mute() {
    if [ -n "$PHY" ]; then
        echo -89.75 > $PHY/out_voltage0_hardwaregain 2>/dev/null
        echo -89.75 > $PHY/out_voltage1_hardwaregain 2>/dev/null
        echo 1 > $PHY/out_altvoltage1_TX_LO_powerdown 2>/dev/null
    else
        iio_attr -q -o -c ad9361-phy voltage0 hardwaregain -- -89.75 >/dev/null 2>&1
        iio_attr -q -o -c ad9361-phy voltage1 hardwaregain -- -89.75 >/dev/null 2>&1
        iio_attr -q -o -c ad9361-phy altvoltage1 powerdown 1 >/dev/null 2>&1
    fi
}
tx_mute

# 2) set up the fabric/AD9361 operating point (zero-IF; offset reg default = 0) with maximum attenuation, then switch the TX LO off again
if [ -x $JD/skypluto-wfm.sh ]; then
    sh $JD/skypluto-wfm.sh start $CARRIER $ATTEN > /root/wfm_boot.log 2>&1
fi
tx_mute
bootmsg "45 Zender dicht"

# 3) start the control/telemetry daemon (UART U9 <-> PicoAudio encoder)
#    Translates the ASCII protocol (docs/serial-protocol.md) to iio_attr + devmem.
#    The Pico is master: it sets F/O/K/L/A/E + PT/PB/PE + CT/CE itself once the link is up.
#    Started via skypluto-supervise.sh, which keeps the daemon alive (restarts after a crash, cleans up leftover measurements).
# Stop the stock Analog Devices web server (busybox httpd -h /www, started by S41network): our web interface uses port 80.
for d in /proc/[0-9]*; do
    l=$(readlink "$d/exe" 2>/dev/null)
    case "$l" in *busybox*) case "$(tr '\0' ' ' < "$d/cmdline" 2>/dev/null)" in 'httpd -h /www'*) kill "$(basename $d)" 2>/dev/null;; esac;; esac
done
if [ -x $JD/skypluto-ctl ]; then
    # first stop any existing supervisor (recognisable by its command line), then any existing daemon (via the exe symlink)
    for d in /proc/[0-9]*; do
        c=$(tr '\0' ' ' < "$d/cmdline" 2>/dev/null)
        case "$c" in *skypluto-supervise*) kill -9 "$(basename $d)" 2>/dev/null;; esac
    done
    for d in /proc/[0-9]*; do
        l=$(readlink "$d/exe" 2>/dev/null)
        case "$l" in *skypluto-ctl*) kill -9 "$(basename $d)" 2>/dev/null;; esac
    done
    if [ -x $JD/skypluto-supervise.sh ]; then
        nohup sh $JD/skypluto-supervise.sh > /dev/null 2>&1 &
    else
        nohup $JD/skypluto-ctl > /tmp/ctl.log 2>&1 &
    fi
fi
