#!/bin/sh
# skypluto-supervise.sh - keeps the control daemon (skypluto-ctl) alive.
# Started by autorun.sh. If the daemon stops (crash or kill), this script cleans up leftover measurements (which hold the RX chain) and restarts the
# daemon after 2 s; after 5 quick crashes in a row (< 10 s) it waits 30 s. The daemon CLOSES the transmitter on every start until the Pico tunes again.
# The log (/tmp/ctl.log, in RAM) is appended to and size-limited; /tmp/ctl_restarts.log counts the restarts.
JD=/mnt/jffs2
fast=0
while true; do
    for d in /proc/[0-9]*; do
        l=$(readlink "$d/exe" 2>/dev/null)
        case "$l" in *skypluto-mask*|*iio_readdev*) kill -9 "$(basename $d)" 2>/dev/null;; esac
    done
    if [ -f /tmp/ctl.log ] && [ "$(wc -c < /tmp/ctl.log)" -gt 600000 ]; then
        tail -c 200000 /tmp/ctl.log > /tmp/ctl.log.new && mv /tmp/ctl.log.new /tmp/ctl.log
    fi
    t0=$(cut -d. -f1 /proc/uptime)
    $JD/skypluto-ctl >> /tmp/ctl.log 2>&1
    rc=$?
    t1=$(cut -d. -f1 /proc/uptime)
    echo "uptime $(cut -d. -f1 /proc/uptime) s: skypluto-ctl beeindigd (rc $rc na $((t1 - t0)) s): herstart" >> /tmp/ctl.log
    echo "uptime $(cut -d. -f1 /proc/uptime) s rc=$rc" >> /tmp/ctl_restarts.log
    if [ $((t1 - t0)) -lt 10 ]; then fast=$((fast + 1)); else fast=0; fi
    if [ $fast -ge 5 ]; then sleep 30; else sleep 2; fi
done
