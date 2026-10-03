#!/bin/sh
# skypluto-supervise.sh - keeps the control daemon (skypluto-ctl) alive.
# Started by autorun.sh. If the daemon stops (crash or kill), this script cleans up leftover measurements (which hold the RX chain) and restarts the
# daemon after 2 s; after 5 quick crashes in a row (< 10 s) it waits 30 s. The daemon CLOSES the transmitter on every start until the Pico tunes again.
# It also starts DHCP on eth0 when the cable is plugged in after boot. The log (/tmp/ctl.log, in RAM) is appended to and size-limited; /tmp/ctl_restarts.log counts the restarts.
JD=/mnt/jffs2

# DHCP watcher: the stock boot runs 'udhcpc -n' once, so with no cable at that moment eth0 never gets an address when the cable is plugged in later.
# Every 2 s: if eth0 has a link but no IPv4 address and no udhcpc is running for it, start one (same options as the stock ifup).
W=$JD/dhcp-watch.log; nlog=0; [ -s $W ] && cp $W $W.prev     # keep the log of the previous boot
wlog() { [ $nlog -lt 30 ] && echo "uptime $(cut -d. -f1 /proc/uptime) s: $1" >> $W; nlog=$((nlog + 1)); }     # events only, capped: it is written to flash
(
    : > $W; wlog "dhcp watcher started"; lastc=x; noaddr=0
    while true; do
        sleep 2
        ip link set eth0 up 2>/dev/null     # a failed 'ifup' at boot can leave eth0 down; a down interface has no readable carrier
        c=$(cat /sys/class/net/eth0/carrier 2>/dev/null)
        [ "$c" != "$lastc" ] && { wlog "eth0 carrier '${c:-?}'"; lastc=$c; }
        if [ "$c" != "1" ] || ip addr show dev eth0 2>/dev/null | grep "inet " | grep -qv "169\.254\."; then noaddr=0; continue; fi
        # (an avahi link-local 169.254.x.x address does not count as an address)
        # a link but no address for 3 checks in a row (6 s): the stock udhcpc (if still around) did not get one, so replace it
        noaddr=$((noaddr + 1)); [ $noaddr -lt 3 ] && continue
        wlog "eth0 has a link but no address; stale udhcpc processes: $(ps | grep '[u]dhcpc' | wc -l)"
        killall udhcpc 2>/dev/null; sleep 1
        udhcpc -R -n -p /var/run/udhcpc.eth0.pid -i eth0 -x hostname:pluto >/dev/null 2>&1
        wlog "udhcpc finished, address: $(ip addr show dev eth0 | grep 'inet ' | tr -s ' ' | cut -d' ' -f3)"
        noaddr=0
    done
) &

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
