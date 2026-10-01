#!/bin/sh
# skypluto-cmd.sh - send a protocol command to the daemon via the TCP console (localhost:5555) and show the reply.
#   skypluto-cmd.sh "?M"      skypluto-cmd.sh H 60      skypluto-cmd.sh "?G"
# Same protocol as the serial line (docs/serial-protocol.md). Note: on every resync the Pico resends E/F/P/K/B/H and thus overwrites
# manually set values again.
exec /mnt/jffs2/skypluto-ctl -c "$*"
