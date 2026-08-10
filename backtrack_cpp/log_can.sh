#!/bin/bash
# Passive (listen-only) DroneCAN capture for a stand/field session.
#
# Assumes can0 is already up in LISTEN-ONLY mode (can0-listen.service does this
# at boot; or: ip link set can0 up type can bitrate 1000000 listen-only on).
# Writes the decoded can_sniff output to field_logs/<date>/. We use our own
# can_sniff (self-contained SocketCAN) because the board is offline and
# can-utils/candump is not installed.
#
# Usage: ./log_can.sh [seconds]   (default 120)
set -u
DUR=${1:-120}
HERE="$(cd "$(dirname "$0")" && pwd)"
D="$HERE/field_logs/$(date +%F)"
mkdir -p "$D"
STAMP=$(date +%H%M%S)
F="$D/sniff_${STAMP}_${DUR}s.log"
CSV="$D/esc_${STAMP}_${DUR}s.csv"
CMD="$D/cmd_${STAMP}_${DUR}s.csv"
echo "logging ${DUR}s (listen-only) -> $F"
echo "  esc.Status -> $CSV ; esc.RawCommand -> $CMD"
"$HERE/build/can_sniff" --seconds "$DUR" --esc-csv "$CSV" --cmd-csv "$CMD" | tee "$F"
