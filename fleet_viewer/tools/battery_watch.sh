#!/bin/bash
# Poll each car's VESC input voltage every 5 min (4S LiPo) over ssh (needs `ssh <car>` to work without a password).
# vesc_voltage.py is streamed to the car, nothing has to be installed there.
# Full log -> ~/.ros/fleet_battery.log ; stdout only when a car needs attention.
#   CARS="nuc1 nuc5" battery_watch.sh
CARS="${CARS:-nuc1 nuc4 nuc5}"
WARN=15.0      # 3.75 V/cell: swap soon
SWAP=14.6      # 3.65 V/cell: swap now
SETUP="${CAR_ROS_SETUP:-}"; [ -n "$SETUP" ] || SETUP='export PATH=$HOME/miniforge3/bin:$PATH; source ~/unicorn_ws/src/unicorn-racing-stack/unicorn.sh; source ~/expo_config/fleet_env.sh'
mkdir -p ~/.ros; LOG=~/.ros/fleet_battery.log
HERE=$(cd "$(dirname "$0")" && pwd)
while true; do
    ts=$(date +%H:%M)
    for c in $CARS; do
        v=$(timeout 30 ssh -o BatchMode=yes -o ConnectTimeout=5 $c "python3 - '$SETUP'" < "$HERE/vesc_voltage.py" 2>/dev/null || echo "ERR ssh")
        echo "$ts $c $v" >> $LOG
        if [[ $v == ERR* ]]; then
            echo "$ts $c 읽기 실패 ($v)"
        elif awk "BEGIN{exit !($v < $SWAP)}"; then
            echo "$ts $c ${v}V — 지금 교체 (<$SWAP V)"
        elif awk "BEGIN{exit !($v < $WARN)}"; then
            echo "$ts $c ${v}V — 곧 교체 (<$WARN V)"
        fi
    done
    sleep 300
done
