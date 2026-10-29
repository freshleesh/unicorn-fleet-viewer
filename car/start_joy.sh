#!/usr/bin/env bash
# pad plugged into this car -> /joy on the private domain (same params as fleet_viewer joy.launch.py)
#   start_joy.sh [device_id]     (device_id N = /dev/input/jsN, default 0)
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PATH=$HOME/miniforge3/bin:$PATH
source "${UNICORN_SETUP:-$HOME/unicorn_ws/src/unicorn-racing-stack/unicorn.sh}" >/dev/null 2>&1
source "$HERE/fleet_env.sh" || exit 1
exec ros2 run joy joy_node --ros-args -p device_id:=${1:-0} -p autorepeat_rate:=20.0 -p deadzone:=0.05
