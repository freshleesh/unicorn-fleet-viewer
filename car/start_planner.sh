#!/usr/bin/env bash
# ring planner (A* lap + pure pursuit) on this car's private domain. Extra args -> ros2 args
#   start_planner.sh [map_name] [-p follow_gap:=0.6 ...]
# Then AUTO in the PC's Autodrive panel (or RB on this car's pad) hands the car over.
MAP="${1:-nuc4_map_1029_1905}"; shift
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PATH=$HOME/miniforge3/bin:$PATH
source "${UNICORN_SETUP:-$HOME/unicorn_ws/src/unicorn-racing-stack/unicorn.sh}" >/dev/null 2>&1
source "$HERE/fleet_env.sh" || exit 1
exec python3 "$HERE/ring_planner.py" --ros-args -p map:=$MAP "$@"
