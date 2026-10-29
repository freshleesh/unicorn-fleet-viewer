#!/usr/bin/env bash
# low_level (VESC + Mid-360 + OAK-D + mux) on this car's private fleet domain. Extra args -> ros2 launch
#   start_low_level.sh geofence_map:=nuc4_map_1029_1905
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PATH=$HOME/miniforge3/bin:$PATH
source "${UNICORN_SETUP:-$HOME/unicorn_ws/src/unicorn-racing-stack/unicorn.sh}" >/dev/null 2>&1
source "$HERE/fleet_env.sh" || exit 1
CAM=true; [ "$FLEET_CAR" = nuc1 ] && CAM=false     # nuc1 camera moved to nuc4 (2026-10-30)
exec ros2 launch stack_master low_level.launch.xml use_camera:=$CAM "$@"
