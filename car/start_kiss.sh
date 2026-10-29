#!/usr/bin/env bash
# kiss_icp_localization on a GLIM map: start_kiss.sh [map_name]  (map at ~/maps/<name>/<name>.pcd + .yaml)
# Waits for a pose: use the "Pose: <car>" tool in the PC's RViz.
MAP="${1:-nuc4_map_1029_1905}"
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PATH=$HOME/miniforge3/bin:$PATH
source "${UNICORN_SETUP:-$HOME/unicorn_ws/src/unicorn-racing-stack/unicorn.sh}" >/dev/null 2>&1
source "$HERE/fleet_env.sh" || exit 1
CFG=$(ros2 pkg prefix kiss_icp_localization)/share/kiss_icp_localization/config/mid360_localization.yaml
export OMP_NUM_THREADS=8
exec ros2 run kiss_icp_localization localization_node --ros-args --params-file "$CFG" \
  -p map_pcd_3d:=$HOME/maps/$MAP/$MAP.pcd -p track_map_yaml:=$HOME/maps/$MAP/$MAP.yaml -r /kiss_loc/odometry:=/car_state/odom
