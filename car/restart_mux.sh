#!/usr/bin/env bash
# Restart only simple_mux with its current parameters (e.g. to reload the geofence map after editing it).
# The car stops for ~2 s and comes back in neither mode (press RB / panel AUTO again). Log: ~/.ros/mux.log
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PATH=$HOME/miniforge3/bin:$PATH
source "${UNICORN_SETUP:-$HOME/unicorn_ws/src/unicorn-racing-stack/unicorn.sh}" >/dev/null 2>&1
source "$HERE/fleet_env.sh" || exit 1
P=~/.ros/simple_mux_params.yaml
ros2 param dump /simple_mux > $P.new && grep -q geofence_map_yaml $P.new && mv $P.new $P && sync
[ -f $P ] || { echo "no mux params (is low_level running?)"; exit 1; }
pkill -f "[s]imple_mux_node.py"; sleep 1
nohup ros2 run stack_master simple_mux_node.py --ros-args -r __node:=simple_mux --params-file $P > ~/.ros/mux.log 2>&1 &
