#!/usr/bin/env bash
# (re)start only the OAK-D driver (same settings as low_level), e.g. after X_LINK_ERROR. Log: ~/.ros/oak.log
pkill -f "[c]amera.launch.py"; pkill -f "[c]omponent_container.*oak"; pkill -f "__node:=oak_container"; sleep 3
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PATH=$HOME/miniforge3/bin:$PATH
source "${UNICORN_SETUP:-$HOME/unicorn_ws/src/unicorn-racing-stack/unicorn.sh}" >/dev/null 2>&1
source "$HERE/fleet_env.sh" || exit 1
CFG=$(ros2 pkg prefix stack_master)/share/stack_master/config/CAR/oak_rgb.yaml
nohup ros2 launch depthai_ros_driver camera.launch.py params_file:=$CFG rectify_rgb:=false use_rviz:=false > ~/.ros/oak.log 2>&1 &
