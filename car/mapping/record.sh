#!/usr/bin/env bash
# mapping bag for glim_mapping: record.sh <name>  -> ~/bag/<name> (mcap, /livox/lidar + /livox/imu). Ctrl+C to stop.
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PATH=$HOME/miniforge3/bin:$PATH
source "${UNICORN_SETUP:-$HOME/unicorn_ws/src/unicorn-racing-stack/unicorn.sh}" >/dev/null 2>&1
source "$HERE/../fleet_env.sh" || exit 1
mkdir -p ~/bag
exec ros2 bag record -s mcap -o ~/bag/"${1:?usage: record.sh <name>}" /livox/lidar /livox/imu
