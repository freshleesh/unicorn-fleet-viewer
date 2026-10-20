#!/usr/bin/env bash
# Stop everything the start_*.sh scripts launched on this car.
pkill -f "[r]ing_planner.py"; pkill -f "[f]leet_relay.py"; pkill -f "[l]ocalization_node"
pkill -f "[l]ow_level.launch.xml"; pkill -f "[c]amera.launch.py"; pkill -f "[s]imple_mux_node.py"
sleep 2; pgrep -af "[r]os2|[c]omponent_container|[v]esc_driver|[l]ivox" || echo "car: nothing left"
