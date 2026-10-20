#!/usr/bin/env bash
# fleet relay: private domain <-> shared domain 1, topics prefixed /<car>/
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PATH=$HOME/miniforge3/bin:$PATH
source "${UNICORN_SETUP:-$HOME/unicorn_ws/src/unicorn-racing-stack/unicorn.sh}" >/dev/null 2>&1
source "$HERE/fleet_env.sh" || exit 1
exec python3 "$HERE/fleet_relay.py" --car $FLEET_CAR --private $FLEET_PRIVATE --shared $FLEET_SHARED --color $FLEET_COLOR
