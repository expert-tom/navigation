#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")"
source /opt/ros/humble/setup.bash
exec python3 real_vehicle.py "$@"
