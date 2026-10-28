#!/usr/bin/env bash
set -euo pipefail
# Base ros image already sources /opt/ros/jazzy in its entrypoint.
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/opt/app/bridge.yaml &
bridge_pid=$!
python3 -u /opt/app/gateway.py &
gateway_pid=$!
trap 'kill "$bridge_pid" "$gateway_pid" 2>/dev/null || true; wait || true' INT TERM EXIT
wait -n "$bridge_pid" "$gateway_pid"
