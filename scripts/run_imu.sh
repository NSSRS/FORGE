#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
source scripts/ros2_env.sh
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
port="${1:-/dev/ttyUSB0}"
if [[ ! -e "$port" ]]; then
  echo "IMU serial device not attached: $port" >&2
  echo 'Check the USB connection and serial port path. Pass a different device as the first argument.' >&2
  exit 1
fi
if [[ ! -r "$port" || ! -w "$port" ]]; then
  echo "IMU is attached, but this terminal lacks permission for $port." >&2
  echo 'Run: newgrp dialout' >&2
  echo 'Then run this launcher again. This refreshes the serial-access group for the shell.' >&2
  exit 1
fi
exec ros2 run imu imu --ros-args -p "port:=$port" -p baud:=9600 -p address:=80
