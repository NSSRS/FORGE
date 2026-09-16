#!/usr/bin/env bash
set -eo pipefail
if [[ $# -ne 1 || "$1" != "--execute" ]]; then
  echo "Usage: sudo $0 --execute" >&2
  exit 2
fi
if [[ $EUID -ne 0 ]]; then
  echo "Hardware EtherCAT access requires sudo." >&2
  exit 2
fi
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/scripts/ros2_env.sh"
set -u
exec ros2 run forge_motor_control forge_motor_driver --ros-args \
  --params-file "$workspace_dir/install/ros2/forge_motor_control/share/forge_motor_control/config/commissioning.yaml" \
  -p simulate:=false -p execute:=true
