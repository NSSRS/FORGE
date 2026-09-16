#!/usr/bin/env bash
# Source this file from any directory.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this script: source scripts/ros2_env.sh" >&2
  exit 2
fi
forge_workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$forge_workspace_dir/install/ros2/setup.bash"
export PYTHONPATH="$forge_workspace_dir/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# Keep commissioning traffic off the dedicated EtherCAT interface and disable
# Fast DDS data sharing. Mixed root/non-root delivery is not reliable on this
# host, so run communicating hardware nodes under the same account.
unset FASTDDS_BUILTIN_TRANSPORTS
export FASTRTPS_DEFAULT_PROFILES_FILE="$forge_workspace_dir/install/ros2/forge_motor_control/share/forge_motor_control/config/fastdds-udp-loopback.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
unset forge_workspace_dir
