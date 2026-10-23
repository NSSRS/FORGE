#!/usr/bin/env bash
# Source after ros2_env.sh for same-account, camera-only ROS processes.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Source this script after scripts/ros2_env.sh" >&2
  exit 2
fi
camera_workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
camera_dds_profile="$camera_workspace_dir/install/ros2/bringup/share/bringup/config/fastdds-camera-shm.xml"
if [[ ! -f "$camera_dds_profile" ]]; then
  echo "Camera DDS profile missing; run bash scripts/setup_stereo_cam.sh" >&2
  unset camera_workspace_dir camera_dds_profile
  return 2
fi
# Let XML own transports; Jazzy LOCALHOST would inject extra transports.
# The XML still restricts UDP to loopback.
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$camera_dds_profile"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
unset camera_workspace_dir camera_dds_profile
