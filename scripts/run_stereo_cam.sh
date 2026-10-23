#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/scripts/ros2_env.sh"
source "$workspace_dir/scripts/camera_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec ros2 launch bringup stereo_cam.launch.py show_video:=true "$@"
