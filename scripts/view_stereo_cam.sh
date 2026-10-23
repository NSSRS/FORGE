#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_dir/scripts/ros2_env.sh"
source "$workspace_dir/scripts/camera_env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# OpenCV Qt windows use XWayland reliably on the native GNOME desktop.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec ros2 run stereo_cam view_camera "$@"
