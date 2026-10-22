#!/usr/bin/env bash
# Install official ROS binary dependencies and rebuild the workspace packages.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-orbbec-camera ros-jazzy-orbbec-description ros-jazzy-cv-bridge python3-opencv python3-numpy
colcon --log-base logs/colcon build --base-paths src --build-base build/ros2 \
  --install-base install/ros2 --symlink-install --packages-up-to bringup
