#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
if [[ $# -ne 0 ]]; then
  echo 'Usage: setup_motor_viewer.sh (uses the current full robot in src/robot_description/mujoco/)' >&2
  exit 2
fi
if [[ ! -f src/robot_description/mujoco/scene.xml || ! -f src/robot_description/mujoco/robot.xml || ! -d src/robot_description/mujoco/assets ]]; then
  echo 'Full robot model missing; restore src/robot_description/mujoco/ or run the Onshape export first.' >&2
  exit 1
fi
# Model files are read directly from this checkout; no historical download or copy.
python3 -m venv --system-site-packages build/motor_viewer/venv
build/motor_viewer/venv/bin/python -m pip install 'mujoco==3.13.0'
echo 'Ready: ./scripts/run_motor_viewer.sh'
