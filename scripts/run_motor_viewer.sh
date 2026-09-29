#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
if [[ ! -x build/motor_viewer/venv/bin/python ]]; then
  echo 'Run ./scripts/setup_motor_viewer.sh first.' >&2
  exit 1
fi
exec build/motor_viewer/venv/bin/python scripts/mujoco_motor_viewer.py "$@"
