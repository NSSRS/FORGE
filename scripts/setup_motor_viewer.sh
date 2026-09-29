#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
# Model-only snapshot; never switch the hardware checkout or execute branch code.
model_revision=7aa258d7105f1c088bb059556a06899433d30cab
if ! git cat-file -e "$model_revision:model/scene.xml" 2>/dev/null; then
  git fetch origin forge_sim
fi
mkdir -p build/motor_viewer
git archive "$model_revision" model | tar -x -C build/motor_viewer
printf '%s\n' "$model_revision" > build/motor_viewer/MODEL_REVISION
python3 -m venv --system-site-packages build/motor_viewer/venv
build/motor_viewer/venv/bin/python -m pip install 'mujoco==3.13.0'
echo 'Ready: ./scripts/run_motor_viewer.sh'
