#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_dir"
# Preserve the pinned bench default; --local-model opts into the current CAD.
case "${1:-}" in
  "")
    model_revision=7aa258d7105f1c088bb059556a06899433d30cab
    if ! git cat-file -e "$model_revision:model/scene.xml" 2>/dev/null; then
      git fetch origin forge_sim
    fi
    mkdir -p build/motor_viewer
    git archive "$model_revision" model | tar -x -C build/motor_viewer
    printf '%s\n' "$model_revision" > build/motor_viewer/MODEL_REVISION
    ;;
  --local-model)
    python3 - <<'PY'
from pathlib import Path
import shutil
source=Path('model'); target=Path('build/motor_viewer/model')
if not (source/'scene.xml').is_file() or not (source/'robot.xml').is_file():
    raise SystemExit('Local model missing; run the Onshape export first.')
target.mkdir(parents=True,exist_ok=True)
for name in ('scene.xml','robot.xml'):
    shutil.copy2(source/name,target/name)
shutil.copytree(source/'assets',target/'assets',dirs_exist_ok=True)
Path('build/motor_viewer/MODEL_REVISION').write_text('local nathan model snapshot; rerun setup after CAD updates\n')
PY
    ;;
  *) echo 'Usage: setup_motor_viewer.sh [--local-model]' >&2; exit 2 ;;
esac
python3 -m venv --system-site-packages build/motor_viewer/venv
build/motor_viewer/venv/bin/python -m pip install 'mujoco==3.13.0'
echo 'Ready: ./scripts/run_motor_viewer.sh'
