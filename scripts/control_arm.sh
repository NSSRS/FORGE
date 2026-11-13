#!/usr/bin/env bash
# Manual control only; no reference/bounds capture workflow.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$script_dir/.."
exec python3 scripts/jog_dm_arm.py --execute --supported --manual-travel \
  --hold-all --smooth-jog --speed-deg-s 10 \
  --plan src/robot_description/config/arm_rig_a1_a6.json --port /dev/ttyACM0 "$@"
