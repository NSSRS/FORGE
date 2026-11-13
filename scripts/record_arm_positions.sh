#!/usr/bin/env bash
# A1/A4/A5/A6 initial positions; A2/A3 each CW then CCW travel ends.
# Startup/close/faults remove torque; rest the arm before changing applications.
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$script_dir/.."
mkdir -p build/arm_tests
run=$(mktemp -d "$PWD/build/arm_tests/positions-$(date +%Y%m%d-%H%M%S)-XXXXXX")
cat <<EOF
Recording to: $run/initial-positions.json
H holds all joints. Select 1–6, then E + hold an arrow to move.
A1: point the arm forward in its initial position.
A2: most clockwise travel end, save; then most counterclockwise travel end, save.
A3: most clockwise travel end, save; then most counterclockwise travel end, save.
A4, A5, A6: their initial positions.
CW/CCW: viewed from each motor's output-shaft side, toward the motor body.
Stop short of hard contact at A2/A3; do not drive into a stop.
At each position: release, Space, wait for Ready, C (or click Capture).
The Position recording panel shows Saved and the measured angle, or NOT SAVED with a reason.
After each Saved, follow the displayed next step. A2/A3 require both ends before Next.
Eight saves total. Other joints keep holding. D / Q / closing disables torque.
EOF
exec python3 scripts/record_arm_positions.py --execute --supported --manual-travel \
  --hold-all --smooth-jog --speed-deg-s 10 \
  --plan src/robot_description/config/arm_rig_a1_a6.json \
  --port /dev/ttyACM0 --capture-mode reference \
  --capture-file "$run/initial-positions.json" --report "$run/session.json"
