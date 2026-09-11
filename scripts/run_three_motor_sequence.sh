#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 4 || "$4" != "--execute" ]]; then
  echo "Usage: sudo $0 ETHERNET_INTERFACE CURRENT_LIMIT_A MOVE_DEG --execute" >&2
  echo "Runs: motors 1,2,3 together; then motor 1; motor 2; motor 3." >&2
  exit 2
fi

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
motion="$workspace_dir/build/encos_query/encos_motion"
if [[ ! -x "$motion" ]]; then
  echo "Missing $motion. Run ./scripts/build.sh first." >&2
  exit 1
fi

interface_name="$1"
current_limit="$2"
move_degrees="$3"
run_stage() {
  local ids="$1"
  echo "=== Motion stage: motor IDs $ids ==="
  "$motion" "$interface_name" "$current_limit" "$move_degrees" --motor-ids "$ids" --execute
}

run_stage 1,2,3
run_stage 1
run_stage 2
run_stage 3
