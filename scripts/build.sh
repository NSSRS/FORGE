#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
for tool in cmake gcc g++ make; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "Missing $tool. Install Ubuntu packages: sudo apt install build-essential cmake" >&2
    exit 1
  fi
done
cmake -S "$workspace_dir/src/encos_query" -B "$workspace_dir/build/encos_query" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$workspace_dir/build/encos_query" --parallel 2
ctest --test-dir "$workspace_dir/build/encos_query" --output-on-failure
