#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
if ! python3 -m venv .venv; then
    echo 'Install venv support: sudo apt-get install python3-venv' >&2
    exit 1
fi
.venv/bin/python -m pip install -r scripts/requirements-sim.txt
.venv/bin/python scripts/preview_sim.py --check
echo 'Ready. Run: source .venv/bin/activate && python scripts/preview_sim.py'
