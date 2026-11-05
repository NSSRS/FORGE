#!/usr/bin/env python3
"""Read-only raw-reply diagnostics for Damiao on MC02."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from motors.arm_diag import main
if __name__ == '__main__':
    raise SystemExit(main())
