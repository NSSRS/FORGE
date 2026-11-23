#!/usr/bin/env python3
"""Run the machine's read-only Modbus probe from a FORGE source checkout."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from welding_machine.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
