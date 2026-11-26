#!/usr/bin/env python3
"""Inspect a saved ENCOS feedback probe journal without opening a transport."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from motors.encos_debug import main

if __name__ == "__main__":
    raise SystemExit(main())
