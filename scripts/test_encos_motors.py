#!/usr/bin/env python3
"""Source-checkout launcher for bounded four-motor ENCOS commissioning."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from motors.commissioning import main

if __name__ == '__main__':
    raise SystemExit(main(default_four_motors=True))
