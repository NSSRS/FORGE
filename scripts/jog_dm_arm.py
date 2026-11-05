#!/usr/bin/env python3
"""Interactive Damiao arm joint jog; simulation by default."""
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src'),str(root/'src/teleop')]
from teleop.arm_keyboard import main
if __name__ == '__main__':raise SystemExit(main())
