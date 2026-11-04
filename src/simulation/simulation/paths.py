"""Shared model location and explicit source-only export workspace."""
from pathlib import Path
from robot_description.paths import model_directory

MODEL_DIR = model_directory()


def source_workspace():
    root = Path(__file__).resolve().parents[3]
    if not (root / 'scripts/export_onshape.py').is_file():
        raise RuntimeError('CAD export/integration requires a source checkout; use scripts/ launchers.')
    return root
