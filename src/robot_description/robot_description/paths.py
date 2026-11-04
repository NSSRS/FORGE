"""Resolve the shared MJCF assets in source checkouts and installed packages."""
import os
from pathlib import Path
import sys


def model_directory():
    override = os.environ.get('FORGE_MODEL_DIR')
    if override:
        path = Path(override).expanduser().resolve()
        if not (path / 'robot.xml').is_file():
            raise FileNotFoundError(f'FORGE_MODEL_DIR has no robot.xml: {path}')
        return path
    package_parent = Path(__file__).resolve().parents[1]
    candidates = [package_parent / 'mujoco',
                  package_parent / 'share/robot_description/mujoco',
                  Path(sys.prefix) / 'share/robot_description/mujoco']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('robot_description')) / 'mujoco')
    except (ImportError, LookupError):
        pass
    for path in candidates:
        if (path / 'robot.xml').is_file():
            return path
    raise FileNotFoundError('Robot assets not found; install robot_description or set FORGE_MODEL_DIR.')
