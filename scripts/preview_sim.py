"""Paused-by-default MuJoCo CAD preview; no control or hardware integration."""
from __future__ import annotations

import argparse
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

MODEL_DIR = Path(__file__).resolve().parents[1] / "src/forge_sim/model"


def check_scene(scene: Path, steps: int = 500) -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for _ in range(steps):
        mujoco.mj_step(model, data)
        if not all(np.isfinite(x).all() for x in (data.qpos, data.qvel, data.qacc)):
            raise RuntimeError("Simulation produced non-finite state")
        if any(w.number for w in data.warning):
            raise RuntimeError("MuJoCo emitted a simulation warning")
    print(f"PASS: {scene.name}; bodies={model.nbody}, joints={model.njnt}, "
          f"actuators={model.nu}, steps={steps}, time={data.time:.3f}s")
    return model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=MODEL_DIR / "scene.xml")
    parser.add_argument("--check", action="store_true", help="Headless load and 500-step smoke check")
    parser.add_argument("--run", action="store_true", help="Start with passive physics running")
    parser.add_argument("--seconds", type=float, help="Close viewer after this many seconds")
    args = parser.parse_args()
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive")
    if args.check:
        check_scene(args.scene.resolve())
        return

    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    running = threading.Event()
    if args.run:
        running.set()

    def on_key(key: int) -> None:
        if key == 32:  # Space
            if running.is_set():
                running.clear()
            else:
                running.set()

    print("MuJoCo preview: Space = pause/run; close the window to exit.", flush=True)
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "placeholder_base") >= 0:
        print("Setup placeholder only; export your Onshape assembly to replace it.", flush=True)
    start = time.monotonic()
    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        while viewer.is_running():
            tick = time.monotonic()
            if args.seconds is not None and tick - start >= args.seconds:
                break
            if running.is_set():
                mujoco.mj_step(model, data)
            viewer.sync()
            interval = model.opt.timestep if running.is_set() else 1 / 60
            time.sleep(max(0.0, interval - (time.monotonic() - tick)))


if __name__ == "__main__":
    main()
