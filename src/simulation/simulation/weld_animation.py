"""Render a checked controller plan in MuJoCo, without physics or motor owners.

Joint coordinates come from the saved raw-shaft path and its measured mapping.
The chassis follows nominal feed; participation switches do not alter arm motion.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess

import numpy as np


def load_plan(path, mapping_path):
    from controls.controller.limits import motor_constraints
    from controls.controller.weld_plan import JointPath
    from motors.arm_calibration import CalibrationMap, digest
    from simulation.arm_calibration import scene_fingerprint
    from simulation.paths import MODEL_DIR

    plan = json.loads(Path(path).read_text())
    mapping = CalibrationMap(json.loads(Path(mapping_path).read_text()))
    root = Path(__file__).resolve().parents[3]
    if plan.get("schema") != "forge.weld_motion_plan" or plan.get("version") != 1:
        raise ValueError("Expected a saved version-1 weld controller plan")
    if plan["mapping_fingerprint"] != mapping.fingerprint:
        raise ValueError("Plan uses a different encoder/model mapping")
    if plan["scene_fingerprint"] != scene_fingerprint(MODEL_DIR / "robot.xml"):
        raise ValueError("Plan uses a different model; regenerate the plan")
    current = motor_constraints(root, plan["max_speed_deg_s"], profile=plan["limit_profile"])
    if digest(current) != digest(plan["optimization"]["constraints"]):
        raise ValueError("Plan uses stale motor constraints; regenerate the plan")
    paths = {name: JointPath.from_document(plan[name]) for name in ("home", "approach", "weld")}
    if not np.allclose(paths["home"].positions[0], plan["current_raw_deg"], atol=1e-7, rtol=0):
        raise ValueError("Home does not start at the saved current pose")
    if not np.allclose(paths["home"].positions[-1], plan["initial_raw_deg"], atol=1e-7, rtol=0):
        raise ValueError("Home does not end at the saved standby")
    for a, b in (("home", "approach"), ("approach", "weld")):
        if not np.allclose(paths[a].positions[-1], paths[b].positions[0], atol=1e-6, rtol=0):
            raise ValueError(f"Discontinuous {a}/{b} transition")
    for name, path in paths.items():
        speed, acceleration = path.peaks()
        limits = plan["stage_limits"][name]
        if np.any(speed > np.array(limits["speed_deg_s"]) + 1e-5) or np.any(
            acceleration > np.array(limits["acceleration_deg_s2"]) + 1e-4
        ):
            raise ValueError(f"{name} exceeds its saved derivative limits")
    return plan, mapping, paths


class Timeline:
    def __init__(self, paths, speedup):
        self.paths = paths
        self.segments = [
            ("home", max(2.0, paths["home"].duration / speedup)),
            ("approach", paths["approach"].duration / speedup),
            ("ready", 1.2),
            ("weld", paths["weld"].duration),
            ("complete", 1.5),
        ]
        self.duration = sum(duration for _, duration in self.segments)
        self.weld_start = sum(duration for name, duration in self.segments[:3])

    def sample(self, movie_t):
        start = 0.0
        for name, duration in self.segments:
            if movie_t < start + duration or name == "complete":
                fraction = np.clip((movie_t - start) / duration, 0, 1)
                if name in self.paths:
                    local = fraction * self.paths[name].duration
                    raw, rate = self.paths[name].sample(float(local))
                    scale = self.paths[name].duration / duration
                else:
                    local = self.paths["weld"].duration if name == "complete" else 0.0
                    raw, _ = self.paths["weld"].sample(float(local))
                    rate = np.zeros(6)
                    scale = 1.0
                return name, float(local), raw, rate, scale
            start += duration
        raise RuntimeError("Invalid timeline")


def render(args):
    # MuJoCo imports after the backend has been selected. No visible GLFW window.
    if "microsoft" in platform.release().lower() and "MUJOCO_GL" not in os.environ:
        os.environ.setdefault("GALLIUM_DRIVER", "d3d12")
        os.environ.setdefault("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")
    import mujoco as mj
    from PIL import Image, ImageDraw, ImageFont
    from controls.controller.weld_plan import WeldGeometry
    from simulation.velocity_control import DRIVEN_JOINTS, WHEEL_DIRECTIONS

    plan, mapping, paths = load_plan(args.plan, args.mapping)
    timeline = Timeline(paths, args.approach_speedup)
    geometry = WeldGeometry(mapping)
    model, data = geometry.model, geometry.data
    seed = geometry.seed.copy()
    target_hz = next(
        row["native_mode2_feasibility"]["requested_hz"]
        for row in plan["optimization"]["candidates"]
        if row["name"] == plan["optimization"]["selected"]
    )
    limits = np.array(plan["raw_speed_limits_deg_s"])
    peaks = paths["weld"].peaks()[0]
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    fonts = {size: ImageFont.truetype(font_path, size) for size in (14, 16, 18, 22, 28)}
    colors = ["#48b7d9", "#8bbd86", "#c2ac70", "#bd94ce", "#e19393", "#80a4e4"]
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    video = output / "cmaes-weld-full.mp4"
    gif = output / "cmaes-weld-preview.gif"
    manifest_path = output / "cmaes-weld-animation.json"
    if not args.preview_only and any(p.exists() for p in (video, gif, manifest_path)):
        raise ValueError("Choose a new output directory; animation already exists")
    option = mj.MjvOption()
    option.geomgroup[3:] = 0
    option.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    overview = mj.MjvCamera()
    overview.lookat[:] = [0.0, 0.0, 0.14]
    overview.distance, overview.azimuth, overview.elevation = 1.32, 100, -34
    closeup = mj.MjvCamera()
    closeup.lookat[:] = [-0.435, 0.0, 0.040]
    closeup.distance, closeup.azimuth, closeup.elevation = 0.31, 90, -38
    model.vis.global_.offwidth = max(832, model.vis.global_.offwidth)
    model.vis.global_.offheight = max(500, model.vis.global_.offheight)
    renderer = mj.Renderer(model, height=500, width=832)
    zoom_renderer = mj.Renderer(model, height=260, width=400)
    trace_times = np.linspace(
        0, paths["weld"].duration, max(601, math.ceil(paths["weld"].duration * 200))
    )
    trace = []
    for time in trace_times:
        data.qpos[:] = seed
        geometry.check(paths["weld"].sample(float(time))[0])
        tip = data.site("wire_tip").xpos.copy()
        trace.append(
            [
                (plan["feed_m_s"] * time + tip[0] - geometry.origin[0]) * 1000,
                (tip[1] - geometry.origin[1]) * 1000,
            ]
        )
    trace = np.asarray(trace)
    feed_length = max(1.0, trace[-1, 0])
    amplitude_mm = float(np.max(np.abs(trace[:, 1])))
    plot_half_width = max(1.0, amplitude_mm * 1.2)
    accent = "#65d8bc"
    max_closure_mm = 0.0
    rendered = 0
    x0, x1, y0, y1 = 908, 1250, 435, 548
    graph = [
        (x0 + x / feed_length * (x1 - x0), (y0 + y1) / 2 - y / plot_half_width * (y1 - y0) / 2)
        for x, y in trace
    ]

    def frame(movie_t):
        nonlocal max_closure_mm, rendered
        stage, local, raw, rate, scale = timeline.sample(movie_t)
        stage_limits = np.array(
            plan["stage_limits"][stage if stage in ("home", "approach") else "weld"]["speed_deg_s"]
        )
        weld_t = local if stage in ("weld", "complete") else 0.0
        data.qpos[:] = seed
        geometry.check(raw)
        feed = plan["feed_m_s"] * weld_t
        # Render nominal forward rolling, independent of hardware output gates.
        data.qpos[0] += feed
        for name, sign in zip(DRIVEN_JOINTS, WHEEL_DIRECTIONS):
            data.joint(name).qpos[0] += sign * feed / 0.04
        mj.mj_forward(model, data)
        closure_mm = (
            np.linalg.norm(
                data.site_xpos[geometry.closure.sites[0]]
                - data.site_xpos[geometry.closure.sites[1]]
            )
            * 1000
        )
        max_closure_mm = max(max_closure_mm, float(closure_mm))
        overview.lookat[0] = feed
        closeup.lookat[0] = -0.435 + feed
        renderer.update_scene(data, camera=overview, scene_option=option)
        zoom_renderer.update_scene(data, camera=closeup, scene_option=option)
        canvas = Image.new("RGB", (1280, 800), "#101a29")
        canvas.paste(Image.fromarray(renderer.render()), (24, 100))
        canvas.paste(Image.fromarray(zoom_renderer.render()), (872, 100))
        draw = ImageDraw.Draw(canvas)

        def label(x, y, text, size=16, fill="#dce6f0"):
            draw.text((x, y), text, font=fonts[size], fill=fill)

        label(24, 15, "FORGE  /  CMA-ES WELD TRAJECTORY", 28)
        label(
            24,
            55,
            f'Target {target_hz:.2f} Hz   |   Achieved {plan["effective_hz"]:.3f} Hz'
            f'   |   Stroke +/-{amplitude_mm:.1f} mm   |   Feed {plan["feed_m_s"] * 1000:.2f} mm/s',
            18,
            accent,
        )
        names = dict(
            home="01  RECORDED / DERIVED STANDBY",
            approach="02  APPROACH WELD START",
            ready="03  READY AT WELD START",
            weld="04  WELDING",
            complete="05  COMPLETE / HOLD",
        )
        draw.rectangle((24, 100, 856, 139), fill="#1b2b40")
        label(38, 107, names[stage], 18)
        clock_label = f"{scale:.1f}x playback" if stage in ("home", "approach") else "1.0x playback"
        label(637, 110, clock_label, 16, accent)
        draw.rectangle((872, 100, 1272, 131), fill="#1b2b40")
        label(886, 106, "WELD START / WIRE TIP", 16)
        label(38, 570, "Full MuJoCo model  |  nominal chassis feed", 14, "#182638")
        draw.rectangle((872, 376, 1272, 600), fill="#19273b")
        label(886, 386, "ACTUAL PLANNED TIP PATH", 16)
        for yy in (y0, (y0 + y1) / 2, y1):
            draw.line((x0, yy, x1, yy), fill="#34445b", width=1)
        draw.line(graph, fill="#47576e", width=1)
        count = int(np.searchsorted(trace_times, weld_t, side="right"))
        if stage in ("weld", "complete") and count > 1:
            draw.line(graph[:count], fill=accent, width=2)
            xx, yy = graph[min(count - 1, len(graph) - 1)]
            draw.ellipse((xx - 4, yy - 4, xx + 4, yy + 4), fill="#f4c16e")
        label(882, 424, f"+{plot_half_width:.0f}", 14)
        label(882, 539, f"-{plot_half_width:.0f}", 14)
        label(910, 560, f"Forward travel 0 - {feed_length:.0f} mm", 14)
        label(24, 613, "OUTPUT-SHAFT SPEED   deg/s     |     instantaneous / weld peak", 16)
        for axis in range(6):
            col, row = axis % 3, axis // 3
            x, y = 24 + col * 278, 647 + row * 55
            label(x, y, f"A{axis+1}", 16, colors[axis])
            draw.rounded_rectangle((x + 37, y + 2, x + 157, y + 16), radius=3, fill="#2d3a50")
            length = min(1.0, abs(rate[axis]) / stage_limits[axis]) * 120
            if length > 1:
                draw.rectangle((x + 37, y + 2, x + 37 + length, y + 16), fill=colors[axis])
            label(x + 166, y - 1, f"{abs(rate[axis]):4.1f} / {peaks[axis]:4.1f}", 14)
            label(x + 37, y + 21, f"Stage limit {stage_limits[axis]:.2f}", 14, "#8c9db3")
        label(886, 623, f'Weld time  {weld_t:.2f} / {paths["weld"].duration:.2f} s', 18)
        label(886, 658, f'Cycles       {weld_t*plan["effective_hz"]:.1f}', 18)
        label(886, 693, f"Feed travel  {feed*1000:.1f} mm", 18)
        label(
            24,
            767,
            "Kinematic plan replay. Weight compensation: deferred LOAD-01. Hardware tracking is not validated.",
            16,
            "#9daec3",
        )
        rendered += 1
        return np.asarray(canvas)

    checkpoints = {
        "standby": 0.0,
        "approach": 2.0 + paths["approach"].duration / args.approach_speedup * 0.55,
        "weld": timeline.weld_start + paths["weld"].duration * 0.45,
        "complete": timeline.duration - 0.2,
    }
    try:
        for name, time in checkpoints.items():
            Image.fromarray(frame(time)).save(output / f"cmaes-weld-{name}.png")
        if args.preview_only:
            print(
                json.dumps(dict(preview_directory=str(output), checkpoints=checkpoints)), flush=True
            )
            return
        import imageio.v2 as imageio

        count = math.ceil(timeline.duration * args.fps)
        with imageio.get_writer(
            video,
            fps=args.fps,
            codec="libx264",
            quality=8,
            macro_block_size=16,
            ffmpeg_params=["-movflags", "+faststart"],
        ) as writer:
            for index in range(count):
                writer.append_data(frame(index / args.fps))
                if index % (args.fps * 4) == 0:
                    print(json.dumps(dict(event="render", frame=index, total=count)), flush=True)
    finally:
        renderer.close()
        zoom_renderer.close()
    # GIF is the real-time welding segment, not the accelerated approach.
    import imageio_ffmpeg

    graph = (
        "fps=15,scale=960:-1:flags=lanczos,split[a][b];"
        "[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=sierra2_4a"
    )
    subprocess.run(
        [
            imageio_ffmpeg.get_ffmpeg_exe(),
            "-hide_banner",
            "-loglevel",
            "error",
            "-ss",
            str(timeline.weld_start),
            "-t",
            str(paths["weld"].duration),
            "-i",
            str(video),
            "-filter_complex",
            graph,
            "-loop",
            "0",
            str(gif),
        ],
        check=True,
    )
    manifest = dict(
        schema="forge.weld_animation",
        version=1,
        source_plan=str(Path(args.plan).resolve()),
        source_plan_sha256=hashlib.sha256(Path(args.plan).read_bytes()).hexdigest(),
        mapping_fingerprint=plan["mapping_fingerprint"],
        scene_fingerprint=plan["scene_fingerprint"],
        target_hz=target_hz,
        effective_hz=plan["effective_hz"],
        feed_mm_s=plan["feed_m_s"] * 1000,
        measured_plan_amplitude_mm=amplitude_mm,
        selected=plan["optimization"]["selected"],
        stage_duration_s={key: p.duration for key, p in paths.items()},
        approach_playback_speedup=args.approach_speedup,
        weld_playback_speedup=1.0,
        video_duration_s=count / args.fps,
        video_fps=args.fps,
        video_frames=count,
        rendered_pose_checks=rendered,
        max_closure_error_mm=max_closure_mm,
        max_speed_deg_s=peaks.tolist(),
        speed_limits_deg_s=limits.tolist(),
        chassis_motion="assumed nominal feed; independent of output participation gates",
        home_start="saved current pose; the offline plan may already start at standby",
        weight_compensation="deferred until after the first arm welding run, LOAD-01",
        hardware_qualified=False,
        scope="kinematic path replay, not loaded dynamics or measured tracking",
    )
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest), flush=True)


def main(argv=None):
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument(
        "--mapping",
        type=Path,
        default=root / "src/robot_description/config/arm_calibration_measured.json",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--approach-speedup", type=float, default=12.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--preview-only", action="store_true")
    args = parser.parse_args(argv)
    if not math.isfinite(args.approach_speedup) or not 1 <= args.approach_speedup <= 60:
        parser.error("Approach playback speedup must be between 1 and 60")
    if not 15 <= args.fps <= 60:
        parser.error("Frame rate must be between 15 and 60")
    render(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
