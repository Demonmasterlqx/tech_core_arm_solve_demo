#!/usr/bin/env python3
"""Render all 40 pose-sweep outcomes with the original C++ scene and CPU OSMesa.

Needs the Release build-planning libraries, libosmesa6-dev, Eigen3 and ffmpeg.
No dynamics or replanning is performed. Cubic interpolation matches validation.
"""
import argparse
import csv
import hashlib
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--still-page", type=int, choices=range(1, 6), default=None)
    parser.add_argument("--still-time", type=float, default=0.)
    args = parser.parse_args()
    if args.width < 1600 or args.height < 900 or args.width % 2 or args.height % 2:
        parser.error("Use even dimensions of at least 1600 x 900")
    if args.fps < 1 or not math.isfinite(args.still_time) or args.still_time < 0:
        parser.error("Invalid frame rate or still time")
    source = args.directory.resolve(); output = args.output.resolve()
    if output.exists():
        parser.error("Output exists; choose a new path")
    required_suffix = ".png" if args.still_page is not None else ".mp4"
    if output.suffix != required_suffix:
        parser.error(f"Output must end in {required_suffix}")
    with (source / "runs.csv").open() as stream:
        runs = list(csv.DictReader(stream))
    with (source / "scenarios.csv").open() as stream:
        scenarios = list(csv.DictReader(stream))
    if len(runs) != 40 or len(scenarios) != 40 or len({r["case"] for r in runs}) != 40:
        parser.error("Exactly 40 distinct cases are required")
    by_case = {r["case"]: r for r in runs}
    if set(by_case) != {s["case"] for s in scenarios}:
        parser.error("Run and scenario case sets differ")
    durations = []; trajectories = {}
    for scenario in scenarios:
        run = by_case[scenario["case"]]
        if run["success"] != "1":
            durations.append(0.); continue
        if run["dense_recheck_valid"] != "1" or scenario.get("q_sweep") != "1":
            parser.error("Successful cases must have a dense-checked Q sweep")
        trajectory = source / run["trajectory"]
        with trajectory.open() as stream:
            knots = list(csv.DictReader(stream))
        durations.append(float(knots[-1]["time"]))
        trajectories[run["case"]] = digest(trajectory)
    pages = [max(6., max(durations[i:i+8])+2.) for i in range(0, 40, 8)]
    output.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("planning_video"); logger.setLevel(logging.INFO)
    logfile = output.with_suffix(".render.log")
    handler = RotatingFileHandler(logfile, maxBytes=1024*1024, backupCount=2)
    logger.addHandler(handler); logger.addHandler(logging.StreamHandler())
    environment = os.environ.copy()
    environment.update(CUDA_VISIBLE_DEVICES="", LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe",
                       LP_NUM_THREADS="4", OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    libraries = [ROOT / "build-planning" / name for name in
                 ("librm26_planning.a", "libtech_core_sim.a", "libtech_core_rules.a")]
    if not all(p.exists() for p in libraries):
        parser.error("Build the Release build-planning targets first; see docs/planning_results.md")
    mujoco = ROOT / ".deps/mujoco-3.3.7"
    metadata = dict(cases=40, successes=sum(r["success"] == "1" for r in runs),
                    case_order=[s["case"] for s in scenarios], page_seconds=pages,
                    motion_speed_scale=4, playback_speed=1, fps=args.fps,
                    width=args.width, height=args.height, still_page=args.still_page,
                    trajectory_sha256=trajectories, scenario_csv_sha256=digest(source/"scenarios.csv"),
                    runs_csv_sha256=digest(source/"runs.csv"),
                    renderer_source_sha256=digest(ROOT/"scripts/render_pose_sweep.cpp"),
                    model_sha256=digest(ROOT/"models/planning/scene.xml"),
                    interpolation="q and mechanism: 3*u^2 - 2*u^3; no mj_step",
                    validation="Every successful trajectory dense-rechecked without cache; rejected cases are static",
                    environment={key:environment[key] for key in
                                 ("CUDA_VISIBLE_DEVICES","LIBGL_ALWAYS_SOFTWARE","GALLIUM_DRIVER","LP_NUM_THREADS")})
    renderer = encoder = None
    with tempfile.TemporaryDirectory(prefix=".render-", dir=output.parent) as temporary:
        temp = Path(temporary); binary = temp/"renderer"; pending = temp/output.name
        compile_command = ["c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic",
                           str(ROOT/"scripts/render_pose_sweep.cpp"),
                           "-I"+str(ROOT/"include"), "-I/usr/include/eigen3", "-I"+str(mujoco/"include"),
                           '-DTECH_CORE_PLANNING_MODEL_PATH="'+str(ROOT/"models/planning/scene.xml")+'"',
                           '-DTECH_CORE_MODEL_PATH="'+str(ROOT/"models/tech_core.xml")+'"',
                           *map(str,libraries), "-L"+str(mujoco/"lib"), "-Wl,-rpath,"+str(mujoco/"lib"),
                           "-lmujoco", "-lOSMesa", "-ldl", "-lpthread", "-o", str(binary)]
        built = subprocess.run(compile_command,env=environment,text=True,capture_output=True,timeout=120)
        if built.stdout or built.stderr: logger.info(built.stdout+built.stderr)
        if built.returncode: raise RuntimeError("Renderer compile failed")
        render_command = [str(binary),str(source),str(args.width),str(args.height),str(args.fps),
                          str(args.still_page-1 if args.still_page is not None else -1),str(args.still_time)]
        encode_command = ["ffmpeg","-hide_banner","-loglevel","error","-nostdin","-f","rawvideo",
                          "-pixel_format","rgb24","-video_size",f"{args.width}x{args.height}",
                          "-framerate",str(args.fps),"-i","pipe:0","-vf","vflip","-filter_threads","1"]
        if args.still_page is not None:
            encode_command += ["-frames:v","1","-update","1"]
        else:
            encode_command += ["-c:v","libx264","-preset","medium","-crf","19","-pix_fmt","yuv420p",
                               "-threads","2","-movflags","+faststart"]
        encode_command += [str(pending)]
        try:
            with (temp/"encoder.log").open("w+") as errors:
                renderer = subprocess.Popen(render_command,env=environment,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
                encoder = subprocess.Popen(encode_command,env=environment,stdin=renderer.stdout,stderr=errors)
                renderer.stdout.close()
                for raw in iter(renderer.stderr.readline,b""):
                    line = raw.decode(errors="replace").strip(); logger.info(line)
                    if line.startswith("OPENGL_RENDERER "): metadata["opengl_renderer"] = line.removeprefix("OPENGL_RENDERER ")
                    if line.startswith("VALIDATION_COMPLETE "): metadata["current_model_recheck"] = line
                render_code = renderer.wait(timeout=30); encode_code = encoder.wait(timeout=120)
                errors.seek(0); error_text = errors.read()
                if error_text: logger.info(error_text)
                if render_code or encode_code:
                    raise RuntimeError(f"Rendering failed: renderer={render_code}, ffmpeg={encode_code}")
            if "llvmpipe" not in metadata.get("opengl_renderer","") and "softpipe" not in metadata.get("opengl_renderer",""):
                raise RuntimeError("Software renderer was not verified")
            probe = subprocess.run(["ffprobe","-v","error","-count_frames","-show_streams","-show_format",
                                    "-of","json",str(pending)],text=True,capture_output=True,check=True,timeout=60)
            metadata["ffprobe"] = json.loads(probe.stdout)
            expected = 1 if args.still_page is not None else sum(math.ceil(seconds*args.fps) for seconds in pages)
            actual = int(metadata["ffprobe"]["streams"][0]["nb_read_frames"])
            if actual != expected: raise RuntimeError(f"Frame count {actual} != {expected}")
            metadata["frames"] = actual; metadata["output_sha256"] = digest(pending)
            metadata["output_bytes"] = pending.stat().st_size
            pending.replace(output)
            output.with_suffix(".json").write_text(json.dumps(metadata,indent=2)+"\n")
            logger.info("COMPLETE %s frames=%d bytes=%d",output,actual,output.stat().st_size)
        finally:
            for process in (renderer,encoder):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try: process.wait(timeout=10)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
            if renderer is not None and renderer.stderr is not None: renderer.stderr.close()
    handler.close()


if __name__ == "__main__":
    main()
