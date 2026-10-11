#!/usr/bin/env python3
"""Replay a validated benchmark CSV using the original C++ scene and CPU OSMesa.

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
    parser.add_argument("--method", default="rigid_cache")
    parser.add_argument("--case", default="dev_nominal")
    parser.add_argument("--repeat", type=int, default=0)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--speed", type=float, default=1.)
    parser.add_argument("--still", type=float, default=None, help="One PNG frame at this trajectory time")
    parser.add_argument("--build-dir", type=Path, default=ROOT/"build-planning")
    args = parser.parse_args()
    if args.width < 960 or args.height < 600 or args.width % 2 or args.height % 2:
        parser.error("Use even dimensions of at least 960 x 600")
    if args.fps < 1 or not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("fps and speed must be positive")
    if args.still is not None and (not math.isfinite(args.still) or args.still < 0):
        parser.error("still time must be finite and nonnegative")
    source = args.directory.resolve(); output = args.output.resolve()
    if output.exists():
        parser.error("Output exists; choose a new path")
    required_suffix = ".png" if args.still is not None else ".mp4"
    if output.suffix != required_suffix:
        parser.error(f"Output must end in {required_suffix}")
    with (source / "runs.csv").open() as stream:
        selected = [r for r in csv.DictReader(stream) if r["method"] == args.method
                    and r["case"] == args.case and int(r["repeat"]) == args.repeat]
    if len(selected) != 1 or selected[0]["success"] != "1" or selected[0]["dense_recheck_valid"] != "1":
        parser.error("Select exactly one successful trajectory with a valid dense recheck")
    run = selected[0]; trajectory = source / run["trajectory"]
    motion_scale = float(run.get("motion_speed_scale", 1))
    with trajectory.open() as stream:
        knots = list(csv.DictReader(stream))
    execution_seconds = float(knots[-1]["time"])
    if args.still is not None and args.still > execution_seconds:
        parser.error("still time exceeds trajectory duration")
    output.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("planning_video"); logger.setLevel(logging.INFO)
    logfile = output.with_suffix(".render.log")
    handler = RotatingFileHandler(logfile, maxBytes=1024*1024, backupCount=2)
    logger.addHandler(handler); logger.addHandler(logging.StreamHandler())
    environment = os.environ.copy()
    environment.update(CUDA_VISIBLE_DEVICES="", LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe",
                       LP_NUM_THREADS="4", OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    libraries = [args.build_dir.resolve() / name for name in
                 ("librm26_planning.a", "libtech_core_sim.a", "libtech_core_rules.a")]
    if not all(p.exists() for p in libraries):
        parser.error("Build the Release build-planning targets first; see docs/planning_results.md")
    mujoco = ROOT / ".deps/mujoco-3.3.7"
    metadata = dict(method=args.method, case=args.case, repeat=args.repeat,
                    planning_mode=run.get("planning_mode", "offline_ready_search"),
                    planning_seconds=float(run["planning_seconds"]), execution_seconds=execution_seconds,
                    motion_speed_scale=motion_scale, mock_velocity_limit_rad_s=motion_scale,
                    mock_acceleration_limit_rad_s2=2*motion_scale*motion_scale,
                    playback_speed=args.speed, fps=args.fps, width=args.width, height=args.height,
                    source_trajectory=str(trajectory), source_trajectory_sha256=digest(trajectory),
                    scenario_csv_sha256=digest(source/"scenarios.csv"),
                    model_sha256=digest(ROOT/"models/planning/scene.xml"),
                    renderer_source_sha256=digest(ROOT/"scripts/render_planning_video.cpp"),
                    interpolation="q and mechanism: 3*u^2 - 2*u^3; no mj_step",
                    validation="Original C++ dense validator, cache disabled, rerun before rendering",
                    environment={key:environment[key] for key in
                                 ("CUDA_VISIBLE_DEVICES","LIBGL_ALWAYS_SOFTWARE","GALLIUM_DRIVER","LP_NUM_THREADS")})
    metadata["q_positive_degrees"] = max(float(k["twist"]) for k in knots)*180/math.pi
    metadata["q_final_degrees"] = float(knots[-1]["twist"])*180/math.pi
    metadata["full_q_sweep"] = run.get("full_q_sweep", "")
    renderer = encoder = None
    with tempfile.TemporaryDirectory(prefix=".render-", dir=output.parent) as temporary:
        temp = Path(temporary); binary = temp/"renderer"; pending = temp/output.name
        compile_command = ["c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic",
                           str(ROOT/"scripts/render_planning_video.cpp"),
                           "-I"+str(ROOT/"include"), "-I/usr/include/eigen3", "-I"+str(mujoco/"include"),
                           '-DTECH_CORE_PLANNING_MODEL_PATH="'+str(ROOT/"models/planning/scene.xml")+'"',
                           '-DTECH_CORE_MODEL_PATH="'+str(ROOT/"models/tech_core.xml")+'"',
                           *map(str,libraries), "-L"+str(mujoco/"lib"), "-Wl,-rpath,"+str(mujoco/"lib"),
                           "-lmujoco", "-lOSMesa", "-ldl", "-lpthread", "-o", str(binary)]
        built = subprocess.run(compile_command,env=environment,text=True,capture_output=True,timeout=120)
        if built.stdout or built.stderr: logger.info(built.stdout+built.stderr)
        if built.returncode: raise RuntimeError("Renderer compile failed")
        render_command = [str(binary),str(source),args.method,args.case,str(args.repeat),
                          str(args.width),str(args.height),str(args.fps),str(args.speed),
                          str(args.still if args.still is not None else -1)]
        encode_command = ["ffmpeg","-hide_banner","-loglevel","error","-nostdin","-f","rawvideo",
                          "-pixel_format","rgb24","-video_size",f"{args.width}x{args.height}",
                          "-framerate",str(args.fps),"-i","pipe:0","-vf","vflip","-filter_threads","1"]
        if args.still is not None:
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
                    if line.startswith("DENSE_RECHECK "): metadata["current_model_recheck"] = line
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
            expected = 1 if args.still is not None else math.ceil((execution_seconds/args.speed+2)*args.fps)
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
