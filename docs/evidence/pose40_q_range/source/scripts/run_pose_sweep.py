#!/usr/bin/env python3
"""Reproduce the fixed 40-pose, level-3 Q sweep experiment using CPU planning."""
import argparse
import csv
import datetime
import hashlib
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def prepare(output):
    rng = random.Random(20261013)
    bounds = [(-.1, -.07), (.17, .23), (.5, .56), (-12, 12), (70, 88), (-15, 15)]
    columns = []
    for lo, hi in bounds:
        values = [lo+(i+rng.random())/39*(hi-lo) for i in range(39)]
        rng.shuffle(values)
        columns.append(values)
    rows = [["pose01", -.09, .2, .5, 0, 80, 10]]
    rows += [[f"pose{i+2:02d}"]+[col[i] for col in columns] for i in range(39)]
    with (output/"requested_poses.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["case", "core_x", "core_y", "core_z", "core_theta_degrees",
                         "core_phi_degrees", "core_alpha_degrees"])
        writer.writerows(rows)
    config = dict(sampling="One prior reference pose plus 39 six-dimensional Latin-hypercube samples; fixed before trials",
                  rng_seed=20261013, requested_cases=40,
                  bounds=dict(zip(["x_m", "y_m", "z_m", "yaw_degrees", "phi_degrees", "roll_degrees"], bounds)),
                  pitch_definition="phi - 90 degrees", q_sequence_degrees=[0, 90, -90],
                  assembly_translation_m=[.2, .2, -.25], arm_base_z_m=-.1, motion_speed_scale=4,
                  max_ready_attempts=96, max_planning_attempts=24, planning_timeout_seconds=3,
                  offline_search_excluded_from_selected_planning_time=True)
    (output/"experiment.json").write_text(json.dumps(config, indent=2)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--online", action="store_true",
                        help="One live request from a fixed physical start; dense validation within 3 s")
    parser.add_argument("--retry", action="store_true",
                        help="Try online approach waypoints within the same 3 s request deadline; requires --online")
    parser.add_argument("--poses", type=Path, help="Reuse an exact requested_poses.csv without reading prior solutions")
    parser.add_argument("--best-effort-q", action="store_true", help="Maximize safe Q travel within the online request")
    parser.add_argument("--build-dir", type=Path, default=ROOT/"build-planning")
    args = parser.parse_args()
    if args.best_effort_q and not args.online:
        parser.error("--best-effort-q requires --online")
    if args.retry and not args.online:
        parser.error("--retry requires --online")
    output = args.output.resolve()
    if output.exists():
        parser.error("Use a new output directory; recorded experiments are not overwritten")
    output.mkdir(parents=True)
    prepare(output)
    if args.poses:
        shutil.copyfile(args.poses.resolve(), output/"requested_poses.csv")
    with (output/"requested_poses.csv").open(newline="") as stream:
        poses = list(csv.DictReader(stream))
    if len(poses) != 40 or len({r["case"] for r in poses}) != 40:
        parser.error("Exactly 40 distinct requested poses are required")
    config = json.loads((output/"experiment.json").read_text())
    config.update(requested_poses_sha256=hashlib.sha256((output/"requested_poses.csv").read_bytes()).hexdigest(),
                  requested_poses_source=str(args.poses.resolve()) if args.poses else "fixed generator")
    if args.online:
        config.update(planning_mode="online_fixed_start", max_ready_attempts=96 if args.retry else 0,
                      max_planning_attempts=24 if args.retry else 1, online_retry=args.retry,
                      offline_search_excluded_from_selected_planning_time=False,
                      physical_start_q_rad=[-.58298870078787879, 1.4014363481415861, 1.4956286729240151,
                                            .51538441396359136, -1.5264021873645037, -.58813609807777278,
                                            2.7857137959190528],
                      ready_search=args.retry, cross_request_warm_start=False, atlas=False,
                      timing_includes="scene configuration and cache reset, start collision check, online waypoint IK and failed retries if enabled, physical-start connection, approach and full task IK, time parameterization, dense cached collision/tracking/rule validation, return",
                      timing_excludes="one-time model load, CSV input/output, uncached equivalence audit, rendering, execution",
                      validation_joint_step_rad=.004, validation_min_substeps=4,
                      approach_joint_knot_step_rad=.02,
                      uncached_audit="Same density and tolerances as the timed validator; any mismatch aborts the experiment")
    config.update(q_best_effort=args.best_effort_q)
    if args.best_effort_q:
        config.update(pre_q_screen="Reject colliding solved endpoints before dense validation; never accept without every dense sample",
                      q_acceptance="Mandatory insert 100 mm, lift 100 mm, P 90 degrees; Q may stop short at a fully validated endpoint",
                      q_search="Positive toward +90 then reverse along checked path to 0 and extend toward -90; 1 degree increments; retain greatest 2*peak-final travel among live candidates",
                      q_range_claim="Largest verified travel found on searched IK branches within budget, not a globally maximal physical range",
                      online_search_limit_seconds=2.94, return_reserve_seconds=.06,
                      q_extension_validation_reserve=".2*prefix check + 1.3*positive checks + .2*negative checks + .06 s; exact-state collision memoization only within configured scene")
    (output/"experiment.json").write_text(json.dumps(config, indent=2)+"\n")
    if args.prepare_only:
        return
    environment = os.environ.copy()
    environment.update(CUDA_VISIBLE_DEVICES="", OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    logger = logging.getLogger("pose_sweep"); logger.setLevel(logging.INFO)
    handler = RotatingFileHandler(output/"planning.log", maxBytes=1024*1024, backupCount=2)
    logger.addHandler(handler)
    mujoco = ROOT/".deps/mujoco-3.3.7"
    libraries = [args.build_dir.resolve()/name for name in
                 ("librm26_planning.a", "libtech_core_sim.a", "libtech_core_rules.a")]
    if not all(path.exists() for path in libraries):
        parser.error("Build build-planning first; see docs/pose40_q_sweep.md")
    try:
        with tempfile.TemporaryDirectory(prefix=".plan-", dir=output) as temporary:
            binary = Path(temporary)/"planner"
            command = ["c++", "-std=c++17", "-O2", str(ROOT/"scripts/plan_pose_sweep.cpp"),
                       "-I"+str(ROOT/"include"), "-I/usr/include/eigen3", "-I"+str(mujoco/"include"),
                       '-DTECH_CORE_PLANNING_MODEL_PATH="'+str(ROOT/"models/planning/scene.xml")+'"',
                       '-DTECH_CORE_MODEL_PATH="'+str(ROOT/"models/tech_core.xml")+'"',
                       *map(str, libraries), "-L"+str(mujoco/"lib"), "-Wl,-rpath,"+str(mujoco/"lib"),
                       "-lmujoco", "-o", str(binary)]
            subprocess.run(command, env=environment, check=True, timeout=120)
            cpu = min(os.sched_getaffinity(0)); os.sched_setaffinity(0, {cpu})
            started = time.monotonic()
            planner_arguments = [str(output), "96", "24"] + (["--online-retry" if args.retry else "--online"] if args.online else [])
            if args.best_effort_q:
                planner_arguments += ["--best-effort-q"]
            metadata = dict(utc_start=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                            cpu_affinity=[cpu], CUDA_VISIBLE_DEVICES="",
                            source_sha256=hashlib.sha256((ROOT/"scripts/plan_pose_sweep.cpp").read_bytes()).hexdigest(),
                            command=["plan_pose_sweep", *planner_arguments],
                            git_head=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                            source_sha256_by_path={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                                   for p in [ROOT/"scripts/plan_pose_sweep.cpp", ROOT/"scripts/run_pose_sweep.py",
                                                             ROOT/"src/planning.cpp", ROOT/"src/planning_ik.cpp", ROOT/"src/planning_scene.cpp",
                                                             ROOT/"models/planning/scene.xml", *libraries]})
            (output/"execution.json").write_text(json.dumps(metadata, indent=2)+"\n")
            process = subprocess.Popen([str(binary), *planner_arguments], env=environment,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                for line in process.stdout:
                    logger.info(line.rstrip())
                    if line.startswith("CASE_COMPLETE"):
                        print(line, end="", flush=True)
                code = process.wait(timeout=10)
                metadata.update(exit_code=code, wall_seconds=time.monotonic()-started)
                (output/"execution.json").write_text(json.dumps(metadata, indent=2)+"\n")
                if code:
                    raise RuntimeError(f"Planner exited {code}; see planning.log")
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
                process.stdout.close()
    finally:
        handler.close()


if __name__ == "__main__":
    main()
