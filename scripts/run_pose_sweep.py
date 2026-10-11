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
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error("Use a new output directory; recorded experiments are not overwritten")
    output.mkdir(parents=True)
    prepare(output)
    if args.prepare_only:
        return
    environment = os.environ.copy()
    environment.update(CUDA_VISIBLE_DEVICES="", OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    logger = logging.getLogger("pose_sweep"); logger.setLevel(logging.INFO)
    handler = RotatingFileHandler(output/"planning.log", maxBytes=1024*1024, backupCount=2)
    logger.addHandler(handler)
    mujoco = ROOT/".deps/mujoco-3.3.7"
    libraries = [ROOT/"build-planning"/name for name in
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
            metadata = dict(utc_start=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                            cpu_affinity=[cpu], CUDA_VISIBLE_DEVICES="",
                            source_sha256=hashlib.sha256((ROOT/"scripts/plan_pose_sweep.cpp").read_bytes()).hexdigest(),
                            command=["plan_pose_sweep", str(output), "96", "24"])
            (output/"execution.json").write_text(json.dumps(metadata, indent=2)+"\n")
            process = subprocess.Popen([str(binary), str(output), "96", "24"], env=environment,
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
