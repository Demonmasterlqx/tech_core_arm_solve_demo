#!/usr/bin/env python3
"""Fixed development/confirmation corpus and reproducible two-backend CPU runs."""
import argparse
import csv
import hashlib
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import random
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
BOUNDS = [(-.29, .11), (0, .4), (.3, .7), (-90, 90), (0, 180), (-90, 90)]


def prepare(path):
    path.mkdir(parents=True, exist_ok=False)
    rows = []
    for split, count, seed in [('development', 32, 20261011), ('confirmation', 100, 20261012)]:
        rng = random.Random(seed)
        columns = []
        for lo, hi in BOUNDS:
            values = [lo + (i + rng.random()) / count * (hi - lo) for i in range(count)]
            rng.shuffle(values)
            columns.append(values)
        poses = list(zip(*columns))
        if split == 'development':
            poses[0] = (-.09, .2, .5, 0, 90, 0)
        for i, pose in enumerate(poses):
            for start in ('zero', 'preset'):
                rows.append([f'{split}_{i:03d}_{start}', split, 'demo', start, *pose])
    legacy = ROOT / 'docs/evidence/pose40_q_range/requested_poses.csv'
    with legacy.open() as stream:
        for row in csv.DictReader(stream):
            fields = ['core_x', 'core_y', 'core_z', 'core_theta_degrees', 'core_phi_degrees', 'core_alpha_degrees']
            rows.append(['legacy_' + row['case'], 'regression', 'legacy', 'legacy', *[float(row[k]) for k in fields]])
    for start in ('zero', 'preset'):
        rows.append([f'blocked_{start}', 'negative', 'blocked', start, -.09, .2, .5, 0, 90, 0])
        rows.append([f'unreachable_{start}', 'negative', 'unreachable', start, -.09, .2, 2, 0, 90, 0])
    suite = path / 'cases.csv'
    with suite.open('w', newline='') as stream:
        out = csv.writer(stream)
        out.writerow(['case', 'split', 'group', 'start', 'core_x', 'core_y', 'core_z', 'core_theta', 'core_phi', 'core_alpha'])
        out.writerows(rows)
    protocol = dict(
        bounds_xyz_yaw_phi_roll=BOUNDS, phi_definition='pitch + 90 degrees',
        physical_installation='Demo defaults: assembly (0.3,0.2,-0.25), yaw pi, robot base_z 0',
        development_poses=32, confirmation_poses=100, starts=['seven joint zero', 'assembly preset'],
        sampling='Independent deterministic six-dimensional Latin hypercubes fixed before optimization',
        primary_population='All confirmation demo requests, separately per backend and starting posture. Reachability certificates are reported separately; no failure is assumed unreachable',
        success='Original best-effort Q contract, mandatory insertion/lift/P complete, full uncached revalidation',
        target_success_rate=.90, request_deadline_seconds=5,
        joint_metric='Approach total absolute joint travel and final joint travel are primary; paired on common successful requests. Mandatory phases separately from optional Q travel',
        q_metrics='Positive, negative and total Q travel; no motion reduction claim by truncating Q',
        legacy_regression='All 40 original poses and original installation/start; reported separately',
        negative_controls='Blocked and unreachable requests excluded from reasonable-pose denominator by construction, required to reject',
        online_seed_policy='No saved per-pose solution, no cross-request warm start',
        suite_sha256=hashlib.sha256(suite.read_bytes()).hexdigest())
    (path / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')


def compile_binary(output, build):
    mujoco = ROOT / '.deps/mujoco-3.3.7'
    binary = output / 'planner'
    sources = [ROOT / 'scripts/benchmark_planner_quality.cpp', ROOT / 'src/planning.cpp',
               ROOT / 'src/planning_ik.cpp', ROOT / 'src/planning_approach.cpp', ROOT / 'src/planning_scene.cpp',
               ROOT / 'include/tech_core/planning.hpp', ROOT / 'include/tech_core/planning_ik.hpp',
               ROOT / 'models/planning/scene.xml']
    libraries = [build / name for name in ('librm26_planning.a', 'libtech_core_sim.a', 'libtech_core_rules.a')]
    command = ['c++', '-std=c++17', '-O2', str(sources[0]), '-I' + str(ROOT/'include'),
               '-I/usr/include/eigen3', '-I' + str(mujoco/'include'),
               '-DTECH_CORE_PLANNING_MODEL_PATH="' + str(ROOT/'models/planning/scene.xml') + '"',
               '-DTECH_CORE_MODEL_PATH="' + str(ROOT/'models/tech_core.xml') + '"', *map(str, libraries),
               '-L' + str(mujoco/'lib'), '-Wl,-rpath,' + str(mujoco/'lib'), '-lmujoco', '-o', str(binary)]
    subprocess.run(command, check=True, timeout=120)
    return binary, {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources + libraries}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path, required=True)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--build-dir', type=Path, default=ROOT/'build')
    parser.add_argument('--binary', type=Path, help='Reuse a frozen evaluator (e.g. the original baseline binary)')
    parser.add_argument('--split', default='development', choices=['development', 'confirmation', 'regression', 'negative', 'all'])
    parser.add_argument('--method', default='rigid_cache', choices=['baseline', 'rigid_cache'])
    parser.add_argument('--cpu', type=int, default=0)
    parser.add_argument('--timeout', type=float, default=5)
    args = parser.parse_args()
    if args.prepare_only:
        prepare(args.suite)
        return
    if args.output is None:
        parser.error('--output is required')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(CUDA_VISIBLE_DEVICES='', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    os.sched_setaffinity(0, {args.cpu})
    binary, sources = (args.binary.resolve(), {}) if args.binary else compile_binary(args.output, args.build_dir.resolve())
    metadata = dict(method=args.method, split=args.split, cpu=args.cpu, CUDA_VISIBLE_DEVICES='',
                    git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                    source_hashes=sources, binary=str(binary), binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                    suite_sha256=hashlib.sha256((args.suite/'cases.csv').read_bytes()).hexdigest(), timeout_seconds=args.timeout)
    command = [str(binary), str(args.suite.resolve()/'cases.csv'), str(args.output.resolve()/'results'),
               args.method, args.split, str(args.timeout)]
    metadata['command'] = command
    (args.output/'execution.json').write_text(json.dumps(metadata, indent=2)+'\n')
    logger = logging.getLogger('quality');logger.setLevel(logging.INFO)
    handler = RotatingFileHandler(args.output/'planning.log', maxBytes=1024*1024, backupCount=2)
    logger.addHandler(handler)
    begin = time.monotonic()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        for line in process.stdout:
            logger.info(line.rstrip())
            print(line, end='', flush=True)
        code = process.wait(timeout=10)
        metadata.update(exit_code=code, wall_seconds=time.monotonic()-begin)
        (args.output/'execution.json').write_text(json.dumps(metadata, indent=2)+'\n')
        if code:
            raise RuntimeError(f'Planner exited {code}')
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill();process.wait()
        process.stdout.close();handler.close()


if __name__ == '__main__':
    main()
