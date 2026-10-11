#!/usr/bin/env python3
"""Audit the fixed quality corpus, preserving all failures and reachability unknowns."""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from types import SimpleNamespace

import numpy as np

from audit_planning_trajectories import (
    UrdfArm, csv_rows, grasp_from_xml, physical_target, scene_pose, trajectory_audit,
)

ROOT = Path(__file__).resolve().parents[1]
PRESET = [-.62770373654521094, 2.3297993305507192, 1.0289871622237392,
          1.2715734368717495, -1.747192543846495, -.81681197466957878, 1.9772845777886507]
LEGACY = [-.58298870078787879, 1.4014363481415861, 1.4956286729240151,
          .51538441396359136, -1.5264021873645037, -.58813609807777278, 2.7857137959190528]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scenario(row):
    legacy = row['start'] == 'legacy'
    q = [0.] * 7 if row['start'] == 'zero' else LEGACY if legacy else PRESET
    result = dict(row, assembly_translation_x=.2 if legacy else .3,
                  assembly_translation_y=.2, assembly_translation_z=-.25,
                  assembly_yaw_radians=math.pi, arm_base_z=-.1 if legacy else 0,
                  target_degrees=-90, q_sweep='1', q_best_effort='1')
    for key in ['theta', 'phi', 'alpha']:
        result[f'core_{key}_degrees'] = row[f'core_{key}']
    result.update({f'start_q{i}': v for i, v in enumerate(q)})
    return result


def preinsert_metrics(path, case):
    rows = csv_rows(path)
    if not rows:
        return {}
    position, rotation, _ = scene_pose(case)
    target, axis = physical_target(position, rotation, [0., 0., 0., 0.])
    target += .04 * axis
    # Both old and new planners export the exact pre-insertion target at the
    # end of the free-space connection. Do not confuse it with alignment at E.
    index = next((i for i, r in enumerate(rows) if r['phase'] == 'approach' and
                  np.linalg.norm(np.array([float(r['target_'+k]) for k in 'xyz'])-target) < 1e-9), None)
    if index is None:
        return {}
    q = np.array([[float(r[f'q{j}']) for j in range(7)] for r in rows[:index+1]])
    delta = q[-1]-q[0]
    travel = np.abs(np.diff(q, axis=0))
    return dict(preinsert_l1=float(np.abs(delta).sum()), preinsert_last_delta=float(delta[6]),
                preinsert_travel=float(travel.sum()), preinsert_last_travel=float(travel[:, 6].sum()))


def stats(values):
    return dict(n=len(values), mean=statistics.mean(values) if values else None,
                p50=float(np.quantile(values, .5)) if values else None,
                p95=float(np.quantile(values, .95)) if values else None,
                max=max(values) if values else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path, required=True)
    parser.add_argument('--runs', type=Path, nargs='+', required=True)
    parser.add_argument('--original', type=Path, nargs='+', required=True)
    parser.add_argument('--certificate-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    cases = {r['case']: r for r in csv_rows(args.suite/'cases.csv')}
    suite_hash = sha(args.suite/'cases.csv')
    certificates = collections.defaultdict(list)
    for folder in sorted(args.certificate_root.iterdir()):
        if not (folder/'execution.json').is_file() or not (folder/'results/runs.csv').is_file():
            continue
        metadata = json.loads((folder/'execution.json').read_text())
        if metadata.get('suite_sha256') != suite_hash:
            continue
        for row in csv_rows(folder/'results/runs.csv'):
            if row['success'] == '1' and row['audit_valid'] == '1':
                if row['case'] not in cases:
                    raise ValueError('Certificate outside the fixed corpus')
                certificates[row['case']].append(str(folder/'results'/row['trajectory']))
    originals = {}
    for folder in args.original:
        for row in csv_rows(folder/'results/runs.csv'):
            key = (row['method'], row['case'])
            if key in originals:
                raise ValueError('Duplicate original run: '+str(key))
            row.update(preinsert_metrics(folder/'results'/row['trajectory'], scenario(cases[row['case']])))
            originals[key] = row
    urdf = ROOT/'reference/rm26_arm/source.urdf'
    manifest = json.loads((ROOT/'reference/rm26_arm/manifest.json').read_text())
    source = next(f for f in manifest['files'] if f['local_path'] == 'reference/rm26_arm/source.urdf')
    if sha(urdf) != source['sha256']:
        raise ValueError('Source URDF differs from the pinned manifest')
    arm = UrdfArm(urdf)
    tcp, _ = grasp_from_xml(ROOT/'models/planning/scene.xml')
    audit_options = SimpleNamespace(online_joint_step=.008, online_min_substeps=2,
                                   position_tolerance=.0005, axis_tolerance=.002,
                                   velocity_limit=4, acceleration_limit=32)
    summaries, audits, failures, nominal = {}, [], [], []
    input_hashes = {}
    for folder in args.runs:
        metadata = json.loads((folder/'execution.json').read_text())
        if metadata.get('exit_code') != 0 or metadata['suite_sha256'] != suite_hash:
            raise ValueError('Incomplete run or corpus mismatch: '+str(folder))
        rows = csv_rows(folder/'results/runs.csv')
        expected = {k for k, v in cases.items() if metadata['split'] == 'all' or v['split'] == metadata['split']}
        if len(rows) != len(expected) or {r['case'] for r in rows} != expected:
            raise ValueError('Missing or duplicated requests: '+str(folder))
        input_hashes[str(folder/'results/runs.csv')] = sha(folder/'results/runs.csv')
        for row in rows:
            case = scenario(cases[row['case']])
            path = folder/'results'/row['trajectory']
            audited = trajectory_audit(path, case, arm, tcp, audit_options, row['success'] == '1')
            audits.append(dict(method=row['method'], case=row['case'], trajectory_sha256=sha(path), **audited))
            if not audited['valid']:
                failures.append(dict(method=row['method'], case=row['case'], errors=audited['errors']))
            row.update(preinsert_metrics(path, case))
            if row['case'] in ('development_000_zero', 'development_000_preset'):
                nominal.append(dict(current=row, original=originals.get((row['method'], row['case']))))
        for split in sorted({r['split'] for r in rows}):
            for start in ['all'] + sorted({r['start'] for r in rows if r['split'] == split}):
                population = [r for r in rows if r['split'] == split and (start == 'all' or r['start'] == start)]
                positive = [r for r in population if r['case'] in certificates]
                successful = [r for r in population if r['success'] == '1']
                paired = [(r, originals[(r['method'], r['case'])]) for r in successful
                          if originals.get((r['method'], r['case']), {}).get('success') == '1']
                metrics = {}
                for metric in ['approach_l1', 'approach_last_joint_l1', 'mandatory_l1', 'assembly_l1',
                               'preinsert_l1', 'preinsert_last_travel', 'q_travel', 'seconds']:
                    a = [float(old[metric]) for _, old in paired if metric in old]
                    b = [float(new[metric]) for new, old in paired if metric in old]
                    metrics[metric] = dict(original=stats(a), current=stats(b),
                        reduction_percent=100*(sum(a)-sum(b))/sum(a) if a and sum(a) else None)
                key = f'{metadata["method"]}/{split}/{start}'
                summaries[key] = dict(total=len(population), successes=len(successful),
                    raw_rate=len(successful)/len(population), certified_reachable=len(positive),
                    certified_successes=sum(r['success'] == '1' for r in positive),
                    certified_rate=sum(r['success'] == '1' for r in positive)/len(positive) if positive else None,
                    unresolved=[r['case'] for r in population if r['case'] not in certificates],
                    missed_certified=[r['case'] for r in positive if r['success'] != '1'],
                    failure_reasons=dict(collections.Counter(r['reason'] for r in population if r['success'] != '1')),
                    seconds=stats([float(r['seconds']) for r in population]), paired_motion=metrics)
    report = dict(suite_sha256=suite_hash, inputs=input_hashes, command=sys.argv,
                  scope='Positive trajectory certificates are a lower bound on reachability. Unknown failures stay in raw totals; conditional rates are not whole-cube rates.',
                  summaries=summaries, nominal=nominal, certificates=certificates,
                  urdf_sha256=sha(urdf),
                  scene_sha256=sha(ROOT/'models/planning/scene.xml'),
                  independent_audit=dict(records=len(audits), failures=failures))
    (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    (args.output/'urdf_audit.json').write_text(json.dumps(audits, indent=2)+'\n')
    for key, value in summaries.items():
        if key.endswith('/all'):
            print(key, 'raw', value['successes'], '/', value['total'],
                  'certified', value['certified_successes'], '/', value['certified_reachable'])
    print('Independent URDF audit:', len(audits), 'records,', len(failures), 'failures')
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
