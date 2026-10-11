#!/usr/bin/env python3
"""Independent URDF/NumPy kinematic and raw-record audit (not a collision proof).

Usage: python scripts/audit_planning_trajectories.py RESULTS_DIRECTORY
Reads runs.csv, scenarios.csv, and every referenced trajectory. Successful paths
are resampled at twice the online segment count, including every online sample.
The seven-axis FK is rebuilt exclusively from source.urdf; only the fixed grasp
and TCP transforms come from the generated MJCF. No MuJoCo or planner import.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
JOINTS = ['j0', 'j1', 'j2', 'j3', 'j4',
          'end_differential_controller/j5', 'end_differential_controller/j6']
PHASES = ['approach', 'insert', 'lift', 'flip', 'twist', 'dwell']


def numbers(text, default):
    return np.array([float(x) for x in (text or default).split()])


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1-math.cos(angle)) * cross @ cross


def rpy(angles):
    x, y, z = angles
    return rotation([0, 0, 1], z) @ rotation([0, 1, 0], y) @ rotation([1, 0, 0], x)


def transform(position=None, orientation=None):
    out = np.eye(4)
    if position is not None:
        out[:3, 3] = position
    if orientation is not None:
        out[:3, :3] = orientation
    return out


def quat_matrix(values):
    q = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) == 0:
        raise ValueError('invalid MJCF quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def xml_transform(element):
    # Generated body/site transforms currently use pos + quat. Fail explicitly
    # on unsupported Euler conventions rather than silently assume URDF RPY.
    if any(k in element.attrib for k in ('euler', 'axisangle', 'xyaxes', 'zaxis')):
        raise ValueError('use explicit quaternion for audited fixed transform: ' + str(element.attrib))
    return transform(numbers(element.get('pos'), '0 0 0'),
                     quat_matrix(numbers(element.get('quat'), '1 0 0 0')))


class UrdfArm:
    def __init__(self, path):
        tree = ET.parse(path).getroot()
        by_child = {j.find('child').get('link'): j for j in tree.findall('joint')}
        chain, link = [], 'ee_link'
        while link != 'base_link':
            if link not in by_child:
                raise ValueError('source URDF has no base_link -> ee_link chain')
            j = by_child[link]
            chain.append(j)
            link = j.find('parent').get('link')
        self.chain = []
        self.lower, self.upper = np.zeros(7), np.zeros(7)
        seen = []
        for j in reversed(chain):
            origin = j.find('origin')
            xyz = numbers(None if origin is None else origin.get('xyz'), '0 0 0')
            angles = numbers(None if origin is None else origin.get('rpy'), '0 0 0')
            fixed = transform(xyz, rpy(angles))
            name, kind = j.get('name'), j.get('type')
            if kind == 'fixed':
                self.chain.append((fixed, None, None))
                continue
            if name not in JOINTS or kind != 'revolute':
                raise ValueError('unexpected moving joint on arm chain: ' + name)
            index = JOINTS.index(name)
            axis = numbers(j.find('axis').get('xyz'), '1 0 0')
            axis /= np.linalg.norm(axis)
            self.chain.append((fixed, index, axis))
            limit = j.find('limit')
            self.lower[index], self.upper[index] = float(limit.get('lower')), float(limit.get('upper'))
            seen.append(name)
        if seen != JOINTS:
            raise ValueError('URDF chain joint order does not match benchmark CSV')

    def fk(self, q):
        q = np.atleast_2d(q)
        result = np.broadcast_to(np.eye(4), (len(q), 4, 4)).copy()
        for fixed, index, axis in self.chain:
            result = result @ fixed
            if index is None:
                continue
            x, y, z = axis
            cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            angles = q[:, index]
            matrices = np.broadcast_to(np.eye(4), (len(q), 4, 4)).copy()
            matrices[:, :3, :3] += (np.sin(angles)[:, None, None] * cross +
                (1-np.cos(angles))[:, None, None] * (cross @ cross))
            result = result @ matrices
        return result


def grasp_from_xml(path):
    root = ET.parse(path).getroot()
    ee = root.find('.//body[@name="ee_link"]')
    if ee is None:
        raise ValueError('MJCF missing ee_link')
    sites = {}

    def visit(body, pose):
        # ee_link's own placement is already supplied by source URDF FK.
        if body is not ee:
            if body.find('joint') is not None or body.find('freejoint') is not None:
                raise ValueError('grasp descendant is not rigid')
            pose = pose @ xml_transform(body)
        for site in body.findall('site'):
            sites[site.get('name')] = pose @ xml_transform(site)
        for child in body.findall('body'):
            visit(child, pose)
    visit(ee, np.eye(4))
    if 'planning_tcp' not in sites or 'carried_unit_opening' not in sites:
        raise ValueError('planning_tcp and carried_unit_opening sites are required; no guessed grasp fallback')
    tcp, opening = sites['planning_tcp'], sites['carried_unit_opening']
    if np.linalg.norm(tcp[:3, 3]-opening[:3, 3]) > 1e-9:
        raise ValueError('planning_tcp is not the physical carried-unit opening')
    if np.linalg.norm(tcp[:3, 2]-opening[:3, 0]) > 1e-9:
        raise ValueError('planning_tcp +Z must align with carried-unit +X')
    return tcp, opening


def csv_rows(path):
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if any(None in row or any(value is None for value in row.values()) for row in rows):
        raise ValueError('truncated/malformed CSV: ' + str(path))
    return rows


def numeric_rows(rows, columns):
    values = np.array([[float(row[k]) for k in columns] for row in rows], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('nonfinite numeric fields: ' + ','.join(columns))
    return values


def scene_pose(scenario):
    fields = ['core_x', 'core_y', 'core_z', 'core_theta_degrees', 'core_phi_degrees',
              'core_alpha_degrees', 'assembly_translation_x', 'assembly_translation_y',
              'assembly_translation_z', 'assembly_yaw_radians', 'target_degrees']
    v = numeric_rows([scenario], fields)[0]
    assembly = rotation([0, 0, 1], v[9])
    core_rotation = assembly @ rpy(np.radians([v[5], v[4]-90, v[3]]))
    core_position = v[6:9] + assembly @ v[:3]
    return core_position, core_rotation, math.radians(v[10])


def physical_target(core_p, core_r, state):
    # Independent homogeneous-transform chain: E -> Q pivot -> P -> carriage
    # -> inserted opening. No use of planner-exported target for assembly phases.
    insertion, lift, flip, twist = state
    pose = (transform(core_p, core_r) @ transform([-.169, 0, .208]) @
            transform(orientation=rotation([1, 0, 0], twist)) @
            transform(orientation=rotation([0, -1, 0], flip)) @
            transform([.169, 0, -.208]) @ transform([0, 0, lift]) @
            transform([-insertion, 0, 0]))
    return pose[:3, 3], pose[:3, 0]


def unit_angle(a, b):
    a = np.asarray(a); b = np.asarray(b)
    a = a / np.linalg.norm(a, axis=-1, keepdims=True)
    b = b / np.linalg.norm(b, axis=-1, keepdims=True)
    return np.arccos(np.clip(np.sum(a*b, axis=-1), -1, 1))


def sphere_interpolate(a, b, t):
    cosine = np.clip(np.dot(a, b), -1, 1)
    theta = math.acos(cosine)
    if theta < 1e-8:
        out = (1-t)*a + t*b
    elif theta > math.pi-1e-7:
        raise ValueError('antipodal approach axes: exported path needs an explicit interpolation plane')
    else:
        out = (math.sin((1-t)*theta)*a + math.sin(t*theta)*b)/math.sin(theta)
    return out / np.linalg.norm(out)


def trajectory_audit(path, scenario, arm, tcp, args, successful):
    rows = csv_rows(path)
    if not rows:
        return {'knots': 0, 'valid': not successful, 'errors': ['successful trajectory is empty'] if successful else []}
    q = numeric_rows(rows, ['q'+str(i) for i in range(7)])
    times = numeric_rows(rows, ['time'])[:, 0]
    state = numeric_rows(rows, ['insertion', 'lift', 'flip', 'twist'])
    target = numeric_rows(rows, ['target_x', 'target_y', 'target_z'])
    axes = numeric_rows(rows, ['direction_x', 'direction_y', 'direction_z'])
    phase = [r['phase'] for r in rows]
    errors = []
    if any(p not in PHASES for p in phase):
        raise ValueError('unknown trajectory phase')
    if [int(r['knot']) for r in rows] != list(range(len(rows))):
        errors.append('nonsequential knot indices')
    if np.any(q < arm.lower-1e-9) or np.any(q > arm.upper+1e-9):
        errors.append('URDF joint limit violation')
    if np.max(np.abs(np.linalg.norm(axes, axis=1)-1)) > 1e-6:
        errors.append('nonunit exported target direction')
    # Failed IK prefixes may have unassigned (all-zero) times. Preserve these as
    # failed records rather than falsely reporting a valid executable path.
    if not successful:
        return {'knots': len(rows), 'valid': not errors, 'errors': errors,
                'scope': 'failed-prefix finite fields and URDF limits only'}
    if set(phase) != set(PHASES):
        errors.append('missing required phase')
    ranks = np.array([PHASES.index(p) for p in phase])
    if ranks[0] != 0 or ranks[-1] != 5 or np.any(np.diff(ranks) < 0) or np.any(np.diff(ranks) > 1):
        errors.append('invalid phase order')
    start = numeric_rows([scenario], ['start_q'+str(i) for i in range(7)])[0]
    if np.linalg.norm(q[0]-start) > 1e-10:
        errors.append('wrong starting configuration')
    if abs(times[0]) > 1e-12 or np.any(np.diff(times) <= 0):
        errors.append('timestamps must start at zero and strictly increase')
        return {'knots': len(rows), 'valid': False, 'errors': errors}
    core_p, core_r, requested_goal = scene_pose(scenario)
    best_effort = scenario.get('q_best_effort', '0') == '1'
    goal = float(state[-1, 3]) if best_effort else requested_goal
    if best_effort and scenario.get('q_sweep') != '1':
        errors.append('best-effort Q requires a sweep')
    expected_ends = {'approach': [0, 0, 0, 0], 'insert': [.1, 0, 0, 0],
                     'lift': [.1, .1, 0, 0], 'flip': [.1, .1, math.pi/2, 0],
                     'twist': [.1, .1, math.pi/2, goal], 'dwell': [.1, .1, math.pi/2, goal]}
    for p, endpoint in expected_ends.items():
        indices = [i for i, name in enumerate(phase) if name == p]
        if indices and np.max(np.abs(state[indices[-1]]-endpoint)) > 1e-8:
            errors.append(p + ' endpoint contract')
    if np.any(state[:, :3] < -1e-9) or np.any(state[:, :3] > [.1+1e-9, .1+1e-9, math.pi/2+1e-9]):
        errors.append('mechanism travel bounds')
    if np.any(np.abs(state[:, 3]) > math.pi/2+1e-9):
        errors.append('Q bound exceeded')
    if scenario.get('q_sweep', '0') == '1':
        sweep = np.array([0.] + [float(s[3]) for s, p in zip(state, phase) if p == 'twist'])
        peak = int(np.argmax(sweep))
        if not best_effort and (abs(goal+math.pi/2) > 1e-9 or abs(sweep[peak]-math.pi/2) > 1e-9 or
                abs(sweep[-1]+math.pi/2) > 1e-9):
            errors.append('Q sweep must visit +90 then finish at -90')
        if np.any(np.diff(sweep[:peak+1]) < -1e-9) or np.any(np.diff(sweep[peak:]) > 1e-9):
            errors.append('Q must turn positive then reverse at most once')
    for i, p in enumerate(phase):
        s, l, f, w = state[i]
        if ((p in ('approach', 'insert') and max(abs(l), abs(f), abs(w)) > 1e-8) or
            (p == 'approach' and abs(s) > 1e-8) or
            (p == 'lift' and max(abs(s-.1), abs(f), abs(w)) > 1e-8) or
            (p == 'flip' and max(abs(s-.1), abs(l-.1), abs(w)) > 1e-8) or
            (p in ('twist', 'dwell') and max(abs(s-.1), abs(l-.1), abs(f-math.pi/2)) > 1e-8)):
            errors.append('mechanism motion out of order at knot ' + str(i)); break
    dq = np.max(np.abs(np.diff(q, axis=0)), axis=1)
    dt = np.diff(times)
    # Cubic Hermite with endpoint velocity zero: derivative extrema occur at
    # u=1/2 (speed) and u=0,1 (acceleration); these are analytic, not samples.
    peak_v = float(np.max(1.5*dq/dt, initial=0))
    peak_a = float(np.max(6*dq/dt**2, initial=0))
    if peak_v > args.velocity_limit+1e-8 or peak_a > args.acceleration_limit+1e-8:
        errors.append('analytic cubic velocity/acceleration violation')
    qs, sample_times, positions, directions, sample_phases = [q[0]], [times[0]], [target[0]], [axes[0]], [phase[0]]
    for i in range(1, len(rows)):
        # EXACT twice-online count includes every online sample, not just a
        # nominal half step that could omit original samples due to rounding.
        # Factor 1.5 bounds d(3u²-2u³)/du for equal-time sampling.
        count = 2 * max(args.online_min_substeps, math.ceil(1.5*dq[i-1]/args.online_joint_step))
        for step in range(1, count+1):
            u = step/count
            blend = u*u*(3-2*u)
            qs.append(q[i-1]+blend*(q[i]-q[i-1]))
            sample_times.append(times[i-1]+u*dt[i-1])
            sample_phases.append(phase[i])
            if phase[i] == 'approach':
                positions.append(target[i-1]+blend*(target[i]-target[i-1]))
                directions.append(sphere_interpolate(axes[i-1], axes[i], blend))
            else:
                p, a = physical_target(core_p, core_r, state[i-1]+blend*(state[i]-state[i-1]))
                positions.append(p); directions.append(a)
    actual = arm.fk(np.array(qs)) @ tcp
    base_z = float(scenario.get('arm_base_z', 0))
    if not math.isfinite(base_z):
        raise ValueError('nonfinite robot base translation')
    actual[:, 2, 3] += base_z
    pos_errors = np.linalg.norm(actual[:, :3, 3]-positions, axis=1)
    axis_errors = unit_angle(actual[:, :3, 2], np.array(directions))
    max_p, max_axis = float(np.max(pos_errors)), float(np.max(axis_errors))
    if max_p > args.position_tolerance+1e-10:
        errors.append('independent URDF dense position error')
    if max_axis > args.axis_tolerance+1e-10:
        errors.append('independent URDF dense axis error')
    export_p, export_a = 0., 0.
    for i, p in enumerate(phase):
        if p == 'approach':
            continue
        desired_p, desired_a = physical_target(core_p, core_r, state[i])
        export_p = max(export_p, float(np.linalg.norm(target[i]-desired_p)))
        export_a = max(export_a, float(unit_angle(axes[i], desired_a)))
    if export_p > 1e-8 or export_a > 1e-7:
        errors.append('exported assembly target disagrees with independent mechanism chain')
    # Infer actual motion from independent FK. Measure aligned uninterrupted
    # suffix duration, not elapsed time on labels supplied by the planner.
    rel_axis = actual[:, :3, 2] @ core_r
    flip = np.arccos(np.clip(rel_axis[:, 0], -1, 1))
    twist = np.arctan2(-rel_axis[:, 1], rel_axis[:, 2])
    aligned = []
    for i, (f, w) in enumerate(zip(flip, twist)):
        undo = rotation([0, 1, 0], f) @ rotation([1, 0, 0], -w)
        local = undo @ (core_r.T @ (actual[i, :3, 3]-core_p)-[-.169, 0, .208])
        insertion, lift = .169-local[0], .208+local[2]
        aligned.append(sample_phases[i] in ('twist', 'dwell') and
            abs(insertion-.1) <= .003+1e-9 and abs(lift-.1) <= .003+1e-9 and
            abs(f-math.pi/2) <= math.radians(2)+1e-9 and abs(w-goal) <= math.radians(5)+1e-9)
    begin = len(aligned)-1
    while begin >= 0 and aligned[begin]:
        begin -= 1
    dwell = sample_times[-1]-sample_times[begin+1] if begin < len(aligned)-1 else 0.
    if dwell < .4-1e-9:
        errors.append('actual FK Q-alignment suffix shorter than .4 s')
    return {'knots': len(rows), 'samples': len(qs), 'valid': not errors, 'errors': errors,
            'max_position_error_m': max_p, 'max_axis_error_rad': max_axis,
            'max_exported_target_position_disagreement_m': export_p,
            'max_exported_target_axis_disagreement_rad': export_a,
            'max_joint_speed_rad_s': peak_v, 'max_joint_acceleration_rad_s2': peak_a,
            'fk_aligned_suffix_seconds': float(dwell), 'execution_seconds': float(times[-1]),
            'q_positive_degrees': float(np.max(state[:, 3])*180/math.pi),
            'q_final_degrees': float(state[-1, 3]*180/math.pi),
            'full_q_sweep': bool(abs(np.max(state[:, 3])-math.pi/2)<1e-9 and abs(state[-1, 3]+math.pi/2)<1e-9),
            'q_best_effort': best_effort}


def distribution(values):
    if not values:
        return {'n': 0, 'p50': None, 'p95': None, 'max': None}
    if not all(math.isfinite(x) and x >= 0 for x in values):
        raise ValueError('statistics contain nonfinite or negative values')
    return {'n': len(values), 'p50': float(np.quantile(values, .5)),
            'p95': float(np.quantile(values, .95)), 'max': float(max(values))}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', nargs='?', type=Path)
    parser.add_argument('--urdf', type=Path, default=ROOT/'reference/rm26_arm/source.urdf')
    parser.add_argument('--scene', type=Path, default=ROOT/'models/planning/scene.xml')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--expected-runs', type=int)
    parser.add_argument('--online-joint-step', type=float, default=.008)
    parser.add_argument('--online-min-substeps', type=int, default=2)
    parser.add_argument('--position-tolerance', type=float, default=.0005)
    parser.add_argument('--axis-tolerance', type=float, default=.002)
    parser.add_argument('--velocity-limit', type=float, default=1.)
    parser.add_argument('--acceleration-limit', type=float, default=2.)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    for k in ('online_joint_step', 'position_tolerance', 'axis_tolerance', 'velocity_limit', 'acceleration_limit'):
        if not math.isfinite(getattr(args, k)) or getattr(args, k) <= 0:
            parser.error(k + ' must be finite and positive')
    if args.online_min_substeps < 1:
        parser.error('online_min_substeps must be positive')
    arm = UrdfArm(args.urdf)
    provenance = ROOT/'reference/rm26_arm/manifest.json'
    if args.urdf.resolve() == (ROOT/'reference/rm26_arm/source.urdf').resolve():
        source_files = json.loads(provenance.read_text())['files']
        source = next(f for f in source_files if f['local_path'] == 'reference/rm26_arm/source.urdf')
        if sha(args.urdf) != source['sha256']:
            raise ValueError('source URDF disagrees with pinned model manifest')
    if args.self_test:
        fixtures = json.loads((ROOT/'models/rm26_arm/verification.json').read_text())['fk_fixtures']
        got = arm.fk(np.array([f['qpos_rad'] for f in fixtures]))
        error_p = max(float(np.max(np.abs(t[:3, 3]-f['tool_position_m']))) for t, f in zip(got, fixtures))
        error_r = max(float(np.max(np.abs(t[:3, :3]-f['tool_rotation_matrix']))) for t, f in zip(got, fixtures))
        if max(error_p, error_r) > 1e-10:
            raise ValueError('source URDF FK fails archived independent MuJoCo fixtures')
        # Physical chain has a simple endpoint derivation independent of the
        # planner implementation: 90-degree P maps local x into +z.
        p, a = physical_target(np.zeros(3), np.eye(3), [.1, .1, math.pi/2, 0])
        if np.linalg.norm(p-[-.061, 0, .277]) > 1e-12 or np.linalg.norm(a-[0, 0, 1]) > 1e-12:
            raise ValueError('mechanism homogeneous-chain self-check')
        print(json.dumps({'URDF_FK_fixtures': len(fixtures), 'max_position_abs_error_m': error_p,
                          'max_rotation_abs_error': error_r, 'mechanism_chain': 'pass'}))
        return 0
    if args.directory is None:
        parser.error('directory required unless --self-test')
    directory = args.directory.resolve()
    tcp, opening = grasp_from_xml(args.scene)
    scenarios_list = csv_rows(directory/'scenarios.csv')
    scenarios = {r['case']: r for r in scenarios_list}
    runs = csv_rows(directory/'runs.csv')
    errors, records, seen = [], [], set()
    if len(scenarios) != len(scenarios_list):
        errors.append('duplicate scenario ids')
    for scenario in scenarios_list:
        scene_pose(scenario)
        numeric_rows([scenario], ['start_q'+str(i) for i in range(7)])
    environment_path = directory/'environment.json'
    environment = json.loads(environment_path.read_text()) if environment_path.exists() else {}
    expected = args.expected_runs if args.expected_runs is not None else environment.get('expected_runs')
    if expected is not None and len(runs) != expected:
        errors.append('run count does not match expected population')
    if expected is None:
        errors.append('expected run count unavailable: supply --expected-runs or environment.json')
    # Catch a stale scene or source switch between execution and independent audit.
    for relative, digest in environment.get('source_sha256', {}).items():
        if relative in ('models/planning/scene.xml', 'scripts/generate_planning_scene.py'):
            current = ROOT/relative
            if current.exists() and sha(current) != digest:
                errors.append('benchmark source changed before audit: ' + relative)
    for row in runs:
        record = {k: row[k] for k in ('method', 'case', 'split', 'repeat', 'success', 'reason')}
        try:
            key = (row['method'], row['case'], int(row['repeat']))
            if key in seen:
                raise ValueError('duplicate method/case/repeat')
            seen.add(key)
            if row['success'] not in ('0', '1') or row.get('plan_success') not in ('0', '1'):
                raise ValueError('invalid success field')
            timing_keys = [k for k in row if k.endswith('_seconds')]
            timing = numeric_rows([row], timing_keys)[0]
            if np.any(timing < 0):
                raise ValueError('negative timing field')
            numeric_keys = [k for k in row if k.startswith('max_') or k in
                ('knots', 'ik_iterations', 'fk_evals', 'collision_states', 'narrowphase_calls',
                 'broadphase_skips', 'validation_samples', 'dense_recheck_samples')]
            if np.any(numeric_rows([row], numeric_keys) < 0):
                raise ValueError('negative counters or error metrics')
            if row['case'] not in scenarios or scenarios[row['case']]['split'] != row['split']:
                raise ValueError('scenario identity/split mismatch')
            trajectory = (directory/row['trajectory']).resolve()
            if directory not in trajectory.parents:
                raise ValueError('trajectory path escapes result directory')
            success = row['success'] == '1'
            # Audit every planner-success path, including those rejected by the
            # optional dense collision recheck. Only accepted successes enter
            # success statistics, with negative fixtures kept separate.
            checked = trajectory_audit(trajectory, scenarios[row['case']], arm, tcp, args,
                                       row['plan_success'] == '1')
            if row['plan_success'] == '1' and scenarios[row['case']].get('q_best_effort') == '1':
                for key in ('q_positive_degrees', 'q_final_degrees'):
                    if not math.isfinite(float(row[key])) or abs(float(row[key])-checked[key]) > 1e-8:
                        checked['valid'] = False; checked['errors'].append('incorrect exported ' + key)
                if (row['full_q_sweep'] == '1') != checked['full_q_sweep']:
                    checked['valid'] = False; checked['errors'].append('incorrect exported full/partial Q label')
            if checked['knots'] != int(row['knots']):
                checked['valid'] = False; checked['errors'].append('CSV knot count mismatch')
            if success and row['plan_success'] != '1':
                checked['valid'] = False; checked['errors'].append('accepted success without planner success')
            if success and row['split'] == 'negative':
                checked['valid'] = False; checked['errors'].append('negative fixture unexpectedly accepted')
            record.update(checked)
            record['trajectory_sha256'] = sha(trajectory)
            if not checked['valid']:
                errors.append('/'.join(map(str, key)) + ': ' + '; '.join(checked['errors']))
        except (ValueError, KeyError, OSError) as exception:
            record.update({'valid': False, 'errors': [str(exception)]})
            errors.append(str(record)[:500])
        records.append(record)
    groups = []
    for method, split in sorted({(r['method'], r['split']) for r in runs}):
        group = [r for r in runs if (r['method'], r['split']) == (method, split)]
        successful = [r for r in group if r['success'] == '1']
        try:
            groups.append({'method': method, 'split': split, 'runs': len(group), 'successes': len(successful),
                'rejections': len(group)-len(successful),
                'all_timing': distribution([float(r['planning_seconds']) for r in group]),
                'successful_timing': distribution([float(r['planning_seconds']) for r in successful])})
        except ValueError:
            errors.append('non-numeric planning_seconds in statistics')
    baseline = {(r['case'], r['repeat']): r for r in runs if r['method'] == 'baseline'}
    paired = []
    for method in sorted({r['method'] for r in runs}-{'baseline'}):
        ratios, regressions, gains = [], [], []
        for row in runs:
            if row['method'] != method or row['split'] == 'negative':
                continue
            base = baseline.get((row['case'], row['repeat']))
            if base is None:
                continue
            label = {'case': row['case'], 'repeat': int(row['repeat']), 'split': row['split']}
            if base['success'] == '1' and row['success'] != '1':
                regressions.append(label)
            elif base['success'] != '1' and row['success'] == '1':
                gains.append(label)
            elif base['success'] == row['success'] == '1':
                t, b = float(row['planning_seconds']), float(base['planning_seconds'])
                if math.isfinite(t) and math.isfinite(b) and t > 0:
                    ratios.append(b/t)
        paired.append({'method': method, 'both_successful_positive_speedup': distribution(ratios),
                       'success_regressions': regressions, 'success_gains': gains})
    audit = {'audit_schema': 1, 'valid': not errors, 'run_count': len(runs), 'expected_runs': expected,
        'successful_runs': sum(r['success'] == '1' for r in runs),
        'successful_positive_runs': sum(r['success'] == '1' and r['split'] != 'negative' for r in runs),
        'scope': 'Independent source-URDF kinematics, grasp, mechanism, timing and raw population audit; no independent mesh collision or physical grasp proof',
        'sampling': {'factor_of_online_segment_count': 2, 'online_joint_step': args.online_joint_step,
                     'online_min_substeps': args.online_min_substeps},
        'thresholds': {'position_m': args.position_tolerance, 'axis_rad': args.axis_tolerance,
                       'velocity_rad_s': args.velocity_limit, 'acceleration_rad_s2': args.acceleration_limit},
        'inputs_sha256': {'urdf': sha(args.urdf), 'scene': sha(args.scene), 'runs_csv': sha(directory/'runs.csv'),
                          'scenarios_csv': sha(directory/'scenarios.csv')},
        'T_ee_tcp': tcp.tolist(), 'T_ee_opening': opening.tolist(), 'groups': groups,
        'paired_positive_speedup': paired,
        'errors': errors, 'trajectories': records}
    output = args.output or directory/'audit.json'
    output.write_text(json.dumps(audit, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'valid': audit['valid'], 'runs': len(runs), 'successes': audit['successful_runs'],
                      'errors': len(errors), 'output': str(output)}))
    return 0 if audit['valid'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, KeyError, OSError) as error:
        print('audit: ' + str(error), file=sys.stderr)
        sys.exit(2)
