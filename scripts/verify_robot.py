#!/usr/bin/env python3
"""Compare every imported link, TCP and Jacobian with independent source URDF FK.

Requires numpy and build/robot_probe, not the Python MuJoCo package.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3)+np.sin(angle)*cross+(1-np.cos(angle))*(cross@cross)


def verify(config, probe):
    profile = json.loads(config.read_text())
    urdf = config.parent/profile.get('source_urdf', profile['urdf'])
    robot = ET.parse(urdf).getroot()
    names = profile['joint_names']
    joints = {j.get('name'): j for j in robot.findall('joint')}
    children = {}
    for joint in joints.values():
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    lower = np.array([float(joints[n].find('limit').get('lower')) for n in names])
    upper = np.array([float(joints[n].find('limit').get('upper')) for n in names])
    def fk(q):
        frames = {profile['root_body']: np.eye(4)}
        def descend(parent):
            for joint in children.get(parent, []):
                child = joint.find('child').get('link')
                if child in profile.get('excluded_links', []):
                    continue
                origin = joint.find('origin')
                xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
                angles = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
                t = np.eye(4)
                t[:3,:3] = rotation([0,0,1], angles[2])@rotation([0,1,0], angles[1])@rotation([1,0,0], angles[0])
                t[:3,3] = xyz
                name = joint.get('name')
                if joint.get('type') != 'fixed':
                    value = q[names.index(name)] if name in names else profile['locked_joints'][name]
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    local = np.eye(4)
                    if joint.get('type') == 'prismatic':
                        local[:3,3] = axis/np.linalg.norm(axis)*value
                    else:
                        local[:3,:3] = rotation(axis, value)
                    t = t@local
                frames[child] = frames[parent]@t
                descend(child)
        descend(profile['root_body'])
        return frames
    samples = np.vstack([profile['home'], np.clip(np.zeros(len(names)),lower,upper), lower, upper,
                         np.random.default_rng(20261013).uniform(lower,upper,(64,len(names)))])
    scene = config.parent/'scene.xml'
    run = subprocess.run([str(probe.resolve()), str(scene.resolve())],
                         input='\n'.join(' '.join(format(x,'.17g') for x in q) for q in samples)+'\n',
                         check=True, text=True, capture_output=True)
    actual = [json.loads(line) for line in run.stdout.splitlines()]
    if len(actual) != len(samples):
        raise ValueError('FK probe sample count mismatch')
    pe, re, je = 0., 0., 0.
    for q, got in zip(samples,actual):
        frames = fk(q)
        for name, t in frames.items():
            values = got['bodies'][name]
            pe = max(pe, float(np.max(np.abs(t[:3,3]-values[:3]))))
            re = max(re, float(np.max(np.abs(t[:3,:3]-np.array(values[3:]).reshape(3,3)))))
        tcp = frames[profile['tool_link']]
        pe = max(pe, float(np.max(np.abs(tcp[:3,3]-got['tool'][:3]))))
        re = max(re, float(np.max(np.abs(tcp[:3,:3]-np.array(got['tool'][3:]).reshape(3,3)))))
        j = np.array(got['jacobian']).reshape(6,len(names))
        for i in range(len(names)):
            delta = np.zeros(len(names)); delta[i] = 1e-6
            plus, minus = fk(q+delta)[profile['tool_link']], fk(q-delta)[profile['tool_link']]
            dp = (plus[:3,3]-minus[:3,3])/2e-6
            skew = ((plus[:3,:3]-minus[:3,:3])/2e-6)@tcp[:3,:3].T
            angular = np.array([skew[2,1],skew[0,2],skew[1,0]])
            je = max(je, float(np.max(np.abs(j[:,i]-np.r_[dp,angular]))))
    if pe>1e-10 or re>1e-10 or je>2e-8:
        raise ValueError(f'URDF/MJCF mismatch: position={pe}, rotation={re}, Jacobian={je}')
    return dict(robot=profile['id'], sample_count=len(samples), link_count=len(frames), joint_count=len(names),
                max_position_abs_error_m=pe,max_rotation_matrix_abs_error=re,max_jacobian_abs_error=je,
                source_urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
                scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
                fk_fixtures=[dict(qpos_rad=q.tolist(),tool_position_m=got['tool'][:3],
                                 tool_rotation_matrix=np.array(got['tool'][3:]).reshape(3,3).tolist())
                             for q,got in zip(samples[:4],actual[:4])])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'models/mec_arm/robot.json')
    parser.add_argument('--probe',type=Path,default=ROOT/'build/robot_probe')
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    result = verify(args.config.resolve(),args.probe)
    text = json.dumps(result,indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    print(text)
