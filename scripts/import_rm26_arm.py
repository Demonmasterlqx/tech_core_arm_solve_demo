#!/usr/bin/env python3
"""Reproduce the pinned RM26 arm-only kinematic MuJoCo model.

Python standard library suffices for download/generation. --verify additionally
requires numpy and a C++ compiler, and uses the repository's MuJoCo SDK.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'models/rm26_arm'
REFERENCE = ROOT / 'reference/rm26_arm'
COMMIT = '6edc630a1a14a3904b42d5a360e9020562eefc99'
REPO = 'HKUSTGZ-ROBOMASTER-PNX/UpperComputerControlOpenSourceDoc'
UPSTREAM = 'src/RM/RMController/core/models/rm26_version3_engineering_model'
RAW = f'https://raw.githubusercontent.com/{REPO}/{COMMIT}/{UPSTREAM}/'
JOINT_NAMES = ['j0', 'j1', 'j2', 'j3', 'j4', 'end_differential_controller/j5', 'end_differential_controller/j6']
HOME = [0, 1.0, 1.4, 0, 0.7, 0, 0]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def numbers(value):
    return [float(v) for v in value.split()]


def fmt(values):
    return ' '.join(format(float(v), '.17g') for v in values)


def quaternion(rpy):
    r, p, y = [v / 2 for v in rpy]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def origin(element):
    node = element.find('origin')
    return {'pos': node.get('xyz', '0 0 0'), 'quat': fmt(quaternion(numbers(node.get('rpy', '0 0 0'))))}


def download(remote, destination, expected=None):
    if destination.exists():
        data = destination.read_bytes()
    else:
        destination.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(RAW + remote, timeout=60) as response:
            data = response.read()
        if expected and digest(data) != expected:
            raise RuntimeError(f'Upstream checksum mismatch: {remote}')
        temporary = destination.with_suffix(destination.suffix + '.part')
        try:
            temporary.write_bytes(data)
            temporary.replace(destination)
        finally:
            temporary.unlink(missing_ok=True)
    if expected and digest(data) != expected:
        raise RuntimeError(f'Local checksum mismatch: {destination}')
    return {'upstream_path': UPSTREAM + '/' + remote, 'local_path': str(destination.relative_to(ROOT)), 'bytes': len(data), 'sha256': digest(data)}


def selected_tree(robot):
    links = {node.get('name'): node for node in robot.findall('link')}
    children = {}
    for joint in robot.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    selected = []
    def descend(joint):
        name = joint.find('child').get('link')
        selected.append((joint, links[name]))
        for child in children.get(name, []):
            descend(child)
    descend(next(j for j in robot.findall('joint') if j.get('name') == 'j0'))
    return selected, children


def generate(fetch):
    REFERENCE.mkdir(parents=True, exist_ok=True)
    MODEL.mkdir(parents=True, exist_ok=True)
    manifest_file = REFERENCE / 'manifest.json'
    previous = json.loads(manifest_file.read_text()) if manifest_file.exists() else {}
    hashes = {entry['local_path']: entry['sha256'] for entry in previous.get('files', [])}
    files = []
    def obtain(remote, destination):
        if not fetch and not destination.exists():
            raise RuntimeError(f'Missing {destination}; run with --download')
        files.append(download(remote, destination, hashes.get(str(destination.relative_to(ROOT)))))
    obtain('urdf/rm26_version3_engineering_model.urdf', REFERENCE / 'source.urdf')
    obtain('package.xml', REFERENCE / 'package.xml')
    robot = ET.parse(REFERENCE / 'source.urdf').getroot()
    selected, children = selected_tree(robot)
    for _, link in selected:
        for visual in link.findall('visual'):
            name = Path(visual.find('geometry/mesh').get('filename')).name
            obtain('meshes/' + name, MODEL / 'meshes' / name)

    scene = ET.Element('mujoco', model='rm26_arm_kinematic')
    ET.SubElement(scene, 'compiler', angle='radian', meshdir='meshes', inertiafromgeom='false', autolimits='true')
    ET.SubElement(scene, 'option', timestep='0.002', gravity='0 0 0')
    ET.SubElement(scene, 'size', nuser_jnt='1')
    visual = ET.SubElement(scene, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='960')
    ET.SubElement(visual, 'headlight', diffuse='.75 .75 .75', ambient='.3 .3 .3', specular='.2 .2 .2')
    ET.SubElement(visual, 'map', znear='.01')
    ET.SubElement(scene, 'statistic', center='0 0 0.45', extent='1.5')
    defaults = ET.SubElement(scene, 'default')
    ET.SubElement(defaults, 'geom', contype='0', conaffinity='0', group='1')
    ET.SubElement(defaults, 'joint', type='hinge', damping='0', armature='0', limited='true')
    assets = ET.SubElement(scene, 'asset')
    for _, link in selected:
        for visual in link.findall('visual'):
            name = Path(visual.find('geometry/mesh').get('filename')).name
            ET.SubElement(assets, 'mesh', name=name[:-4], file=name)
    world = ET.SubElement(scene, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 .3 -1', diffuse='.8 .8 .8')
    ET.SubElement(world, 'geom', name='ground_visual', type='plane', size='2 2 .01', rgba='.16 .19 .24 1', group='0')
    # A mount marker, not the removed chassis. Source base_link is the world frame.
    ET.SubElement(world, 'geom', name='mount_visual', type='cylinder', pos='-.181 0 .04', size='.085 .04', rgba='.26 .31 .37 1')
    bodies = {'base_link': world}
    for joint, link in selected:
        body = ET.SubElement(bodies[joint.find('parent').get('link')], 'body', name=link.get('name'), **origin(joint))
        bodies[link.get('name')] = body
        if joint.get('name') in JOINT_NAMES:
            limit = joint.find('limit')
            ET.SubElement(body, 'joint', name=joint.get('name'), axis=joint.find('axis').get('xyz'), range=f"{limit.get('lower')} {limit.get('upper')}", user='1')
            # Numerical placeholders only. URDF j1 inertia violates triangle inequalities.
            ET.SubElement(body, 'inertial', pos='0 0 0', mass='1', diaginertia='.001 .001 .001')
        for visual in link.findall('visual'):
            name = Path(visual.find('geometry/mesh').get('filename')).name
            color = visual.find('material/color')
            ET.SubElement(body, 'geom', name=link.get('name')+'_visual', type='mesh', mesh=name[:-4], rgba=color.get('rgba') if color is not None else '.8 .8 .8 1', **origin(visual))
        if link.get('name') == 'ee_link':
            ET.SubElement(body, 'site', name='arm_tool', size='.008', rgba='1 .35 .08 1', group='2')
    keyframe = ET.SubElement(scene, 'keyframe')
    ET.SubElement(keyframe, 'key', name='home', qpos=fmt(HOME))
    ET.indent(scene, space='  ')
    ET.ElementTree(scene).write(MODEL / 'scene.xml', encoding='utf-8', xml_declaration=True)
    metadata = {
        'repository': 'https://github.com/' + REPO, 'commit': COMMIT,
        'upstream_model_directory': UPSTREAM,
        'license': 'package.xml declares BSD; repository root and model package have no standalone license text at this commit; no more specific BSD variant asserted',
        'world_frame': 'source base_link at identity; j0 origin retained exactly; no chassis body or geometry',
        'joint_order': JOINT_NAMES, 'home_qpos_rad': HOME,
        'tool_frame': 'source ee_link (ee_joint fixed transform retained)',
        'frozen_joints_at_zero': [j.get('name') for j, _ in selected if j.get('name') not in JOINT_NAMES and j.get('type') != 'fixed'],
        'mock_joint_velocity_limit_rad_s': 1.0,
        'dynamics': 'Kinematic only. Explicit placeholder 1 kg / isotropic 0.001 kg m^2 inertia on each moving link, zero gravity, no actuators, no contacts. Source inertias not used.',
        'files': sorted(files, key=lambda entry: entry['local_path']),
    }
    manifest_file.write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'Generated {MODEL / "scene.xml"}; {len(files)} pinned source files, {sum(f["bytes"] for f in files)} bytes')


def verify(sdk):
    import numpy as np
    robot = ET.parse(REFERENCE / 'source.urdf').getroot()
    selected, _ = selected_tree(robot)
    joint_by_name = {j.get('name'): j for j in robot.findall('joint')}
    def rotation(axis, angle):
        x, y, z = axis
        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)
    def transform(joint):
        result = np.eye(4)
        o = joint.find('origin')
        roll, pitch, yaw = numbers(o.get('rpy'))
        result[:3, :3] = rotation([0,0,1], yaw) @ rotation([0,1,0], pitch) @ rotation([1,0,0], roll)
        result[:3, 3] = numbers(o.get('xyz'))
        return result
    def fk(q):
        frames = {'base_link': np.eye(4)}
        axes, positions = [], []
        for joint, link in selected:
            frame = frames[joint.find('parent').get('link')] @ transform(joint)
            if joint.get('name') in JOINT_NAMES:
                i = JOINT_NAMES.index(joint.get('name'))
                axis = numbers(joint.find('axis').get('xyz'))
                axes.append(frame[:3,:3] @ axis)
                positions.append(frame[:3,3].copy())
                local = np.eye(4)
                local[:3,:3] = rotation(axis, q[i])
                frame = frame @ local
            frames[link.get('name')] = frame
        tool = frames['ee_link']
        jac = np.array([np.r_[np.cross(a, tool[:3,3]-p), a] for a, p in zip(axes, positions)]).T
        return frames, jac
    ranges = np.array([[float(joint_by_name[n].find('limit').get(k)) for k in ['lower','upper']] for n in JOINT_NAMES])
    rng = np.random.default_rng(26)
    samples = np.vstack([HOME, np.zeros(7), ranges[:,0], ranges[:,1], rng.uniform(ranges[:,0], ranges[:,1], size=(64,7))])
    body_names = [l.get('name') for _, l in selected]
    body_initializer = ','.join(json.dumps(name) for name in body_names)
    cpp = r'''#include <mujoco/mujoco.h>
#include <iostream>
#include <iomanip>
int main(int argc,char**argv) {
  char error[2048]; mjModel* m=mj_loadXML(argv[1],nullptr,error,sizeof(error));
  if(!m){std::cerr<<error;return 1;} mjData* d=mj_makeData(m);
  if(m->nq!=7 || m->nv!=7 || m->nu!=0){std::cerr<<"Unexpected DOFs";return 2;}
  for(int i=0;i<m->ngeom;i++)if(m->geom_contype[i]||m->geom_conaffinity[i])return 3;
  const char* names[]={BODY_NAMES};
  std::cout<<std::setprecision(17);
  while(std::cin>>d->qpos[0]){
    for(int i=1;i<7;i++)std::cin>>d->qpos[i]; mj_forward(m,d);
    for(const char* name:names){int b=mj_name2id(m,mjOBJ_BODY,name);if(b<0)return 4;
      for(int i=0;i<3;i++)std::cout<<d->xpos[3*b+i]<<' ';
      for(int i=0;i<9;i++)std::cout<<d->xmat[9*b+i]<<' ';}
    int s=mj_name2id(m,mjOBJ_SITE,"arm_tool");if(s<0)return 5;
    for(int i=0;i<3;i++)std::cout<<d->site_xpos[3*s+i]<<' ';
    for(int i=0;i<9;i++)std::cout<<d->site_xmat[9*s+i]<<' ';
    std::cout<<'\n';
  }
  mj_deleteData(d);mj_deleteModel(m);
}'''.replace('BODY_NAMES', body_initializer)
    with tempfile.TemporaryDirectory(prefix='rm26-fk-') as directory:
        source = Path(directory) / 'verify.cpp'
        executable = Path(directory) / 'verify'
        source.write_text(cpp)
        subprocess.run(['c++', '-std=c++17', str(source), '-I'+str(sdk/'include'), '-L'+str(sdk/'lib'), '-Wl,-rpath,'+str(sdk/'lib'), '-lmujoco', '-o', str(executable)], check=True)
        output = subprocess.run([str(executable), str(MODEL/'scene.xml')], input='\n'.join(fmt(q) for q in samples)+'\n', capture_output=True, text=True, check=True).stdout
    actual = np.array([numbers(line) for line in output.splitlines()])
    expected = []
    for q in samples:
        frames, _ = fk(q)
        expected.append(np.concatenate([np.r_[frames[n][:3,3], frames[n][:3,:3].ravel()] for n in body_names+['ee_link']]))
    differences = np.abs(actual-np.array(expected)).reshape(len(samples),-1,12)
    position_error = float(differences[:,:,:3].max())
    rotation_error = float(differences[:,:,3:].max())
    if position_error > 1e-10 or rotation_error > 1e-10:
        raise RuntimeError(f'URDF/MJCF FK mismatch {position_error}, {rotation_error}')
    frames, jac = fk(HOME)
    singular_values = np.linalg.svd(jac, compute_uv=False)
    inertia_audit = []
    for _, link in selected:
        inertia = link.find('inertial/inertia')
        if inertia is None:
            continue
        a,b,c,d,e,f = [float(inertia.get(k)) for k in ['ixx','iyy','izz','ixy','ixz','iyz']]
        eigenvalues = np.linalg.eigvalsh([[a,d,e],[d,b,f],[e,f,c]])
        inertia_audit.append({'link': link.get('name'), 'principal_moments_kg_m2': eigenvalues.tolist(), 'triangle_margin_kg_m2': float(eigenvalues[0]+eigenvalues[1]-eigenvalues[2]), 'physically_valid_tensor': bool(eigenvalues[0]>0 and eigenvalues[0]+eigenvalues[1]>=eigenvalues[2])})
    manifest = json.loads((REFERENCE/'manifest.json').read_text())
    for entry in manifest['files']:
        if digest((ROOT/entry['local_path']).read_bytes()) != entry['sha256']:
            raise RuntimeError('Source checksum mismatch: '+entry['local_path'])
    report = {'mujoco_sdk': str(sdk), 'sample_count': len(samples), 'compared_body_count': len(body_names), 'joint_count': 7, 'actuator_count': 0, 'contacts_disabled': True, 'seed': 26, 'max_position_abs_error_m': position_error, 'max_rotation_matrix_abs_error': rotation_error, 'home_tool_position_m': frames['ee_link'][:3,3].tolist(), 'fk_fixtures': [{'qpos_rad': q.tolist(), 'tool_position_m': fk(q)[0]['ee_link'][:3,3].tolist(), 'tool_rotation_matrix': fk(q)[0]['ee_link'][:3,:3].tolist()} for q in samples[:4]], 'home_pose_jacobian_singular_values_mixed_units': singular_values.tolist(), 'source_inertia_audit': inertia_audit, 'scene_sha256': digest((MODEL/'scene.xml').read_bytes()), 'source_checksums_verified': len(manifest['files'])}
    (MODEL/'verification.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download', action='store_true', help='fetch missing files at pinned commit')
    parser.add_argument('--verify', action='store_true', help='compile temporary MuJoCo FK checker and compare 68 URDF poses')
    parser.add_argument('--mujoco-dir', type=Path, default=ROOT/'.deps/mujoco-3.3.7')
    args = parser.parse_args()
    generate(args.download)
    if args.verify:
        verify(args.mujoco_dir.resolve())

if __name__ == '__main__':
    main()
