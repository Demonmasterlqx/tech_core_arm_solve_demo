#!/usr/bin/env python3
"""Build a fixed-base kinematic robot package from URDF + robot.json (stdlib only).

The source URDF is authoritative for frames, axes, limits and visual/collision
geometry. Movable joints must be declared explicitly; unsupported joints fail.
Regeneration is offline. --download restores the pinned MecArm source manifest.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET
from robot_description import metadata, collision_metadata
from assembly_scene import task_components

ROOT = Path(__file__).resolve().parents[1]


def fmt(values):
    return ' '.join(format(float(x), '.17g') for x in values)


def origin(element):
    node = element.find('origin')
    xyz = node.get('xyz', '0 0 0') if node is not None else '0 0 0'
    r, p, y = [float(x)/2 for x in (node.get('rpy', '0 0 0') if node is not None else '0 0 0').split()]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return dict(pos=xyz, quat=fmt([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]))


def restore_sources(manifest_path, download=False):
    manifest = json.loads(manifest_path.read_text())
    for entry in manifest['files']:
        path = ROOT/entry['local_path']
        if not path.exists() and download:
            url = manifest['repository'].replace('https://github.com/', 'https://raw.githubusercontent.com/') + '/' + manifest['commit'] + '/' + urllib.request.quote(entry['upstream_path'])
            with urllib.request.urlopen(url, timeout=60) as response:
                data = response.read()
            if hashlib.sha256(data).hexdigest() != entry['sha256']:
                raise ValueError('Upstream checksum mismatch: '+url)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
            raise ValueError('Missing or changed pinned source: '+str(path))


def generate(config):
    profile = json.loads(config.read_text())
    if profile.get('schema_version', 1) != 1:
        raise ValueError('Unsupported robot profile schema_version')
    directory = config.parent
    source = ET.parse(directory/profile['urdf']).getroot()
    names = profile['joint_names']
    if not names or len(set(names)) != len(names) or any(not n or any(c.isspace() for c in n) for n in names):
        raise ValueError('joint_names must be nonempty and unique')
    for field in ('home', 'planning_home', 'velocity_limits'):
        if len(profile[field]) != len(names) or not all(math.isfinite(x) for x in profile[field]):
            raise ValueError('Invalid '+field)
    if any(x <= 0 for x in profile['velocity_limits']):
        raise ValueError('Velocity limits must be positive')
    scene = ET.Element('mujoco', model=profile['id']+'_kinematic')
    ET.SubElement(scene, 'compiler', angle='radian', autolimits='true', inertiafromgeom='false')
    ET.SubElement(scene, 'option', timestep='.002', gravity='0 0 0')
    visual = ET.SubElement(scene, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='960')
    ET.SubElement(visual, 'headlight', diffuse='.75 .75 .75', ambient='.3 .3 .3')
    ET.SubElement(scene, 'statistic', center='0 0 .4', extent='1.5')
    default = ET.SubElement(scene, 'default')
    ET.SubElement(default, 'geom', contype='0', conaffinity='0', group='1')
    asset = ET.SubElement(scene, 'asset')
    world = ET.SubElement(scene, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -1 2', dir='0 .3 -1')
    ET.SubElement(world, 'geom', name='ground_visual', type='plane', pos='0 0 -.25', size='2 2 .01', rgba='.16 .19 .24 1', group='0')
    ET.SubElement(world, 'geom', name='mount_visual', type='cylinder', pos='0 0 -.04', size='.085 .04', rgba='.26 .31 .37 1')
    links = {link.get('name'): link for link in source.findall('link')}
    if len(links) != len(source.findall('link')):
        raise ValueError('Duplicate URDF link names')
    joint_names = [joint.get('name') for joint in source.findall('joint')]
    if len(set(joint_names)) != len(joint_names):
        raise ValueError('Duplicate URDF joint names')
    children = {}
    for joint in source.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    meshes, visited, order, arm_geoms = {}, set(), [], []
    def geometry(body, element, name, collision):
        shape = element.find('geometry')
        attrs = dict(name=name, **origin(element), group='3' if collision else '1')
        if collision:
            attrs['rgba'] = '0 0 0 0'
        else:
            color = element.find('material/color')
            attrs['rgba'] = color.get('rgba') if color is not None else '.6 .6 .7 1'
        mesh = shape.find('mesh')
        if mesh is not None:
            filename = mesh.get('filename')
            # Package URIs map to this package's meshes; paths cannot escape it.
            rel = Path('meshes')/Path(filename).name if filename.startswith('package://') else Path(filename)
            path = (directory/rel).resolve()
            if not path.is_relative_to(directory.resolve()) or not path.is_file():
                raise ValueError('Missing or external mesh: '+str(path))
            key = (rel.as_posix(), mesh.get('scale', '1 1 1'))
            if key not in meshes:
                meshes[key] = 'robot_mesh_'+str(len(meshes))
                ET.SubElement(asset, 'mesh', name=meshes[key], file=key[0], scale=key[1])
            attrs.update(type='mesh', mesh=meshes[key])
        elif shape.find('box') is not None:
            attrs.update(type='box', size=fmt(float(x)/2 for x in shape.find('box').get('size').split()))
        elif shape.find('cylinder') is not None:
            node = shape.find('cylinder')
            attrs.update(type='cylinder', size=fmt([float(node.get('radius')), float(node.get('length'))/2]))
        elif shape.find('sphere') is not None:
            attrs.update(type='sphere', size=shape.find('sphere').get('radius'))
        else:
            raise ValueError('Unsupported geometry: '+name)
        ET.SubElement(body, 'geom', **attrs)
        if collision:
            arm_geoms.append(name)
    def descend(parent, link_name, joint=None):
        if link_name in visited:
            raise ValueError('URDF is not a tree: '+link_name)
        visited.add(link_name)
        body = ET.SubElement(parent, 'body', name=link_name, **(origin(joint) if joint is not None else {}))
        if joint is not None and joint.get('type') != 'fixed':
            name = joint.get('name')
            if name not in names or joint.get('type') not in ('revolute', 'prismatic') or joint.find('mimic') is not None:
                raise ValueError('Unsupported or undeclared movable joint: '+name)
            limit = joint.find('limit')
            lo, hi = float(limit.get('lower')), float(limit.get('upper'))
            index = names.index(name)
            if not math.isfinite(lo+hi) or lo >= hi or any(not lo <= profile[key][index] <= hi for key in ('home','planning_home')):
                raise ValueError('Invalid limits or home: '+name)
            ET.SubElement(body, 'joint', name=name, type='hinge' if joint.get('type') == 'revolute' else 'slide',
                          axis=joint.find('axis').get('xyz') if joint.find('axis') is not None else '1 0 0', range=fmt([lo,hi]))
            ET.SubElement(body, 'inertial', pos='0 0 0', mass='1', diaginertia='.001 .001 .001')
            order.append(name)
        link = links[link_name]
        for i, element in enumerate(link.findall('visual')):
            geometry(body, element, link_name+'_visual_'+str(i), False)
        collisions = link.findall('collision') or link.findall('visual')
        for i, element in enumerate(collisions):
            geometry(body, element, 'collision_arm_'+link_name+'_'+str(i), True)
        if link_name == profile['tool_link']:
            ET.SubElement(body, 'site', name='arm_tool', size='.008', rgba='1 .35 .08 1', group='2')
        for child in children.get(link_name, []):
            descend(body, child.find('child').get('link'), child)
    descend(world, profile['root_body'])
    if set(order) != set(names) or profile['tool_link'] not in visited:
        raise ValueError('Selected tree does not contain all joints/tool')
    key = ET.SubElement(ET.SubElement(scene, 'keyframe'), 'key', name='home', qpos=fmt(profile['home'][names.index(n)] for n in order))
    metadata(scene, profile)
    write(scene, directory/'scene.xml')
    if 'carried_unit' not in profile:
        return
    task_asset, station, carried, block, _, _ = task_components(directory)
    asset.extend(list(task_asset))
    payload = profile['carried_unit']
    carried.set('pos', fmt(payload['pos']))
    carried.set('quat', fmt(payload['quat']))
    parent = scene.find(f'.//body[@name="{payload["parent"]}"]')
    if parent is None:
        raise ValueError('Missing payload parent')
    parent.append(carried)
    world.append(station)
    world.append(block)
    for geom in scene.iter('geom'):
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
    key.set('qpos', fmt([profile['planning_home'][names.index(n)] for n in order]+[0,0,0]))
    collision_metadata(scene, arm_geoms, profile['mount_bodies'])
    write(scene, directory/'planning.xml')
    report = dict(robot=profile['id'], joint_order=names, mujoco_joint_order=order, link_count=len(visited),
                  source_urdf_sha256=hashlib.sha256((directory/profile['urdf']).read_bytes()).hexdigest(),
                  dynamics='Kinematic mock: placeholder inertias, no actuators or contact response.',
                  collision='Source collision geometry (visual fallback), convex hull per mesh; same welded/adjacent bodies excluded, all other pairs checked.',
                  payload=payload, files_sha256={name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in ('scene.xml','planning.xml')})
    (directory/'import_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


def write(scene, path):
    ET.indent(scene, space='  ')
    ET.ElementTree(scene).write(path, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'models/mec_arm/robot.json')
    parser.add_argument('--download', action='store_true', help='restore pinned MecArm sources from manifest')
    args = parser.parse_args()
    if args.config.resolve() == (ROOT/'models/mec_arm/robot.json').resolve():
        restore_sources(ROOT/'reference/mec_arm/manifest.json', args.download)
    generate(args.config.resolve())
