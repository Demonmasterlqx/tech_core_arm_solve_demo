#!/usr/bin/env python3
"""Rebuild the four pinned CAD arm packages, offline.

Runtime has no Python dependency. Regeneration uses numpy, scipy, trimesh,
fast-simplification and shapely (versions recorded in each geometry report).
The original export is byte-preserved in reference/exported_arms. Only generated
visual meshes are simplified; collision hulls/prisms enclose source triangles.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh
import shapely
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import constrained_delaunay_triangles

from import_robot import generate, fmt, write

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'reference/exported_arms'
ROBOTS = ('hkust_eng', 'pie_eng', 'zhuimeng_single', 'zhuimeng_dual')


def stl_triangles(path):
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], 'little')
    if len(data) != 84 + 50 * count:
        raise ValueError('Invalid binary STL: ' + str(path))
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    return np.frombuffer(data, dtype=dtype, offset=84)['vertices'].astype(float)


def hull(points, path):
    points = np.unique(points.reshape(-1, 3), axis=0)
    poly = ConvexHull(points)
    faces = poly.simplices.copy()
    normals = np.cross(points[faces[:, 1]]-points[faces[:, 0]], points[faces[:, 2]]-points[faces[:, 0]])
    reverse = np.einsum('ij,ij->i', normals, poly.equations[:, :3]) < 0
    faces[reverse] = faces[reverse, ::-1]
    mesh = trimesh.Trimesh(vertices=points, faces=faces, process=False)
    mesh.remove_unreferenced_vertices()
    mesh.export(path)


def split_triangles(triangles, depth):
    """Every original triangle belongs to a hull, including long boundary faces."""
    if depth == 0 or len(triangles) < 20:
        return [triangles]
    centres = triangles.mean(axis=1)
    axis = np.argmax(np.ptp(centres, axis=0))
    order = np.argsort(centres[:, axis], kind='stable')
    half = len(order) // 2
    return split_triangles(triangles[order[:half]], depth-1) + split_triangles(triangles[order[half:]], depth-1)


def jaw_prisms(triangles, axis):
    """Conservative projected silhouette; convex partition; full axial extent."""
    x = np.asarray(axis, dtype=float)
    x /= np.linalg.norm(x)
    y = np.cross(x, [1, 0, 0] if abs(x[0]) < .8 else [0, 1, 0])
    y /= np.linalg.norm(y)
    basis = np.column_stack((x, y, np.cross(x, y)))
    local = triangles @ basis
    polygons = [Polygon(t[:, 1:]) for t in local]
    shadow = unary_union([p for p in polygons if p.area > 1e-18])
    pieces = [shadow] if shadow.geom_type == 'Polygon' else list(shadow.geoms)
    # Fill internal screw holes, preserve the external concave jaw opening.
    filled = unary_union([Polygon(p.exterior) for p in pieces if p.geom_type == 'Polygon'])
    filled = filled.buffer(1e-7, quad_segs=1, join_style='mitre').simplify(1e-10, preserve_topology=True)
    parts = list(constrained_delaunay_triangles(filled).geoms)
    # Merge neighbouring triangles only when the union remains convex.
    changed = True
    while changed:
        changed = False
        for i, a in enumerate(parts):
            for j in range(i+1, len(parts)):
                b = parts[j]
                if not a.intersects(b):
                    continue
                union = a.union(b)
                convex = union.convex_hull
                if convex.area - union.area < 1e-16:
                    parts[i] = convex
                    parts.pop(j)
                    changed = True
                    break
            if changed:
                break
    missing = shadow.difference(unary_union(parts)).area
    if missing > 1e-15:
        raise ValueError('Jaw projection lost source area')
    lo, hi = local[:, :, 0].min()-1e-7, local[:, :, 0].max()+1e-7
    vertices = []
    for part in parts:
        yz = np.array(part.exterior.coords)[:-1]
        vertices.append(np.vstack([np.column_stack((np.full(len(yz), a), yz)) for a in (lo, hi)]) @ basis.T)
    return vertices, missing


def freeze(joint, value):
    origin = joint.find('origin')
    if origin is None:
        origin = ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    pos = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    rot = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
    axis /= np.linalg.norm(axis)
    # The pinned export labels auxiliary ranges as viewer defaults. Fixed
    # carrying poses are explicit assembly assumptions, not actuator limits.
    if not np.isfinite(value):
        raise ValueError('Nonfinite frozen joint coordinate')
    if joint.get('type') == 'prismatic':
        pos += rot.apply(axis * value)
    else:
        rot = rot * Rotation.from_rotvec(axis * value)
    origin.set('xyz', fmt(pos))
    origin.set('rpy', fmt(rot.as_euler('xyz')))
    joint.set('type', 'fixed')
    for tag in ('axis', 'limit', 'mimic'):
        node = joint.find(tag)
        if node is not None:
            joint.remove(node)


def build(robot, meshes=True, collision_only=False, only_links=None):
    directory = ROOT / 'models' / robot
    profile = json.loads((directory / 'robot.json').read_text())
    source = SOURCE / robot
    manifest = json.loads((source / 'manifest.json').read_text())
    tree = ET.parse(source / (robot+'.urdf')).getroot()
    removed = set(profile.get('excluded_links', []))
    for joint in list(tree.findall('joint')):
        if joint.find('child').get('link') in removed:
            tree.remove(joint)
        elif joint.get('name') in profile['locked_joints']:
            freeze(joint, profile['locked_joints'][joint.get('name')])
    report = dict(robot=robot, visual_target_faces=40000,
                  collision='Full source convex hulls; wrist spatial triangle partitions; jaws convex projected prisms. All retained source triangles are covered. Internal holes may be filled.',
                  versions={name: __import__(name).__version__ for name in ('numpy', 'scipy', 'trimesh', 'shapely', 'fast_simplification')}, links={})
    (directory / 'meshes').mkdir(exist_ok=True)
    (directory / 'collision').mkdir(exist_ok=True)
    previous = json.loads((directory / 'geometry_report.json').read_text()) if (directory / 'geometry_report.json').exists() else {}
    for link in list(tree.findall('link')):
        name = link.get('name')
        if name in removed:
            tree.remove(link)
            continue
        vis_path = directory / 'meshes' / (name+'.stl')
        if meshes and (not only_links or name in only_links):
            triangles = stl_triangles(source / 'meshes' / (name+'.stl'))
            if collision_only:
                visual_count = previous['links'][name]['visual_triangles']
            else:
                visual = trimesh.Trimesh(vertices=triangles.reshape(-1, 3), faces=np.arange(3*len(triangles)).reshape(-1, 3), process=True)
                if len(visual.faces) > 40000:
                    visual = visual.simplify_quadric_decimation(face_count=40000)
                visual.export(vis_path)
                visual_count = len(visual.faces)
            collision_paths = []
            missed = 0.
            if 'jaw' in name:
                # Export component order is also STL triangle order; jaws have
                # no collapsed source facets (verified before slicing).
                components = [c['triangles'] for c in manifest['components'] if c.get('link') == name and c.get('triangles')]
                if sum(components) != len(triangles):
                    raise ValueError('Cannot recover exact jaw component slices')
                if profile.get('merge_jaw_projection', False):
                    components = [len(triangles)]
                offset = 0
                for count in components:
                    parts, missing = jaw_prisms(triangles[offset:offset+count], profile['grasp_axis'])
                    missed = max(missed, missing)
                    offset += count
                    for points in parts:
                        path = directory / 'collision' / f'{name}_{len(collision_paths)}.obj'
                        hull(points, path)
                        collision_paths.append(path)
            elif name in profile.get('collision_source_components', []):
                components = [c['triangles'] for c in manifest['components'] if c.get('link') == name and c.get('triangles')]
                if sum(components) != len(triangles):
                    raise ValueError('Cannot recover exact source component slices: '+name)
                offset = 0
                for count in components:
                    path = directory / 'collision' / f'{name}_{len(collision_paths)}.obj'
                    hull(triangles[offset:offset+count], path)
                    offset += count
                    collision_paths.append(path)
            else:
                depth = profile.get('collision_partition_depth', {}).get(name, 0)
                for part in split_triangles(triangles, depth):
                    path = directory / 'collision' / f'{name}_{len(collision_paths)}.obj'
                    hull(part, path)
                    collision_paths.append(path)
            report['links'][name] = dict(source_triangles=len(triangles), visual_triangles=visual_count,
                                         source_sha256=hashlib.sha256((source/'meshes'/(name+'.stl')).read_bytes()).hexdigest(),
                                         collision_files=[p.relative_to(directory).as_posix() for p in collision_paths],
                                         projected_uncovered_area_m2=missed)
        else:
            report['links'][name] = previous['links'][name]
        link.find('visual/geometry/mesh').set('filename', 'meshes/'+name+'.stl')
        for node in list(link.findall('collision')):
            link.remove(node)
        for file in report['links'][name]['collision_files']:
            collision = ET.SubElement(link, 'collision')
            ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='0 0 0')
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=file)
        print(robot, name, report['links'][name]['source_triangles'], '->', report['links'][name]['visual_triangles'],
              'collision parts', len(report['links'][name]['collision_files']), flush=True)
    tree.insert(0, ET.Comment('Generated from pinned export. Gripper locked for rigid carrying; source URDF is unchanged.'))
    write(tree, directory / profile['urdf'])
    used = {directory/file for entry in report['links'].values() for file in entry['collision_files']}
    for path in (directory / 'collision').glob('*.obj'):
        if path not in used and any(path.stem.startswith(name+'_') for name in report['links']):
            path.unlink()  # This directory contains generated proxies only.
    outputs = used | {directory/profile['urdf'], directory/'robot.json'} | {
        directory/'meshes'/(name+'.stl') for name in report['links']}
    report['generated_sha256'] = {p.relative_to(directory).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in sorted(outputs)}
    (directory / 'geometry_report.json').write_text(json.dumps(report, indent=2)+'\n')
    generate(directory / 'robot.json')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', choices=ROBOTS, action='append')
    parser.add_argument('--reuse-meshes', action='store_true', help='Regenerate frames/scenes using existing visual/collision assets')
    parser.add_argument('--collision-only', action='store_true', help='Rebuild collision meshes, retaining generated visuals')
    parser.add_argument('--link', action='append', help='Rebuild only selected links; requires existing other assets')
    args = parser.parse_args()
    for robot in args.robot or ROBOTS:
        build(robot, not args.reuse_meshes, args.collision_only, args.link)
