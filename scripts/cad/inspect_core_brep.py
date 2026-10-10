#!/usr/bin/env python3
"""Audit extracted official technology-core BREP without changing its geometry."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Cone
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID, TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Iterator, TopoDS_Shape


def bounds(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    return list(box.Get())


def descend(shape, kind):
    result = []
    exp = TopExp_Explorer(shape, kind)
    while exp.More():
        result.append(exp.Current())
        exp.Next()
    return result


def compare_hkust(path, output):
    """Measure exported OBJ fronts independently of the exporter's bounds data."""
    manifest = json.loads(path.read_text())
    markers = []
    for part in manifest['meshes']:
        if not part['name'].startswith('pillar_L_light_'):
            continue
        obj = path.parent / part['file']
        if hashlib.sha256(obj.read_bytes()).hexdigest() != part['sha256']:
            raise ValueError(f'OBJ hash mismatch: {obj}')
        vertices, triangles = [], []
        for line in obj.read_text().splitlines():
            if line.startswith('v '):
                vertices.append(list(map(float, line.split()[1:])))
            elif line.startswith('f '):
                triangles.append([int(v.split('/')[0]) - 1 for v in line.split()[1:]])
        vertices = np.array(vertices) * 1000
        triangles = np.array(triangles)
        front = vertices[:, 0].max()
        selected = np.all(np.abs(vertices[triangles][:, :, 0] - front) < 1e-6, axis=1)
        points = vertices[triangles[selected]]
        area = np.linalg.norm(np.cross(points[:, 1] - points[:, 0],
                                       points[:, 2] - points[:, 0]), axis=1).sum() / 2
        markers.append({'name': part['name'], 'front_area_mm2': float(area),
                        'bounds_mm': [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()]})
    result = {'hkust_source': manifest['source'], 'hkust_sha256': manifest['source_sha256'],
              'source_commit': manifest['source_commit'], 'actual_obj_markers': markers,
              'total_marker_area_mm2': sum(m['front_area_mm2'] for m in markers),
              'hkust_shell_bounds_m': next(m['bounds_m'] for m in manifest['meshes'] if m['name'] == 'shell'),
              'hkust_fan_object_count': len(manifest['fan_components'])}
    (output / 'hkust_comparison.json').write_text(json.dumps(result, indent=2) + '\n')


def audit(source, output):
    shape = TopoDS_Shape()
    if not BRepTools.Read_s(shape, str(source), BRep_Builder()):
        raise RuntimeError('BREP read failed')
    output.mkdir(parents=True, exist_ok=True)
    parts = []
    shapes = []
    iterator = TopoDS_Iterator(shape)
    while iterator.More():
        part = iterator.Value()
        shapes.append(part)
        part_id = len(parts)
        b = bounds(part)
        row = {'id': part_id, 'shape_type': str(part.ShapeType()).split('.')[-1],
               'bounds_mm': b, 'size_mm': [b[i + 3] - b[i] for i in range(3)],
               'solid_count': len(descend(part, TopAbs_SOLID)),
               'shell_count': len(descend(part, TopAbs_SHELL)), 'faces': []}
        for face_index, face_shape in enumerate(descend(part, TopAbs_FACE)):
            face = TopoDS.Face_s(face_shape)
            surface = BRepAdaptor_Surface(face)
            prop = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, prop)
            item = {'id': face_index, 'surface': str(surface.GetType()).split('.')[-1],
                    'area_mm2': prop.Mass(), 'centroid_mm': list(prop.CentreOfMass().Coord()),
                    'bounds_mm': bounds(face)}
            if surface.GetType() == GeomAbs_Cylinder:
                cylinder = surface.Cylinder()
                item.update(radius_mm=cylinder.Radius(),
                            axis_point_mm=list(cylinder.Axis().Location().Coord()),
                            axis_direction=list(cylinder.Axis().Direction().Coord()))
            elif surface.GetType() == GeomAbs_Plane:
                plane = surface.Plane()
                item.update(plane_point_mm=list(plane.Location().Coord()),
                            plane_normal=list(plane.Axis().Direction().Coord()))
            elif surface.GetType() == GeomAbs_Cone:
                cone = surface.Cone()
                item.update(radius_mm=cone.RefRadius(), semi_angle_rad=cone.SemiAngle(),
                            axis_point_mm=list(cone.Axis().Location().Coord()),
                            axis_direction=list(cone.Axis().Direction().Coord()))
            row['faces'].append(item)
        row['surface_counts'] = dict(Counter(f['surface'] for f in row['faces']))
        parts.append(row)
        iterator.Next()
    manifest = {'source': str(source), 'source_bytes': source.stat().st_size,
                'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                'units': 'millimetres', 'bounds_mm': bounds(shape), 'parts': parts,
                'topology_note': 'Direct children are geometry components, not verified STEP product names. Shells must not be silently treated as closed solids.'}
    (output / 'components.json').write_text(json.dumps(manifest, indent=2) + '\n')
    # These component numbers identify this extracted source, not arbitrary BREP files.
    if len(parts) == 418 and len(parts[365]['faces']) == 117:
        reference = parts[365]['faces'][8]
        if reference.get('radius_mm') is None or abs(reference['radius_mm'] - 18) > 1e-6:
            raise ValueError('Known peg reference cylinder no longer matches source')
        origin = np.asarray(reference['axis_point_mm'])
        xaxis = np.asarray(reference['axis_direction'])
        yaxis = np.cross([0, 0, 1], xaxis)
        yaxis /= np.linalg.norm(yaxis)
        zaxis = np.cross(xaxis, yaxis)
        rotation = np.array([xaxis, yaxis, zaxis])
        matrix = np.column_stack([rotation, -rotation @ origin])
        transform = gp_Trsf()
        transform.SetValues(*matrix.flatten())
        local_parts = []
        for part_id, part in enumerate(shapes):
            local = BRepBuilderAPI_Transform(part, transform, False).Shape()
            b = bounds(local)
            local_parts.append({'id': part_id, 'bounds_mm': b,
                                'size_mm': [b[i + 3] - b[i] for i in range(3)]})
        local_manifest = {'source_sha256': manifest['source_sha256'],
                          'source_E_mm': origin.tolist(), 'source_to_core_mm': matrix.tolist(),
                          'frame': '+X outward peg axis, +Z closest to source global vertical; right-handed. Front camera screen right is +Y.',
                          'peg_reference': {'component': 365, 'face': 8, 'radius_mm': 18,
                                            'origin_proof': 'Cylindrical axis point lies in the front end planar faces at local X=0.'},
                          'parts': local_parts}
        (output / 'local_components.json').write_text(json.dumps(local_manifest, indent=2) + '\n')
        marker_results = []
        for face_id, face in enumerate(descend(shapes[370], TopAbs_FACE)):
            # Same-plane mask cut is an explicit check; it is not a full visibility renderer.
            cut = BRepAlgoAPI_Cut(face, shapes[369])
            cut.Build()
            if not cut.IsDone():
                raise RuntimeError('MARK coplanar mask subtraction failed')
            visible = cut.Shape()
            prop = GProp_GProps()
            BRepGProp.SurfaceProperties_s(visible, prop)
            local = BRepBuilderAPI_Transform(visible, transform, False).Shape()
            marker_results.append({'id': face_id, 'visible_area_mm2': prop.Mass(),
                                   'bounds_mm': bounds(local), 'boolean_ok': True,
                                   'scope': 'Component 370 face minus component 369 coplanar black mask; other depth occluders not included.'})
        (output / 'marker_visible_boolean.json').write_text(json.dumps(marker_results, indent=2) + '\n')
    print(json.dumps({'parts': len(parts), 'bounds_mm': manifest['bounds_mm'],
                      'faces': sum(len(p['faces']) for p in parts),
                      'solids': sum(p['solid_count'] for p in parts),
                      'shells': sum(p['shell_count'] for p in parts)}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--hkust-manifest', type=Path)
    args = parser.parse_args()
    audit(args.source, args.output)
    if args.hkust_manifest:
        compare_hkust(args.hkust_manifest, args.output)
