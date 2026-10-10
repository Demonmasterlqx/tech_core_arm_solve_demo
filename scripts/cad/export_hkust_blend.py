#!/usr/bin/env python3
"""Blender 4.5: export the user-designated HKUST core without geometry simplification.

Run with Blender's Python, for example:
  blender --background --factory-startup --python scripts/cad/export_hkust_blend.py

All evaluated mesh vertices are retained. OBJ files contain triangulated faces and
per-corner normals, separated by object/material for MuJoCo lighting control.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[2]
ROTATION = Matrix(((0, 0, 1), (1, 0, 0), (0, 1, 0)))
DATUM_NAME = "空物体.001"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def bound(vertices):
    return {
        "min": [min(v[k] for v in vertices) for k in range(3)],
        "max": [max(v[k] for v in vertices) for k in range(3)],
    }


def srgb(value):
    return 12.92 * value if value <= 0.0031308 else 1.055 * value ** (1 / 2.4) - 0.055


def rgba_srgb(values):
    return [srgb(c) for c in values[:3]] + [values[3]]


def socket_value(node, name, default):
    socket = node.inputs.get(name)
    if socket is None:
        return default
    value = socket.default_value
    return list(value) if hasattr(value, "__iter__") else float(value)


def material_info(material):
    if material is None:
        return {"name": None, "rgba": [0.8, 0.8, 0.8, 1], "emission_strength": 0.0}
    nodes = list(material.node_tree.nodes) if material.use_nodes else []
    principled = next((n for n in nodes if n.type == "BSDF_PRINCIPLED"), None)
    emission = next((n for n in nodes if n.type == "EMISSION"), None)
    base = socket_value(principled, "Base Color", list(material.diffuse_color)) if principled else list(material.diffuse_color)
    emit = socket_value(principled, "Emission Color", [0, 0, 0, 1]) if principled else [0, 0, 0, 1]
    strength = socket_value(principled, "Emission Strength", 0) if principled else 0
    if emission is not None:
        emit = socket_value(emission, "Color", emit)
        strength = socket_value(emission, "Strength", strength)
        if principled is None:
            base = emit[:]
    graph = []
    for node in nodes:
        parameters = {}
        for socket in node.inputs:
            if socket.name not in ("Base Color", "Color", "Emission Color", "Emission Strength", "Strength", "Metallic", "Roughness", "Alpha"):
                continue
            if not hasattr(socket, "default_value"):
                continue
            value = socket.default_value
            parameters[socket.name] = {
                "value": list(value) if hasattr(value, "__iter__") else value,
                "linked": socket.is_linked,
            }
        if parameters:
            graph.append({"name": node.name, "type": node.type, "inputs": parameters})
    return {
        "name": material.name,
        "rgba": base,
        "base_color_linear_rgba": base,
        "base_color_srgb_rgba": rgba_srgb(base),
        "emission_color_linear_rgba": emit,
        "emission_color_srgb_rgba": rgba_srgb(emit),
        "emission_strength": strength,
        "metallic": socket_value(principled, "Metallic", 0) if principled else 0,
        "roughness": socket_value(principled, "Roughness", 0.5) if principled else 0.5,
        "shader_parameter_evidence": graph,
        "shader_conversion_note": "Scalar colors are node defaults. Blender procedural bump, mixed BSDF and Cycles emission are preserved in the source blend but are not equivalent to MuJoCo Phong materials.",
    }


def group(name):
    if name == "pillar_slide":
        return "lift"
    if name.startswith("pillar_"):
        return "pillar"
    return "station"


def write_obj(path, mesh, triangles, positions, normal_matrix, object_name):
    vertex_indices = sorted({i for triangle in triangles for i in triangle.vertices})
    vertex_map = {i: j + 1 for j, i in enumerate(vertex_indices)}
    loop_indices = sorted({i for triangle in triangles for i in triangle.loops})
    loop_map = {i: j + 1 for j, i in enumerate(loop_indices)}
    with path.open("w", encoding="utf-8", newline="\n") as target:
        target.write("# HKUST ENTERPRIZE exchange.blend; MIT; see reference/hkust/LICENSE\n")
        target.write("# SI metres; E origin; +X outward; +Z up; evaluated source geometry, no LOD\n")
        target.write(f"o {object_name}\n")
        for index in vertex_indices:
            target.write("v " + " ".join(format(v, ".17g") for v in positions[index]) + "\n")
        for index in loop_indices:
            normal = (normal_matrix @ mesh.corner_normals[index].vector).normalized()
            target.write("vn " + " ".join(format(v, ".17g") for v in normal) + "\n")
        for triangle in triangles:
            corners = zip(triangle.vertices, triangle.loops)
            target.write("f " + " ".join(f"{vertex_map[v]}//{loop_map[n]}" for v, n in corners) + "\n")
    return vertex_indices, len(loop_indices)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "reference/hkust/exchange.blend")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "models/meshes/core")
    args = parser.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    source_metadata_path = ROOT / "reference/hkust/source.json"
    source_metadata = json.loads(source_metadata_path.read_text(encoding="utf-8"))
    source_hash = sha256(args.source)
    if source_hash != source_metadata["sha256"]:
        raise RuntimeError("Blender source differs from the pinned, copied HKUST source")
    bpy.ops.wm.open_mainfile(filepath=str(args.source.resolve()))
    scene = bpy.context.scene
    scale = scene.unit_settings.scale_length
    datum = bpy.data.objects.get(DATUM_NAME)
    if datum is None:
        raise RuntimeError(f"Source E datum missing: {DATUM_NAME}")
    origin = datum.matrix_world.translation.copy()
    transform = ROTATION.to_4x4()
    transform.translation = -(ROTATION @ origin) * scale
    for row in range(3):
        for column in range(3):
            transform[row][column] *= scale
    objects = sorted((o for o in bpy.data.objects if o.type == "MESH"), key=lambda o: o.name)
    if len(objects) != 59:
        raise RuntimeError(f"Expected 59 source mesh objects, got {len(objects)}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    dependency_graph = bpy.context.evaluated_depsgraph_get()
    manifest = {
        "source": "reference/hkust/exchange.blend",
        "source_sha256": source_hash,
        "source_commit": source_metadata["source_commit"],
        "license": "reference/hkust/LICENSE",
        "exporter": "scripts/cad/export_hkust_blend.py",
        "blender_version": bpy.app.version_string,
        "source_units": {"system": scene.unit_settings.system, "scale_length_m": scale},
        "canonical_frame": {
            "origin": "E, the explicit source pillar-tip datum 空物体.001",
            "source_origin_m": list(origin * scale),
            "formula": "canonical_xyz = [source_z - E_z, source_x - E_x, source_y - E_y] * scene_scale_length",
            "transform_source_world_to_canonical_m": [list(row) for row in transform],
            "rotation_determinant": ROTATION.determinant(),
            "axes": {"X": "out of the pillar/front panel", "Y": "Blender world +X", "Z": "up, Blender world +Y"},
            "marker_front_x_m": (0.012000000104308128 - origin.z) * scale,
            "geometry_note": "The evaluated central electromagnet protrudes beyond nominal E; no source vertices were moved to force it behind the datum.",
        },
        "source_empty_datums": [{"name": o.name, "matrix_world": [list(r) for r in o.matrix_world]} for o in bpy.data.objects if o.type == "EMPTY"],
        "export_policy": {"evaluated_modifiers": True, "corner_normals": True, "per_object_material_parts": True, "lod_or_decimation": False, "triangulation": "Blender mesh.calc_loop_triangles", "uv_exported": False},
        "objects": [],
        "materials": {},
        "parts": [],
    }
    all_positions = []
    for obj in objects:
        evaluated = obj.evaluated_get(dependency_graph)
        mesh = evaluated.to_mesh(preserve_all_data_layers=True, depsgraph=dependency_graph)
        try:
            mesh.calc_loop_triangles()
            matrix = transform @ evaluated.matrix_world
            normal_matrix = matrix.to_3x3().inverted().transposed()
            positions = [matrix @ vertex.co for vertex in mesh.vertices]
            all_positions.extend(positions)
            used_materials = sorted({triangle.material_index for triangle in mesh.loop_triangles})
            object_record = {
                "name": obj.name,
                "source_mesh": obj.data.name,
                "source_matrix_world": [list(row) for row in obj.matrix_world],
                "source_vertices": len(obj.data.vertices),
                "evaluated_vertices": len(mesh.vertices),
                "triangles": len(mesh.loop_triangles),
                "bounds_m": bound(positions),
                "motion_group": group(obj.name),
                "modifiers": [{"name": modifier.name, "type": modifier.type, "viewport": modifier.show_viewport, "render": modifier.show_render} for modifier in obj.modifiers],
            }
            manifest["objects"].append(object_record)
            for material_index in used_materials:
                material = mesh.materials[material_index] if material_index < len(mesh.materials) else None
                material_name = material.name if material else "default"
                manifest["materials"][material_name] = material_info(material)
                suffix = "" if len(used_materials) == 1 else "__" + re.sub(r"[^A-Za-z0-9_-]", "_", material_name)
                filename = f"{obj.name}{suffix}.obj"
                triangles = [triangle for triangle in mesh.loop_triangles if triangle.material_index == material_index]
                path = args.output_dir / filename
                used_vertices, normal_count = write_obj(path, mesh, triangles, positions, normal_matrix, obj.name)
                manifest["parts"].append({
                    "filename": filename,
                    "source_object": obj.name,
                    "source_material": material_name,
                    "material_index": material_index,
                    "motion_group": group(obj.name),
                    "rgba": manifest["materials"][material_name]["rgba"],
                    "bounds_m": bound([positions[i] for i in used_vertices]),
                    "vertices": len(used_vertices),
                    "corner_normals": normal_count,
                    "triangles": len(triangles),
                    "sha256": sha256(path),
                })
        finally:
            evaluated.to_mesh_clear()
    # Stable integration contract used by the MJCF generator.
    manifest["meshes"] = [{
        "name": Path(part["filename"]).stem,
        "file": part["filename"],
        "object": part["source_object"],
        "material": part["source_material"],
        "rgba_linear": manifest["materials"][part["source_material"]]["base_color_linear_rgba"],
        "rgba_srgb": manifest["materials"][part["source_material"]]["base_color_srgb_rgba"],
        "emission_linear": manifest["materials"][part["source_material"]]["emission_color_linear_rgba"],
        "emission_strength": manifest["materials"][part["source_material"]]["emission_strength"],
        "motion_group": part["motion_group"],
        "vertices": part["vertices"],
        "triangles": part["triangles"],
        "bounds_m": part["bounds_m"],
        "sha256": part["sha256"],
    } for part in manifest["parts"]]
    manifest["frame"] = manifest["canonical_frame"]
    manifest["object_count"] = len(objects)
    manifest["part_count"] = len(manifest["parts"])
    manifest["total_triangles"] = sum(part["triangles"] for part in manifest["parts"])
    manifest["bounds_m"] = bound(all_positions)
    manifest["marker_components"] = [o.name for o in objects if o.name.startswith("pillar_L_light_")]
    manifest["fan_components"] = [o.name for o in objects if o.name.startswith("back_light_bar_")]
    if manifest["part_count"] != 60 or len(manifest["marker_components"]) != 6 or len(manifest["fan_components"]) != 36:
        raise RuntimeError("Source object/material decomposition differs from the inspected source")
    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Exported {len(objects)} source objects, {manifest['part_count']} material parts, {manifest['total_triangles']} triangles")
    print("Canonical transform:", manifest["canonical_frame"]["transform_source_world_to_canonical_m"])
    print("Bounds (m):", manifest["bounds_m"])


if __name__ == "__main__":
    main()
