#!/usr/bin/env python3
"""Add the RM2026 rulebook p64 bore interface to the untouched official energy CAD.

Only four INTERNAL source faces change. All other original OBJ vertices, normals
and face records are copied literally. The new cylinder and blind stop are visual
interfaces, not a claim about an unprovided manufacturing wall/iron thickness.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool

from import_energy import Mesh, SOURCE_SHA256, faces, triangulate_face, write_obj

GRAY_LINEAR = (0.147319004, 0.147319004, 0.147319004)
GRAY_SRGB = (0.420000006, 0.420000006, 0.420000006)
INTERNAL_FACES_REPLACED = (187, 188, 189, 254)
RADIUS_M = .020
STOP_X_M = .100


def load_obj(path: Path) -> tuple[np.ndarray, np.ndarray]:
    vertices: list[list[float]] = []
    triangles: list[list[int]] = []
    for line in path.read_text().splitlines():
        values = line.split()
        if not values:
            continue
        if values[0] == "v":
            vertices.append([float(v) for v in values[1:]])
        elif values[0] == "f":
            triangles.append([int(v.split("/")[0]) - 1 for v in values[1:]])
    return np.asarray(vertices), np.asarray(triangles, dtype=np.int64)


def metadata(mesh: Mesh, path: Path) -> dict[str, object]:
    vertices = np.asarray(mesh.vertices)
    return {"name": mesh.name, "file": path.name, "role": mesh.role,
            "default_visible": True, "rgb_linear": mesh.rgb_linear,
            "rgb_srgb": mesh.rgb_srgb, "vertices": len(mesh.vertices),
            "triangles": len(mesh.triangles), "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "bounds_m": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()]}


def axial_hit_x(points: np.ndarray, tris: np.ndarray, radius_y: float, radius_z: float) -> float:
    """First ray/triangle intersection without external acceleration dependencies."""
    origin = np.array([-.001, radius_y, radius_z])
    direction = np.array([1.0, 0.0, 0.0])
    a, b, c = (points[tris[:, k]] for k in range(3))
    edge1, edge2 = b - a, c - a
    h = np.cross(np.broadcast_to(direction, edge2.shape), edge2)
    determinant = np.einsum("ij,ij->i", edge1, h)
    valid = np.abs(determinant) > 1e-15
    inverse = np.zeros_like(determinant)
    inverse[valid] = 1 / determinant[valid]
    relative = origin - a
    u = inverse * np.einsum("ij,ij->i", relative, h)
    q = np.cross(relative, edge1)
    v = inverse * q[:, 0]
    t = inverse * np.einsum("ij,ij->i", edge2, q)
    valid &= (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8) & (t > 1e-8)
    if not valid.any():
        raise AssertionError("Bore ray escaped without reaching a blind stop")
    return float(t[valid].min() + origin[0])


def correct_energy_bore(source: Path, original: Path, output: Path) -> dict[str, object]:
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("Unexpected official STEP checksum")
    baseline = json.loads((original / "manifest.json").read_text())
    for asset in baseline["meshes"]:
        if hashlib.sha256((original / asset["file"]).read_bytes()).hexdigest() != asset["sha256"]:
            raise ValueError(f"Original official mesh was changed: {asset['file']}")
    if original.resolve() == output.resolve():
        raise ValueError("Corrected and original directories must differ")
    output.mkdir(parents=True, exist_ok=True)
    gray = next(m for m in baseline["meshes"] if m["name"] == "energy_gray_body")
    original_lines = (original / gray["file"]).read_text().splitlines(keepends=True)
    skip_indices: set[int] = set()
    offset = 0
    for face in baseline["faces"]:
        if face["mesh"] == gray["name"]:
            if face["index"] in INTERNAL_FACES_REPLACED:
                skip_indices.update(range(offset, offset + face["triangles"]))
            offset += face["triangles"]
    destination = output / "energy_gray_body_rule.obj"
    retained_lines: list[str] = []
    triangle_index = 0
    for line in original_lines:
        if line.startswith("f "):
            if triangle_index not in skip_indices:
                retained_lines.append(line)
            triangle_index += 1
        else:
            retained_lines.append(line)
    destination.write_text("".join(retained_lines), encoding="utf-8", newline="\n")
    # Preserve white panels byte-for-byte, including exact normals.
    white = next(m for m in baseline["meshes"] if m["name"] == "energy_white_panels")
    shutil.copyfile(original / white["file"], output / white["file"])
    reader = STEPCAFControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise RuntimeError("STEP loading failed")
    doc = TDocStd_Document(TCollection_ExtendedString("energy_bore_revision"))
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    source_faces = faces(tool.GetShape_s(roots.Value(1)))
    cutter = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, -1, 0), gp_Dir(0, 1, 0)), 20, 102).Shape()
    repair = Mesh("energy_bore_internal_repairs", "rule_corrected_internal_surface", GRAY_LINEAR, GRAY_SRGB)
    taper_max_x = 0.0
    for index in (187, 188, 254):
        cut = BRepAlgoAPI_Cut(source_faces[index], cutter)
        cut.Build()
        if not cut.IsDone():
            raise RuntimeError(f"Bore trim failed for source face {index}")
        result = cut.Shape()
        BRepMesh_IncrementalMesh(result, .05, False, math.radians(8), False)
        for face in faces(result):
            points, normals, triangles = triangulate_face(face)
            if index in (187, 188):
                taper_max_x = max(taper_max_x, float(points[:, 0].max()))
            offset = len(repair.vertices)
            repair.vertices.extend(points.tolist())
            repair.normals.extend(normals.tolist())
            repair.triangles.extend((triangles + offset).tolist())
    liner = Mesh("energy_bore_liner", "rule_bore_visual_interface", GRAY_LINEAR, GRAY_SRGB)
    stop = Mesh("energy_bore_stop", "rule_iron_stop_visual_interface", GRAY_LINEAR, GRAY_SRGB)
    sectors = 180  # 2 degrees, radial chord sag <= 0.00305 mm.
    for i in range(sectors):
        angle = 2 * math.pi * i / sectors
        y, z = RADIUS_M * math.cos(angle), RADIUS_M * math.sin(angle)
        liner.vertices.extend([[taper_max_x, y, z], [STOP_X_M, y, z]])
        liner.normals.extend([[0, -math.cos(angle), -math.sin(angle)]] * 2)
        stop.vertices.append([STOP_X_M, y, z])
        stop.normals.append([-1, 0, 0])
    stop.vertices.append([STOP_X_M, 0, 0])
    stop.normals.append([-1, 0, 0])
    for i in range(sectors):
        j = (i + 1) % sectors
        liner.triangles.extend([[2*i, 2*j+1, 2*j], [2*i, 2*i+1, 2*j+1]])
        stop.triangles.append([sectors, j, i])
    new_metadata = []
    for mesh in (repair, liner, stop):
        path = output / f"{mesh.name}.obj"
        write_obj(mesh, path)
        new_metadata.append(metadata(mesh, path))
    gray_rule = {**gray, "name": "energy_gray_body_rule", "file": destination.name,
                 "role": "official_geometry_internal_faces_removed",
                 "triangles": gray["triangles"] - len(skip_indices),
                 "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
                 "bytes": destination.stat().st_size,
                 "source_face_indices": [i for i in gray["source_face_indices"] if i not in INTERNAL_FACES_REPLACED],
                 "face_count": gray["face_count"] - len(INTERNAL_FACES_REPLACED)}
    meshes = [gray_rule, white, *new_metadata]
    all_vertices, all_triangles = [], []
    vertex_offset = 0
    for asset in meshes:
        vertices, triangles = load_obj(output / asset["file"])
        all_vertices.append(vertices)
        all_triangles.append(triangles + vertex_offset)
        vertex_offset += len(vertices)
    points, triangles = np.vstack(all_vertices), np.vstack(all_triangles)
    rays = []
    for y, z in ((0, 0), (.010, 0), (-.010, 0), (.0199, 0), (-.0199, 0), (0, .0199), (0, -.0199)):
        hit = axial_hit_x(points, triangles, y, z)
        if abs(hit - STOP_X_M) > 1e-8:
            raise AssertionError(f"Premature bore obstruction at {hit} m")
        rays.append({"y_m": y, "z_m": z, "first_hit_x_m": hit})
    # Bounds use only referenced vertices; unused removed-face vertices cannot mask changes.
    corrected_bounds = np.array([points[np.unique(triangles)].min(axis=0), points[np.unique(triangles)].max(axis=0)])
    raw_points = []
    for asset in (gray, white):
        p, t = load_obj(original / asset["file"])
        raw_points.append(p[np.unique(t)])
    raw_points_array = np.vstack(raw_points)
    raw_bounds = np.array([raw_points_array.min(axis=0), raw_points_array.max(axis=0)])
    if not np.array_equal(corrected_bounds, raw_bounds):
        raise AssertionError("Rule bore correction changed exterior bounding box")
    manifest: dict[str, object] = {
        "schema_version": 1, "asset": "RM2026 energy unit with rulebook V2.2.0 bore interface",
        "source": baseline["source"], "frame": baseline["frame"],
        "official_baseline_manifest_sha256": hashlib.sha256((original / "manifest.json").read_bytes()).hexdigest(),
        "rule_source": {"file": "RoboMaster_2026_机甲大师超级对抗赛比赛规则手册V2.2.0（20260807）.pdf",
                        "page": 64, "figure": "4-39", "bore_diameter_mm": 40, "mouth_to_iron_stop_mm": 100},
        "changes": {"source_face_indices_removed": list(INTERNAL_FACES_REPLACED),
                    "source_face_189": "Remove incorrect shallow closing disk at CAD Y=5 mm",
                    "source_faces_187_188": "Retain original conical surface outside radius 20 mm; only trim its sub-20 mm throat",
                    "source_face_254": "Retain internal CAD Y=10 mm cavity end disk outside radius 20 mm",
                    "liner": "Zero-thickness radius 20 mm inward-facing cylindrical visual interface to X=100 mm",
                    "iron_stop": "Zero-thickness disk at X=100 mm; no unprovided iron thickness or new mass assumed",
                    "unchanged": "All other official solid surface vertex, normal and face records; all white-panel OBJ bytes; original official asset directory"},
        "geometry": {"bore_radius_m": RADIUS_M, "bore_throat_x_m": taper_max_x,
                     "mouth_plane_x_m": 0, "iron_stop_x_m": STOP_X_M, "body_length_m": .150,
                     "cylinder_sectors": sectors,
                     "cylinder_max_radial_sag_mm": RADIUS_M * (1-math.cos(math.pi/sectors))*1000,
                     "bore_and_stop_are_visual_interfaces": True,
                     "manufacturing_wall_thickness_known": False},
        "verification": {"exterior_bounds_bitwise_unchanged": True,
                         "bounds_m": raw_bounds.tolist(), "axial_first_hit_rays": rays,
                         "retained_gray_triangle_records": gray_rule["triangles"],
                         "retained_gray_vertex_and_normal_records_byte_identical": True,
                         "white_obj_byte_identical": True},
        "meshes": meshes,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def save_section_image(original: Path, corrected: Path, destination: Path) -> None:
    """Compare actual triangle intersections; this is not a schematic silhouette."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    figure, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, sharey=True)
    for axis, directory, title in zip(axes, (original, corrected), (
            "Official STEP tessellation: closed shallow recess",
            "Rulebook p64 interface: 40 mm bore, 100 mm blind-stop depth")):
        manifest = json.loads((directory / "manifest.json").read_text())
        for asset in manifest["meshes"]:
            if not asset["default_visible"]:
                continue
            points, triangles = load_obj(directory / asset["file"])
            segments = []
            for triangle in points[triangles]:
                cuts = []
                for i, j in ((0, 1), (1, 2), (2, 0)):
                    a, b = triangle[i], triangle[j]
                    if abs(a[1]) < 1e-10:
                        cuts.append(a)
                    if a[1] * b[1] < 0:
                        cuts.append(a + (b-a) * (-a[1]) / (b[1]-a[1]))
                unique = np.unique(np.round(cuts, 10), axis=0) if cuts else []
                if len(unique) == 2:
                    segments.append(np.asarray(unique)[:, [0, 2]] * 1000)
            is_rule_interface = asset["name"] in ("energy_bore_liner", "energy_bore_stop", "energy_bore_internal_repairs")
            axis.add_collection(LineCollection(segments, colors="#1976b9" if is_rule_interface else "#555555",
                                               linewidths=1.1 if is_rule_interface else .7))
        axis.set_title(title)
        axis.set_aspect("equal")
        axis.set_xlim(-3, 153)
        axis.set_ylim(-52, 52)
        axis.set_ylabel("Local Z / mm")
        axis.grid(alpha=.2)
    axes[1].annotate("First axial hit = 100 mm", xy=(100, 0), xytext=(48, 8),
                     arrowprops={"arrowstyle": "->", "color": "#1976b9"}, color="#1976b9")
    axes[1].set_xlabel("Local X / mm (mouth plane at 0)")
    figure.tight_layout()
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--original", type=Path, default=Path("models/meshes/energy_unit"))
    parser.add_argument("--output", type=Path, default=Path("models/meshes/energy_unit_rule"))
    parser.add_argument("--section-image", type=Path, help="Optional CPU matplotlib comparison of actual mesh sections")
    args = parser.parse_args()
    manifest = correct_energy_bore(args.source, args.original, args.output)
    if args.section_image:
        save_section_image(args.original, args.output, args.section_image)
    print(json.dumps({"geometry": manifest["geometry"], "verification": manifest["verification"]}, indent=2))


if __name__ == "__main__":
    main()
