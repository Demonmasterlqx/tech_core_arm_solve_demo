#!/usr/bin/env python3
"""Tessellate the official RM2026 energy unit STEP without remodeling its surfaces.

This is an offline asset tool; the simulation does not need Python or OpenCascade.
Install cadquery-ocp==7.8.1.1 and numpy, then pass the downloaded STEP as --source.
The official STEP has a shallow closed recess, not a 100 mm external blind bore.
The import deliberately preserves that geometry and reports the discrepancy.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepLProp import BRepLProp_SLProps
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Face, TopoDS_Shape
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool

SOURCE_SHA256 = "35d105a221f4527e5c5e84ac607c94c797ec2e19c734974de4c5485c2c2d5ee7"
SOURCE_URL = "https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/21cbbecaaceb47729bc477cd12f8de5b1761107116144/能量单元.stp"
POST_URL = "https://bbs.robomaster.com/article/814728"
# Proper rotation: the mouth centre is CAD origin, +Y enters the unit.
ROTATION = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


@dataclass
class Mesh:
    name: str
    role: str
    rgb_linear: tuple[float, float, float] | None
    rgb_srgb: tuple[float, float, float] | None
    vertices: list[list[float]] = field(default_factory=list)
    normals: list[list[float]] = field(default_factory=list)
    triangles: list[list[int]] = field(default_factory=list)
    face_indices: list[int] = field(default_factory=list)


def bounds(shape: TopoDS_Shape) -> list[list[float]]:
    """Exact-surface optimal box; avoid the loose B-spline control-pole box."""
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    values = box.Get()
    return [list(values[:3]), list(values[3:])]


def faces(shape: TopoDS_Shape) -> list[TopoDS_Face]:
    result: list[TopoDS_Face] = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        result.append(TopoDS.Face_s(explorer.Current()))
        explorer.Next()
    return result


def triangulate_face(face: TopoDS_Face) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Keep face seams sharp and use CAD surface normals, not cross-face averages."""
    location = TopLoc_Location()
    triangulation = BRep_Tool.Triangulation_s(face, location)
    if triangulation is None or triangulation.NbTriangles() == 0:
        raise RuntimeError("OpenCascade failed to tessellate a source face")
    placement = location.Transformation()
    points = np.array([
        triangulation.Node(i).Transformed(placement).Coord()
        for i in range(1, triangulation.NbNodes() + 1)
    ])
    triangles = np.array([
        triangulation.Triangle(i).Get()
        for i in range(1, triangulation.NbTriangles() + 1)
    ], dtype=np.int64) - 1
    reversed_face = face.Orientation() == TopAbs_REVERSED
    if reversed_face:
        triangles = triangles[:, [0, 2, 1]]
    # Fallback normals at parametric singularities only.
    normal_sum = np.zeros_like(points)
    cross = np.cross(points[triangles[:, 1]] - points[triangles[:, 0]],
                     points[triangles[:, 2]] - points[triangles[:, 0]])
    for corner in range(3):
        np.add.at(normal_sum, triangles[:, corner], cross)
    norms = np.linalg.norm(normal_sum, axis=1)
    normals = normal_sum / np.maximum(norms[:, None], 1e-30)
    surface = BRepAdaptor_Surface(face)
    if triangulation.HasUVNodes():
        for i in range(triangulation.NbNodes()):
            uv = triangulation.UVNode(i + 1)
            prop = BRepLProp_SLProps(surface, uv.X(), uv.Y(), 1, 1e-10)
            if prop.IsNormalDefined():
                normals[i] = np.array(prop.Normal().Coord()) * (-1 if reversed_face else 1)
    if not np.isfinite(normals).all() or (np.linalg.norm(normals, axis=1) < 0.99).any():
        raise RuntimeError("Invalid tessellation normal")
    return points @ ROTATION.T * 0.001, normals @ ROTATION.T, triangles


def write_obj(mesh: Mesh, path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("# Official RM2026 energy unit; metres; +X mouth to end cap.\n")
        stream.write(f"# Source SHA256 {SOURCE_SHA256}\n")
        stream.write(f"o {mesh.name}\n")
        for vertex in mesh.vertices:
            stream.write("v " + " ".join(f"{v:.10f}" for v in vertex) + "\n")
        for normal in mesh.normals:
            stream.write("vn " + " ".join(f"{v:.9f}" for v in normal) + "\n")
        for tri in mesh.triangles:
            stream.write("f " + " ".join(f"{i + 1}//{i + 1}" for i in tri) + "\n")


def import_energy(source: Path, output: Path, deflection_mm: float = 0.05,
                  angle_deg: float = 8.0) -> dict[str, object]:
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != SOURCE_SHA256:
        raise ValueError(f"Unexpected STEP revision: {digest}; refusing unverified geometry")
    if not 0 < deflection_mm <= 0.1 or not 0 < angle_deg <= 10:
        raise ValueError("Use a positive deflection <= 0.1 mm and angle <= 10 degrees")
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise RuntimeError("STEP reader failed")
    document = TDocStd_Document(TCollection_ExtendedString("energy_unit"))
    if not reader.Transfer(document):
        raise RuntimeError("STEPCAF document transfer failed")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(document.Main())
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    if roots.Length() != 1:
        raise ValueError("Expected one energy-unit assembly")
    shape = shape_tool.GetShape_s(roots.Value(1))
    solid_explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    solids: list[TopoDS_Shape] = []
    while solid_explorer.More():
        solids.append(solid_explorer.Current())
        solid_explorer.Next()
    if len(solids) != 1:
        raise ValueError("Expected the official single solid plus auxiliary surfaces")
    solid_faces = faces(solids[0])
    all_faces = faces(shape)
    if not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError("Official source is not a valid BRep; do not silently repair it")
    solid_shells: list[dict[str, object]] = []
    shell_explorer = TopExp_Explorer(solids[0], TopAbs_SHELL)
    while shell_explorer.More():
        shell = shell_explorer.Current()
        solid_shells.append({"closed": shell.Closed(),
                             "orientation": str(shell.Orientation()).split(".")[-1],
                             "valid": BRepCheck_Analyzer(shell).IsValid(),
                             "source_bounds_mm": bounds(shell)})
        shell_explorer.Next()
    BRepMesh_IncrementalMesh(shape, deflection_mm, False, math.radians(angle_deg), False)
    mesh_groups: dict[str, Mesh] = {}
    face_metadata: list[dict[str, object]] = []
    for face_index, face in enumerate(all_faces):
        is_solid = any(face.IsSame(f) for f in solid_faces)
        color = Quantity_Color()
        has_color = any(color_tool.GetColor(face, kind, color) for kind in (
            XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen))
        linear = tuple(round(v, 9) for v in (color.Red(), color.Green(), color.Blue())) if has_color else None
        srgb = tuple(round(v, 9) for v in color.Values(Quantity_TOC_sRGB)) if has_color else None
        name = "energy_auxiliary_surfaces" if not is_solid else (
            "energy_white_panels" if srgb == (1.0, 1.0, 1.0) else "energy_gray_body")
        role = "physical_solid_surface" if is_solid else "cad_auxiliary_surface"
        if is_solid and srgb not in ((1.0, 1.0, 1.0), (0.420000006, 0.420000006, 0.420000006)):
            raise ValueError(f"Unexpected solid face color: {srgb}")
        group = mesh_groups.setdefault(name, Mesh(name, role, linear, srgb))
        points, normals, triangles = triangulate_face(face)
        offset = len(group.vertices)
        group.vertices.extend(points.tolist())
        group.normals.extend(normals.tolist())
        group.triangles.extend((triangles + offset).tolist())
        group.face_indices.append(face_index)
        surface = BRepAdaptor_Surface(face)
        item: dict[str, object] = {
            "index": face_index, "mesh": name, "role": role,
            "surface_type": str(surface.GetType()).split(".")[-1],
            "source_bounds_mm": bounds(face), "triangles": len(triangles),
        }
        if surface.GetType() == GeomAbs_Cylinder:
            cylinder = surface.Cylinder()
            item.update(radius_mm=cylinder.Radius(),
                        axis_point_mm=list(cylinder.Axis().Location().Coord()),
                        axis_direction=list(cylinder.Axis().Direction().Coord()))
        elif surface.GetType() == GeomAbs_Plane:
            plane = surface.Plane()
            item.update(plane_point_mm=list(plane.Location().Coord()),
                        plane_normal=list(plane.Axis().Direction().Coord()))
        face_metadata.append(item)
    output.mkdir(parents=True, exist_ok=True)
    mesh_metadata: list[dict[str, object]] = []
    for mesh in mesh_groups.values():
        path = output / f"{mesh.name}.obj"
        write_obj(mesh, path)
        vertices = np.asarray(mesh.vertices)
        mesh_metadata.append({
            "name": mesh.name, "file": path.name, "role": mesh.role,
            "default_visible": mesh.role == "physical_solid_surface",
            "rgb_linear": mesh.rgb_linear, "rgb_srgb": mesh.rgb_srgb,
            "face_count": len(mesh.face_indices), "source_face_indices": mesh.face_indices,
            "vertices": len(mesh.vertices), "triangles": len(mesh.triangles),
            "bounds_m": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "bytes": path.stat().st_size,
        })
    volume = GProp_GProps()
    BRepGProp.VolumeProperties_s(solids[0], volume)
    centerline = [{"source_y_mm": y, "state": str(BRepClass3d_SolidClassifier(
        solids[0], gp_Pnt(0, y, 0), 1e-6).State()).split(".")[-1]}
        for y in (0.1, 4.9, 5.1, 9.9, 10.1, 100.0, 144.9, 145.1, 149.9)]
    manifest: dict[str, object] = {
        "schema_version": 1, "asset": "RoboMaster 2026 official energy unit",
        "source": {"url": SOURCE_URL, "post_url": POST_URL, "sha256": digest,
                   "bytes": source.stat().st_size, "file_units": "millimetres",
                   "product": "111000111", "cad_timestamp": "2025-10-21T19:58:35",
                   "generator": "CREO PARAMETRIC BY PTC INC, 2019164"},
        "tessellation": {"engine": "OpenCascade 7.8.1 / cadquery-ocp 7.8.1.1",
                         "linear_deflection_mm": deflection_mm, "angle_deg": angle_deg,
                         "relative": False, "normals": "analytic per CAD face, seam-preserving"},
        "frame": {"units": "metres", "origin": "centre of CAD Y=0 mouth plane",
                  "axis": "+X from mouth to closed far cap", "mapping": "[x,y,z] = [CAD_Y,-CAD_X,CAD_Z] / 1000",
                  "matrix_cad_mm_to_local_m": [[0, .001, 0, 0], [-.001, 0, 0, 0], [0, 0, .001, 0], [0, 0, 0, 1]],
                  "opening_center_m": [0, 0, 0], "far_cap_center_m": [.15, 0, 0],
                  "nominal_grip_axial_interval_m": [.035, .110]},
        "cad": {"assembly_roots": roots.Length(), "solid_count": len(solids),
                "brep_valid": True, "solid_shells": solid_shells,
                "faces": len(all_faces), "solid_faces": len(solid_faces),
                "auxiliary_faces": len(all_faces) - len(solid_faces),
                "source_bounds_mm": bounds(shape), "solid_volume_mm3": volume.Mass(),
                "surface_counts": dict(Counter(str(BRepAdaptor_Surface(f).GetType()).split(".")[-1] for f in all_faces))},
        "geometry_evidence": {"axial_length_mm": 150, "large_cap_cylinder_diameter_mm": 95,
                              "small_cap_cylinder_diameter_mm": 80,
                              "centerline_solid_classification": centerline,
                              "external_recess_floor_source_y_mm": 5,
                              "sealed_internal_cavity_source_y_range_mm": [10, 145],
                              "warning": "The published CAD closes the front recess at Y=5 mm. It does not supply an externally open 100 mm blind bore. No faces were removed or remodeled."},
        "auxiliary_policy": "All 33 non-solid source faces are retained in a separate OBJ. They are CAD SURFACE/COPY_GEOM layers, including duplicates, and are excluded from the default physical visualization.",
        "meshes": mesh_metadata, "faces": face_metadata,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("models/meshes/energy_unit"))
    parser.add_argument("--deflection-mm", type=float, default=.05)
    parser.add_argument("--angle-deg", type=float, default=8.0)
    args = parser.parse_args()
    manifest = import_energy(args.source, args.output, args.deflection_mm, args.angle_deg)
    print(json.dumps({"cad": manifest["cad"], "meshes": manifest["meshes"]}, indent=2))


if __name__ == "__main__":
    main()
