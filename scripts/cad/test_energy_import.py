"""CAD-dependent offline tests. Run with the CAD tool environment, not C++ runtime."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from import_energy import ROTATION, import_energy
from correct_energy_bore import correct_energy_bore, load_obj


class OfficialEnergyImportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = Path(__file__).resolve().parents[2] / "build/fidelity-research/official/energy_unit.stp"
        if not cls.source.is_file():
            raise unittest.SkipTest("Download the checksum-pinned official STEP first")
        cls.temporary = tempfile.TemporaryDirectory(prefix="rm2026_energy_test_")
        cls.output = Path(cls.temporary.name)
        import_energy(cls.source, cls.output)
        cls.manifest = json.loads((cls.output / "manifest.json").read_text())

    @classmethod
    def tearDownClass(cls) -> None:
        if hasattr(cls, "temporary"):
            cls.temporary.cleanup()

    def test_complete_source_face_partition_and_cad_validity(self) -> None:
        cad = self.manifest["cad"]
        self.assertEqual((cad["faces"], cad["solid_faces"], cad["auxiliary_faces"]), (291, 258, 33))
        self.assertTrue(cad["brep_valid"])
        self.assertEqual([s["orientation"] for s in cad["solid_shells"]],
                         ["TopAbs_FORWARD", "TopAbs_REVERSED"])
        self.assertTrue(all(s["closed"] and s["valid"] for s in cad["solid_shells"]))
        indices = [i for mesh in self.manifest["meshes"] for i in mesh["source_face_indices"]]
        self.assertEqual(sorted(indices), list(range(291)))
        self.assertTrue(all(face["triangles"] > 0 for face in self.manifest["faces"]))

    def test_scale_axis_and_source_appearance(self) -> None:
        self.assertAlmostEqual(float(np.linalg.det(ROTATION)), 1.0)
        np.testing.assert_allclose(ROTATION @ [0, 150, 0] * .001, [.15, 0, 0])
        meshes = self.manifest["meshes"]
        gray = next(m for m in meshes if m["name"] == "energy_gray_body")
        white = next(m for m in meshes if m["name"] == "energy_white_panels")
        np.testing.assert_allclose(gray["rgb_srgb"], [.42] * 3, atol=1e-7)
        self.assertEqual(white["rgb_srgb"], [1, 1, 1])
        self.assertAlmostEqual(gray["bounds_m"][1][0] - gray["bounds_m"][0][0], .15, places=8)
        self.assertAlmostEqual(gray["bounds_m"][1][1] - gray["bounds_m"][0][1], .095, places=6)

    def test_normals_follow_actual_source_surface_winding(self) -> None:
        for mesh in self.manifest["meshes"]:
            vertices, normals, triangles = [], [], []
            for line in (self.output / mesh["file"]).read_text().splitlines():
                values = line.split()
                if not values:
                    continue
                if values[0] == "v":
                    vertices.append([float(v) for v in values[1:]])
                elif values[0] == "vn":
                    normals.append([float(v) for v in values[1:]])
                elif values[0] == "f":
                    triangles.append([int(v.split("/")[0]) - 1 for v in values[1:]])
            points = np.array(vertices)
            directions = np.array(normals)
            faces = np.array(triangles)
            np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1, atol=2e-8)
            cross = np.cross(points[faces[:, 1]] - points[faces[:, 0]],
                             points[faces[:, 2]] - points[faces[:, 0]])
            area = np.linalg.norm(cross, axis=1)
            good = area > 1e-14
            dot = np.einsum("ij,ij->i", cross[good] / area[good, None], directions[faces[good]].mean(axis=1))
            self.assertGreater(float(dot.min()), 0, mesh["name"])

    def test_closed_recess_is_disclosed_not_falsely_claimed_as_bore(self) -> None:
        samples = self.manifest["geometry_evidence"]["centerline_solid_classification"]
        states = {p["source_y_mm"]: p["state"] for p in samples}
        self.assertEqual(states[4.9], "TopAbs_OUT")
        self.assertEqual(states[5.1], "TopAbs_IN")
        self.assertEqual(states[10.1], "TopAbs_OUT")
        self.assertEqual(states[145.1], "TopAbs_IN")

    def test_rule_bore_preserves_exterior_and_opens_depth(self) -> None:
        rule_output = self.output / "rule"
        result = correct_energy_bore(self.source, self.output, rule_output)
        self.assertEqual(result["changes"]["source_face_indices_removed"], [187, 188, 189, 254])
        self.assertTrue(result["verification"]["exterior_bounds_bitwise_unchanged"])
        self.assertTrue(result["geometry"]["bore_and_stop_are_visual_interfaces"])
        for ray in result["verification"]["axial_first_hit_rays"]:
            self.assertAlmostEqual(ray["first_hit_x_m"], .1, places=8)
        liner_points, _ = load_obj(rule_output / "energy_bore_liner.obj")
        np.testing.assert_allclose(np.linalg.norm(liner_points[:, 1:], axis=1), .020, atol=1e-10)
        source_gray = (self.output / "energy_gray_body.obj").read_text().splitlines()
        revised_gray = (rule_output / "energy_gray_body_rule.obj").read_text().splitlines()
        self.assertEqual([line for line in source_gray if not line.startswith("f ")],
                         [line for line in revised_gray if not line.startswith("f ")])
        source_faces = {line for line in source_gray if line.startswith("f ")}
        self.assertTrue(all(line in source_faces for line in revised_gray if line.startswith("f ")))
        self.assertEqual((self.output / "energy_white_panels.obj").read_bytes(),
                         (rule_output / "energy_white_panels.obj").read_bytes())

    def test_rule_interface_normals_and_winding(self) -> None:
        rule_output = self.output / "normal_rule"
        result = correct_energy_bore(self.source, self.output, rule_output)
        for mesh in result["meshes"]:
            if mesh["name"] not in ("energy_bore_internal_repairs", "energy_bore_liner", "energy_bore_stop"):
                continue
            points, triangles = load_obj(rule_output / mesh["file"])
            normals = np.asarray([[float(v) for v in line.split()[1:]]
                                  for line in (rule_output / mesh["file"]).read_text().splitlines()
                                  if line.startswith("vn ")])
            cross = np.cross(points[triangles[:, 1]] - points[triangles[:, 0]],
                             points[triangles[:, 2]] - points[triangles[:, 0]])
            area = np.linalg.norm(cross, axis=1)
            good = area > 1e-14
            dot = np.einsum("ij,ij->i", cross[good] / area[good, None], normals[triangles[good]].mean(axis=1))
            self.assertGreater(float(dot.min()), 0, mesh["name"])


if __name__ == "__main__":
    unittest.main()
