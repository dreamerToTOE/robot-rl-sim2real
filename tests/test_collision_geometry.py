"""Synthetic geometry only: vendor assets and Isaac Sim are not required."""

import itertools
from pathlib import Path
import sys
import struct
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
try:
    import numpy as np
    from scipy.spatial import ConvexHull
except ModuleNotFoundError as error:
    if error.name not in {"numpy", "scipy"}:
        raise
    missing_dependency = error.name
else:
    missing_dependency = None
    from check_model_collisions import CollisionModel, hull_planes, intersection_inradius


@unittest.skipIf(missing_dependency, f"Optional collision dependency unavailable: {missing_dependency}")
class TestCollisionGeometry(unittest.TestCase):
    def setUp(self):
        self.cube = np.array(list(itertools.product((-1.0, 1.0), repeat=3)))
        self.planes = hull_planes(self.cube)

    def test_separated_cubes(self):
        other = hull_planes(self.cube + (3, 0, 0))
        self.assertAlmostEqual(intersection_inradius(self.planes, other), -0.5)

    def test_contact_and_overlap(self):
        touching = hull_planes(self.cube + (2, 0, 0))
        overlap = hull_planes(self.cube + (1, 0, 0))
        self.assertAlmostEqual(intersection_inradius(self.planes, touching), 0)
        self.assertAlmostEqual(intersection_inradius(self.planes, overlap), 0.5)

    def test_rigid_transform_invariance(self):
        angle = 0.37
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        a = hull_planes(self.cube @ rotation.T + (0.4, 1.2, -0.7))
        b = hull_planes((self.cube + (3, 0, 0)) @ rotation.T + (0.4, 1.2, -0.7))
        self.assertAlmostEqual(intersection_inradius(a, b), -0.5)

    def test_invalid_input_fails_closed(self):
        with self.assertRaises(ValueError):
            hull_planes([[float("nan"), 0, 0]])
        with self.assertRaises(ValueError):
            intersection_inradius(self.planes * 2, self.planes)

    def test_local_stl_loading_aabb_certificate_and_lp_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "urdf").mkdir()
            (root / "meshes").mkdir()
            faces = ConvexHull(self.cube).simplices
            raw = b"synthetic cube".ljust(80, b"\0") + struct.pack("<I", len(faces))
            for face in faces:
                raw += struct.pack("<12fH", 0, 0, 0, *self.cube[face].flatten(), 0)
            (root / "meshes/cube.stl").write_bytes(raw)
            collision = '<collision><geometry><mesh filename="../meshes/cube.stl"/></geometry></collision>'
            template = ('<robot name="synthetic"><link name="a">' + collision + '</link>'
                        '<link name="b">' + collision + '</link>'
                        '<joint name="fixed" type="fixed"><parent link="a"/><child link="b"/>'
                        '<origin xyz="{} 0 0"/></joint></robot>')
            source = root / "urdf/model.urdf"
            source.write_text(template.format(3))
            model = CollisionModel(source)
            fast = model.check({}, include_adjacent=True, aabb_fast=True)[0]
            self.assertTrue(fast["separated"])
            self.assertEqual(fast["aabb_distance_lower_bound_m"], 1.0)
            self.assertIsNone(fast["intersection_inradius_m"])
            source.write_text(template.format(1))
            overlap = CollisionModel(source).check({}, include_adjacent=True, aabb_fast=True)[0]
            self.assertFalse(overlap["separated"])
            self.assertAlmostEqual(overlap["intersection_inradius_m"], 0.5)


if __name__ == "__main__":
    unittest.main()
