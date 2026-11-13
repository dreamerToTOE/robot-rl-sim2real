"""Synthetic models only: no vendor model / SDK / simulator needed."""

import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from compare_model_fk import compare, error, parse_sdf, parse_urdf, pose


URDF = """<robot name="synthetic">
  <link name="base"/><link name="arm"/><link name="forearm"/><link name="tip"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="0 0 1"/>
    <limit lower="-2" upper="2" effort="0" velocity="0"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="arm"/><child link="forearm"/>
    <origin xyz="0.5 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="0" velocity="0"/>
  </joint>
  <joint name="tool" type="fixed">
    <parent link="forearm"/><child link="tip"/><origin xyz="0.3 0 0"/>
  </joint>
</robot>"""

SDF = """<sdf version="1.11"><model name="synthetic">
  <link name="base"/>
  <link name="arm"><pose relative_to="shoulder"/></link>
  <link name="forearm"><pose relative_to="elbow"/></link>
  <link name="tip"><pose relative_to="tool"/></link>
  <frame name="pivot" attached_to="arm"><pose>0.5 0 0 0 0 0</pose></frame>
  <joint name="shoulder" type="revolute">
    <parent>base</parent><child>arm</child>
    <pose relative_to="base">1 0 0 0 0 1.5707963267948966</pose>
    <axis><xyz>0 0 -1</xyz><limit><lower>-2</lower><upper>2</upper></limit></axis>
  </joint>
  <joint name="elbow" type="revolute">
    <parent>arm</parent><child>forearm</child><pose relative_to="pivot"/>
    <axis><xyz expressed_in="__model__">-1 0 0</xyz><limit><lower>-2</lower><upper>2</upper></limit></axis>
  </joint>
  <joint name="tool" type="fixed">
    <parent>forearm</parent><child>tip</child><pose relative_to="forearm">0.3 0 0 0 0 0</pose>
  </joint>
</model></sdf>"""


class TestModelFK(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def parse(self, content, parser):
        path = Path(self.directory.name) / "model.xml"
        path.write_text(content, encoding="utf-8")
        return parser(path)

    def models(self, sdf=SDF):
        return self.parse(URDF, parse_urdf), self.parse(sdf, parse_sdf)

    def test_analytic_two_axis_fk_and_fixed_tool(self):
        u, s = self.models()
        uq = {"shoulder": math.pi/2, "elbow": math.pi/2}
        sq = {"shoulder": -math.pi/2, "elbow": math.pi/2}
        expected = pose((0.5, 0, -0.3), (0, math.pi/2, math.pi))
        for model, q in ((u, uq), (s, sq)):
            e = error(model.fk(q)["tip"], expected)
            self.assertLess(e["position_m"], 1e-14)
            self.assertLess(e["orientation_rad"], 1e-14)

    def test_sign_search_relative_pose_and_axis_frame(self):
        u, s = self.models()
        result = compare(u, s, "tip", count=8)
        self.assertEqual(result["sign_mapping_sdf_from_urdf"], {"shoulder": -1, "elbow": 1})
        self.assertEqual(result["configuration_count"], 19)
        self.assertEqual(result["source_boundary_count"], 8)
        self.assertEqual(result["sign_candidates_tested"], 4)
        self.assertTrue(result["registered_tip_within_numeric_tolerance"])
        self.assertLess(result["families"]["random"]["mapped_same_base"]["max_position_m"], 1e-14)
        for row in result["direction_comparison"]:
            self.assertGreater(row["min_angular_cosine"], 0.999999999)

    def test_source_limits_are_checked_outside_shared_domain(self):
        u, s = self.models(SDF.replace('<lower>-2</lower>', '<lower>-1</lower>').replace('<upper>2</upper>', '<upper>1</upper>'))
        result = compare(u, s, "tip", count=8)
        urdf_cases = [row for row in result["source_boundary_samples"] if row["source"] == "urdf"]
        sdf_cases = [row for row in result["source_boundary_samples"] if row["source"] == "sdf"]
        self.assertTrue(all(row["sdf_limit_violations"] for row in urdf_cases))
        self.assertFalse(any(row["urdf_limit_violations"] for row in sdf_cases))

    def test_tip_registration_does_not_imply_whole_model_agreement(self):
        u, s = self.models(SDF.replace("0.5 0 0 0 0 0", "0.5 0 0.0007 0 0 0"))
        result = compare(u, s, "tip", count=8)
        self.assertTrue(result["registered_tip_within_numeric_tolerance"])
        self.assertAlmostEqual(result["families"]["random"]["mapped_same_base"]["max_position_m"], 0.0007)
        self.assertAlmostEqual(result["all_link_comparison"]["base"]["registered_max_m"], 0.0007)
        self.assertAlmostEqual(result["all_link_comparison"]["arm"]["registered_max_m"], 0.0007)

    def test_geometry_change_cannot_be_hidden_by_zero_pose_registration(self):
        u, s = self.models(SDF.replace("0.5 0 0 0 0 0", "0.6 0 0 0 0 0"))
        result = compare(u, s, "tip", count=8)
        self.assertFalse(result["registered_tip_within_numeric_tolerance"])
        self.assertGreater(result["families"]["random"]["mapped_registered"]["max_position_m"], 0.01)

    def test_sdf_default_joint_pose_is_in_child_frame(self):
        sdf = """<sdf version="1.7"><model name="default_pose">
          <link name="base"/><link name="child"><pose>1 0 0 0 0 0</pose></link>
          <joint name="j" type="revolute"><parent>base</parent><child>child</child>
          <axis><xyz>0 0 1</xyz><limit><lower>-2</lower><upper>2</upper></limit></axis>
          </joint></model></sdf>"""
        result = self.parse(sdf, parse_sdf).fk({"j": math.pi/2})["child"]
        e = error(result, pose((1, 0, 0), (0, 0, math.pi/2)))
        self.assertLess(e["position_m"], 1e-14)
        self.assertLess(e["orientation_rad"], 1e-14)

    def test_euler_representations_compared_as_rotations(self):
        first = pose(rpy=(0.1, 0.2, 0.3))
        second = pose(rpy=(0.1+math.pi, math.pi-0.2, 0.3+math.pi))
        self.assertLess(error(first, second)["orientation_rad"], 1e-14)
        self.assertAlmostEqual(error(pose(), pose(rpy=(math.pi, 0, 0)))["orientation_rad"], math.pi)

    def test_unsupported_or_invalid_kinematics_fail_closed(self):
        cases = [
            SDF.replace('<model name="synthetic">', '<model name="synthetic"><model name="nested"/>'),
            SDF.replace('relative_to="pivot"', 'relative_to="other::pivot"'),
            SDF.replace('attached_to="arm"', 'attached_to="pivot"'),
            SDF.replace('<pose relative_to="shoulder"/>', '<pose relative_to="arm"/>'),
            SDF.replace('<pose relative_to="pivot"/>', '<pose relative_to="pivot" rotation_format="quat_xyzw">0 0 0 0 0 0 1</pose>'),
            SDF.replace('<pose relative_to="pivot"/>', '<pose relative_to="pivot" degrees="true"/>'),
            SDF.replace('<xyz>0 0 -1</xyz>', '<xyz>0 0 -1</xyz><use_parent_model_frame>true</use_parent_model_frame>'),
            SDF.replace('<xyz>0 0 -1</xyz>', '<xyz>0 0 -1</xyz><mimic joint="elbow"/>'),
            SDF.replace('<xyz>0 0 -1</xyz>', '<xyz>0 0 0</xyz>'),
            SDF.replace('<lower>-2</lower>', '<lower>3</lower>'),
            SDF.replace('version="1.11"', 'version="1.6"'),
            SDF.replace('name="pivot"', 'name="arm"'),
        ]
        for content in cases:
            with self.subTest(content=content), self.assertRaises(ValueError):
                self.parse(content, parse_sdf)
        with self.assertRaises(ValueError):
            self.parse(URDF.replace('type="revolute"', 'type="continuous"'), parse_urdf)
        with self.assertRaises(ValueError):
            self.parse(URDF.replace('<axis xyz="0 0 1"/>', '<axis xyz="0 0 1"/><mimic joint="elbow"/>'), parse_urdf)


if __name__ == "__main__":
    unittest.main()
