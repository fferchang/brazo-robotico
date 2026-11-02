"""Reglas geométricas del brazo real: planificador 2D y firmware del ESP32."""
from pathlib import Path
import math
import re
import unittest

import numpy as np

from brazoia import planner
from brazoia.config import load_config
from brazoia.gripper_geometry import GripperGeometry


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "firmware_secuencias.ino"
# Agarre real bajo e inclinado (prueba_20261026_213214) y su traslado_1.
LOW_GRASP = {1: 200.7, 2: 207.2, 3: 168.3, 4: 244.7, 5: 289.9, 6: 44.2}
LIFT = {1: 200.8, 2: 170.0, 3: 168.4, 4: 244.7, 5: 290.0}


def random_poses(count, seed):
    rng = np.random.default_rng(seed)
    for _ in range(count):
        yield {1: 180.0, 2: rng.uniform(90, 270), 3: rng.uniform(90, 276), 4: rng.uniform(75, 278),
               5: rng.uniform(0, 340), 6: rng.uniform(0, 140)}


class PlanarModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = GripperGeometry(ROOT / "assets")

    def plane(self, transforms, link):
        # Plano del brazo con M1 = 180°: x hacia adelante (−y del URDF), y hacia arriba.
        point = transforms[link][:3, 3]
        origin = transforms["upper_arm"][:3, 3]
        return np.array([-(point[1] - origin[1]) * 1000, (point[2] - self.geometry.floor_z) * 1000])

    def test_planar_chain_matches_calibrated_3d_model(self):
        self.assertAlmostEqual(planner.H_BASE, self.geometry.shoulder_height_mm, places=6)
        for pose in random_poses(50, 1):
            transforms, _ = self.geometry.transforms(pose)
            points, _ = planner.compute_geometry((pose[2], pose[3], pose[4]))
            for index, link in ((2, "lower_arm"), (3, "wrist"), (4, "gripper")):
                np.testing.assert_allclose(points[index], self.plane(transforms, link), atol=0.05)

    def test_floor_envelope_never_above_real_gripper_for_any_roll_or_jaw(self):
        checked = 0
        for pose in random_poses(1500, 2):
            real = self.geometry.clearance(pose)["height_mm"]
            if real > 150:
                continue
            checked += 1
            self.assertLessEqual(planner.floor_clearance((pose[2], pose[3], pose[4]), pose[6]), real + 1e-6)
        self.assertGreater(checked, 500)

    def test_envelope_table_covers_the_gripper_meshes(self):
        fixed = np.vstack([v for _, name, v in self.geometry.meshes if not name.endswith("Moving_Jaw.stl")])
        moving = next(v for _, name, v in self.geometry.meshes if name.endswith("Moving_Jaw.stl"))
        for m6 in np.arange(0.0, 140.01, 0.5):
            transforms, _ = self.geometry.transforms({1: 180, 2: 180, 3: 90, 4: 180, 5: 263, 6: m6})
            relative = np.linalg.inv(transforms["gripper"]) @ transforms["jaw"]
            vertices = np.vstack([fixed, moving @ relative[:3, :3].T + relative[:3, 3]])
            along = -vertices[:, 1] * 1000
            radius = np.hypot(vertices[:, 0], vertices[:, 2]) * 1000
            covered = np.zeros(len(vertices), bool)
            for start, end, limit in planner.gripper_envelope(float(m6)):
                covered |= (along >= start) & (along <= end) & (radius <= limit)
            self.assertTrue(covered.all(), f"M6={m6:.1f}° tiene puntos fuera del contorno")

    def test_closed_gripper_vertical_grasp_is_allowed_and_table_contact_is_not(self):
        grasp = (196.0, 175.0, 255.0)
        height = self.geometry.clearance({1: 180, 2: grasp[0], 3: grasp[1], 4: grasp[2], 5: 263, 6: 137})["height_mm"]
        self.assertGreater(height, 15.0)
        self.assertEqual(planner.classify_pose(grasp, 137.0), (True, "OK"))
        lower = (grasp[0] + 12.0, grasp[1], grasp[2])
        self.assertGreater(-1, self.geometry.clearance(
            {1: 180, 2: lower[0], 3: lower[1], 4: lower[2], 5: 263, 6: 137})["height_mm"] - 0)
        self.assertEqual(planner.classify_pose(lower, 137.0), (False, "PISO"))

    def test_unknown_gripper_uses_the_most_conservative_envelope(self):
        pose = (200.0, 160.0, 250.0)
        known = [planner.floor_clearance(pose, m6) for m6 in (0, 30, 90, 137)]
        self.assertLessEqual(planner.floor_clearance(pose, None), min(known) + 1e-9)

    def test_escape_lets_the_arm_lift_after_a_low_grasp(self):
        grasp = tuple(LOW_GRASP[i] for i in (2, 3, 4))
        self.assertEqual(planner.classify_pose(grasp, LOW_GRASP[6]), (False, "PISO"))
        self.assertGreater(GripperGeometry(ROOT / "assets").clearance(LOW_GRASP)["height_mm"], 5.0)
        groups = planner.build_motion_plan(LOW_GRASP, LIFT, load_config().limits)
        self.assertTrue(groups)
        # El escape no permite acercarse más a la mesa.
        down = {**LIFT, 2: LOW_GRASP[2] + 4.0}
        with self.assertRaises(ValueError):
            planner.build_motion_plan(LOW_GRASP, down, load_config().limits)

    def test_m4_reaches_278_degrees(self):
        limits = load_config().limits
        self.assertEqual(limits[4], (75.0, 278.0))
        self.assertEqual(planner.PLANAR_LIMITS[4], (75.0, 278.0))
        self.assertEqual(planner.dynamic_m4_max(180.0, 180.0), 278.0)
        self.assertEqual(planner.classify_pose((180.0, 180.0, 278.5))[1], "LIMITE")


class FirmwareConsistencyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = FIRMWARE.read_text(encoding="utf-8")

    def constant(self, name):
        match = re.search(rf"static const float {name} = (-?[0-9.]+)f;", self.source)
        self.assertIsNotNone(match, name)
        return float(match.group(1))

    def envelope(self, name):
        match = re.search(rf"static const EnvelopeSegment {name}\[\] = \{{(.*?)\}};", self.source, re.S)
        self.assertIsNotNone(match, name)
        values = [float(v) for v in re.findall(r"-?[0-9.]+(?=f)", match.group(1))]
        return tuple(tuple(values[i:i + 3]) for i in range(0, len(values), 3))

    def test_geometry_constants_match_planner(self):
        for name, value in (("GEOM_H_BASE", planner.H_BASE), ("GEOM_W_BASE", planner.W_BASE),
                            ("GEOM_L1", planner.L1), ("GEOM_OFFSET_M3", planner.OFFSET_M3),
                            ("GEOM_L2", planner.L2), ("GEOM_L3", planner.L3),
                            ("GEOM_M2_ZERO", planner.M2_ZERO), ("GEOM_M3_ZERO", planner.M3_ZERO),
                            ("GEOM_M4_ZERO", planner.M4_ZERO), ("GEOM_GRIPPER_LEN", planner.GRIPPER_LEN),
                            ("GEOM_GRIPPER_WIDTH", planner.GRIPPER_WIDTH),
                            ("FLOOR_TOUCH_THRESHOLD", planner.FLOOR_TOUCH),
                            ("FLOOR_ESCAPE_TOLERANCE", planner.ESCAPE_TOLERANCE)):
            self.assertAlmostEqual(self.constant(name), value, places=4, msg=name)

    def test_gripper_envelopes_match_planner(self):
        names = ("ENVELOPE_CLOSED", "ENVELOPE_HOLDING", "ENVELOPE_HALF_OPEN", "ENVELOPE_OPEN")
        tops = ("ENVELOPE_CLOSED_MAX_M6", "ENVELOPE_HOLDING_MAX_M6", "ENVELOPE_HALF_OPEN_MAX_M6")
        for name, (top, segments) in zip(names, planner.GRIPPER_ENVELOPES):
            self.assertEqual(self.envelope(name), segments, name)
        for name, (top, _) in zip(tops, planner.GRIPPER_ENVELOPES):
            self.assertEqual(self.constant(name), top, name)
        self.assertEqual(self.envelope("ENVELOPE_UNKNOWN"), planner.UNKNOWN_GRIPPER_ENVELOPE)

    def test_m4_limits_match_config(self):
        self.assertIn("case 4:\n      minDeg = 75.0f;\n      maxDeg = 278.0f;", self.source)
        self.assertIn("float maxM4 = 278.0f;", self.source)
        self.assertTrue(math.isclose(load_config().limits[4][1], 278.0))


if __name__ == "__main__":
    unittest.main()
