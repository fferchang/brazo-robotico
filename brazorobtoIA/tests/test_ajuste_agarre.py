"""Corrimiento del punto de agarre (config.json «agarre»), sin recalibrar las cámaras."""
import json
import math
import tempfile
import unittest
from pathlib import Path

from brazoia.config import load_config, load_grasp_offset
from brazoia.politica_geometrica import GeometricPolicy

ROOT = Path(__file__).resolve().parents[1]


def policy(offset):
    instance = GeometricPolicy.__new__(GeometricPolicy)
    instance.grasp_offset_mm = offset
    return instance


class GraspPointTests(unittest.TestCase):
    def test_radial_moves_away_from_the_base(self):
        x, y = policy((10.0, 0.0)).grasp_point((0.0, -0.200))
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, -0.210)
        x, y = policy((10.0, 0.0)).grasp_point((0.120, -0.160))
        self.assertAlmostEqual(math.hypot(x, y), 0.210)
        self.assertAlmostEqual(math.atan2(y, x), math.atan2(-0.160, 0.120))

    def test_lateral_is_to_the_left_looking_out_from_the_base(self):
        # Mirando desde la base hacia -y, la izquierda es +x.
        x, y = policy((0.0, 5.0)).grasp_point((0.0, -0.200))
        self.assertAlmostEqual(x, 0.005)
        self.assertAlmostEqual(y, -0.200)

    def test_zero_leaves_the_camera_estimate(self):
        self.assertEqual(policy((0.0, 0.0)).grasp_point((0.1, -0.2)), (0.1, -0.2))


class GraspConfigTests(unittest.TestCase):
    def test_loaded_from_config_json(self):
        raw = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
        self.assertEqual(load_config().grasp_offset_mm, load_grasp_offset(raw.get("agarre", {})))

    def test_missing_section_is_zero_and_range_is_checked(self):
        self.assertEqual(load_grasp_offset({}), (0.0, 0.0))
        with self.assertRaises(ValueError):
            load_grasp_offset({"ajuste_radial_mm": 45})

    def test_config_without_section_still_loads(self):
        raw = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
        raw.pop("agarre", None)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            self.assertEqual(load_config(path).grasp_offset_mm, (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
