from pathlib import Path
import tempfile
import tkinter as tk
import unittest
from unittest.mock import Mock, patch

import numpy as np

from brazoia.calibration import (fingertip_gap_mm, fit_touches, gripper_offset, model_m6_for_gap,
                                 proposed_changes, touch_heights)
from brazoia.gripper_geometry import GripperGeometry


ROOT = Path(__file__).resolve().parents[1]
TRUE_OFFSETS = {2: 2.5, 3: -1.5, 4: 3.0}
TRUE_TABLE_MM = 6.0


def synthetic_touch(geometry, m1, m3, m4):
    """Lectura de telemetría con la garra apoyada en una mesa real conocida."""
    def height(m2):
        pose = {1: m1, 2: m2, 3: m3, 4: m4, 5: 263.0, 6: 0.0}
        return touch_heights(geometry, [pose], offsets=TRUE_OFFSETS, table_mm=TRUE_TABLE_MM)[0]

    low, high = 120.0, 270.0
    if height(low) * height(high) > 0:
        return None
    for _ in range(50):
        middle = (low + high) / 2
        if (height(middle) > 0) == (height(low) > 0):
            low = middle
        else:
            high = middle
    return {1: m1, 2: (low + high) / 2, 3: m3, 4: m4, 5: 263.0, 6: 0.0}


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = GripperGeometry(ROOT / "assets")
        configurations = [(150 + 5 * index, m3, m4) for index, (m3, m4) in enumerate(
            (m3, m4) for m3 in (130, 160, 190, 215, 235) for m4 in (230, 255, 275))]
        cls.touches = [pose for pose in (synthetic_touch(cls.geometry, *config) for config in configurations) if pose]

    def test_fit_recovers_table_and_zero_offsets(self):
        fit = fit_touches(self.geometry, self.touches)
        self.assertTrue(fit.identifiable)
        self.assertAlmostEqual(fit.table_mm, TRUE_TABLE_MM, delta=0.05)
        for motor, expected in TRUE_OFFSETS.items():
            self.assertAlmostEqual(fit.offsets[motor], expected, delta=0.02)
        self.assertLess(fit.rms_after_mm, 0.05)
        changes = proposed_changes(self.geometry, fit)
        self.assertAlmostEqual(changes["shoulder_height_mm"],
                               self.geometry.shoulder_height_mm - TRUE_TABLE_MM, delta=0.05)
        self.assertAlmostEqual(changes["visual_offset_degrees"][2],
                               self.geometry.calibration[2]["visual_offset_degrees"] + 2.5, delta=0.02)

    def test_similar_touches_do_not_estimate_every_zero(self):
        similar = [pose for pose in (synthetic_touch(self.geometry, 180, 160 + k, 255) for k in range(7)) if pose]
        fit = fit_touches(self.geometry, similar)
        self.assertFalse(fit.identifiable)
        self.assertLessEqual(len(fit.offsets), 1)
        self.assertEqual(set(proposed_changes(self.geometry, fit)["visual_offset_degrees"]), set(fit.offsets))

    def test_little_tilt_variety_estimates_table_and_m2_only(self):
        # Parecido a los toques reales del 30/10: garra casi vertical y lecturas con ruido.
        rng = np.random.default_rng(2)
        poses = []
        for m3, m4 in ((147, 231), (139, 255), (139, 245), (206, 227), (210, 243), (190, 244),
                       (239, 228), (246, 250), (241, 222), (238, 228), (236, 234), (217, 233)):
            def height(m2, m3=m3, m4=m4):
                pose = {1: 180, 2: m2, 3: m3, 4: m4, 5: 270, 6: 1}
                return touch_heights(self.geometry, [pose], offsets={2: 1.5}, table_mm=-20.0)[0]
            low, high = 100.0, 270.0
            if height(low) * height(high) > 0:
                continue
            for _ in range(50):
                middle = (low + high) / 2
                if (height(middle) > 0) == (height(low) > 0):
                    low = middle
                else:
                    high = middle
            poses.append({1: 180, 2: (low + high) / 2 + rng.normal(0, 0.3), 3: m3 + rng.normal(0, 0.3),
                          4: m4 + rng.normal(0, 0.3), 5: 270, 6: 1})
        fit = fit_touches(self.geometry, poses)
        self.assertEqual(set(fit.offsets), {2})
        self.assertEqual(set(fit.missing) | set(fit.unneeded), {3, 4})
        self.assertAlmostEqual(fit.offsets[2], 1.5, delta=0.5)
        self.assertAlmostEqual(fit.table_mm, -20.0, delta=2.0)

    def test_requires_enough_touches(self):
        with self.assertRaises(ValueError):
            fit_touches(self.geometry, self.touches[:3])

    def test_model_gripper_closes_below_zero_and_opens_with_m6(self):
        gaps = [fingertip_gap_mm(self.geometry, m6) for m6 in (0, 10, 20, 30)]
        self.assertEqual(gaps, sorted(gaps))
        closing = model_m6_for_gap(self.geometry)
        self.assertAlmostEqual(fingertip_gap_mm(self.geometry, closing), 0.0, delta=0.1)
        self.assertLess(closing, 0.0)  # La garra real cerrada deja ~7 mm entre las puntas.
        self.assertAlmostEqual(gripper_offset(self.geometry, 1.0), closing - 1.0, places=6)


class CalibrationWindowTests(unittest.TestCase):
    def setUp(self):
        serial_patch = patch("serial.Serial", side_effect=AssertionError("Prohibido abrir puerto"))
        self.serial = serial_patch.start()
        self.addCleanup(serial_patch.stop)
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = tk.Tk()
        self.root.withdraw()
        self.addCleanup(self.root.destroy)

    def test_captures_read_only_saves_and_fits(self):
        from calibracion_app import CalibrationApp, analyze

        geometry = GripperGeometry(ROOT / "assets")
        touches = [pose for pose in (synthetic_touch(geometry, 150 + 5 * k, m3, m4) for k, (m3, m4) in enumerate(
            ((130, 255), (160, 230), (190, 275), (215, 255), (235, 230), (160, 275), (190, 230), (215, 275))))
                   if pose]
        source = Mock(side_effect=touches + [{1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0, 6: 1.2}])
        ui = CalibrationApp(self.root, project_root=ROOT, real_pose_provider=source, data_dir=self.folder.name)
        for _ in touches:
            ui.capture_touch()
        ui.calculate()
        self.assertIn("Mesa real", ui.result.get())
        ui.capture_gripper()
        self.assertIn("M6 = 1.2°", ui.gripper_result.get())
        self.assertEqual(source.call_count, len(touches) + 1)
        report = analyze(ui.path, project_root=ROOT)
        self.assertIn(f"{len(touches)} toques", report)
        self.serial.assert_not_called()

    def test_without_provider_capture_explains_and_does_not_save(self):
        from calibracion_app import CalibrationApp

        ui = CalibrationApp(self.root, project_root=ROOT, data_dir=self.folder.name)
        ui.capture_touch()
        self.assertIn("aplicación principal", ui.status.get())
        self.assertFalse(ui.path.exists())


if __name__ == "__main__":
    unittest.main()
