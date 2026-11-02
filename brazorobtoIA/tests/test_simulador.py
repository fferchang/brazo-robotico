import tkinter as tk
import unittest
from unittest.mock import Mock, patch

from brazoia.config import INITIAL_POSE
from brazoia.gripper_geometry import joint_angle
from brazoia.rerun_view import RerunArmView
from simulador_app import ROOT, SimulatorApp


class SimulatorTests(unittest.TestCase):
    def setUp(self):
        self.serial_patch = patch("serial.Serial", side_effect=AssertionError("Prohibido conectar hardware"))
        self.serial = self.serial_patch.start()
        self.addCleanup(self.serial_patch.stop)
        self.view_patch = patch("simulador_app.RerunArmView")
        self.view_class = self.view_patch.start()
        self.addCleanup(self.view_patch.stop)
        self.view = self.view_class.return_value
        self.view.available = False
        root = tk.Tk()
        root.withdraw()
        self.ui = SimulatorApp(root, auto_open=False)
        self.addCleanup(self.ui.close)

    def test_all_six_controls_publish_only_virtual_angles(self):
        self.view.available = True
        targets = {1: 200, 2: 190, 3: 120, 4: 170, 5: 263, 6: 140}
        for motor, value in targets.items():
            self.ui.sliders[motor].set(value)
        self.ui.publish()
        self.assertEqual(self.ui.pose, targets)
        self.view.log_pose.assert_called_with(targets)
        self.assertIn("M6", self.ui.detail.get())
        self.assertFalse(self.view_class.call_args.kwargs["clamp_joint_limits"])
        self.serial.assert_not_called()

    def test_entry_decimal_comma_invalid_numbers_and_ranges(self):
        self.ui.entries[1].set("185,5")
        self.ui.apply_entry(1)
        self.assertEqual(self.ui.pose[1], 185.5)
        for value in ("nan", "inf", "texto", "999", ""):
            self.ui.entries[1].set(value)
            self.ui.apply_entry(1)
            self.assertEqual(self.ui.pose[1], 185.5)
            self.assertIn("inválido", self.ui.status.get())
        self.serial.assert_not_called()

    def test_reset_open_view_and_close_do_not_send_motor_commands(self):
        self.ui.set_angle(6, 140)
        self.ui.reset_pose()
        self.assertEqual(self.ui.pose, {**INITIAL_POSE, 6: 0})
        self.view.open.side_effect = lambda: setattr(self.view, "available", True)
        self.ui.open_view()
        self.view.open.assert_called_once()
        self.view.log_pose.assert_called_with(self.ui.pose)
        self.ui.close()
        self._cleanups.pop()  # Ya se cerró esta ventana.
        self.serial.assert_not_called()

    def test_floor_contact_is_visible_and_does_not_block_simulation(self):
        self.ui.geometry = Mock()
        self.ui.geometry.clearance.return_value = {"height_mm": -3.0, "clipped_motors": ()}
        self.view.available = True
        self.ui.publish()
        self.assertIn("CRUZA / TOCA", self.ui.height_text.get())
        self.view.log_pose.assert_called()


class SimulatorRerunTests(unittest.TestCase):
    def test_simulator_draws_requested_m6_angle_without_clamping(self):
        with patch("rerun.RecordingStream"):
            view = RerunArmView(ROOT / "assets", clamp_joint_limits=False)
            view.open()
            self.assertTrue(view.available, view.error)
            joint = Mock(limit_lower=-.2, limit_upper=2.0)
            view.joints["6"] = joint
            view.log_pose({**INITIAL_POSE, 6: 140})
            joint.compute_transform.assert_called_once_with(
                joint_angle(view.calibration[6], 6, 140), clamp=False)


if __name__ == "__main__":
    unittest.main()
