from types import SimpleNamespace
import time
import tkinter as tk
import unittest
from unittest.mock import Mock, patch

from brazoia.config import INITIAL_POSE, load_config
from brazoia.simulation_path import FloorPathPlanner
from trayecto_app import TrajectoryApp


POSE = {**INITIAL_POSE, 6: 0.0}


class TrajectoryAppTests(unittest.TestCase):
    def setUp(self):
        self.serial_patch = patch("serial.Serial", side_effect=AssertionError("Prohibido abrir puerto"))
        self.serial = self.serial_patch.start()
        self.addCleanup(self.serial_patch.stop)
        self.view_patch = patch("trayecto_app.RerunArmView")
        self.view_class = self.view_patch.start()
        self.addCleanup(self.view_patch.stop)
        self.view = self.view_class.return_value
        self.view.available = True
        self.root = tk.Tk()
        self.root.withdraw()
        self.source = Mock(return_value=POSE)
        self.ui = TrajectoryApp(self.root, real_pose_provider=self.source, auto_open=False)
        self.addCleanup(self.ui.close)

    def test_captures_read_only_and_worker_animates_without_hardware(self):
        self.ui.capture("start", "real")
        self.source.return_value = {**POSE, 1: 179}
        self.ui.capture("goal", "real")
        self.assertEqual(self.source.call_count, 2)
        self.ui.calculate()
        deadline = time.monotonic() + 5
        while self.ui.plan is None and time.monotonic() < deadline:
            self.root.update()
            time.sleep(.01)
        self.assertIsNotNone(self.ui.plan, self.ui.status.get())
        self.view.log_pose.assert_called()
        self.ui.seek("100")
        self.assertEqual(self.ui.last_pose, {**POSE, 1: 179})
        self.ui.close()
        self.serial.assert_not_called()

    def test_edited_pose_invalidates_plan_and_stale_worker_result(self):
        self.ui.fill_pose("start", POSE)
        self.ui.fill_pose("goal", POSE)
        plan = FloorPathPlanner(self.ui.geometry, self.ui.limits).plan(POSE, POSE)
        revision = self.ui.revision
        self.ui.plan = plan
        self.ui.fields["goal"][1].set("190")
        self.ui.results.put((revision, plan, None))
        self.assertIsNone(self.ui.plan)
        self.ui.poll()
        self.assertIsNone(self.ui.plan)

    def test_invalid_capture_does_not_replace_fields_or_send_commands(self):
        self.ui.fill_pose("start", POSE)
        self.source.side_effect = ValueError("Telemetría antigua")
        self.ui.capture("start", "real")
        self.assertEqual(self.ui.read_pose("start"), POSE)
        self.assertIn("antigua", self.ui.status.get())
        self.serial.assert_not_called()

    def test_main_application_adapter_only_reads_snapshot(self):
        from app import App
        robot = Mock(connected=True)
        robot.send.side_effect = AssertionError("Prohibido enviar órdenes")
        robot.snapshot.return_value = SimpleNamespace(positions=POSE, received_at={i: time.monotonic() for i in POSE})
        holder = SimpleNamespace(robot=robot, config=load_config())
        self.assertEqual(App.simulation_snapshot_pose(holder), POSE)
        robot.send.assert_not_called()
        robot.connect.assert_not_called()
        self.serial.assert_not_called()


if __name__ == "__main__":
    unittest.main()
