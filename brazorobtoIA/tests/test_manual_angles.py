import queue
import time
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

from app import App, manual_motor_proposal
from brazoia.config import INITIAL_POSE, load_config
from brazoia.execution import MotionExecutor
from brazoia.robot import RobotSnapshot


class Value:
    def __init__(self, value=""):
        self.value = value
    def get(self):
        return self.value
    def set(self, value):
        self.value = value


class ManualAnglesTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.positions = {**INITIAL_POSE, 6: 20.0}
        self.robot = Mock(connected=True)
        self.robot.snapshot.side_effect = lambda: RobotSnapshot(
            dict(self.positions), {i: time.monotonic() for i in range(1, 7)}, True, "idle", "")
        self.ui = SimpleNamespace(robot=self.robot, config=self.config, execution=None,
                                  execution_busy=False, manual_executor=None, status=Value(),
                                  manual_angle_vars={i: Value("190") for i in range(1, 7)},
                                  worker_messages=queue.Queue(), run_async=Mock())

    def proposal(self, motor=1, target=190, snapshot=None):
        return manual_motor_proposal(snapshot or self.robot.snapshot(), motor, target,
                                     self.config.limits, 0.5, time.monotonic())

    def test_only_selected_motor_is_commanded_and_accepts_decimal_comma(self):
        proposal = self.proposal(target="190,5")
        self.assertEqual(proposal.target, {1: 190.5})
        self.assertEqual(proposal.groups[0].targets, ((1, 190.5),))
        self.assertIsNone(proposal.gripper_action)

    def test_invalid_or_out_of_range_angles(self):
        for target in ("", "abc", "nan", "inf", "-inf", "59", "301"):
            with self.subTest(target=target), self.assertRaises(ValueError):
                self.proposal(target=target)

    def test_torque_fault_gripper_and_stale_telemetry(self):
        snapshot = self.robot.snapshot()
        for change in (dict(torque_enabled=False), dict(torque_enabled=None), dict(fault="error"),
                       dict(gripper_state="closing"), dict(received_at={}), dict(positions={})):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.proposal(snapshot=replace(snapshot, **change))

    def test_planar_unsafe_path_rejected_without_other_motor_compensation(self):
        with patch("app.edge_safe", return_value=False) as check:
            with self.assertRaisesRegex(ValueError, "geometría"):
                self.proposal(motor=2, target=190)
            check.assert_called_once()

    def test_gripper_limits_and_single_motor(self):
        self.assertEqual(self.proposal(6, 130).groups[0].targets, ((6, 130.0),))
        with self.assertRaises(ValueError):
            self.proposal(6, 141)

    def test_disconnected_or_busy_never_requests_execution(self):
        for change in (dict(robot=None), dict(execution_busy=True), dict(execution=object())):
            ui = SimpleNamespace(**{**vars(self.ui), **change})
            App.move_manual_motor(ui, 1)
        self.ui.run_async.assert_not_called()
        self.robot.send.assert_not_called()

    def test_cancel_does_not_send(self):
        with patch("app.messagebox.askyesno", return_value=False):
            App.move_manual_motor(self.ui, 1)
        self.ui.run_async.assert_not_called()
        self.robot.send.assert_not_called()

    def test_execution_sends_only_selected_motor_and_confirms_arrival(self):
        def send(command):
            self.assertEqual(command, "1 190.00")
            self.positions[1] = 190
        self.robot.send.side_effect = send
        executor = MotionExecutor(self.robot)
        executor.execute(self.proposal())
        self.robot.send.assert_called_once_with("1 190.00")

    def test_after_confirmation_revalidates_before_execution_and_cleans_state(self):
        with patch("app.messagebox.askyesno", return_value=True):
            App.move_manual_motor(self.ui, 1)
        self.assertTrue(self.ui.execution_busy)
        args, kwargs = self.ui.run_async.call_args
        self.robot.snapshot.side_effect = lambda: RobotSnapshot({}, {}, True, "idle", "")
        with self.assertRaises(ValueError) as raised:
            args[0]()
        kwargs["failure"](raised.exception)
        self.assertFalse(self.ui.execution_busy)
        self.assertIsNone(self.ui.manual_executor)
        self.robot.send.assert_not_called()

    def test_copy_angles_does_not_send(self):
        App.copy_manual_angles(self.ui)
        self.assertEqual(self.ui.manual_angle_vars[5].get(), "263.0")
        self.robot.send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
