"""Diagnóstico del enlace botón -> serie, sin abrir COM ni accionar motores.

Las pruebas de estado documentan el comportamiento actual, incluso sus
limitaciones. Que pasen no certifica la protección eléctrica del firmware.
"""
from __future__ import annotations

import ast
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from app import App
from brazoia.robot import RobotSerial


class GripperButtonDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.robot = RobotSerial("SIMULATED_NO_PORT", 115200)
        self.wire = Mock(is_open=True)
        self.robot.connection = self.wire
        self.app = SimpleNamespace(
            robot=self.robot, execution_busy=False, status=Mock()
        )
        self.app.send_robot = lambda command: App.send_robot(self.app, command)
        source = Path(__file__).resolve().parents[1] / "app.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        callbacks = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            kwargs = {kw.arg: kw.value for kw in node.keywords}
            label = kwargs.get("text")
            if isinstance(label, ast.Constant) and label.value == "Cerrar garra por presión":
                callbacks.append(kwargs["command"])
        self.assertEqual(len(callbacks), 1)
        self.press = eval(
            compile(ast.Expression(callbacks[0]), str(source), "eval"),
            {"self": self.app},
        )

    def test_one_press_sends_exactly_one_close_line(self):
        self.press()
        self.wire.write.assert_called_once_with(b"gripper close\n")
        self.wire.flush.assert_called_once_with()
        event = self.robot.events.get_nowait()
        self.assertEqual((event.kind, event.payload), ("tx", "gripper close"))

    def test_no_position_or_global_torque_command_is_added(self):
        self.press()
        commands = [call.args[0] for call in self.wire.write.call_args_list]
        self.assertEqual(commands, [b"gripper close\n"])

    def test_motion_in_progress_blocks_the_manual_button(self):
        self.app.execution_busy = True
        self.press()
        self.wire.write.assert_not_called()
        self.app.status.set.assert_called_once()

    def test_disconnected_port_reports_error_without_sending(self):
        self.wire.is_open = False
        self.press()
        self.wire.write.assert_not_called()
        self.assertIn("no está conectado", self.app.status.set.call_args.args[0])

    def test_received_start_and_contact_are_recognized(self):
        self.robot._consume_line("GRIPPER_CLOSE_START from=140 contact_raw=92", time.monotonic())
        self.assertEqual(self.robot.snapshot().gripper_state, "closing")
        self.robot._consume_line("GRIPPER_HOLD reason=contact current_raw=95", time.monotonic())
        self.assertEqual(self.robot.snapshot().gripper_state, "holding")

    def test_repeated_presses_are_currently_sent_again(self):
        self.press()
        self.robot._consume_line("GRIPPER_CLOSE_START from=140 contact_raw=92", time.monotonic())
        self.press()
        self.assertEqual(self.wire.write.call_count, 2)

    def test_read_failure_currently_leaves_display_state_closing(self):
        self.robot._consume_line("GRIPPER_CLOSE_START from=140 contact_raw=92", time.monotonic())
        self.robot._consume_line("GRIPPER_STOP reason=communication_error detail=ReadCurrent_sin_respuesta", time.monotonic())
        self.assertEqual(self.robot.snapshot().gripper_state, "closing")
        self.assertIn("communication_error", self.robot.snapshot().fault)

    def test_hold_label_currently_does_not_distinguish_contact_from_closed_limit(self):
        for reason in ("contact", "closed_limit", "manual", "timeout"):
            with self.subTest(reason=reason):
                robot = RobotSerial("SIMULATED_NO_PORT", 115200)
                robot._consume_line(f"GRIPPER_HOLD reason={reason}", time.monotonic())
                self.assertEqual(robot.snapshot().gripper_state, "holding")


if __name__ == "__main__":
    unittest.main()
