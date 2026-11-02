"""Diagnóstico del enlace botón -> serie, sin abrir COM ni accionar motores.

La lógica de corriente del firmware se prueba en tests/firmware_host. Que estas
pruebas pasen no certifica la protección eléctrica real de la garra.
"""
from __future__ import annotations

import ast
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
import tempfile
from unittest.mock import Mock

from app import App
from brazoia.execution import MotionExecutor
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

    def test_repeated_press_is_sent_and_ignored_by_firmware(self):
        # La aplicación vuelve a enviar la orden; el firmware responde
        # GRIPPER_CLOSE_IGNORED sin reiniciar el cierre ni marcar un fallo.
        self.press()
        self.robot._consume_line("GRIPPER_CLOSE_START from=140 contact_raw=92", time.monotonic())
        self.press()
        self.robot._consume_line("GRIPPER_CLOSE_IGNORED reason=already_closing", time.monotonic())
        self.assertEqual(self.wire.write.call_count, 2)
        self.assertEqual(self.robot.snapshot().gripper_state, "closing")
        self.assertEqual(self.robot.snapshot().fault, "")

    def test_read_failure_marks_gripper_error(self):
        self.robot._consume_line("GRIPPER_CLOSE_START from=140 contact_raw=92", time.monotonic())
        self.robot._consume_line("GRIPPER_READ_FAIL detail=ReadCurrent_sin_respuesta count=1", time.monotonic())
        self.assertEqual(self.robot.snapshot().gripper_state, "closing")
        self.assertEqual(self.robot.snapshot().fault, "")
        self.robot._consume_line("GRIPPER_STOP reason=communication_error detail=ReadCurrent_sin_respuesta", time.monotonic())
        self.assertEqual(self.robot.snapshot().gripper_state, "error")
        self.assertIn("communication_error", self.robot.snapshot().fault)
        self.robot._consume_line("GRIPPER_TORQUE_OFF reason=communication_error current_raw=-1 result=1 attempts=1", time.monotonic())
        self.assertEqual(self.robot.snapshot().gripper_state, "released")

    def test_hold_reason_distinguishes_contact_from_empty_close(self):
        expected = {"contact": "holding", "closed_limit": "closed", "manual": "stopped", "timeout": "stopped"}
        for reason, state in expected.items():
            with self.subTest(reason=reason):
                robot = RobotSerial("SIMULATED_NO_PORT", 115200)
                robot._consume_line(f"GRIPPER_HOLD reason={reason}", time.monotonic())
                snapshot = robot.snapshot()
                self.assertEqual(snapshot.gripper_state, state)
                self.assertEqual(snapshot.gripper_reason, reason)

    def test_unconfirmed_torque_off_is_a_fault(self):
        self.robot._consume_line("GRIPPER_TORQUE_OFF reason=hard_current current_raw=250 result=0 attempts=3", time.monotonic())
        self.robot._consume_line("GRIPPER_FAULT reason=torque_off_sin_confirmar source=hard_current", time.monotonic())
        self.assertEqual(self.robot.snapshot().gripper_state, "error")
        self.assertIn("GRIPPER_FAULT", self.robot.snapshot().fault)

    def test_current_telemetry_is_parsed(self):
        self.robot._consume_line("GRIPPER_I raw=87 mode=position", time.monotonic())
        self.assertEqual(self.robot.snapshot().gripper_current_raw, 87)

    def test_global_torque_off_releases_gripper(self):
        self.robot._consume_line("GRIPPER_HOLD reason=contact", time.monotonic())
        self.robot._consume_line("TORQUE_OFF_OK", time.monotonic())
        self.assertEqual(self.robot.snapshot().gripper_state, "released")
        robot = RobotSerial("SIMULATED_NO_PORT", 115200)
        robot._consume_line("GRIPPER_HOLD reason=contact", time.monotonic())
        robot._consume_line("ARM_TORQUE_OFF_OK", time.monotonic())
        self.assertEqual(robot.snapshot().gripper_state, "holding")


class GripperConfirmationTests(unittest.TestCase):
    """El ejecutor solo acepta confirmaciones de la garra posteriores a su orden."""

    def setUp(self):
        self.robot = RobotSerial("SIMULATED_NO_PORT", 115200)
        self.robot.connection = Mock(is_open=True)
        self.executor = MotionExecutor(self.robot)

    def test_state_before_the_command_is_not_a_confirmation(self):
        self.robot._consume_line("GRIPPER_HOLD reason=contact", time.monotonic())
        since = time.monotonic() + 0.001
        with self.assertRaisesRegex(RuntimeError, "no confirmó"):
            self.executor._wait_gripper("holding", timeout=0.2, since=since)

    def test_new_contact_confirms(self):
        since = time.monotonic()
        self.robot._consume_line("GRIPPER_HOLD reason=contact", time.monotonic())
        self.executor._wait_gripper("holding", timeout=0.5, since=since)

    def test_empty_close_stops_the_task(self):
        since = time.monotonic()
        self.robot._consume_line("GRIPPER_HOLD reason=closed_limit", time.monotonic())
        with self.assertRaisesRegex(RuntimeError, "sin el cubo"):
            self.executor._wait_gripper("holding", timeout=0.5, since=since)


class SerialLogTests(unittest.TestCase):
    def test_tx_and_rx_are_written_to_one_file_per_connection(self):
        with tempfile.TemporaryDirectory() as folder:
            robot = RobotSerial("SIMULATED_NO_PORT", 115200, log_folder=Path(folder))
            robot._open_log()
            robot.connection = Mock(is_open=True)
            robot.send("gripper close")
            robot._consume_line("GRIPPER_I raw=40 mode=closing", time.monotonic())
            robot.connection = None
            robot.disconnect()
            files = list(Path(folder).glob("serie_*.log"))
            self.assertEqual(len(files), 1)
            lines = files[0].read_text(encoding="utf-8").splitlines()
            self.assertEqual([line.split("\t")[1:] for line in lines],
                             [["tx", "gripper close"], ["rx", "GRIPPER_I raw=40 mode=closing"]])


if __name__ == "__main__":
    unittest.main()
