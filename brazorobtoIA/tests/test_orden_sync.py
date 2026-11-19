"""Orden `sync`: hasta cinco motores del brazo en un solo mensaje (firmware, gemelo y PC)."""
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

from brazoia import gemelo
from brazoia.execution import MotionExecutor, MotionProposal, MotionSegment
from brazoia.gemelo import DigitalTwin
from brazoia.planner import MotionGroup
from brazoia.robot import RobotSerial

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "firmware_secuencias.ino"
HOST_TESTS = ROOT / "firmware" / "prueba_pc"


class FirmwareConstantsTests(unittest.TestCase):
    def test_sync_constants_match_the_twin(self) -> None:
        source = FIRMWARE.read_text(encoding="utf-8")
        self.assertRegex(source, r"SYNC_MAX_SPEED = MOVE_SPEED;")
        self.assertEqual(gemelo.SYNC_MAX_SPEED, gemelo.MOVE_SPEED)
        for name in ("SYNC_MIN_SPEED", "SYNC_MAX_MOTORS", "SYNC_MAX_DURATION_MS"):
            match = re.search(rf"{name} = (\d+);", source)
            self.assertIsNotNone(match, name)
            self.assertEqual(int(match.group(1)), getattr(gemelo, name), name)

    @unittest.skipUnless(shutil.which("g++"), "sin g++: la prueba del firmware en la PC se saltea")
    def test_firmware_sync_on_the_pc(self) -> None:
        """Compila el .ino con stubs de Arduino y SCServo y corre las pruebas de `sync`."""
        with tempfile.TemporaryDirectory() as folder:
            build = Path(folder)
            for name in ("Arduino.h", "SCServo.h", "test_sync.cpp"):
                shutil.copy(HOST_TESTS / name, build / name)
            converted = subprocess.run([sys.executable, str(HOST_TESTS / "make_host.py"), str(FIRMWARE)],
                                       capture_output=True, text=True, check=True).stdout
            (build / "firmware_host.cpp").write_text(converted, encoding="utf-8")
            compiled = subprocess.run(
                ["g++", "-std=gnu++17", "-I.", "-include", "Arduino.h", "test_sync.cpp", "-o", "test_sync"],
                cwd=build, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr[-3000:])
            result = subprocess.run([str(build / "test_sync")], capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class TwinSyncTests(unittest.TestCase):
    def setUp(self):
        self.twin = DigitalTwin(seed=4)

    def run_line(self, line: str) -> list[str]:
        count = self.twin.log_count
        self.twin.handle_line(line)
        return self.twin.log_since(count)

    def test_caps_announces_sync(self):
        self.assertEqual(self.run_line("caps"), ["FIRMWARE_CAPS sync=1 sync_max_motors=5 sync_max_speed=450"])

    def test_five_motors_start_together_with_proportional_speeds(self):
        reply = self.run_line("sync 0 1 190 2 175 3 95 4 182 5 260")
        self.assertEqual(len(reply), 1)
        self.assertTrue(reply[0].startswith("SYNC_MOVE_OK n=5 dur_ms=0 ids=1,2,3,4,5 speeds=450,"), reply)
        speeds = [int(value) for value in reply[0].split("speeds=")[1].split(",")]
        self.assertTrue(all(gemelo.SYNC_MIN_SPEED <= speed <= gemelo.SYNC_MAX_SPEED for speed in speeds))
        self.assertEqual({motor: self.twin.profiles[motor].target for motor in range(1, 6)},
                         {1: 190.0, 2: 175.0, 3: 95.0, 4: 182.0, 5: 260.0})

    def test_duration_lowers_the_speed_but_not_above_the_ceiling(self):
        self.assertIn("speeds=113", self.run_line("sync 1000 1 190")[0])
        self.twin.last_move = -1e9
        self.assertIn("speeds=450", self.run_line("sync 50 1 180")[0])

    def test_cooldown_is_shared_with_the_other_orders(self):
        self.run_line("sync 0 1 185")
        self.assertEqual(self.run_line("sync 0 1 190"), ["SYNC_REJECTED reason=cooldown"])
        self.assertEqual(self.twin.profiles[1].target, 185.0)

    def test_malformed_or_out_of_range_orders_never_move(self):
        cases = {
            "sync": "formato", "sync 0": "formato", "sync x 1 190": "formato",
            "sync 0 6 90": "motores", "sync 0 0 90": "motores", "sync 0 1 190 1 185": "motores",
            "sync 0 1 190 2": "formato", "sync 0 1 nan": "formato", "sync 0 1 1_0": "formato",
            "sync 0 1 50": "fuera_de_rango", "sync 20000 1 190": "formato",
            "sync 0 1 190 2 180 3 90 4 180 5 263 1 180": "formato",
        }
        targets = {motor: self.twin.profiles[motor].target for motor in range(1, 6)}
        for line, reason in cases.items():
            with self.subTest(line=line):
                self.twin.last_move = -1e9
                self.assertEqual(self.run_line(line), [f"SYNC_REJECTED reason={reason}"])
        self.assertEqual({motor: self.twin.profiles[motor].target for motor in range(1, 6)}, targets)

    def test_unsafe_path_is_refused_like_pair(self):
        """La pose final es válida, pero la recta desde la actual pasa por la mesa."""
        pair = self.run_line("pair 2 270 3 276")
        self.twin.last_move = -1e9
        sync = self.run_line("sync 0 2 270 3 276")
        self.assertTrue(pair[0].startswith("PAIR_REJECTED reason=trayectoria_insegura"), pair)
        self.assertTrue(sync[0].startswith("SYNC_REJECTED reason=trayectoria_insegura"), sync)
        self.assertEqual(self.twin.profiles[2].target, 180.0)

    def test_busy_while_a_firmware_point_runs(self):
        self.run_line("waypoint 1 1 1 184")
        self.assertEqual(self.run_line("sync 0 1 190"), ["WAYPOINT_BUSY id=1"])

    def test_old_firmware_neither_announces_nor_moves(self):
        self.twin.sync_firmware = False
        self.assertEqual(self.run_line("caps"), ["Comando invalido. Usa 'help'."])
        self.assertEqual(self.run_line("sync 0 1 190"), ["ID invalido. Usa 1..6."])
        self.assertEqual(self.twin.profiles[1].target, 180.0)


class HostCapabilityTests(unittest.TestCase):
    def test_caps_reply_enables_sync_and_its_ack_counts(self):
        robot = RobotSerial("SIMULADO", 115200)
        self.assertFalse(robot.sync_supported)
        robot._consume_line("FIRMWARE_CAPS sync=1 sync_max_motors=5 sync_max_speed=450", 1.0)
        self.assertTrue(robot.sync_supported)
        robot._consume_line("SYNC_MOVE_OK n=2 dur_ms=0 ids=1,5 speeds=450,300", 1.1)
        self.assertEqual(robot.snapshot().move_acks, 1)
        self.assertEqual(robot.snapshot().fault, "")
        robot._consume_line("SYNC_REJECTED reason=trayectoria_insegura detail=PISO", 1.2)
        self.assertIn("SYNC_REJECTED", robot.snapshot().fault)

    def test_old_firmware_reply_keeps_pair(self):
        robot = RobotSerial("SIMULADO", 115200)
        robot._consume_line("Comando invalido. Usa 'help'.", 1.0)
        self.assertFalse(robot.sync_supported)
        self.assertEqual(robot.snapshot().fault, "")


class InstantSerial(RobotSerial):
    """Acusa cada orden y deja los motores en su destino al instante."""

    def __init__(self, supported: bool):
        super().__init__("SIMULADO", 115200)
        self.sync_supported = supported
        self.torque_enabled = True
        self.sent: list[str] = []
        self._place({1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0, 6: 0.0})

    def _place(self, targets):
        now = time.monotonic()
        with self.state_lock:
            for motor, value in targets.items():
                self.positions[motor] = value
                self.received_at[motor] = now

    def snapshot(self):
        # Como el firmware: una lectura nueva de cada motor cada 50 ms.
        now = time.monotonic()
        with self.state_lock:
            for motor in self.received_at:
                if now - self.received_at[motor] >= 0.05:
                    self.received_at[motor] = now
        return super().snapshot()

    def send(self, command):
        self.sent.append(command)
        parts = command.split()
        if parts[0] == "sync":
            self._place({int(m): float(v) for m, v in zip(parts[2::2], parts[3::2])})
            self._consume_line("SYNC_MOVE_OK", time.monotonic())
        elif parts[0] == "pair":
            self._place({int(parts[1]): float(parts[2]), int(parts[3]): float(parts[4])})
            self._consume_line("PAIR_MOVE_OK", time.monotonic())
        else:
            self._place({int(parts[0]): float(parts[1])})
            self._consume_line(f"Moviendo ID {parts[0]}", time.monotonic())


def together_point(groups):
    target = {motor: value for group in groups for motor, value in group.targets}
    return MotionProposal(1, "aproximacion_1", "Aproximación 1", target, tuple(groups), None,
                          segments=(MotionSegment(0, len(groups), True, True),))


GROUPS = (MotionGroup(((1, 184.0), (5, 266.0))), MotionGroup(((2, 183.0), (3, 93.0))),
          MotionGroup(((4, 184.0),)))


class ExecutorSyncTests(unittest.TestCase):
    def moves(self, robot):
        return [command for command in robot.sent if not command.startswith("gripper")]

    def test_together_point_goes_in_one_sync_order(self):
        robot = InstantSerial(supported=True)
        MotionExecutor(robot).execute(together_point(GROUPS))
        self.assertEqual(self.moves(robot), ["sync 0 1 184.00 5 266.00 2 183.00 3 93.00 4 184.00"])

    def test_firmware_without_sync_keeps_pair(self):
        robot = InstantSerial(supported=False)
        MotionExecutor(robot).execute(together_point(GROUPS))
        self.assertEqual(self.moves(robot), ["pair 1 184.00 5 266.00", "pair 2 183.00 3 93.00", "4 184.00"])

    def test_sync_can_be_turned_off(self):
        robot = InstantSerial(supported=True)
        MotionExecutor(robot, sync_orders=False).execute(together_point(GROUPS))
        self.assertEqual(len(self.moves(robot)), 3)

    def test_a_motor_twice_in_a_point_keeps_the_order(self):
        """Un desvío del planificador (el mismo motor en dos grupos) no se aplasta en uno."""
        robot = InstantSerial(supported=True)
        groups = (MotionGroup(((2, 183.0), (3, 93.0))), MotionGroup(((2, 185.0),)))
        MotionExecutor(robot).execute(together_point(groups))
        self.assertEqual(self.moves(robot), ["pair 2 183.00 3 93.00", "2 185.00"])

    def test_sequential_points_stay_sequential(self):
        """Un punto que no va «junto» (orden de seguridad del planificador) sigue en orden."""
        robot = InstantSerial(supported=True)
        proposal = MotionProposal(1, "aproximacion_1", "Aproximación 1", {2: 183.0, 4: 184.0}, GROUPS[1:], None,
                                  segments=(MotionSegment(0, 2, False, True),))
        MotionExecutor(robot).execute(proposal)
        self.assertEqual(self.moves(robot), ["pair 2 183.00 3 93.00", "4 184.00"])


class TwinEndToEndTests(unittest.TestCase):
    def test_executor_moves_the_twin_with_sync(self):
        twin = DigitalTwin(seed=4)
        robot = RobotSerial(gemelo.VIRTUAL_PORT, 115200, opener=twin.open_port)
        try:
            robot.connect()
            deadline = time.monotonic() + 3.0
            while not robot.sync_supported and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertTrue(robot.sync_supported)
            robot.torque_enabled = True
            count = twin.log_count
            MotionExecutor(robot).execute(together_point(GROUPS))
            replies = twin.log_since(count)
            self.assertEqual(sum(line.startswith("SYNC_MOVE_OK n=5") for line in replies), 1, replies)
            self.assertFalse(any("PAIR_MOVE_OK" in line for line in replies))
            positions = robot.snapshot().positions
            for group in GROUPS:
                for motor, value in group.targets:
                    self.assertLess(abs(positions[motor] - value), 1.5, motor)
        finally:
            robot.disconnect()
            twin.stop()


if __name__ == "__main__":
    unittest.main()
