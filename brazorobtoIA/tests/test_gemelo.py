"""Gemelo digital: el firmware virtual responde como firmware_secuencias.ino."""
from __future__ import annotations

import time
import unittest
from pathlib import Path

from brazoia.config import detection_parameters, load_config
from brazoia.execution import GRIPPER_OPEN_SPEED_DEG_S, gripper_open_timeout
from brazoia.robot import is_fault_line, parse_positions

try:
    import mujoco
except ImportError:  # La simulación es opcional para el resto de la aplicación.
    mujoco = None


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "firmware_secuencias.ino"


class OpenTimeoutTest(unittest.TestCase):
    def test_open_from_closed_waits_longer_than_the_servo_takes(self):
        travel = 140.0 / GRIPPER_OPEN_SPEED_DEG_S
        self.assertGreater(travel, 5.0)  # por eso no alcanzaban los 5 s de antes
        self.assertGreater(gripper_open_timeout(0.0), travel + 1.0)
        self.assertGreater(gripper_open_timeout(None), travel + 1.0)
        self.assertEqual(5.0, gripper_open_timeout(139.0))

    def test_speed_matches_the_firmware_open_command(self):
        text = FIRMWARE.read_text(encoding="utf-8", errors="replace")
        self.assertIn("servoBus.WritePosEx(6, (int)degToPos(realDegrees), 300, 25);", text)


class LoadedArrivalTest(unittest.TestCase):
    """El ejecutor acepta un motor quieto a menos de 3°, pero no uno que sigue moviéndose."""

    def executor(self, positions):
        from types import SimpleNamespace

        from brazoia.execution import MotionExecutor

        sequence = iter(positions)
        state = {"value": positions[0]}

        def snapshot():
            state["value"] = next(sequence, state["value"])
            return SimpleNamespace(positions={2: state["value"]}, fault="", torque_enabled=True)

        robot = SimpleNamespace(send=lambda command: None, snapshot=snapshot)
        messages = []
        return MotionExecutor(robot, status=messages.append), messages

    def test_still_motor_within_three_degrees_is_accepted(self):
        from brazoia.planner import MotionGroup

        executor, messages = self.executor([200.0, 210.0, 218.0, 218.6] + [218.6] * 40)
        executor._move_group(MotionGroup(((2, 220.0),)))
        self.assertTrue(any("Llegada con desvío" in text for text in messages))

    def test_far_or_moving_motor_is_not_accepted(self):
        from brazoia.planner import MotionGroup

        executor, _ = self.executor([200.0] + [216.0] * 80)  # quieto, pero a 4°
        with self.assertRaises(RuntimeError):
            executor._move_group(MotionGroup(((2, 220.0),)))


class PlanningStartTest(unittest.TestCase):
    def test_encoder_slightly_past_a_limit_still_plans(self):
        from brazoia.planner import build_motion_plan, planning_start

        limits = load_config().limits
        current = {1: 180.0, 2: 180.0, 3: 89.8, 4: 180.0, 5: 263.0, 6: 0.0}
        with self.assertRaises(ValueError):
            build_motion_plan(current, {3: 120.0}, limits)
        start = planning_start(current, limits)
        self.assertEqual(90.5, start[3])
        self.assertEqual(0.0, start[6])
        self.assertTrue(build_motion_plan(start, {3: 120.0}, limits))

    def test_far_outside_a_limit_is_refused(self):
        from brazoia.planner import planning_start

        with self.assertRaises(ValueError):
            planning_start({4: 290.0}, load_config().limits)


class DetectionParametersTest(unittest.TestCase):
    def test_frontal_uses_its_own_thresholds(self):
        vision = load_config().vision
        frontal, lateral = detection_parameters(vision, 0), detection_parameters(vision, 1)
        self.assertEqual(vision.frontal_minimum_lime_area_ratio, frontal["minimum_lime_area_ratio"])
        self.assertEqual(vision.minimum_lime_area_ratio, lateral["minimum_lime_area_ratio"])
        self.assertEqual(0.0, lateral["minimum_lime_confidence"])


@unittest.skipIf(mujoco is None, "MuJoCo no está instalado")
class VirtualFirmwareTest(unittest.TestCase):
    """Sin hilo: cada prueba avanza el gemelo a mano con `tick()`."""

    def setUp(self):
        from brazoia.gemelo import DigitalTwin
        from brazoia.mujoco_sim import Placement

        self.twin = DigitalTwin(placement=Placement((0.08, -0.25), 0.0, (-0.12, -0.22), 0.0))
        self.lines = []
        self.twin.println = self._capture(self.twin.println)

    def _capture(self, original):
        def println(text=""):
            self.lines.append(text)
            original(text)
        return println

    def run_for(self, seconds):
        for _ in range(int(round(seconds / 0.01))):
            self.twin.tick()

    def send(self, command, seconds=0.0):
        self.lines.clear()
        self.twin.handle_line(command)
        self.run_for(seconds)
        return [line for line in self.lines if not line.startswith("POS ID")]

    def telemetry(self):
        values = {}
        for line in self.lines:
            if line.startswith("POS ID"):
                values.update(parse_positions(line))
        return values

    def test_telemetry_parses_like_the_real_one(self):
        self.lines.clear()
        self.run_for(0.06)
        values = self.telemetry()
        self.assertEqual(set(range(1, 7)), set(values))
        self.assertAlmostEqual(180.0, values[1], delta=0.2)
        self.assertAlmostEqual(0.0, values[6], delta=1.0)  # M6 lógico: el firmware informa real = lógico + 10
        posall = self.send("posall")
        self.assertEqual(set(range(1, 7)), set(parse_positions(posall[0])))

    def test_single_move_reaches_target_at_servo_speed(self):
        responses = self.send("1 220")
        self.assertTrue(responses[0].startswith("Moviendo ID 1 a 220.0 deg"))
        self.run_for(0.6)
        self.assertLess(self.twin.logical_pose()[1], 210.0)  # 40° a ~40°/s: todavía no llegó
        self.run_for(1.2)
        self.assertAlmostEqual(220.0, self.twin.logical_pose()[1], delta=1.0)

    def test_limits_are_clamped_and_flagged_as_faults(self):
        responses = self.send("4 10")
        self.assertEqual("AJUSTE LIMITE ID 4: 10.00 -> 75.00", responses[0])
        self.assertTrue(is_fault_line(responses[0]))

    def test_pair_is_validated_like_the_firmware(self):
        self.assertEqual("PAIR_REJECTED reason=fuera_de_rango ID=2", self.send("pair 2 300 3 120")[0])
        self.run_for(0.06)
        # M2 ≤ 100 y M3 ≤ 110 limitan M4 a 130°.
        rejected = self.send("pair 2 100 4 270")[0]
        self.assertEqual("PAIR_REJECTED reason=trayectoria_insegura detail=REGLA_DINAMICA_M4", rejected)
        self.assertTrue(is_fault_line(rejected))
        accepted = self.send("pair 2 150 3 120", 3.0)[0]
        self.assertTrue(accepted.startswith("PAIR_MOVE_OK ID1=2 target1=150.00"))
        pose = self.twin.logical_pose()
        self.assertAlmostEqual(150.0, pose[2], delta=1.2)
        self.assertAlmostEqual(120.0, pose[3], delta=1.2)

    def test_pose_rules_are_the_firmware_ones_not_the_planner_ones(self):
        from brazoia.gemelo import firmware_pose_reason
        from brazoia.planner import classify_pose

        # M3 unas décimas bajo su límite: el planificador lo rechaza, el firmware no lo mira.
        pose = (98.7, 89.9, 119.9)
        self.assertEqual("LIMITE", classify_pose(pose, 0.0)[1])
        self.assertEqual("OK", firmware_pose_reason(pose, 0.0, None))

    def test_cooldown_between_moves(self):
        self.send("1 190")
        self.assertEqual("PAIR_REJECTED reason=cooldown", self.send("pair 1 200 5 250")[0])

    def test_torque_commands_answer_like_the_firmware(self):
        self.assertEqual("ARM_TORQUE_OFF_OK", self.send("arm torque off")[-1])
        self.assertFalse(any(self.twin.torque[motor] for motor in range(1, 6)))
        self.assertTrue(self.twin.torque[6])
        self.assertEqual("TORQUE_ON_OK", self.send("torque on")[-1])
        self.assertTrue(all(self.twin.torque.values()))

    def test_gripper_open_takes_what_the_servo_takes(self):
        self.assertEqual("GRIPPER_OPEN_OK target=140.0", self.send("gripper open", 4.5)[0])
        self.assertLess(self.twin.logical_pose()[6], 130.0)
        self.run_for(1.5)
        self.assertAlmostEqual(140.0, self.twin.logical_pose()[6], delta=1.0)

    def test_closing_on_nothing_stops_at_the_closed_limit(self):
        self.send("gripper open", 6.0)
        responses = self.send("gripper close", 10.0)
        self.assertTrue(responses[0].startswith("GRIPPER_CLOSE_START from=1"))
        self.assertIn("GRIPPER_STOP_DETAIL reason=closed_limit", " ".join(responses))
        self.assertTrue(any(line.startswith("GRIPPER_HOLD reason=closed_limit") for line in responses))
        self.assertFalse(self.twin.gripper_closing)

    def test_unknown_command(self):
        self.assertEqual("Comando invalido. Usa 'help'.", self.send("hola")[0])
        self.assertEqual("ID invalido. Usa 1..6.", self.send("9 100")[0])


@unittest.skipIf(mujoco is None, "MuJoCo no está instalado")
class VirtualPortTest(unittest.TestCase):
    def test_robot_serial_talks_to_the_twin(self):
        from brazoia.gemelo import VIRTUAL_PORT, DigitalTwin
        from brazoia.robot import RobotSerial

        twin = DigitalTwin(pace=0)
        robot = RobotSerial(VIRTUAL_PORT, 115200, opener=twin.open_port)
        try:
            robot.connect()
            robot.send("arm torque on")
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                snapshot = robot.snapshot()
                if snapshot.torque_enabled and len(snapshot.positions) == 6:
                    break
                time.sleep(0.02)
            snapshot = robot.snapshot()
            self.assertTrue(snapshot.torque_enabled)
            self.assertAlmostEqual(263.0, snapshot.positions[5], delta=0.3)
            self.assertEqual("", snapshot.fault)
        finally:
            robot.disconnect()
            twin.stop()


if __name__ == "__main__":
    unittest.main()
