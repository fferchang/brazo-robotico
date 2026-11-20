"""Garra más rápida: cierre por presión con tramo rápido y apertura con `gripper speed`."""
import unittest

from brazoia import gemelo
from brazoia.config import MotionConfig, load_motion
from brazoia.execution import GRIPPER_FAST_CLOSE_EMPTY_UNTIL, GRIPPER_FAST_CLOSE_UNTIL, MotionExecutor
from brazoia.gemelo import DigitalTwin, speed_to_deg_s
from brazoia.robot import RobotSerial, configure_motion


class TwinFastCloseTests(unittest.TestCase):
    def setUp(self):
        self.twin = DigitalTwin(seed=4)
        self.twin.handle_line("6 90")
        for _ in range(400):  # la garra llega a 90° (logico)
            self.twin.tick()
        self.assertGreater(self.twin._logical_from_encoder(6), 85.0)

    def run_line(self, line):
        count = self.twin.log_count
        self.twin.handle_line(line)
        return self.twin.log_since(count)

    def test_goes_fast_to_the_cutoff_then_as_the_usual_close_ends(self):
        start = self.run_line("gripper close 55")[0]
        self.assertRegex(start, r"GRIPPER_CLOSE_START from=\d+\.\d contact_raw=\d+ fast_until=55\.0")
        profile = self.twin.profiles[6]
        self.assertAlmostEqual(profile.speed, speed_to_deg_s(gemelo.GRIPPER_FAST_CLOSE_SPEED))
        self.assertEqual(profile.target, 55.0)
        while self.twin.gripper_fast_until is not None:
            self.twin.tick()
        self.twin.tick()
        self.twin.gripper_last_step = -1e9
        self.twin.tick()
        # Igual que al final del cierre de siempre: consigna cerrada, servo a 220.
        self.assertLessEqual(self.twin._logical_from_encoder(6), 55.0 + gemelo.GRIPPER_FAST_HANDOFF_DEGREES + 1.0)
        self.assertEqual(profile.target, 0.0)
        self.assertAlmostEqual(profile.speed, speed_to_deg_s(220))

    def test_without_cutoff_or_already_close_it_is_the_usual_close(self):
        self.assertNotIn("fast_until", self.run_line("gripper close")[0])
        self.twin.gripper_closing = False
        self.twin.handle_line("6 58")
        for _ in range(200):
            self.twin.tick()
        self.assertNotIn("fast_until", self.run_line("gripper close 55")[0])

    def test_bad_cutoffs_are_refused(self):
        for line in ("gripper close 200", "gripper close -1", "gripper close x", "gripper close 55 1"):
            self.assertEqual(self.run_line(line),
                             ["GRIPPER_ERROR reason=fuera_de_rango detail=gripper_close_hasta_0_a_140"], line)
        self.assertFalse(self.twin.gripper_closing)

    def test_opening_uses_gripper_speed_and_closing_by_angle_does_not(self):
        self.assertEqual(self.run_line("gripper speed 900"), ["GRIPPER_SPEED open=900"])
        self.twin.last_move = -1e9
        self.run_line("6 120")
        self.assertAlmostEqual(self.twin.profiles[6].speed, speed_to_deg_s(900))
        self.twin.last_move = -1e9
        self.run_line("6 30")
        self.assertAlmostEqual(self.twin.profiles[6].speed, speed_to_deg_s(gemelo.MOVE_SPEED))
        for line in ("gripper speed 50", "gripper speed 2000", "gripper speed x"):
            self.assertEqual(self.run_line(line), ["GRIPPER_SPEED_REJECTED reason=fuera_de_rango"], line)

    def test_old_firmware_does_not_understand_either(self):
        self.twin.sync_firmware = False
        self.assertNotIn("FIRMWARE_CAPS", " ".join(self.run_line("caps")))
        self.assertFalse(any("GRIPPER_SPEED" in line for line in self.run_line("gripper speed 900")))


class Recorder(RobotSerial):
    def __init__(self, fast):
        super().__init__("SIMULADO", 115200)
        self.gripper_fast_supported = fast
        self.guard_supported = fast
        self.sent = []

    def send(self, command):
        self.sent.append(command)


class HostGripperTests(unittest.TestCase):
    def test_close_command_depends_on_firmware_and_stage(self):
        old = Recorder(fast=False)
        MotionExecutor(old)._send_gripper_close("agarre")
        self.assertEqual(old.sent[-1], "gripper close")
        new = Recorder(fast=True)
        executor = MotionExecutor(new)
        executor._send_gripper_close("agarre")
        self.assertEqual(new.sent[-1], f"gripper close {GRIPPER_FAST_CLOSE_UNTIL:.1f}")
        executor._send_gripper_close("guardado")
        self.assertEqual(new.sent[-1], f"gripper close {GRIPPER_FAST_CLOSE_EMPTY_UNTIL:.1f}")
        MotionExecutor(new, fast_gripper_close=False)._send_gripper_close("agarre")
        self.assertEqual(new.sent[-1], "gripper close")
        self.assertTrue(all(sent == "gripper current 60" for sent in new.sent[:4:2]))

    def test_open_speed_goes_with_the_configuration(self):
        robot = Recorder(fast=True)
        commands = configure_motion(robot, MotionConfig(gripper_open_speed=800), timeout=0.05)
        self.assertEqual(commands[-1], "gripper speed 800")
        robot._consume_line("FIRMWARE_CAPS sync=1 sync_max_motors=5 sync_max_speed=450 guard=1 "
                            "speed_hard_max=1000 gripper_fast=1", 1.0)
        self.assertTrue(robot.gripper_fast_supported)
        robot._consume_line("GRIPPER_SPEED_REJECTED reason=fuera_de_rango", 1.1)
        self.assertIn("GRIPPER_SPEED_REJECTED", robot.snapshot().fault)

    def test_configuration_range(self):
        self.assertEqual(load_motion({}).gripper_open_speed, 900)
        self.assertTrue(load_motion({}).fast_gripper_close)
        self.assertFalse(load_motion({"cierre_rapido": False}).fast_gripper_close)
        for value in (50, 2000):
            with self.subTest(value=value), self.assertRaises(ValueError):
                load_motion({"velocidad_garra_abrir": value})


if __name__ == "__main__":
    unittest.main()
