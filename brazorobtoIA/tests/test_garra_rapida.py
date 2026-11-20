"""Garra más rápida: cierre por presión con tramo rápido y apertura con `gripper speed`."""
import time
import unittest

from brazoia import gemelo
from brazoia.config import MotionConfig, load_motion
from brazoia.execution import (
    GRIP_SETTLE_SECONDS,
    GRIP_SETTLE_TIMEOUT_SECONDS,
    GRIPPER_FAST_CLOSE_EMPTY_UNTIL,
    GRIPPER_FAST_CLOSE_UNTIL,
    MotionExecutor,
)
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


class ClosingOnCube:
    """Cierre por presión en el gemelo que toca el cubo (sin pruebas propias)."""

    def setUp(self):
        self.twin = DigitalTwin(seed=4)

    def close_on_contact(self):
        """Cierre en curso: primera lectura de contacto a 43,6° y confirmación a 42,8°."""
        twin = self.twin
        twin.gripper_closing = True
        twin.gripper_started = twin.now - 1.0
        twin.gripper_command = 0.0
        twin.contact_threshold = 60
        twin._logical_from_encoder = lambda motor: self.position if motor == 6 else 0.0
        for position, current in [(43.6, 100), (42.8, 120), (42.8, 120)]:
            self.position = position
            twin._gripper_current_raw = lambda value=current: value
            twin.gripper_last_step = -1e9
            twin._process_gripper_closing()
        return twin.log_since(0)


class TwinSqueezeTests(ClosingOnCube, unittest.TestCase):
    def test_hold_from_first_contact_minus_squeeze(self):
        self.assertEqual(self.twin.log_since(0)[-1:], [])
        self.twin.handle_line("gripper squeeze 2")
        self.assertIn("GRIPPER_SQUEEZE deg=2.0", self.twin.log_since(0))
        log = self.close_on_contact()
        hold = [line for line in log if line.startswith("GRIPPER_HOLD reason=contact")]
        self.assertEqual(len(hold), 1)
        self.assertTrue(hold[0].endswith("squeeze_deg=2.0"), hold[0])
        self.assertAlmostEqual(self.twin.profiles[6].target, 41.6)

    def test_without_squeeze_it_keeps_the_old_behaviour(self):
        log = self.close_on_contact()
        hold = [line for line in log if line.startswith("GRIPPER_HOLD reason=contact")][0]
        self.assertNotIn("squeeze_deg", hold)

    def test_bad_values_are_refused(self):
        for line in ("gripper squeeze 11", "gripper squeeze -1", "gripper squeeze x"):
            count = self.twin.log_count
            self.twin.handle_line(line)
            self.assertEqual(self.twin.log_since(count), ["GRIPPER_SQUEEZE_REJECTED reason=fuera_de_rango"])
        self.assertEqual(self.twin.gripper_squeeze, 0.0)


class TwinSettleTests(ClosingOnCube, unittest.TestCase):
    def watch(self, current):
        twin = self.twin
        twin._gripper_current_raw = lambda value=current: value
        twin.gripper_watch_started -= 0.1  # 100 ms más desde el agarre
        twin.gripper_last_watch = -1e9
        count = twin.log_count
        twin._process_gripper_watch()
        return twin.log_since(count)

    def test_settled_when_the_current_is_stable(self):
        self.twin.handle_line("gripper squeeze 5")
        self.close_on_contact()
        self.assertEqual(self.watch(110), [])
        self.assertEqual(self.watch(141), [])
        self.assertEqual(self.watch(143), ["GRIPPER_SETTLED reason=stable current_raw=143 elapsed_ms=300"])
        self.assertEqual(self.watch(143), [])

    def test_unstable_current_settles_by_time(self):
        self.close_on_contact()
        replies = []
        for index in range(25):
            replies += self.watch(60 if index % 2 else 150)
        self.assertEqual(len(replies), 1)
        self.assertTrue(replies[0].startswith("GRIPPER_SETTLED reason=timeout"), replies)


class SettleSerial(RobotSerial):
    def __init__(self, settle_after=None):
        super().__init__("SIMULADO", 115200)
        self.gripper_settle_supported = True
        self.started = time.monotonic()
        self.settle_after = settle_after

    def snapshot(self):
        if self.settle_after is not None and time.monotonic() - self.started >= self.settle_after:
            self.settle_after = None
            self._consume_line("GRIPPER_SETTLED reason=stable current_raw=140 elapsed_ms=300", time.monotonic())
        return super().snapshot()


class ExecutorSettleTests(unittest.TestCase):
    def test_lifts_as_soon_as_the_grip_settles(self):
        robot = SettleSerial(settle_after=0.3)
        started = time.monotonic()
        MotionExecutor(robot)._wait_grip_settle(0)
        self.assertLess(time.monotonic() - started, 0.6)

    def test_without_the_notice_it_waits_the_maximum_and_warns(self):
        robot = SettleSerial()
        messages = []
        started = time.monotonic()
        MotionExecutor(robot, status=messages.append)._wait_grip_settle(0)
        self.assertGreaterEqual(time.monotonic() - started, GRIP_SETTLE_TIMEOUT_SECONDS - 0.05)
        self.assertIn("no avisó", messages[-1])

    def test_old_firmware_keeps_the_fixed_wait(self):
        robot = SettleSerial()
        robot.gripper_settle_supported = False
        started = time.monotonic()
        MotionExecutor(robot)._wait_grip_settle(0)
        self.assertAlmostEqual(time.monotonic() - started, GRIP_SETTLE_SECONDS, delta=0.15)


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
        robot.gripper_squeeze_supported = True
        commands = configure_motion(robot, MotionConfig(gripper_squeeze_degrees=2.0), timeout=0.05)
        self.assertEqual(commands[-1], "gripper squeeze 2.0")
        robot._consume_line("FIRMWARE_CAPS sync=1 sync_max_motors=5 sync_max_speed=450 guard=1 "
                            "speed_hard_max=1000 gripper_fast=1", 1.0)
        self.assertTrue(robot.gripper_fast_supported)
        robot._consume_line("GRIPPER_SPEED_REJECTED reason=fuera_de_rango", 1.1)
        self.assertIn("GRIPPER_SPEED_REJECTED", robot.snapshot().fault)

    def test_configuration_range(self):
        self.assertEqual(load_motion({}).gripper_open_speed, 900)
        self.assertEqual(load_motion({}).gripper_squeeze_degrees, 5.0)
        self.assertTrue(load_motion({}).fast_gripper_close)
        self.assertFalse(load_motion({"cierre_rapido": False}).fast_gripper_close)
        for value in (50, 2000):
            with self.subTest(value=value), self.assertRaises(ValueError):
                load_motion({"velocidad_garra_abrir": value})
        for value in (-1, 11):
            with self.subTest(value=value), self.assertRaises(ValueError):
                load_motion({"apriete_grados": value})


if __name__ == "__main__":
    unittest.main()
