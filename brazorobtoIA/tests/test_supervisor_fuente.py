"""Supervisor de la fuente (`power guard`) y velocidad máxima del brazo (`speed max`)."""
import json
import tempfile
import time
import unittest
from pathlib import Path

from brazoia import gemelo
from brazoia.config import MotionConfig, load_config, load_motion
from brazoia.execution import MotionExecutor, MotionProposal, MotionSegment
from brazoia.gemelo import DigitalTwin, speed_to_deg_s
from brazoia.planner import MotionGroup
from brazoia.robot import RobotSerial, configure_motion, motion_commands


class TwinGuardTests(unittest.TestCase):
    def setUp(self):
        self.twin = DigitalTwin(seed=4)

    def run_line(self, line):
        count = self.twin.log_count
        self.twin.handle_line(line)
        return self.twin.log_since(count)

    def sample(self, count=1):
        """Avanza hasta que el gemelo haya tomado `count` lecturas nuevas de la fuente."""
        replies = []
        for _ in range(count):
            start = self.twin.log_count
            self.twin.last_power_telemetry = -1e9
            self.twin._process_power_telemetry()
            replies += self.twin.log_since(start)
        return replies

    def test_speed_max_sets_the_ceiling_of_every_arm_order(self):
        self.assertEqual(self.run_line("speed max 700 60"), ["ARM_SPEED max=700 acc=60"])
        self.assertIn("sync_max_speed=700", self.run_line("caps")[0])
        self.assertIn("speeds=700", self.run_line("sync 0 1 190 2 175")[0])
        self.assertAlmostEqual(self.twin.profiles[1].speed, speed_to_deg_s(700))
        self.assertAlmostEqual(self.twin.profiles[1].acceleration, gemelo.acc_to_deg_s2(60))
        for line in ("speed max 1200", "speed max 50", "speed max 700 200", "speed max x"):
            self.assertEqual(self.run_line(line), ["ARM_SPEED_REJECTED reason=fuera_de_rango"], line)

    def test_off_by_default(self):
        self.twin.injected_voltage_dv = 70
        self.assertEqual(self.sample(3), [])

    def test_low_voltage_halves_the_speed_and_resends_pending_goals(self):
        self.run_line("power guard on")
        self.run_line("sync 0 1 200 2 170")
        self.twin.injected_voltage_dv = 90
        replies = self.sample()
        self.assertEqual(replies, ["POWER_GUARD scale=50 reason=voltaje v_min_dv=90 total_ma="
                                   + replies[0].split("total_ma=")[1]])
        self.assertAlmostEqual(self.twin.profiles[1].speed, speed_to_deg_s(225))
        self.assertEqual(self.twin.profiles[1].target, 200.0)  # mismo destino
        self.assertTrue(self.sample()[0].startswith("POWER_GUARD scale=25"))
        self.assertEqual(self.sample(), [])  # no baja de 25%
        self.twin.last_move = -1e9  # sin ticks el reloj del gemelo no avanza: saltear el enfriamiento
        self.assertIn("speeds=112", self.run_line("sync 0 3 95")[0])
        self.twin.injected_voltage_dv = None
        self.assertEqual(self.sample(9), [])
        self.assertTrue(self.sample()[0].startswith("POWER_GUARD scale=50 reason=normal"))

    def test_high_current_slows_down_too(self):
        self.run_line("power guard on")
        self.twin.injected_current_raw = {2: 300, 3: 170}
        self.assertTrue(self.sample()[0].startswith("POWER_GUARD scale=50 reason=corriente"))

    def test_sustained_cut_level_holds_the_arm_where_it_is(self):
        self.run_line("power guard on")
        self.run_line("sync 0 1 220")
        for _ in range(30):
            self.twin.tick()
        self.twin.injected_voltage_dv = 70
        self.assertFalse(any("HOLD" in line for line in self.sample()))
        replies = self.sample()
        self.assertTrue(any(line.startswith("POWER_GUARD_HOLD reason=voltaje v_min_dv=70") for line in replies))
        self.assertLess(self.twin.profiles[1].target, 219.0)  # ya no va a 220
        self.assertEqual(self.twin.arm_goals, {})

    def test_torque_changes_forget_old_goals(self):
        self.run_line("power guard on")
        self.run_line("sync 0 1 220")
        self.run_line("arm torque off")
        self.run_line("arm torque on")
        self.twin.injected_voltage_dv = 90
        self.sample()
        self.assertNotEqual(self.twin.profiles[1].target, 220.0)

    def test_limits_are_configurable_and_checked(self):
        self.assertEqual(self.run_line("power guard 100 85 2500 4000"),
                         ["POWER_GUARD_LIMITS slow_dv=100 hold_dv=85 slow_ma=2500 hold_ma=4000"])
        for line in ("power guard 80 85 2500 4000", "power guard 100 85 4000 2500", "power guard 100 85"):
            self.assertEqual(self.run_line(line), ["POWER_GUARD_REJECTED reason=fuera_de_rango"], line)

    def test_resting_arm_current_is_close_to_the_real_measurement(self):
        """Calibración ARM_CURRENT_MA_PER_NM: en reposo, lejos de los ~7 A de antes."""
        for _ in range(100):
            self.twin.tick()
        currents, voltage = self.twin._read_power()
        self.assertLess(sum(currents[:5]) * 6.5, 300.0)
        self.assertGreater(voltage, 110)

    def test_old_firmware_has_neither(self):
        self.twin.sync_firmware = False
        self.assertEqual(self.run_line("speed max 700"), ["ID invalido. Usa 1..6."])
        self.assertNotIn("POWER_GUARD", " ".join(self.run_line("power guard on")))


class HostGuardTests(unittest.TestCase):
    def test_scale_hold_and_capabilities_are_parsed(self):
        robot = RobotSerial("SIMULADO", 115200)
        robot._consume_line("FIRMWARE_CAPS sync=1 sync_max_motors=5 sync_max_speed=450 guard=1 speed_hard_max=1000", 1.0)
        self.assertTrue(robot.guard_supported)
        robot._consume_line("POWER_GUARD scale=50 reason=voltaje v_min_dv=90 total_ma=200", 1.1)
        self.assertEqual(robot.snapshot().speed_scale, 0.5)
        self.assertEqual(robot.snapshot().fault, "")
        robot._consume_line("ARM_SPEED max=700 acc=45", 1.2)
        self.assertEqual(robot.arm_max_speed, 700)
        robot._consume_line("POWER_GUARD=on", 1.3)
        self.assertEqual(robot.snapshot().speed_scale, 1.0)
        robot._consume_line("POWER_GUARD_HOLD reason=voltaje v_min_dv=70 total_ma=900", 1.4)
        self.assertIn("POWER_GUARD_HOLD", robot.snapshot().fault)

    def test_configuration_goes_only_to_firmware_that_announces_it(self):
        sent = []

        class Recorder(RobotSerial):
            def send(self, command):
                sent.append(command)

        robot = Recorder("SIMULADO", 115200)
        self.assertEqual(configure_motion(robot, MotionConfig(), timeout=0.05), [])
        self.assertEqual(sent, [])
        robot.guard_supported = True
        commands = configure_motion(robot, MotionConfig(max_speed=700, guard_slow_volts=10.0), timeout=0.05)
        self.assertEqual(commands, ["speed max 700 45", "power guard 100 80 3000 4500", "power guard on"])
        self.assertEqual(sent, commands)
        self.assertEqual(motion_commands(MotionConfig(guard_enabled=False))[-1], "power guard off")


class ConfigTests(unittest.TestCase):
    def test_defaults_and_ranges(self):
        motion = load_config().motion
        self.assertEqual((motion.max_speed, motion.acceleration), (450, 45))
        self.assertTrue(motion.guard_enabled)
        self.assertEqual(load_motion({}), MotionConfig())
        for bad in ({"velocidad_max": 1500}, {"aceleracion": 5},
                    {"guardia_fuente": {"lento_v": 8.0, "corte_v": 9.0}},
                    {"guardia_fuente": {"lento_ma": 5000, "corte_ma": 4000}}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                load_motion(bad)


class SlowedArmSerial(RobotSerial):
    """El supervisor bajó la velocidad a 25%: el motor llega en ~3 s, no en ~0,5 s."""

    def __init__(self):
        super().__init__("SIMULADO", 115200)
        self.torque_enabled = True
        self.speed_scale = 0.25
        self.start = None
        self._set(180.0)

    def _set(self, value):
        with self.state_lock:
            for motor in range(1, 7):
                self.positions[motor] = value if motor == 1 else 0.0
                self.received_at[motor] = time.monotonic()

    def snapshot(self):
        if self.start is not None:
            amount = min(1.0, (time.monotonic() - self.start) / 3.0)
            self._set(180.0 + 10.0 * amount)
        return super().snapshot()

    def send(self, command):
        self.start = time.monotonic()
        self._consume_line("Moviendo ID 1", time.monotonic())


class ExecutorScaleTests(unittest.TestCase):
    def test_arrival_timeout_stretches_with_the_guard_scale(self):
        robot = SlowedArmSerial()
        proposal = MotionProposal(1, "aproximacion_1", "Aproximación 1", {1: 190.0},
                                  (MotionGroup(((1, 190.0),)),), None, segments=(MotionSegment(0, 1),))
        MotionExecutor(robot, sync_orders=False).execute(proposal)  # sin estirar: 2,0 s y falla
        self.assertGreater(time.monotonic() - robot.start, 2.5)


if __name__ == "__main__":
    unittest.main()
