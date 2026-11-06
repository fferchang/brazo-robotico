"""Garra: revisión de piso al abrir y vigilancia de corriente al abrir y al sostener.

La vigilancia se prueba en el firmware virtual del gemelo, que imita
processGripperWatch() del firmware real; las constantes deben coincidir.
"""
from __future__ import annotations

import re
import unittest
from pathlib import Path

import numpy as np

from brazoia.config import load_config
from brazoia.execution import SupervisedExecution
from brazoia.planner import floor_clearance, gripper_sweep_problem
from brazoia.robot import is_fault_line

ROOT = Path(__file__).resolve().parents[1]
# Garra horizontal y baja, válida para las reglas: cerrada queda a 40 mm de la mesa,
# abierta (contorno de cualquier giro de M5) la barre.
LOW_HORIZONTAL = (130.0, 245.0, 220.0)
HIGH = (180.0, 90.0, 180.0)


class GripperSweepTests(unittest.TestCase):
    def test_opening_low_and_horizontal_is_rejected(self) -> None:
        self.assertGreater(floor_clearance(LOW_HORIZONTAL, 1.5), 5.0)
        problem = gripper_sweep_problem(LOW_HORIZONTAL, 1.5, 140.0)
        self.assertIsNotNone(problem)
        self.assertIn("mm de la mesa", problem)

    def test_closing_and_opening_up_high_are_allowed(self) -> None:
        self.assertIsNone(gripper_sweep_problem(LOW_HORIZONTAL, 137.0, 0.0))  # cerrar es el agarre
        self.assertIsNone(gripper_sweep_problem(HIGH, 1.5, 140.0))
        self.assertIsNone(gripper_sweep_problem(HIGH, None, 140.0))

    def test_without_m6_reading_the_whole_opening_is_checked(self) -> None:
        self.assertIsNotNone(gripper_sweep_problem(LOW_HORIZONTAL, None, 140.0))

    def test_a_stage_that_opens_there_is_not_proposed(self) -> None:
        config = load_config()

        class Policy:
            canonical_start = np.array([180.0, 180.0, 90.0, 180.0, 263.0])
            canonical_guard = canonical_start

            def predict(self, stage_index, features):
                return {1: 180.0, 2: LOW_HORIZONTAL[0], 3: LOW_HORIZONTAL[1], 4: LOW_HORIZONTAL[2], 5: 263.0}

        execution = SupervisedExecution(Policy(), config.limits)
        execution.next_stage_index = 2  # Aproximación 2/3 abre la garra
        execution.scene_features = np.zeros(32)
        current = {1: 180.0, 2: LOW_HORIZONTAL[0], 3: LOW_HORIZONTAL[1], 4: LOW_HORIZONTAL[2], 5: 263.0, 6: 1.5}
        with self.assertRaisesRegex(ValueError, "no se puede abrir la garra"):
            execution.prepare(current, [])
        self.assertIsNone(execution.proposal)


class FirmwareConstantsTests(unittest.TestCase):
    """Los umbrales de la vigilancia son los mismos en el firmware y en el gemelo."""

    def test_watch_constants_match(self) -> None:
        from brazoia import gemelo

        source = (ROOT / "firmware" / "firmware_secuencias.ino").read_text(encoding="utf-8")
        pairs = {
            "GRIPPER_WATCH_INTERVAL_MS": gemelo.GRIPPER_WATCH_INTERVAL_S * 1000,
            "GRIPPER_MOVE_INRUSH_IGNORE_MS": gemelo.GRIPPER_MOVE_INRUSH_IGNORE_S * 1000,
            "GRIPPER_MOVE_WATCH_MS": gemelo.GRIPPER_MOVE_WATCH_S * 1000,
            "GRIPPER_HARD_CURRENT_SAMPLES": gemelo.GRIPPER_HARD_CURRENT_SAMPLES,
            "GRIPPER_HOLD_LIMIT_RAW": gemelo.GRIPPER_HOLD_LIMIT_RAW,
            "GRIPPER_HOLD_LIMIT_SAMPLES": gemelo.GRIPPER_HOLD_LIMIT_SAMPLES,
            "GRIPPER_HOLD_RELIEF_DEGREES": gemelo.GRIPPER_HOLD_RELIEF_DEGREES,
            "GRIPPER_HOLD_MAX_RELIEFS": gemelo.GRIPPER_HOLD_MAX_RELIEFS,
        }
        for name, expected in pairs.items():
            match = re.search(rf"{name} = ([0-9.]+)f?;", source)
            self.assertIsNotNone(match, name)
            self.assertAlmostEqual(float(match.group(1)), float(expected), msg=name)


class VirtualGripperWatchTests(unittest.TestCase):
    def setUp(self):
        from brazoia.gemelo import DigitalTwin
        from brazoia.mujoco_sim import Placement

        self.twin = DigitalTwin(placement=Placement((0.08, -0.25), 0.0, (-0.12, -0.22), 0.0))
        self.lines = []
        original = self.twin.println

        def println(text=""):
            self.lines.append(text)
            original(text)

        self.twin.println = println

    def run_for(self, seconds):
        for _ in range(int(round(seconds / 0.01))):
            self.twin.tick()

    def send(self, command, seconds=0.0):
        self.lines.clear()
        self.twin.handle_line(command)
        self.run_for(seconds)
        return self.events()

    def events(self):
        return [line for line in self.lines if not line.startswith("POS ID")]

    def force_current(self, raw):
        self.twin._gripper_current_raw = lambda: raw

    def test_opening_where_it_would_sweep_the_table_is_refused(self):
        self.twin._planar_now = lambda: LOW_HORIZONTAL
        responses = self.send("gripper open", 0.1)
        self.assertTrue(responses[0].startswith("GRIPPER_ERROR reason=floor"))
        self.assertTrue(is_fault_line(responses[0]))
        self.assertFalse(any(line.startswith("GRIPPER_OPEN_OK") for line in responses))

    def test_hard_current_while_holding_turns_m6_off(self):
        self.send("gripper stop", 0.05)  # sostiene donde está: vigilancia de sujeción
        self.force_current(210)
        self.run_for(0.35)
        events = self.events()
        stop = next(line for line in events if line.startswith("GRIPPER_STOP_DETAIL"))
        self.assertIn("reason=hard_current state=holding", stop)
        self.assertTrue(is_fault_line(stop))
        self.assertTrue(any(line.startswith("GRIPPER_TORQUE_OFF proteccion de sobrecorriente") for line in events))
        self.assertFalse(self.twin.torque[6])

    def test_sustained_high_holding_current_loosens_then_turns_off(self):
        self.send("gripper stop", 0.05)
        start = self.twin.gripper_command
        self.force_current(180)  # sobre 170, bajo el límite duro de 200
        self.lines.clear()
        self.run_for(2.15)
        reliefs = [line for line in self.events() if line.startswith("GRIPPER_HOLD_RELIEF")]
        self.assertEqual(1, len(reliefs))
        self.assertFalse(is_fault_line(reliefs[0]))
        self.assertAlmostEqual(start + 1.5, self.twin.gripper_command, places=3)
        self.run_for(6.2)  # dos aflojadas más y después corta
        events = self.events()
        self.assertEqual(3, sum(line.startswith("GRIPPER_HOLD_RELIEF") for line in events))
        self.assertTrue(any("detail=sigue_alta_despues_de_aflojar" in line for line in events))
        self.assertFalse(self.twin.torque[6])

    def test_opening_inrush_is_ignored_but_a_jam_is_not(self):
        self.force_current(250)
        responses = self.send("gripper open", 0.2)
        self.assertTrue(any(line.startswith("GRIPPER_OPEN_OK") for line in responses))
        self.assertFalse(any("hard_current" in line for line in responses))
        self.run_for(0.3)
        self.assertTrue(any("reason=hard_current state=moving" in line for line in self.events()))

    def test_normal_holding_current_is_left_alone(self):
        self.send("gripper stop", 0.05)
        self.force_current(140)  # lo que mide el gemelo sosteniendo el cubo
        self.run_for(3.0)
        self.assertFalse(any("RELIEF" in line or "TORQUE_OFF" in line for line in self.events()))

    def test_hard_current_while_closing_cuts_torque_without_holding_first(self):
        self.send("gripper open", 6.0)
        self.send("gripper close", 0.3)
        self.force_current(230)
        self.run_for(0.1)
        events = self.events()
        self.assertTrue(any("reason=hard_current" in line for line in events))
        self.assertFalse(any(line.startswith("GRIPPER_HOLD") for line in events))
        self.assertFalse(self.twin.torque[6])
        self.assertFalse(self.twin.gripper_closing)

    def test_a_second_close_does_not_restart_the_closing(self):
        self.send("gripper open", 6.0)
        self.send("gripper close", 0.3)
        started = self.twin.gripper_started
        self.assertEqual(["GRIPPER_CLOSE_IGNORED reason=ya_esta_cerrando"], self.send("gripper close")[:1])
        self.assertEqual(started, self.twin.gripper_started)

    def test_torque_off_cancels_closing_and_watch(self):
        self.send("gripper open", 6.0)  # el gemelo arranca con la garra cerrada
        self.send("gripper close", 0.1)
        self.assertTrue(self.twin.gripper_closing)
        responses = self.send("torque off", 0.1)
        self.assertIn("GRIPPER_STOP reason=torque_off detail=se_cancelo_el_cierre", responses)
        self.assertFalse(self.twin.gripper_closing)
        self.assertEqual("none", self.twin.gripper_watch)


if __name__ == "__main__":
    unittest.main()
