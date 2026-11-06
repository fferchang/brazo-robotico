"""Botón HACER LA TAREA SOLA: límites antes de arrancar y lo que queda si se bloquea."""
from __future__ import annotations

import types
import unittest
from unittest.mock import Mock

import tkinter as tk

from app import App
from brazoia.politica_geometrica import MAX_CUBE_DISTANCE_CM, cube_distance_problem


class CubeDistanceTests(unittest.TestCase):
    def test_only_cubes_beyond_the_limit_are_rejected(self) -> None:
        self.assertIsNone(cube_distance_problem({"cubo_a_la_base_cm": 24.8}))
        self.assertIsNone(cube_distance_problem({"cubo_a_la_base_cm": MAX_CUBE_DISTANCE_CM}))
        self.assertIn("33 cm", cube_distance_problem({"cubo_a_la_base_cm": 33.4}))
        self.assertIsNone(cube_distance_problem({"tipo": "aprendido"}))

    def test_scene_problem_uses_the_policy_summary_and_ignores_learned_models(self) -> None:
        far = types.SimpleNamespace(policy=types.SimpleNamespace(summary=lambda features: {"cubo_a_la_base_cm": 34.0}),
                                    scene_features=[0.0])
        self.assertIn("34 cm", App._scene_problem(None, far))
        learned = types.SimpleNamespace(policy=object(), scene_features=[0.0])
        self.assertIsNone(App._scene_problem(None, learned))

        def broken(features):
            raise ValueError("No hay poses alcanzables")

        failing = types.SimpleNamespace(policy=types.SimpleNamespace(summary=broken), scene_features=[0.0])
        self.assertIsNone(App._scene_problem(None, failing))


class AutonomousFailureTests(unittest.TestCase):
    def ui(self, stop: bool, emergency: bool = False):
        auto_stop = Mock(is_set=Mock(return_value=stop))
        execution = types.SimpleNamespace(finished=False, next_stage_index=6)
        return types.SimpleNamespace(execution_busy=True, auto_stop=auto_stop, execution=execution,
                                     emergency_active=emergency, auto_status=Mock(), log=Mock())

    def test_emergency_message_stays_and_does_not_offer_to_resume(self) -> None:
        ui = self.ui(stop=True, emergency=True)
        App._autonomous_failed(ui, RuntimeError("Movimiento detenido por el operador."))
        text = ui.auto_status.set.call_args.args[0]
        self.assertTrue(text.startswith("EMERGENCIA"))
        self.assertNotIn("Hacer la tarea sola", text)

    def test_a_blocked_stage_says_where_the_task_stayed(self) -> None:
        ui = self.ui(stop=False)
        App._autonomous_failed(ui, RuntimeError("Sin confirmación de llegada"))
        text = ui.auto_status.set.call_args.args[0]
        self.assertFalse(ui.execution_busy)
        self.assertIn("BLOQUEADO: Sin confirmación de llegada", text)
        self.assertIn("Agarre", text)

    def test_stop_by_operator_is_not_reported_as_a_block(self) -> None:
        ui = self.ui(stop=True)
        App._autonomous_failed(ui, RuntimeError("Movimiento detenido por el operador."))
        self.assertTrue(ui.auto_status.set.call_args.args[0].startswith("Detenido por el operador."))


class InicioSceneTests(unittest.TestCase):
    """La foto con la que se planifica todo: actual y con el cubo fuera del recipiente."""

    def ui(self):
        from brazoia.config import load_config

        return types.SimpleNamespace(config=load_config())

    def pair(self, frame, age: float = 0.0):
        import time

        from brazoia.capture import FrameSnapshot

        stamp = time.monotonic() - age
        return [FrameSnapshot(frame, stamp, 1, index, 180, "OK") for index in range(2)]

    def test_rejects_old_frames_and_a_cube_already_inside(self) -> None:
        from tests.test_registro_ejecucion import observe, scene

        outside, inside = scene(outside=True), scene(inside=True)
        ui = self.ui()
        self.assertIsNone(App._inicio_problem(ui, self.pair(outside), observe([outside, outside])))
        self.assertIn("imagen actual", App._inicio_problem(ui, self.pair(outside, age=5.0),
                                                           observe([outside, outside])))
        self.assertIn("ya está dentro", App._inicio_problem(ui, self.pair(inside), observe([inside, inside])))
        self.assertIn("detección", App._inicio_problem(ui, self.pair(outside), observe([outside])))


class ExternalAbortTests(unittest.TestCase):
    def test_a_stop_requested_between_stages_is_not_lost(self) -> None:
        import threading

        from brazoia.execution import MotionExecutor, MotionProposal
        from brazoia.planner import MotionGroup

        robot = Mock()
        robot.snapshot.return_value = types.SimpleNamespace(fault="", torque_enabled=True, positions={1: 180.0})
        abort = threading.Event()
        executor = MotionExecutor(robot, abort=abort)
        abort.set()  # Detener entre dos etapas, antes de que execute() borre su propia señal
        proposal = MotionProposal(1, "aproximacion_1", "Aproximación 1/3", {1: 150.0},
                                  (MotionGroup(((1, 150.0),)),), None)
        with self.assertRaisesRegex(RuntimeError, "detenido por el operador"):
            executor.execute(proposal)
        robot.send.assert_not_called()


class AutonomousButtonTests(unittest.TestCase):
    def test_button_exists_and_needs_cameras_and_robot(self) -> None:
        root = tk.Tk()
        root.withdraw()
        try:
            ui = App(root)
            root.update_idletasks()
            pending, buttons = [root], []
            while pending:
                widget = pending.pop()
                pending.extend(widget.winfo_children())
                if isinstance(widget, tk.Button) and "HACER LA TAREA SOLA" in widget.cget("text"):
                    buttons.append(widget)
            self.assertEqual(len(buttons), 1)
            buttons[0].invoke()
            self.assertIn("conectá el robot", ui.status.get())
            self.assertFalse(ui.execution_busy)
            self.assertIsNone(ui.execution)
        finally:
            for timer in root.tk.call("after", "info"):
                root.after_cancel(timer)
            root.destroy()


if __name__ == "__main__":
    unittest.main()
