"""Demo sin recipiente: el cubo de un punto de la mesa al otro, en loop."""
from __future__ import annotations

import time
import types
import unittest
import unittest.mock

import mujoco
import numpy as np
import tkinter as tk

from app import App
from brazoia.calibracion_camaras import fit_calibration, settings_from_config
from brazoia.config import INITIAL_POSE, KEYFRAME_STAGES, load_config
from brazoia.demo import (
    DEMO_SPOTS,
    RELEASE_DROPS,
    DemoPolicy,
    TablePlaceExpert,
    arrival_error_mm,
    farther_spot,
    twin_demo_placement,
)
from brazoia.execution import SupervisedExecution
from brazoia.gripper_geometry import model_fingerprint
from brazoia.mujoco_sim import GRIPPER_HOLD_ESTIMATE, Placement, VirtualArm
from brazoia.planner import GRIPPER_OPEN_TARGET, gripper_sweep_problem
from brazoia.politica_geometrica import object_center
from brazoia.vision import Detection, SceneObservation
from tests.test_calibracion_camaras import FRONTAL, LATERAL, project, samples_for, scene_features, workspace_points

CONFIG = load_config()
NO_CONTAINER = (("frontal", "recipiente"), ("lateral", "recipiente"))
CUBE = (DEMO_SPOTS[0][0] + 0.006, DEMO_SPOTS[0][1] - 0.008)


def cube_only(cube_xy):
    return scene_features(cube_xy, (0.0, 0.0), missing=NO_CONTAINER)


def observations_without_container(cube_xy):
    result = []
    for matrix in (FRONTAL, LATERAL):
        cube = Detection("red_cube", *project(matrix, cube_xy), 0.05, 0.05, 0.003, 0.9, 1.0, 0.9, (0, 0, 1, 1))
        result.append(SceneObservation(cube, None))
    return result


def pinza(arm, pose):
    data = mujoco.MjData(arm.model)
    for motor in range(1, 6):
        data.qpos[arm.qadr[motor]] = arm.to_qpos(motor, pose[motor])
    mujoco.mj_kinematics(arm.model, data)
    return data.site_xpos[arm.site_id].copy()


class DestinationTests(unittest.TestCase):
    def test_the_cube_goes_to_the_farther_spot(self) -> None:
        first, second = DEMO_SPOTS
        self.assertEqual(farther_spot(first), second)
        self.assertEqual(farther_spot(second), first)
        self.assertEqual(farther_spot((first[0] + 0.03, first[1])), second)
        # Movido a mano cerca del segundo punto: vuelve al primero.
        self.assertEqual(farther_spot((second[0] - 0.02, second[1] + 0.01)), first)

    def test_spots_are_inside_the_reach_that_works_on_the_real_arm(self) -> None:
        for spot in DEMO_SPOTS:
            self.assertTrue(0.18 <= float(np.hypot(*spot)) <= 0.29, spot)

    def test_twin_scene_puts_the_container_away_and_the_cube_near_a_spot(self) -> None:
        placement = twin_demo_placement(np.random.default_rng(4))
        self.assertGreater(float(np.hypot(*placement.container_xy)), 1.0)
        self.assertLess(min(np.hypot(placement.cube_xy[0] - x, placement.cube_xy[1] - y) for x, y in DEMO_SPOTS),
                        0.02)


class TablePlaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.arm = VirtualArm(limits=CONFIG.limits)
        cls.expert = TablePlaceExpert(cls.arm)
        cls.placement = Placement(CUBE, 0.0, farther_spot(CUBE), 0.0)
        cls.plan = cls.expert.plan(cls.placement)
        calibration = fit_calibration(samples_for(workspace_points(14)), settings_from_config(CONFIG.cameras),
                                      model=model_fingerprint(cls.arm.assets_root))
        cls.policy = DemoPolicy(calibration, CONFIG.limits, arm=cls.arm, expert=cls.expert)

    def stage(self, name):
        return self.plan.keyframes[KEYFRAME_STAGES.index(name)]

    def test_release_lowers_the_cube_onto_the_destination(self) -> None:
        self.assertEqual(self.plan.warnings, [])
        release, above = self.stage("soltado"), self.stage("traslado_3")
        np.testing.assert_allclose(object_center(self.arm, {**release.pose, 6: GRIPPER_HOLD_ESTIMATE}),
                                   self.placement.container_xy, atol=0.005)
        low = pinza(self.arm, release.pose)[2] - self.arm.floor_z
        self.assertLessEqual(low, self.expert.grasp_height + max(RELEASE_DROPS) + 0.001)
        self.assertGreater(pinza(self.arm, above.pose)[2] - self.arm.floor_z, low + 0.05)
        # Baja en vertical: todos los puntos previos sobre el destino.
        for pose in release.via:
            np.testing.assert_allclose(pinza(self.arm, pose)[:2], pinza(self.arm, release.pose)[:2], atol=0.002)
        # Abrir ahí pasa la misma revisión de mesa que hace la ejecución.
        self.assertIsNone(gripper_sweep_problem((release.pose[2], release.pose[3], release.pose[4]), None,
                                                GRIPPER_OPEN_TARGET))

    def test_guardado_first_rises_straight_up_with_the_gripper_open(self) -> None:
        release, guard = self.stage("soltado"), self.stage("guardado")
        self.assertEqual(guard.pose, dict(INITIAL_POSE))
        self.assertGreaterEqual(len(guard.via), 2)
        start = pinza(self.arm, release.pose)
        heights = [pinza(self.arm, pose)[2] for pose in guard.via]
        self.assertEqual(heights, sorted(heights))
        np.testing.assert_allclose(pinza(self.arm, guard.via[0])[:2], start[:2], atol=0.002)

    def test_policy_locates_only_the_cube_and_runs_all_stages(self) -> None:
        features = cube_only(CUBE)
        placement, _ = self.policy.plan(features)
        np.testing.assert_allclose(placement.cube_xy, CUBE, atol=1e-3)
        self.assertEqual(placement.container_xy, farther_spot(CUBE))
        summary = self.policy.summary(features)
        self.assertEqual(summary["tipo"], "demo")
        self.assertAlmostEqual(summary["traslado_cm"], 19.0, delta=1.5)
        self.assertLess(arrival_error_mm(self.policy, cube_only(placement.container_xy), placement.container_xy), 1.0)

        execution = SupervisedExecution(self.policy, CONFIG.limits)
        current = {**INITIAL_POSE, 6: 0.0}
        observations = observations_without_container(CUBE)
        stages = []
        while not execution.finished:
            proposal = execution.prepare(current, observations)
            stages.append(proposal.stage)
            execution.mark_completed()
            current = {**current, **proposal.target}
        self.assertEqual(stages, list(KEYFRAME_STAGES[1:]))


    def test_transfer_skip_keeps_middle_stage_if_carried_cube_check_rejects(self) -> None:
        samples = []

        def clears_table():
            samples.append(False)
            return False

        execution = SupervisedExecution(
            self.policy, CONFIG.limits, direct_demo=True, carried_cube_check=lambda: clears_table,
        )
        execution.scene_features = cube_only(CUBE)
        execution.next_stage_index = KEYFRAME_STAGES.index("traslado_2")
        current = {**self.policy.predict(execution.next_stage_index - 1, execution.scene_features),
                   6: GRIPPER_HOLD_ESTIMATE}
        proposal = execution.prepare(current, [])
        self.assertTrue(samples)
        self.assertEqual(proposal.stage, "traslado_2")
        execution.mark_completed()
        self.assertEqual(KEYFRAME_STAGES[execution.next_stage_index], "traslado_3")


class DemoInicioTests(unittest.TestCase):
    def ui(self):
        return types.SimpleNamespace(config=CONFIG)

    def pair(self, age=0.0):
        from brazoia.capture import FrameSnapshot

        frame = np.zeros((480, 640, 3), np.uint8)
        return [FrameSnapshot(frame, time.monotonic() - age, 1, index, 180, "OK") for index in range(2)]

    def test_needs_current_frames_and_the_cube_in_both_cameras(self) -> None:
        seen = observations_without_container(CUBE)
        self.assertIsNone(App._demo_inicio_problem(self.ui(), self.pair(), seen))
        self.assertIn("imagen actual", App._demo_inicio_problem(self.ui(), self.pair(age=5.0), seen))
        hidden = [seen[0], SceneObservation(None, None)]
        self.assertIn("lateral no ve el cubo", App._demo_inicio_problem(self.ui(), self.pair(), hidden))


class DemoRetryTests(unittest.TestCase):
    """Tras una vuelta fallida, la demo se recupera sola y sigue (con un tope de fallas)."""

    def fake(self, outcomes, *, fault=None):
        import queue
        import threading

        calls = {"cycles": 0, "recover": []}

        def cycle(_policy, _who, _cycle, _cycles, done, *_rest):
            calls["cycles"] += 1
            outcome = outcomes.pop(0) if outcomes else None
            if outcome is not None:
                raise outcome
            done[0] += 1

        ui = types.SimpleNamespace(
            config=types.SimpleNamespace(root=CONFIG.root), twin=None, auto_stop=threading.Event(),
            emergency_active=False, operator_stop=False, worker_messages=queue.Queue(),
            robot=types.SimpleNamespace(snapshot=lambda: types.SimpleNamespace(fault=fault)),
            _set_auto_status_from_worker=lambda _text: None,
        )
        ui._demo_cycle = cycle
        ui._demo_recover = lambda _policy, rejected: calls["recover"].append(rejected)
        return ui, calls

    def test_retries_after_failures_and_counts_only_complete_laps(self) -> None:
        from app import _Rejected

        ui, calls = self.fake([RuntimeError("bloqueo"), _Rejected("no ve el cubo")])
        done = [0]
        App._demo_cycles(ui, None, "Brazo", 2, done, retry=True)
        self.assertEqual(done[0], 2)
        self.assertEqual(calls["cycles"], 4)
        self.assertEqual(calls["recover"], [False, True])

    def test_gives_up_after_too_many_failures_in_a_row(self) -> None:
        from app import DEMO_MAX_RETRIES

        ui, calls = self.fake([RuntimeError("bloqueo")] * (DEMO_MAX_RETRIES + 1))
        with self.assertRaisesRegex(RuntimeError, "seguidas"):
            App._demo_cycles(ui, None, "Brazo", 0, [0], retry=True)
        self.assertEqual(len(calls["recover"]), DEMO_MAX_RETRIES)

    def test_no_retry_when_disabled_stopped_or_with_a_fault(self) -> None:
        ui, calls = self.fake([RuntimeError("bloqueo")])
        with self.assertRaises(RuntimeError):
            App._demo_cycles(ui, None, "Brazo", 1, [0], retry=False)
        ui, calls = self.fake([RuntimeError("Detenido por el operador.")])
        ui.auto_stop.set()
        with self.assertRaises(RuntimeError):
            App._demo_cycles(ui, None, "Brazo", 1, [0], retry=True)
        ui, calls = self.fake([RuntimeError("sobrecorriente")], fault="M6 sobrecorriente")
        with self.assertRaises(RuntimeError):
            App._demo_cycles(ui, None, "Brazo", 1, [0], retry=True)
        self.assertEqual(calls["recover"], [])


    def test_needs_hands_is_never_retried(self) -> None:
        from app import _NeedsHands

        ui, calls = self.fake([_NeedsHands("el brazo no vuelve a Inicio")])
        with self.assertRaises(_NeedsHands):
            App._demo_cycles(ui, None, "Brazo", 0, [0], retry=True)
        self.assertEqual(calls["recover"], [])

    def test_twin_cube_loss_is_retried_but_an_operator_stop_is_kept(self) -> None:
        from app import _CubeLost

        ui, calls = self.fake([_CubeLost("se le cayó")])
        ui.auto_stop.set()  # lo puso el monitor del cubo del gemelo
        done = [0]
        App._demo_cycles(ui, None, "Gemelo", 1, done, retry=True)
        self.assertEqual((done[0], calls["recover"]), (1, [False]))
        ui, calls = self.fake([_CubeLost("se le cayó")])
        ui.operator_stop = True  # y además el operador tocó Detener
        ui.auto_stop.set()
        with self.assertRaises(_CubeLost):
            App._demo_cycles(ui, None, "Gemelo", 1, [0], retry=True)
        self.assertTrue(ui.auto_stop.is_set())
        self.assertEqual(calls["recover"], [])


class GripperHasObjectTests(unittest.TestCase):
    def test_only_a_contact_hold_at_holding_angle_counts(self) -> None:
        from app import gripper_has_object

        self.assertTrue(gripper_has_object("holding", "contact", 40.0))
        self.assertFalse(gripper_has_object("holding", "closed_limit", 1.2))  # cerró en vacío
        self.assertFalse(gripper_has_object("holding", "contact", 1.2))
        self.assertFalse(gripper_has_object("holding", "contact", 90.0))  # abierta por ángulo después
        self.assertFalse(gripper_has_object("opening", "contact", 40.0))
        self.assertFalse(gripper_has_object("holding", "contact", None))


class FakeGripperRobot:
    """Lo mínimo del brazo para la recuperación: M6, estado de la garra y las órdenes."""

    def __init__(self, m6, state, reason, holds=3):
        import threading

        self.lock = threading.Lock()
        self.m6, self.state, self.reason, self.holds = m6, state, reason, holds
        self.sent = []

    def send(self, command):
        self.sent.append(command)
        if command == "gripper open":
            self.m6, self.state = 137.2, "opening"
        elif command == "gripper close":
            self.m6, self.state, self.reason, self.holds = 1.2, "holding", "closed_limit", self.holds + 1

    def snapshot(self):
        return types.SimpleNamespace(positions={6: self.m6}, gripper_state=self.state,
                                     gripper_hold_reason=self.reason, gripper_holds=self.holds, fault="")


class DemoRecoverTests(unittest.TestCase):
    """La recuperación suelta el cubo solo si lo sostiene y sube antes de girar la base."""

    LOW = {1: 150.0, 2: 189.5, 3: 191.9, 4: 248.7, 5: 226.4}   # agarre (garra a ~28 mm)
    HIGH = {1: 160.0, 2: 150.0, 3: 175.0, 4: 255.0, 5: 240.0}  # traslado

    def recover(self, robot, pose):
        import threading

        calls = []
        policy = types.SimpleNamespace(canonical_start=[INITIAL_POSE[m] for m in range(1, 6)])
        ui = types.SimpleNamespace(
            robot=robot, config=CONFIG, twin=None, auto_stop=threading.Event(),
            _wait_stable_pose=lambda _robot: {**pose, 6: robot.m6},
            _demo_lift=lambda _policy, current, m6: calls.append(("subir", round(m6))),
            executor=types.SimpleNamespace(execute=lambda proposal: calls.append(("a_inicio", proposal.stage))),
        )
        with unittest.mock.patch("app.DEMO_RETRY_SECONDS", 0.01):
            App._demo_recover(ui, policy, rejected=False)
        return calls

    def test_empty_closed_gripper_is_not_opened_and_rises_before_turning(self) -> None:
        robot = FakeGripperRobot(1.2, "holding", "closed_limit")  # «cerró hasta el tope sin agarrar»
        calls = self.recover(robot, self.LOW)
        self.assertEqual(robot.sent, [])
        self.assertEqual(calls, [("subir", 1), ("a_inicio", "recuperacion_inicio")])

    def test_held_cube_is_released_then_the_gripper_rises_returns_and_closes(self) -> None:
        robot = FakeGripperRobot(40.0, "holding", "contact")
        calls = self.recover(robot, self.HIGH)
        self.assertEqual(robot.sent[0], "gripper open")
        self.assertEqual(robot.sent[-1], "gripper close")
        self.assertEqual(calls, [("subir", 137), ("a_inicio", "recuperacion_inicio")])

    def test_closing_on_something_at_inicio_stops_the_demo(self) -> None:
        robot = FakeGripperRobot(137.2, "opening", "contact")
        original = robot.send

        def send(command):
            original(command)
            if command == "gripper close":
                robot.m6, robot.reason = 30.0, "contact"  # algo entre los dedos

        robot.send = send
        with self.assertRaisesRegex(RuntimeError, "volvió a agarrar"):
            self.recover(robot, self.HIGH)


class DemoLiftTests(unittest.TestCase):
    def test_lift_rises_straight_up_without_turning_the_base_or_wrist(self) -> None:
        from brazoia.demo import lift_waypoints

        arm = VirtualArm(limits=CONFIG.limits)
        expert = TablePlaceExpert(arm)
        plan = expert.plan(Placement(DEMO_SPOTS[0], 0.0, DEMO_SPOTS[1], 0.0))
        grasp = dict(plan.keyframes[KEYFRAME_STAGES.index("agarre")].pose)
        points = lift_waypoints(expert, {**grasp, 6: 1.0}, 1.0)
        self.assertGreaterEqual(len(points), 3)
        self.assertAlmostEqual(pinza(arm, points[-1])[2] - arm.floor_z, 0.10, delta=0.002)
        for point in points:
            self.assertLess(abs(point[1] - grasp[1]), 1.0)
            self.assertLess(abs(point[5] - grasp[5]), 1.0)
        start_xy = pinza(arm, grasp)[:2]
        for point in points:
            self.assertLess(np.linalg.norm(pinza(arm, point)[:2] - start_xy), 0.004)  # en vertical
        self.assertEqual(lift_waypoints(expert, {**points[-1], 6: 1.0}, 1.0), [])  # ya arriba

    def test_unsolvable_lift_asks_for_hands_instead_of_retrying(self) -> None:
        import app

        policy = types.SimpleNamespace(expert=object())
        ui = types.SimpleNamespace(config=CONFIG)
        from brazoia.mujoco_sim import PlanningError

        with unittest.mock.patch("brazoia.demo.lift_waypoints", side_effect=PlanningError("sin pose")):
            with self.assertRaises(app._NeedsHands):
                App._demo_lift(ui, policy, {**INITIAL_POSE, 6: 1.0}, 1.0)


class DemoButtonTests(unittest.TestCase):
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
                if isinstance(widget, tk.Button) and "DEMO" in widget.cget("text"):
                    buttons.append(widget)
            self.assertEqual(len(buttons), 1)
            buttons[0].invoke()
            self.assertIn("conectá el robot", ui.status.get())
            self.assertFalse(ui.execution_busy)
            self.assertIsNone(ui.executor)
        finally:
            for timer in root.tk.call("after", "info"):
                root.after_cancel(timer)
            root.destroy()


if __name__ == "__main__":
    unittest.main()
