"""Demo sin recipiente: el cubo de un punto de la mesa al otro, en loop."""
from __future__ import annotations

import time
import types
import unittest

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
