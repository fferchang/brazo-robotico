"""Política geométrica: calibración exacta + experto de cinemática inversa (MuJoCo)."""
from __future__ import annotations

import dataclasses
import tempfile
import unittest
from pathlib import Path

import numpy as np

from brazoia.calibracion_camaras import fit_calibration, settings_from_config
from brazoia.config import INITIAL_POSE, KEYFRAME_STAGES, load_config
from brazoia.execution import SupervisedExecution
from brazoia.gripper_geometry import model_fingerprint
from brazoia.mujoco_sim import GRIPPER_HOLD_ESTIMATE, PickPlaceExpert, Placement, VirtualArm
from brazoia.politica_geometrica import GeometricPolicy, calibration_path, load_geometric_policy, object_center
from brazoia.vision import Detection, SceneObservation
from tests.test_calibracion_camaras import FRONTAL, LATERAL, project, samples_for, scene_features, workspace_points


CONFIG = load_config()
PLACEMENT = Placement((0.08, -0.25), 0.0, (-0.12, -0.22), 0.0)


def observations_for(placement):
    result = []
    for matrix in (FRONTAL, LATERAL):
        cube = Detection("red_cube", *project(matrix, placement.cube_xy), 0.05, 0.05, 0.003, 0.9, 1.0, 0.9, (0, 0, 1, 1))
        box = Detection("lime_container", *project(matrix, placement.container_xy), 0.1, 0.1, 0.01, 0.9, 0.8, 0.9,
                        (0, 0, 1, 1))
        result.append(SceneObservation(cube, box))
    return result


class GeometricPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.arm = VirtualArm(limits=CONFIG.limits)
        cls.expert = PickPlaceExpert(cls.arm)
        cls.plan = cls.expert.plan(PLACEMENT)
        points = workspace_points(14)
        cls.model = model_fingerprint(cls.arm.assets_root)
        cls.calibration = fit_calibration(samples_for(points) + samples_for(points, "recipiente"),
                                          settings_from_config(CONFIG.cameras), model=cls.model)
        cls.policy = GeometricPolicy(cls.calibration, CONFIG.limits, arm=cls.arm, expert=cls.expert)
        cls.scene = scene_features(PLACEMENT.cube_xy, PLACEMENT.container_xy)

    def test_object_center_comes_back_from_the_expert_poses(self) -> None:
        grasp = self.plan.keyframes[KEYFRAME_STAGES.index("agarre")].pose
        release = self.plan.keyframes[KEYFRAME_STAGES.index("soltado")].pose
        # El experto deja 3 mm de holgura contra el dedo fijo antes de cerrar.
        np.testing.assert_allclose(object_center(self.arm, {**grasp, 6: GRIPPER_HOLD_ESTIMATE}),
                                   PLACEMENT.cube_xy, atol=0.005)
        np.testing.assert_allclose(object_center(self.arm, {**release, 6: GRIPPER_HOLD_ESTIMATE}),
                                   PLACEMENT.container_xy, atol=0.005)

    def test_policy_returns_the_expert_plan_for_the_located_scene(self) -> None:
        placement, _ = self.policy.plan(self.scene)
        np.testing.assert_allclose(placement.cube_xy, PLACEMENT.cube_xy, atol=1e-3)
        np.testing.assert_allclose(placement.container_xy, PLACEMENT.container_xy, atol=1e-3)
        for index in range(1, len(KEYFRAME_STAGES) - 1):
            predicted = self.policy.predict(index, self.scene)
            expected = self.plan.keyframes[index].pose
            for motor in range(1, 6):
                self.assertAlmostEqual(predicted[motor], float(np.clip(expected[motor], *CONFIG.limits[motor])),
                                       delta=1.0, msg=f"{KEYFRAME_STAGES[index]} M{motor}")
        np.testing.assert_allclose(self.policy.canonical_start, [INITIAL_POSE[m] for m in range(1, 6)])
        self.assertTrue(self.policy.via(KEYFRAME_STAGES.index("descenso_abierto"), self.scene))

    def test_supervised_execution_goes_through_the_vertical_descent(self) -> None:
        execution = SupervisedExecution(self.policy, CONFIG.limits)
        current = {**INITIAL_POSE, 6: 0.0}
        observations = observations_for(PLACEMENT)
        while True:
            proposal = execution.prepare(current, observations)
            if proposal.stage == "descenso_abierto":
                break
            execution.mark_completed()
            current = {**current, **proposal.target}
        via = self.policy.via(proposal.stage_index, execution.scene_features)
        visited, pose = [], dict(current)
        for group in proposal.groups:
            pose.update(group.targets)
            visited.append(dict(pose))
        for waypoint in via:
            self.assertTrue(any(all(abs(item[m] - waypoint[m]) <= 0.6 for m in range(1, 6)) for item in visited),
                            waypoint)
        self.assertTrue(all(abs(pose[m] - proposal.target[m]) <= 0.6 for m in range(1, 6)))

    def test_scene_outside_the_calibrated_zone_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "fuera de la zona calibrada"):
            self.policy.predict(3, scene_features((0.40, -0.25), PLACEMENT.container_xy))

    def test_loading_requires_a_matching_calibration(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            config = dataclasses.replace(CONFIG, root=Path(temporary))
            with self.assertRaisesRegex(ValueError, "Falta la calibración"):
                load_geometric_policy(config, arm=self.arm, expert=self.expert)
            self.calibration.save(calibration_path(config))
            self.assertIsInstance(load_geometric_policy(config, arm=self.arm, expert=self.expert), GeometricPolicy)
            cameras = dict(CONFIG.cameras)
            cameras["lateral"] = dataclasses.replace(cameras["lateral"], rotation_degrees=0)
            with self.assertRaisesRegex(ValueError, "no coincide"):
                load_geometric_policy(dataclasses.replace(config, cameras=cameras), arm=self.arm, expert=self.expert)

    def test_loading_refuses_a_calibration_from_another_3d_model(self) -> None:
        # Con otro modelo (por ejemplo, M1 sin invertir) la misma foto daría el lado espejado.
        with tempfile.TemporaryDirectory() as temporary:
            config = dataclasses.replace(CONFIG, root=Path(temporary))
            for model, message in ((None, "no registra con qué modelo 3D"), ("0123456789abcdef", "otro modelo 3D")):
                dataclasses.replace(self.calibration, model=model).save(calibration_path(config))
                with self.assertRaisesRegex(ValueError, message):
                    load_geometric_policy(config, arm=self.arm, expert=self.expert)


class AppModeTests(unittest.TestCase):
    """Selector de modo de la ejecución automática, sin cámaras ni ESP32."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.arm = VirtualArm(limits=CONFIG.limits)
        cls.expert = PickPlaceExpert(cls.arm)

    def ui(self, root, mode, policy=None):
        from types import SimpleNamespace
        from unittest.mock import Mock

        return SimpleNamespace(automatic_mode=Mock(get=Mock(return_value=mode)), config=dataclasses.replace(CONFIG, root=root),
                               geometric_model=(self.arm, self.expert), policy=policy, status=Mock(), log=Mock())

    def test_geometric_mode_needs_a_saved_calibration(self) -> None:
        from app import App

        with tempfile.TemporaryDirectory() as temporary:
            ui = self.ui(Path(temporary), "geometrico")
            self.assertIsNone(App.automatic_policy(ui))
            self.assertIn("Falta la calibración", ui.status.set.call_args.args[0])
            points = workspace_points(12)
            fit_calibration(samples_for(points), settings_from_config(CONFIG.cameras),
                            model=model_fingerprint(self.arm.assets_root)).save(calibration_path(ui.config))
            self.assertIsInstance(App.automatic_policy(ui), GeometricPolicy)

    def test_learned_mode_keeps_requiring_a_model(self) -> None:
        from app import App

        with tempfile.TemporaryDirectory() as temporary:
            ui = self.ui(Path(temporary), "aprendido")
            self.assertIsNone(App.automatic_policy(ui))
            self.assertIn("Entrená", ui.status.set.call_args.args[0])
            model = object()
            ui.policy = model
            self.assertIs(App.automatic_policy(ui), model)

    def test_calibration_scene_requires_both_cameras(self) -> None:
        from types import SimpleNamespace

        from app import App

        ui = SimpleNamespace(cameras=[], snapshots=[], observations=[], config=CONFIG)
        with self.assertRaisesRegex(ValueError, "dos cámaras"):
            App.calibration_scene(ui)


if __name__ == "__main__":
    unittest.main()
