"""Preparación anticipada de Guardado: plan válido para una banda de M6 y adopción segura."""
from __future__ import annotations

import unittest

from brazoia.calibracion_camaras import fit_calibration, settings_from_config
from brazoia.config import INITIAL_POSE, KEYFRAME_STAGES, load_config
from brazoia.execution import (
    ANTICIPATED_POSE_TOLERANCE,
    GRIPPER_OPEN_TOLERANCE,
    SupervisedExecution,
    _independent_policy,
)
from brazoia.gripper_geometry import model_fingerprint
from brazoia.mujoco_sim import PickPlaceExpert, Placement, VirtualArm
from brazoia.politica_geometrica import GeometricPolicy
from tests.test_calibracion_camaras import samples_for, workspace_points
from tests.test_politica_geometrica import observations_for

CONFIG = load_config()
PLACEMENT = Placement((0.08, -0.25), 0.0, (-0.12, -0.22), 0.0)
GUARDADO = KEYFRAME_STAGES.index("guardado")


def make_policy():
    arm = VirtualArm(limits=CONFIG.limits)
    calibration = fit_calibration(samples_for(workspace_points(14)) + samples_for(workspace_points(14), "recipiente"),
                                  settings_from_config(CONFIG.cameras), model=model_fingerprint(arm.assets_root))
    return GeometricPolicy(calibration, CONFIG.limits, arm=arm, expert=PickPlaceExpert(arm))


class AnticipatedGuardadoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy = make_policy()
        cls.observations = observations_for(PLACEMENT)

    def run_until_guardado(self, sync=True):
        """Prepara las etapas 1–11 como si el brazo llegara justo a cada destino; el plan
        anticipado arranca después de la etapa 1, como en la app."""
        execution = SupervisedExecution(self.policy, CONFIG.limits, sync_orders=sync)
        current = {**INITIAL_POSE, 6: 0.0}
        ahead = None
        while execution.next_stage_index < GUARDADO:
            proposal = execution.prepare(current, self.observations)
            if proposal.stage_index == 1:
                ahead = execution.start_ahead(self.observations)
            execution.mark_completed()
            for group in proposal.groups:
                current.update(dict(group.targets))
            if proposal.gripper_action == "angle":
                current[6] = proposal.gripper_degrees
            elif proposal.gripper_action == "close":
                current[6] = 40.0
        return execution, ahead, current

    def test_the_anticipated_plan_matches_a_normal_plan_from_the_same_pose(self) -> None:
        execution, ahead, current = self.run_until_guardado()
        self.assertIsNotNone(ahead)
        proposal, detail = execution.adopt_ahead(ahead, current)
        self.assertIsNotNone(proposal, detail)
        self.assertEqual(proposal.stage, "guardado")
        normal = SupervisedExecution(self.policy, CONFIG.limits, sync_orders=True)
        normal.scene_features, normal.next_stage_index = execution.scene_features, GUARDADO
        reference = normal.prepare(current, self.observations)
        # Mismos puntos de destino y misma acción de garra; la banda de M6 solo agrega revisiones.
        self.assertEqual(proposal.target, reference.target)
        self.assertEqual(proposal.gripper_action, reference.gripper_action)
        self.assertEqual(proposal.groups[-1], reference.groups[-1])
        self.assertTrue(proposal.segments[-1].final)

    def test_pose_or_gripper_off_the_assumption_is_replanned(self) -> None:
        cases = {
            "brazo": lambda pose: pose.__setitem__(2, pose[2] + ANTICIPATED_POSE_TOLERANCE + 0.2),
            "garra": lambda pose: pose.__setitem__(6, pose[6] + GRIPPER_OPEN_TOLERANCE + 0.5),
            "lectura": lambda pose: pose.pop(4),
        }
        for name, change in cases.items():
            with self.subTest(name):
                execution, ahead, current = self.run_until_guardado()
                change(current)
                proposal, detail = execution.adopt_ahead(ahead, current)
                self.assertIsNone(proposal, name)
                self.assertIsNone(execution.proposal)
                self.assertTrue(detail)

    def test_small_deviations_inside_the_tolerances_are_accepted(self) -> None:
        execution, ahead, current = self.run_until_guardado()
        current = {**current, 1: current[1] + ANTICIPATED_POSE_TOLERANCE * 0.9,
                   6: current[6] - GRIPPER_OPEN_TOLERANCE * 0.9}
        proposal, detail = execution.adopt_ahead(ahead, current)
        self.assertIsNotNone(proposal, detail)
        self.assertIs(execution.proposal, proposal)

    def test_only_guardado_is_anticipated_and_never_twice(self) -> None:
        execution = SupervisedExecution(self.policy, CONFIG.limits)
        self.assertIsNone(execution.start_ahead(self.observations))  # todavía no hay escena
        execution.prepare({**INITIAL_POSE, 6: 0.0}, self.observations)
        self.assertIsNone(execution.start_ahead(self.observations, stage="soltado"))
        plan = execution.start_ahead(self.observations)
        self.assertIsNotNone(plan)
        plan.thread.join(30)
        # Etapa equivocada: no se adopta.
        self.assertEqual(execution.adopt_ahead(plan, {**INITIAL_POSE, 6: 90.0})[0], None)


class BandHelpersTests(unittest.TestCase):
    def setUp(self):
        self.execution = SupervisedExecution(make_policy(), CONFIG.limits)

    def test_without_a_band_nothing_changes(self) -> None:
        ex = self.execution
        self.assertEqual(ex._static_m6_values(90.0), (90.0,))
        self.assertEqual(ex._rules_m6(90.0), 90.0)
        self.assertEqual(ex._sweep_m6_values(90.0, 0.0), ex._gripper_sweep_samples(90.0, 0.0))

    def test_band_samples_cover_the_extremes_and_the_worst_envelope(self) -> None:
        ex = self.execution
        ex._m6_band = (84.0, 96.0)
        values = ex._static_m6_values(90.0)
        self.assertEqual((values[0], values[-1]), (84.0, 96.0))
        self.assertTrue(all(b - a <= 8.0 + 1e-9 for a, b in zip(values, values[1:])))
        self.assertEqual(ex._rules_m6(90.0), 90.0)  # misma forma de garra en toda la banda
        sweep = ex._sweep_m6_values(90.0, 0.0)
        self.assertEqual((min(sweep), max(sweep)), (0.0, 96.0))
        ex._m6_band = (40.0, 52.0)  # cruza el límite de contorno de 47°
        self.assertIsNone(ex._rules_m6(46.0))

    def test_the_copy_uses_its_own_3d_model_with_the_same_answers(self) -> None:
        policy = self.execution.policy
        clone = _independent_policy(policy)
        self.assertIsNot(clone.arm, policy.arm)
        self.assertIsNot(clone.arm._check_data, policy.arm._check_data)
        for pose in ({**INITIAL_POSE, 6: 0.0}, {1: 180.0, 2: 250.0, 3: 160.0, 4: 150.0, 5: 263.0, 6: 90.0}):
            self.assertEqual(clone.arm.pose_problems(pose), policy.arm.pose_problems(pose))


if __name__ == "__main__":
    unittest.main()
