from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

from brazoia.config import INITIAL_POSE, KEYFRAME_STAGES

try:
    import mujoco
except ImportError:  # La simulación es opcional para el resto de la aplicación.
    mujoco = None


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(mujoco is None, "MuJoCo no está instalado")
class MujocoSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from brazoia.mujoco_sim import PickPlaceExpert, VirtualArm

        cls.arm = VirtualArm(ROOT / "assets")
        cls.expert = PickPlaceExpert(cls.arm)

    def test_logical_angle_conversion_round_trips(self):
        for motor in range(1, 7):
            for value in (0.0, 37.5, 180.0, 263.0):
                self.assertAlmostEqual(self.arm.to_logical(motor, self.arm.to_qpos(motor, value)), value, places=9)

    def test_kinematics_match_rerun_geometry(self):
        rng = np.random.default_rng(0)
        data = mujoco.MjData(self.arm.model)
        for _ in range(10):
            pose = {i: rng.uniform(*self.arm.limits[i]) for i in range(1, 7)}
            transforms, _ = self.arm.geometry.transforms(pose)
            for motor, value in pose.items():
                data.qpos[self.arm.qadr[motor]] = self.arm.to_qpos(motor, value)
            mujoco.mj_kinematics(self.arm.model, data)
            for link in ("lower_arm", "gripper", "jaw"):
                body = self.arm.model.body(link).id
                np.testing.assert_allclose(data.xpos[body], transforms[link][:3, 3], atol=1e-9)
                np.testing.assert_allclose(data.xmat[body].reshape(3, 3), transforms[link][:3, :3], atol=1e-9)

    def test_simulated_ranges_cover_current_limits_and_urdf_stops(self):
        for motor, (low, high) in self.arm.limits.items():
            simulated_low, simulated_high = self.arm.joint_limits[motor]
            self.assertLessEqual(simulated_low, low)
            self.assertGreaterEqual(simulated_high, high)
            self.assertGreaterEqual(simulated_low, 0.0)
            self.assertLessEqual(simulated_high, 360.0)
        self.assertGreater(self.arm.joint_limits[4][1], 277.3)  # Valor alcanzado en una demo real.

    def test_physics_has_no_contact_margin(self):
        # El margen de planificación vive en una copia: en la física alteraba el agarre.
        self.assertEqual(float(self.arm.model.geom_margin.max()), 0.0)
        self.assertGreater(float(self.arm.check_model.geom_margin.max()), 0.0)

    def test_initial_pose_has_no_collisions(self):
        for gripper in (0.0, 137.0):
            self.assertEqual(self.arm.pose_problems({**INITIAL_POSE, 6: gripper}), [])

    def test_detects_parts_passing_through_each_other(self):
        problems = self.arm.pose_problems({1: 180, 2: 120, 3: 275, 4: 180, 5: 263, 6: 0})
        self.assertIn("autocolisión antebrazo ↔ hombro", problems)

    def test_ik_reaches_vertical_grasp(self):
        position = [0.05, -0.24, self.arm.floor_z + 0.02]
        solution = self.arm.solve_ik(position, 0.3, list(self.expert.SEEDS))
        self.assertTrue(solution.ok, solution)
        self.assertEqual(solution.tilt_deg, 0.0)
        self.assertLessEqual(solution.position_error_mm, 1.0)

    def test_expert_completes_pick_and_place(self):
        from brazoia.mujoco_sim import run_episode, sample_placement

        rng = np.random.default_rng(3)
        for _ in range(2):
            result = run_episode(self.arm, self.expert, sample_placement(rng, self.arm.dimensions))
            self.assertTrue(result.success, result.reasons)
            self.assertEqual([record.stage for record in result.records], list(KEYFRAME_STAGES))
            self.assertEqual(result.records[KEYFRAME_STAGES.index("agarre")].gripper_state, "holding")
            self.assertGreater(result.minimum_keyframe_clearance_mm, 5.0)
            self.assertEqual(result.contacts, {})

    def test_current_rules_are_reported_not_enforced(self):
        from brazoia.mujoco_sim import current_rule_warnings

        warnings = current_rule_warnings({1: 153.6, 2: 215.5, 3: 150.2, 4: 279.0, 5: 263.4, 6: 37.6},
                                         self.arm.limits)
        self.assertTrue(any(warning.startswith("M4=279.0°") for warning in warnings))

    def test_simulation_does_not_import_serial_controller(self):
        code = ("import sys, brazoia.mujoco_sim, simulacion_mujoco; "
                "print(int('serial' in sys.modules or 'brazoia.robot' in sys.modules))")
        output = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, check=True)
        self.assertEqual(output.stdout.strip(), "0")


if __name__ == "__main__":
    unittest.main()
