import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import rerun as rr

from brazoia.gripper_geometry import SHOULDER_HEIGHT_MM, GripperGeometry, joint_angle, model_fingerprint
from brazoia.rerun_view import RerunArmView


ASSETS = Path(__file__).resolve().parents[1] / "assets"
POSE = {1: 180, 2: 180, 3: 90, 4: 180, 5: 263, 6: 0}


class GripperGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = GripperGeometry(ASSETS)

    def test_m1_turns_the_arm_like_the_real_one(self):
        # Agarre real del 29/10 (prueba_20261029_234857): con M1 = 153,4° la cámara frontal,
        # que mira al brazo de frente, muestra la garra a su derecha, que es la izquierda
        # del brazo (+x del URDF). Con el sentido viejo de M1 el modelo la ponía en −x.
        grasp = {1: 153.4, 2: 226.5, 3: 135.2, 4: 260.0, 5: 263.4, 6: 44.7}
        release = {1: 213.0, 2: 159.2, 3: 204.5, 4: 260.0, 5: 263.4, 6: 137.4}
        self.assertGreater(self.geometry.transforms(grasp)[0]["gripper"][0, 3], 0.08)
        self.assertLess(self.geometry.transforms(release)[0]["gripper"][0, 3], -0.05)

    def test_model_fingerprint_ignores_formatting_but_not_the_model(self):
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as temporary:
            copy = Path(temporary)
            (copy / "modelo").mkdir()
            urdf = (ASSETS / "modelo" / "so100.urdf").read_bytes().replace(b"\r\n", b"\n")
            (copy / "modelo" / "so100.urdf").write_bytes(urdf.replace(b"\n", b"\r\n"))
            calibration = json.loads((ASSETS / "calibracion.json").read_text(encoding="utf-8"))
            (copy / "calibracion.json").write_text(json.dumps(calibration), encoding="utf-8")
            original = model_fingerprint(ASSETS)
            self.assertEqual(model_fingerprint(copy), original)
            self.assertNotEqual(model_fingerprint(copy, shoulder_height_mm=SHOULDER_HEIGHT_MM + 1), original)
            calibration["1"]["direction"] = -float(calibration["1"]["direction"])
            (copy / "calibracion.json").write_text(json.dumps(calibration), encoding="utf-8")
            self.assertNotEqual(model_fingerprint(copy), original)

    def test_floor_reference_is_calibrated_height_below_m2_axis(self):
        # 121,9 mm según los toques de mesa del 30/10/2026 (antes, 100 mm medidos a mano).
        transforms, _ = self.geometry.transforms(POSE)
        self.assertAlmostEqual(transforms["upper_arm"][2, 3] - self.geometry.floor_z, SHOULDER_HEIGHT_MM / 1000)
        self.assertAlmostEqual(SHOULDER_HEIGHT_MM, 121.9)

    def test_transform_chain_matches_rerun_including_offsets_and_clamping(self):
        tree = rr.urdf.UrdfTree.from_file_path(ASSETS / "modelo" / "so100.urdf")
        joints = {joint.name: joint for joint in tree.joints()}
        for pose in (POSE, {1: 125, 2: 210, 3: 170, 4: 250, 5: 100, 6: 140}):
            for clamp in (False, True):
                expected = {"base": np.eye(4)}
                for i, config in self.geometry.calibration.items():
                    joint = joints[config["joint"]]
                    angle = joint_angle(config, i, pose[i])
                    if clamp:
                        angle = max(joint.limit_lower, min(joint.limit_upper, angle))
                    transform = joint.compute_transform(angle, clamp=False)
                    xyz = transform.translation.as_arrow_array().to_pylist()[0]
                    x, y, z, w = transform.quaternion.as_arrow_array().to_pylist()[0]
                    matrix = np.eye(4)
                    matrix[:3, :3] = [
                        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
                    matrix[:3, 3] = xyz
                    expected[joint.child_link] = expected[joint.parent_link] @ matrix
                actual, _ = self.geometry.transforms(pose, clamp=clamp)
                for link in expected:
                    # Rerun entrega transforms float32; el cálculo usa float64.
                    np.testing.assert_allclose(actual[link], expected[link], atol=1e-6)

    def test_height_is_lowest_mesh_vertex_not_wrist_center(self):
        transforms, _ = self.geometry.transforms(POSE)
        all_points = []
        for link, _, vertices in self.geometry.meshes:
            homogeneous = np.column_stack([vertices, np.ones(len(vertices))])
            all_points.extend((transforms[link] @ homogeneous.T).T[:, :3])
        expected = min(p[2] for p in all_points) - self.geometry.floor_z
        self.assertAlmostEqual(self.geometry.clearance(POSE)["height_mm"], expected * 1000)

    def test_rotation_and_opening_both_affect_height(self):
        pose = {**POSE, 2: 210, 3: 160, 4: 240}
        heights = [self.geometry.clearance({**pose, 5: m5, 6: m6})["height_mm"]
                   for m5, m6 in ((0, 0), (90, 0), (0, 45))]
        self.assertGreater(abs(heights[0] - heights[1]), 1)
        self.assertGreater(abs(heights[0] - heights[2]), 1)

    def test_known_height_plane_contact_and_margin(self):
        geometry = GripperGeometry(ASSETS)
        geometry.meshes = [("base", "synthetic", np.array([[0, 0, geometry.floor_z + .01]]))]
        self.assertAlmostEqual(geometry.clearance(POSE)["height_mm"], 10)
        geometry.meshes[0][2][0, 2] -= .01
        self.assertAlmostEqual(geometry.clearance(POSE)["height_mm"], 0)
        geometry.meshes[0][2][0, 2] -= .005
        self.assertAlmostEqual(geometry.clearance(POSE)["height_mm"], -5)

    def test_clipped_motor_reported_and_missing_pose_rejected(self):
        result = self.geometry.clearance({**POSE, 6: 140})
        self.assertIn(6, result["clipped_motors"])
        for bad in ({1: 180}, {**POSE, 6: float("nan")}):
            with self.assertRaises(ValueError):
                self.geometry.clearance(bad)

    def test_rerun_logs_diagnostic_without_hardware_and_clears_missing_pose(self):
        with patch("serial.Serial", side_effect=AssertionError("No abrir hardware")) as serial:
            view = RerunArmView(ASSETS)
            view.geometry = self.geometry
            view.recording = Mock()
            view._log_floor_clearance(POSE)
            self.assertIsNotNone(view.clearance)
            view._log_floor_clearance({1: 180})
            self.assertIsNone(view.clearance)
            self.assertTrue(any(isinstance(call.args[1], rr.Clear) for call in view.recording.log.call_args_list))
            serial.assert_not_called()

    def test_view_open_and_pose_use_installed_rerun_api_without_spawning(self):
        with patch("rerun.RecordingStream") as recording_class:
            view = RerunArmView(ASSETS)
            view.open()
            self.assertTrue(view.available, view.error)
            view.log_pose(POSE)
            self.assertIsNotNone(view.clearance)
            paths = [call.args[0] for call in recording_class.return_value.log.call_args_list]
            self.assertIn("diagnostico_piso/plano", paths)
            self.assertIn("diagnostico_piso/medicion/punta", paths)


if __name__ == "__main__":
    unittest.main()
