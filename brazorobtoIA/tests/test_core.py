from __future__ import annotations

import json
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from app import build_reference_pose_plan
from brazoia.capture import (
    FrameSnapshot,
    adjust_brightness,
    adjust_contrast,
    adjust_saturation,
    apply_zoom,
)
from brazoia.config import (
    INITIAL_POSE,
    KEYFRAME_STAGES,
    REFERENCE_POSE,
    load_config,
    validate_stage_gripper,
)
from brazoia.dataset import DemonstrationSession, load_successful_demonstrations
from brazoia.execution import SupervisedExecution
from brazoia.planner import build_motion_plan
from brazoia.policy import MINIMUM_DEMONSTRATIONS, StagePolicy, train_policy
from brazoia.robot import RobotSerial, parse_positions
from brazoia.runlog import ExecutionLog
from brazoia.vision import SceneObservation, detect_scene, observation_features


class ConfigurationTests(unittest.TestCase):
    def test_final_sequence_has_exactly_twelve_ordered_keyframes(self) -> None:
        self.assertEqual(
            KEYFRAME_STAGES,
            (
                "inicio",
                "aproximacion_1",
                "aproximacion_2",
                "aproximacion_3",
                "apertura_sobre_cubo",
                "descenso_abierto",
                "agarre",
                "traslado_1",
                "traslado_2",
                "traslado_3",
                "soltado",
                "guardado",
            ),
        )

    def test_configuration_contains_all_motor_limits(self) -> None:
        configuration = load_config()
        self.assertEqual(set(configuration.limits), set(range(1, 7)))
        self.assertEqual(configuration.cameras["frontal"].contrast_percent, 50.0)
        self.assertEqual(configuration.cameras["lateral"].contrast_percent, 50.0)
        self.assertEqual(configuration.cameras["frontal"].brightness_percent, -25.0)
        self.assertEqual(configuration.cameras["frontal"].saturation_percent, 10.0)
        self.assertEqual(configuration.cameras["lateral"].brightness_percent, -15.0)
        self.assertEqual(configuration.cameras["lateral"].saturation_percent, 0.0)
        self.assertEqual(configuration.cameras["frontal"].zoom_factor, 1.0)

    def test_gripper_state_matches_each_stage(self) -> None:
        self.assertEqual(validate_stage_gripper("inicio", 2.0, "holding"), "")
        self.assertNotEqual(validate_stage_gripper("inicio", 140.0, "opening"), "")
        self.assertEqual(validate_stage_gripper("aproximacion_2", 140.0, "opening"), "")
        self.assertNotEqual(validate_stage_gripper("aproximacion_3", 20.0, "idle"), "")
        self.assertEqual(validate_stage_gripper("aproximacion_3", 2.0, "holding"), "")
        self.assertNotEqual(validate_stage_gripper("aproximacion_3", 140.0, "opening"), "")
        for stage in ("apertura_sobre_cubo", "descenso_abierto"):
            self.assertEqual(validate_stage_gripper(stage, 140.0, "opening"), "")
            self.assertNotEqual(validate_stage_gripper(stage, 2.0, "holding"), "")
        self.assertEqual(validate_stage_gripper("traslado_2", 70.0, "holding"), "")
        self.assertNotEqual(validate_stage_gripper("traslado_2", 70.0, "idle"), "")

    def test_reference_pose_targets_only_motors_one_through_five(self) -> None:
        self.assertEqual(
            REFERENCE_POSE,
            {1: 180.0, 2: 90.0, 3: 270.0, 4: 180.0, 5: 270.0},
        )
        self.assertNotIn(6, REFERENCE_POSE)
        source = (Path(__file__).resolve().parents[1] / "app.py").read_text(
            encoding="utf-8"
        )
        method = source.split("    def move_to_reference_pose", 1)[1].split(
            "    def open_rerun", 1
        )[0]
        self.assertIn('robot.send("arm torque on")', method)
        self.assertNotIn('robot.send("torque on")', method)

    def test_reference_plan_corrects_m2_slightly_below_its_limit(self) -> None:
        current = {1: 180.0, 2: 88.4, 3: 273.3, 4: 172.5, 5: 264.1, 6: 1.6}
        groups = build_reference_pose_plan(current, load_config().limits)
        commanded = {
            motor_id: target
            for group in groups
            for motor_id, target in group.targets
        }
        self.assertEqual(commanded[2], 90.0)
        self.assertNotIn(6, commanded)

    def test_initial_pose_plan_reaches_new_angles_without_touching_m6(self) -> None:
        current = {**REFERENCE_POSE, 6: 37.0}
        groups = build_reference_pose_plan(current, load_config().limits, INITIAL_POSE)
        reached = dict(current)
        for group in groups:
            for motor_id, target in group.targets:
                self.assertNotEqual(motor_id, 6)
                reached[motor_id] = target
        self.assertEqual(reached, {1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0, 6: 37.0})
        self.assertEqual(current, {**REFERENCE_POSE, 6: 37.0})
        with self.assertRaisesRegex(ValueError, "únicamente M1–M5"):
            build_reference_pose_plan(current, load_config().limits, {**INITIAL_POSE, 6: 0.0})

    def test_contrast_is_increased_around_mid_gray(self) -> None:
        frame = np.asarray([[[64, 128, 192]]], dtype=np.uint8)
        adjusted = adjust_contrast(frame, 50.0)
        self.assertLess(int(adjusted[0, 0, 0]), 64)
        self.assertAlmostEqual(int(adjusted[0, 0, 1]), 128, delta=1)
        self.assertGreater(int(adjusted[0, 0, 2]), 192)

    def test_first_camera_brightness_reduction(self) -> None:
        frame = np.full((1, 1, 3), 200, dtype=np.uint8)
        adjusted = adjust_brightness(frame, -25.0)
        self.assertTrue(np.all(adjusted == 150))

    def test_saturation_and_zoom_keep_frame_shape(self) -> None:
        frame = np.zeros((100, 160, 3), dtype=np.uint8)
        frame[:, :80] = (0, 0, 255)
        saturated = adjust_saturation(frame, 50.0)
        zoomed = apply_zoom(saturated, 2.0)
        self.assertEqual(zoomed.shape, frame.shape)


class RobotProtocolTests(unittest.TestCase):
    def test_compact_positions_and_gripper_offset(self) -> None:
        parsed = parse_positions(
            "POS M1=2048(180.04deg) M2=NA M6=1706(150.00deg)"
        )
        self.assertAlmostEqual(parsed[1], 180.04)
        self.assertIsNone(parsed[2])
        self.assertAlmostEqual(parsed[6], 140.0)

    def test_single_position_line(self) -> None:
        parsed = parse_positions("POS ID 3: 1024 (~90.02 deg)")
        self.assertAlmostEqual(parsed[3], 90.02)

    def test_arm_only_torque_confirmations_update_arm_state(self) -> None:
        robot = RobotSerial("TEST", 115200)
        robot._consume_line("ARM_TORQUE_ON_OK", time.monotonic())
        self.assertTrue(robot.snapshot().torque_enabled)
        robot._consume_line("ARM_TORQUE_OFF_OK", time.monotonic())
        self.assertFalse(robot.snapshot().torque_enabled)
        self.assertEqual(robot.snapshot().gripper_state, "idle")

    def test_firmware_has_separate_arm_and_global_torque_commands(self) -> None:
        firmware = (
            Path(__file__).resolve().parents[1]
            / "firmware"
            / "firmware_secuencias.ino"
        ).read_text(encoding="utf-8")
        arm_branch = firmware.index('line.equalsIgnoreCase("arm torque off")')
        global_branch = firmware.index('line.equalsIgnoreCase("torque off")')
        self.assertLess(arm_branch, global_branch)
        self.assertIn("for (uint8_t id = MIN_ID; id <= 5; id++)", firmware)
        self.assertIn("ARM_TORQUE_OFF_OK", firmware)

    def test_interface_uses_backward_compatible_torque_on(self) -> None:
        source = (Path(__file__).resolve().parents[1] / "app.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('command=lambda: self.send_robot("torque on")', source)
        self.assertIn('command=lambda: self.send_robot("arm torque off")', source)


class VisionTests(unittest.TestCase):
    def test_detects_red_square_and_lime_container(self) -> None:
        frame = np.full((360, 640, 3), (35, 35, 35), dtype=np.uint8)
        cv2.rectangle(frame, (80, 160), (150, 230), (0, 0, 255), -1)
        cv2.rectangle(frame, (330, 120), (560, 285), (0, 255, 128), -1)
        observation = detect_scene(frame)
        self.assertIsNotNone(observation.red_cube)
        self.assertIsNotNone(observation.lime_container)
        self.assertGreater(observation.red_cube.shape_ratio, 0.85)

    def test_does_not_treat_the_old_gray_container_as_lime(self) -> None:
        path = Path(__file__).resolve().parents[1] / "assets" / "referencia_recipiente_gris.png"
        frame = cv2.imread(str(path))
        self.assertIsNotNone(frame)
        observation = detect_scene(frame)
        self.assertIsNone(observation.lime_container)

    def test_frontal_strict_profile_keeps_object_and_rejects_table_edge(self) -> None:
        root = Path(__file__).resolve().parents[1]
        configuration = load_config()
        vision = configuration.vision
        parameters = {
            "minimum_lime_area_ratio": vision.frontal_minimum_lime_area_ratio,
            "lime_hsv_lower": vision.frontal_lime_hsv_lower,
            "lime_hsv_upper": vision.frontal_lime_hsv_upper,
            "minimum_lime_shape_ratio": vision.frontal_minimum_lime_shape_ratio,
            "minimum_lime_confidence": vision.frontal_minimum_lime_confidence,
        }
        correct = cv2.imread(
            str(root / "assets" / "referencias_vision" / "frontal_recipiente_correcto.png")
        )
        incorrect = cv2.imread(
            str(root / "assets" / "referencias_vision" / "frontal_falso_positivo.png")
        )
        self.assertIsNotNone(detect_scene(correct, **parameters).lime_container)
        self.assertIsNone(detect_scene(incorrect, **parameters).lime_container)

    def test_feature_vector_is_fixed_length(self) -> None:
        vector = observation_features(
            [SceneObservation(None, None), SceneObservation(None, None)]
        )
        self.assertEqual(vector.shape, (32,))


class DatasetTests(unittest.TestCase):
    def test_complete_successful_demonstration_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session = DemonstrationSession(root, {"test": True})
            self.assertTrue(session.path.is_dir())
            self.assertTrue(session.path.name.startswith("prueba_"))
            self.assertTrue((session.path / "metadata.json").is_file())
            frame = np.zeros((48, 64, 3), dtype=np.uint8)
            snapshots = [
                FrameSnapshot(frame, time.monotonic(), 1, 2, 180, "OK"),
                FrameSnapshot(frame, time.monotonic(), 1, 1, 180, "OK"),
            ]
            observations = [SceneObservation(None, None), SceneObservation(None, None)]
            positions = {motor_id: 100.0 + motor_id for motor_id in range(1, 7)}
            for stage in KEYFRAME_STAGES:
                if stage in {"inicio", "aproximacion_1", "aproximacion_3", "guardado"}:
                    positions[6] = 2.0
                    gripper_state = "holding"
                elif stage in {"aproximacion_2", "apertura_sobre_cubo", "descenso_abierto", "soltado"}:
                    positions[6] = 140.0
                    gripper_state = "opening"
                else:
                    positions[6] = 70.0
                    gripper_state = "holding"
                session.capture(
                    stage,
                    snapshots,
                    positions,
                    observations,
                    torque_enabled=True,
                    gripper_state=gripper_state,
                )
            session.finish("success")

            loaded = load_successful_demonstrations(root)
            self.assertEqual(len(loaded), 1)
            self.assertEqual(len(loaded[0]["stages"]), 12)
            self.assertEqual(len(list(session.path.glob("*.png"))), 24)
            self.assertIsNone(session.next_stage)
            with self.assertRaisesRegex(ValueError, "finalizada"):
                session.capture("guardado", snapshots, positions, observations,
                                torque_enabled=True, gripper_state="holding")

    def test_old_ten_stage_demonstration_is_not_loaded(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session = DemonstrationSession(root, {})
            session.data.update(status="finished", result="success", stages=[
                {"stage": stage} for stage in KEYFRAME_STAGES
                if stage not in {"apertura_sobre_cubo", "descenso_abierto"}
            ])
            session._write()
            self.assertEqual(load_successful_demonstrations(root), [])

    def test_operator_can_save_a_gripper_mismatch_as_a_warning(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            session = DemonstrationSession(Path(temporary), {"test": True})
            frame = np.zeros((24, 32, 3), dtype=np.uint8)
            snapshots = [
                FrameSnapshot(frame, time.monotonic(), 1, 2, 180, "OK"),
                FrameSnapshot(frame, time.monotonic(), 1, 1, 180, "OK"),
            ]
            record = session.capture(
                "inicio",
                snapshots,
                {motor_id: 140.0 for motor_id in range(1, 7)},
                [SceneObservation(None, None), SceneObservation(None, None)],
                torque_enabled=False,
                gripper_state="opening",
                warnings=["Garra abierta en Inicio."],
            )
            self.assertEqual(record["capture_warnings"], ["Garra abierta en Inicio."])
            self.assertEqual(session.next_stage, "aproximacion_1")


class ExecutionLogTests(unittest.TestCase):
    def test_each_execution_creates_its_own_folder_immediately(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = ExecutionLog(root)
            second = ExecutionLog(root)
            self.assertNotEqual(first.path, second.path)
            for execution in (first, second):
                self.assertTrue(execution.path.is_dir())
                self.assertTrue(execution.path.name.startswith("prueba_"))
                self.assertTrue(execution.metadata_path.is_file())


class PolicyTests(unittest.TestCase):
    def test_model_with_old_sequence_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "old.npz"
            stages = [stage for stage in KEYFRAME_STAGES
                      if stage not in {"apertura_sobre_cubo", "descenso_abierto"}]
            np.savez(path, metadata_json=np.asarray(json.dumps({"stages": stages})))
            with self.assertRaisesRegex(ValueError, "otra secuencia"):
                StagePolicy.load(path)

    def _write_demo(self, root: Path, demo_index: int) -> None:
        path = root / f"demo_{demo_index:03d}"
        path.mkdir()
        base = -0.8 + demo_index * 0.08
        stages = []
        for stage_index, stage_name in enumerate(KEYFRAME_STAGES):
            features = np.linspace(0.1, 0.9, 32) + base * 0.01
            features[1] = 0.45 + base * 0.10
            features[17] = 0.55 - base * 0.08
            joints = {
                str(motor_id): 120.0
                + 7.0 * motor_id
                + 2.0 * stage_index
                + base * (motor_id * 0.8)
                for motor_id in range(1, 6)
            }
            joints["6"] = 140.0
            stages.append(
                {
                    "stage": stage_name,
                    "features": features.tolist(),
                    "joint_degrees_logical": joints,
                }
            )
        metadata = {
            "schema": 4,
            "status": "finished",
            "result": "success",
            "stages": stages,
        }
        (path / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")

    def test_training_and_loading_local_policy(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "demos"
            root.mkdir()
            for index in range(25):
                self._write_demo(root, index)
            model_path = Path(temporary) / "policy.npz"
            report = train_policy(root, model_path, load_config().limits)
            self.assertEqual(report.demonstrations, 25)
            self.assertLess(report.mean_absolute_error_degrees, 2.0)
            model = StagePolicy.load(model_path)
            np.testing.assert_allclose(model.canonical_start, model.canonical_guard)
            features = np.linspace(0.1, 0.9, 32)
            features[1] = 0.45
            features[17] = 0.55
            prediction = model.predict(4, features)
            self.assertEqual(set(prediction), set(range(1, 6)))

    def test_five_demonstrations_enable_pilot_training(self) -> None:
        self.assertEqual(MINIMUM_DEMONSTRATIONS, 5)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "demos"
            root.mkdir()
            for index in range(5):
                self._write_demo(root, index)
            report = train_policy(
                root, Path(temporary) / "pilot.npz", load_config().limits
            )
            self.assertEqual(report.demonstrations, 5)

    def test_bad_pilot_is_rejected_before_writing_model(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "demos"
            root.mkdir()
            for index in range(5):
                self._write_demo(root, index)
            metadata_path = root / "demo_001" / "metadata.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            for stage in metadata["stages"]:
                for motor_id in range(1, 6):
                    stage["joint_degrees_logical"][str(motor_id)] += 150.0
            metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
            model_path = Path(temporary) / "bad.npz"
            with self.assertRaisesRegex(ValueError, "Modelo rechazado"):
                train_policy(root, model_path, load_config().limits)
            self.assertFalse(model_path.exists())
            experimental_path = Path(temporary) / "experimental.npz"
            report = train_policy(root, experimental_path, load_config().limits, export_unvalidated=True)
            self.assertFalse(report.validation_passed)
            self.assertTrue(experimental_path.exists())
            with np.load(experimental_path, allow_pickle=False) as saved:
                info = json.loads(str(saved["metadata_json"].item()))
            self.assertTrue(info["experimental_only"])
            self.assertEqual(len(info["source_demonstrations"]), 5)
            with self.assertRaisesRegex(ValueError, "experimental no validado"):
                StagePolicy.load(experimental_path)

    def test_capture_rule_warnings_do_not_exclude_training_samples(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "demos"
            root.mkdir()
            for index in range(5):
                self._write_demo(root, index)
                path = root / f"demo_{index:03d}" / "metadata.json"
                data = json.loads(path.read_text(encoding="utf-8"))
                for stage in data["stages"]:
                    stage["capture_warnings"] = ["REGLA_M4", "Torque del brazo no confirmado como activo."]
                path.write_text(json.dumps(data), encoding="utf-8")
            model_path = Path(temporary) / "warnings.npz"
            report = train_policy(root, model_path, load_config().limits)
            model = StagePolicy.load(model_path)
            self.assertEqual(report.demonstrations, 5)
            self.assertEqual(model.metadata["captures_with_warnings"], 60)


class PlannerTests(unittest.TestCase):
    def test_motion_groups_never_command_more_than_two_motors(self) -> None:
        limits = load_config().limits
        current = {1: 180.0, 2: 180.0, 3: 160.0, 4: 180.0, 5: 170.0}
        target = {1: 185.0, 2: 180.0, 3: 160.0, 4: 180.0, 5: 175.0}
        groups = build_motion_plan(current, target, limits)
        self.assertTrue(groups)
        self.assertTrue(all(1 <= len(group.targets) <= 2 for group in groups))


class ExecutionSequenceTests(unittest.TestCase):
    def test_automatic_gripper_actions_match_corrected_sequence(self) -> None:
        class FakePolicy:
            canonical_start = np.asarray([180.0, 180.0, 160.0, 180.0, 170.0])
            canonical_guard = canonical_start.copy()

            def predict(self, _stage_index, _features):
                return {1: 180.0, 2: 180.0, 3: 160.0, 4: 180.0, 5: 170.0}

        execution = SupervisedExecution(FakePolicy(), load_config().limits)
        current = {1: 180.0, 2: 180.0, 3: 160.0, 4: 180.0, 5: 170.0, 6: 0.0}
        observations = [SceneObservation(None, None), SceneObservation(None, None)]
        expected = {
            1: None,
            2: "open",
            3: "close",
            4: "open",
            5: None,
            6: "close",
            7: None,
            8: None,
            9: None,
            10: "open",
            11: "close",
        }
        for stage_index, action in expected.items():
            proposal = execution.prepare(current, observations)
            self.assertEqual(proposal.stage_index, stage_index)
            self.assertEqual(proposal.gripper_action, action)
            execution.mark_completed()
            self.assertEqual(execution.finished, stage_index == 11)


if __name__ == "__main__":
    unittest.main()
