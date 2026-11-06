"""Verificación visual final y registro de cada ejecución automática."""
from __future__ import annotations

import json
import queue
import tempfile
import time
import types
import unittest
from pathlib import Path

import cv2
import numpy as np

from app import App
from brazoia.capture import FrameSnapshot
from brazoia.config import detection_parameters, load_config
from brazoia.execution import MotionProposal
from brazoia.planner import MotionGroup
from brazoia.runlog import ExecutionLog
from brazoia.vision import check_container, detect_scene, verify_cube_in_container

LIME = (0, 200, 100)  # BGR: verde lima para las dos cámaras
RED = (0, 0, 220)


def scene(*, container: bool = True, inside: bool = False, outside: bool = False) -> np.ndarray:
    frame = np.full((480, 640, 3), 90, np.uint8)
    if container:
        cv2.rectangle(frame, (250, 220), (450, 370), LIME, -1)
    if inside:
        # Asoma un poco por arriba de las paredes: chico, cerca del borde superior.
        cv2.rectangle(frame, (330, 235), (352, 252), RED, -1)
    if outside:
        # Algo rojo grande lejos del recipiente: el «cubo» con más confianza.
        cv2.rectangle(frame, (40, 60), (140, 160), RED, -1)
    return frame


def observe(frames):
    vision = load_config().vision
    return [detect_scene(frame, **detection_parameters(vision, index)) for index, frame in enumerate(frames)]


class ContainerCheckTests(unittest.TestCase):
    def test_small_red_inside_counts_even_with_bigger_red_elsewhere(self) -> None:
        frame = scene(inside=True, outside=True)
        observation = observe([frame, frame])[0]
        # detect_scene elige el rojo grande de afuera como cubo...
        self.assertLess(observation.red_cube.center_x, 0.3)
        # ...pero la verificación busca el rojo dentro del recipiente.
        check = check_container(frame, observation)
        self.assertEqual(check.state, "dentro")
        self.assertGreater(check.cube.center_x, 0.45)

    def test_empty_container_and_missing_container(self) -> None:
        empty = scene(outside=True)
        self.assertEqual(check_container(empty, observe([empty, empty])[0]).state, "vacio")
        missing = scene(container=False, outside=True)
        self.assertEqual(check_container(missing, observe([missing, missing])[0]).state, "sin_recipiente")

    def test_verdict_needs_one_camera_inside_and_none_empty(self) -> None:
        inside, empty, missing = scene(inside=True), scene(), scene(container=False)
        cases = (
            ((inside, inside), True),
            ((inside, missing), True),   # una cámara que no ve el recipiente no decide
            ((inside, empty), False),    # si una lo ve vacío, no se confirma
            ((missing, missing), False),
            ((empty, empty), False),
        )
        for frames, expected in cases:
            with self.subTest(frames=[id(frame) for frame in frames]):
                passed, checks = verify_cube_in_container(list(frames), observe(frames))
                self.assertIs(passed, expected)
                self.assertEqual(len(checks), 2)

    def test_without_both_cameras_it_is_not_confirmed(self) -> None:
        frame = scene(inside=True)
        self.assertEqual(verify_cube_in_container([frame], observe([frame])), (False, []))


class ExecutionLogTests(unittest.TestCase):
    def snapshots(self, *frames):
        return [FrameSnapshot(frame, time.monotonic(), 1, index, 180, "OK") for index, frame in enumerate(frames)]

    def test_records_scene_photos_joints_notes_and_verification(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            log = ExecutionLog(Path(temporary), {"modo": "geometrico"})
            log.scene({"tipo": "geometrico", "cubo_a_la_base_cm": 24.0})
            frame = scene(inside=True)
            frames = [frame, frame]
            record = log.capture("inicio", self.snapshots(*frames), observe(frames),
                                 {1: 180.0, 6: 137.25}, "open")
            self.assertEqual(record["images"], ["01_inicio_frontal.png", "01_inicio_lateral.png"])
            for name in record["images"]:
                self.assertTrue((log.path / name).is_file())
            log.proposal(MotionProposal(1, "aproximacion_1", "Aproximación 1/3", {1: 150.0},
                                        (MotionGroup(((1, 150.0),)),), None))
            log.note("aviso", "Llegada con desvío (motor quieto): M2 a 1,3°")
            log.completed("aproximacion_1", {1: 150.04}, "open")
            passed, checks = verify_cube_in_container(frames, observe(frames))
            log.verification(passed, checks, record["index"])
            log.finish("success", passed)

            data = json.loads(log.metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(data["schema"], 2)
            self.assertEqual(data["configuration"], {"modo": "geometrico"})
            self.assertEqual(data["scene"]["cubo_a_la_base_cm"], 24.0)
            self.assertEqual(data["captures"][0]["joints"], {"1": 180.0, "6": 137.25})
            self.assertEqual([event["kind"] for event in data["events"]], ["proposal", "aviso", "completed"])
            self.assertEqual(data["events"][2]["joints"], {"1": 150.04})
            self.assertEqual(data["events"][2]["gripper_state"], "open")
            self.assertTrue(data["verification"]["passed"])
            self.assertEqual(data["verification"]["cameras"]["lateral"]["estado"], "dentro")
            self.assertTrue(data["automatic_verification"])

    def test_a_missing_frame_is_noted_and_does_not_stop_the_task(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            log = ExecutionLog(Path(temporary))
            frame = scene()
            record = log.capture("traslado_1", self.snapshots(frame, None), observe([frame]))
            self.assertEqual(record["images"], ["01_traslado_1_frontal.png"])
            self.assertEqual(record["warnings"], ["Sin imagen de la cámara lateral."])
            self.assertIsNone(record["joints"])


class _Var:
    def __init__(self):
        self.value = ""

    def set(self, value):
        self.value = value


class _AppStub:
    """Lo mínimo de App para sus métodos de registro, sin ventana ni robot."""

    _robot_state = App._robot_state
    _run_capture = App._run_capture
    _log_scene = App._log_scene
    verify_final_result = App.verify_final_result

    def __init__(self, root: Path, frames):
        self.robot = None
        self.snapshots = [FrameSnapshot(frame, time.monotonic(), 1, index, 180, "OK")
                          for index, frame in enumerate(frames)]
        self.observations = observe(frames)
        self.execution_log = ExecutionLog(root)
        self.worker_messages = queue.Queue()
        self.auto_status = _Var()
        self.logged_scene = None
        self.automatic_verification = None


class AppRunLogTests(unittest.TestCase):
    def test_final_verification_is_saved_with_its_photos(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            frame = scene(inside=True, outside=True)
            app = _AppStub(Path(temporary), [frame, frame])
            app.verify_final_result()
            self.assertTrue(app.automatic_verification)
            self.assertIn("frontal dentro, lateral dentro", app.auto_status.value)
            data = json.loads(app.execution_log.metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(data["captures"][0]["label"], "verificacion_final")
            self.assertEqual(data["verification"]["capture"], 1)

    def test_scene_of_inicio_is_logged_once(self) -> None:
        class Policy:
            def summary(self, features):
                return {"tipo": "geometrico", "cubo_a_la_base_cm": 22.5}

        with tempfile.TemporaryDirectory() as temporary:
            frame = scene()
            app = _AppStub(Path(temporary), [frame, frame])
            execution = types.SimpleNamespace(policy=Policy(), scene_features=np.zeros(32))
            app._log_scene(execution, app.execution_log)
            app._log_scene(execution, app.execution_log)  # la misma propuesta preparada otra vez
            data = json.loads(app.execution_log.metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(data["scene"]["cubo_a_la_base_cm"], 22.5)
            self.assertEqual([item["label"] for item in data["captures"]], ["inicio"])


if __name__ == "__main__":
    unittest.main()
