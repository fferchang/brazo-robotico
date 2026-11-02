import json
import tempfile
import time
import tkinter as tk
from tkinter import ttk
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np

from app import App
from brazoia.capture import FrameSnapshot
from brazoia.config import KEYFRAME_STAGES, load_config
from brazoia.dataset import DemonstrationSession, load_successful_demonstrations
from brazoia.robot import RobotSnapshot
from brazoia.vision import SceneObservation


class RetryPhotoTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.session = DemonstrationSession(self.root, {})
        self.positions = {i: 180.0 for i in range(1, 7)}
        self.observations = [SceneObservation(None, None), SceneObservation(None, None)]

    def snapshots(self, value=0):
        return [FrameSnapshot(np.full((48, 64, 3), value, dtype=np.uint8), time.monotonic(),
                              value + 1, i, 180, "OK") for i in (0, 1)]

    def capture(self, retry=False, value=0):
        stage = self.session.data["stages"][-1]["stage"] if retry else self.session.next_stage
        return self.session.capture(stage, self.snapshots(value), self.positions, self.observations,
                                    torque_enabled=True, gripper_state="holding", replace_last=retry)

    def test_retry_updates_both_images_angles_and_keeps_stage_counter(self):
        original = self.capture()
        self.positions[1] = 195
        updated = self.capture(True, 50)
        self.assertEqual(len(self.session.data["stages"]), 1)
        self.assertEqual(self.session.next_stage, "aproximacion_1")
        self.assertEqual(updated["index"], 0)
        self.assertEqual(updated["stage"], "inicio")
        self.assertEqual(updated["joint_degrees_logical"]["1"], 195)
        self.assertEqual(self.session.data["replaced_captures"], [original])
        for name in original["images"]:
            self.assertTrue(np.all(cv2.imread(str(self.session.path / name)) == 0))
        for name in updated["images"]:
            self.assertTrue(np.all(cv2.imread(str(self.session.path / name)) == 50))
        disk = json.loads((self.session.path / "metadata.json").read_text(encoding="utf-8"))
        self.assertEqual(disk, self.session.data)

    def test_repeated_retries_then_next_capture_keep_order(self):
        self.capture()
        self.capture(True, 50)
        self.capture(True, 80)
        self.capture()
        self.assertEqual([r["stage"] for r in self.session.data["stages"]], list(KEYFRAME_STAGES[:2]))
        self.assertEqual(len(self.session.data["replaced_captures"]), 2)

    def test_last_stage_retry_still_has_twelve_training_samples(self):
        for _ in KEYFRAME_STAGES:
            self.capture()
        self.capture(True, 100)
        self.assertTrue(self.session.complete)
        self.session.finish("success")
        loaded = load_successful_demonstrations(self.root)
        self.assertEqual(len(loaded[0]["stages"]), 12)
        self.assertIn("retoma", loaded[0]["stages"][-1]["images"][0])
        with self.assertRaisesRegex(ValueError, "finalizada"):
            self.capture(True)

    def test_empty_session_cannot_retry(self):
        with self.assertRaisesRegex(ValueError, "Todavía"):
            self.session.capture("inicio", self.snapshots(), self.positions, self.observations,
                                 torque_enabled=True, gripper_state="holding", replace_last=True)

    def test_write_failure_keeps_original_metadata_and_images(self):
        original = self.capture()
        metadata = (self.session.path / "metadata.json").read_bytes()
        for target in ("brazoia.dataset.cv2.imwrite", "brazoia.dataset._atomic_json"):
            with self.subTest(target=target), patch(target, side_effect=OSError("disco")):
                with self.assertRaises(OSError):
                    self.capture(True, 50)
            self.assertEqual(self.session.data["stages"], [original])
            self.assertEqual((self.session.path / "metadata.json").read_bytes(), metadata)
        for name in original["images"]:
            self.assertTrue(np.all(cv2.imread(str(self.session.path / name)) == 0))

    def test_second_image_failure_does_not_commit(self):
        original = self.capture()
        write = cv2.imwrite
        calls = 0
        def failing_write(path, frame):
            nonlocal calls
            calls += 1
            return write(path, frame) if calls == 1 else False
        with patch("brazoia.dataset.cv2.imwrite", side_effect=failing_write):
            with self.assertRaises(RuntimeError):
                self.capture(True, 50)
        self.assertEqual(self.session.data["stages"], [original])

    def test_button_confirmation_and_cancel(self):
        self.capture()
        ui = SimpleNamespace(demonstration=self.session, status=Mock(), take_keyframe=Mock())
        with patch("app.messagebox.askyesno", return_value=False):
            App.retry_last_keyframe(ui)
        ui.take_keyframe.assert_not_called()
        with patch("app.messagebox.askyesno", return_value=True):
            App.retry_last_keyframe(ui)
        ui.take_keyframe.assert_called_once_with(replace_last=True)

    def test_ui_recaptures_last_not_next_and_never_moves_robot(self):
        self.capture()
        robot = Mock()
        robot.snapshot.return_value = RobotSnapshot(self.positions, {}, True, "holding", "")
        robot.stable_pose.return_value = (self.positions, "")
        ui = SimpleNamespace(demonstration=self.session, robot=robot, config=load_config(),
                             snapshots=self.snapshots(90), observations=self.observations,
                             status=Mock(), demo_status=Mock(), log=Mock(), reject_keyframe=Mock())
        App.take_keyframe(ui, replace_last=True)
        ui.reject_keyframe.assert_not_called()
        self.assertEqual(self.session.data["stages"][-1]["stage"], "inicio")
        self.assertEqual(len(self.session.data["stages"]), 1)
        self.assertIn("reemplazada", ui.status.set.call_args.args[0])
        robot.send.assert_not_called()

    def test_retry_button_exists_and_handles_empty_session(self):
        root = tk.Tk()
        root.withdraw()
        try:
            ui = App(root)
            root.update_idletasks()
            pending, buttons = [root], []
            while pending:
                widget = pending.pop()
                pending.extend(widget.winfo_children())
                if isinstance(widget, ttk.Button) and widget.cget("text") == "Reintentar última foto":
                    buttons.append(widget)
            self.assertEqual(len(buttons), 1)
            buttons[0].invoke()
            self.assertIn("Primero tomá una foto", ui.status.get())
            self.assertIsNone(ui.robot)
            self.assertEqual(ui.cameras, [])
        finally:
            for timer in root.tk.call("after", "info"):
                root.after_cancel(timer)
            root.destroy()


if __name__ == "__main__":
    unittest.main()
