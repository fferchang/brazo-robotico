import json
import tempfile
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from brazoia.config import INITIAL_POSE, KEYFRAME_STAGES, load_config
from brazoia.offline import OfflineModel
from brazoia.policy import StagePolicy
from offline_app import OfflineApp


class OfflineTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        folder = self.root / "modelos" / "no_validados"
        folder.mkdir(parents=True)
        self.path = folder / "experimental.npz"
        self.metadata = {"stages": list(KEYFRAME_STAGES), "experimental_only": True,
                         "validation_passed": False, "validation_sources": ["demo"],
                         "limits": load_config().limits}
        payload = dict(mean=np.zeros(32), scale=np.ones(32), feature_min=np.zeros((12, 32)),
                       feature_max=np.ones((12, 32)), canonical_start=np.array(list(INITIAL_POSE.values())),
                       canonical_guard=np.array(list(INITIAL_POSE.values())),
                       metadata_json=np.asarray(json.dumps(self.metadata)))
        for i in range(12):
            weight = np.zeros((561, 5))
            weight[0] = [1000, 180, 90, 180, 263]
            payload[f"weights_{i}"] = weight
        np.savez(self.path, **payload)
        self.demo_path = self.root / "datos" / "demostraciones" / "demo"
        self.demo_path.mkdir(parents=True)
        cv2.imwrite(str(self.demo_path / "photo.png"), np.zeros((48, 64, 3), dtype=np.uint8))
        self.stages = [{"stage": name, "features": [0.5]*32, "images": ["photo.png", "photo.png"],
                        "joint_degrees_logical": {str(k): v for k, v in {**INITIAL_POSE, 6: 20}.items()},
                        "capture_warnings": ["aviso conservado"]} for name in KEYFRAME_STAGES]
        (self.demo_path / "metadata.json").write_text(json.dumps({"schema": 4, "status": "finished",
            "result": "success", "stages": self.stages}), encoding="utf-8")

    def test_experimental_offline_is_not_an_executable_policy(self):
        model = OfflineModel(self.path)
        self.assertNotIsInstance(model, StagePolicy)
        self.assertFalse(hasattr(model, "predict"))
        with self.assertRaisesRegex(ValueError, "experimental no validado"):
            StagePolicy.load(self.path)

    def test_raw_error_not_hidden_by_clamped_visualization(self):
        result = OfflineModel(self.path).evaluate(4, self.stages[4])
        self.assertEqual(result["raw"][0], 1000)
        self.assertEqual(result["preview"][1], 300)
        self.assertEqual(result["preview"][6], 20)
        self.assertAlmostEqual(result["mae"], 164)
        self.assertIn("recortada", " ".join(result["notes"]))

    def test_home_and_guard_show_canonical_pose(self):
        model = OfflineModel(self.path)
        for index in (0, 11):
            result = model.evaluate(index, self.stages[index])
            self.assertEqual(result["preview"][1], INITIAL_POSE[1])
            self.assertEqual(result["raw"][0], 1000)

    def test_ood_is_visible_not_silently_suppressed(self):
        stage = {**self.stages[4], "features": [99.0]*32}
        result = OfflineModel(self.path).evaluate(4, stage)
        self.assertIn("Fuera de distribución", " ".join(result["notes"]))

    def test_bad_input_is_rejected(self):
        model = OfflineModel(self.path)
        for features in ([0.0]*31, [float("nan")]*32):
            with self.assertRaises(ValueError):
                model.evaluate(4, {**self.stages[4], "features": features})
        with self.assertRaises(ValueError):
            model.evaluate(4, self.stages[0])

    def test_window_stage_navigation_playback_and_rerun_never_open_serial(self):
        root = tk.Tk()
        root.withdraw()
        ui = None
        with patch("serial.Serial", side_effect=AssertionError("No abrir hardware")) as serial, \
             patch("offline_app.RerunArmView") as view_class:
            try:
                view_class.return_value.available = False
                ui = OfflineApp(root, project_root=self.root)
                root.update_idletasks()
                self.assertIn("EXPERIMENTAL", ui.model_text.get())
                self.assertEqual(ui.split_text.get(), "VALIDACIÓN")
                self.assertEqual(len(ui.table.get_children()), 6)
                self.assertEqual(len(ui.photos), 2)
                ui.step(1)
                self.assertEqual(ui.stage_combo.current(), 1)
                ui.play()
                self.assertIsNotNone(ui.timer)
                ui.pause()
                self.assertIsNone(ui.timer)
                for _ in range(11):
                    ui.step(1)
                self.assertEqual(ui.stage_combo.current(), 11)
                view_class.return_value.available = True
                ui.open_view()
                view_class.return_value.log_pose.assert_called()
                serial.assert_not_called()
            finally:
                if ui is not None:
                    ui.close()
                else:
                    root.destroy()


if __name__ == "__main__":
    unittest.main()
