"""Ventana de calibración de cámaras con escena y telemetría simuladas."""
from __future__ import annotations

import json
import tempfile
import tkinter as tk
import unittest
from pathlib import Path

from brazoia.calibracion_camaras import CameraCalibration, settings_from_config
from brazoia.config import load_config
from calibracion_camaras_app import CameraCalibrationApp
from tests.test_calibracion_camaras import scene_features, workspace_points


class CameraCalibrationWindowTests(unittest.TestCase):
    def test_capture_register_calculate_and_save(self) -> None:
        root = tk.Tk()
        root.withdraw()
        cubes, containers = workspace_points(10), workspace_points(10, seed=3)
        state = {"index": 0, "missing": set()}

        def scene():
            return scene_features(cubes[state["index"]], containers[state["index"]], missing=state["missing"])

        def locator(_pose):
            # Agarre del cubo o cubo sobre el recipiente, según lo que se registra.
            source = containers if state.get("kind") == "recipiente" else cubes
            return tuple(source[state["index"]])

        def register(kind):
            state["kind"] = kind
            app.register(kind)

        with tempfile.TemporaryDirectory() as temporary:
            app = CameraCalibrationApp(
                root, project_root=Path(temporary), scene_provider=scene,
                pose_provider=lambda: {motor: 180.0 for motor in range(1, 7)},
                camera_settings=settings_from_config(load_config().cameras), locator=locator)
            try:
                register("cubo")
                self.assertIn("Primero tomá la foto", app.status.get())
                state["missing"] = {("frontal", "cubo")}
                app.capture_scene()
                register("cubo")
                self.assertIn("no se ve el cubo rojo en la cámara frontal", app.status.get())
                state["missing"] = set()
                for index in range(10):
                    state["index"] = index
                    app.capture_scene()
                    register("cubo")
                    register("cubo")
                    self.assertIn("ya se usó", app.status.get())
                    register("recipiente")
                self.assertEqual(len(app.samples), 20)
                saved = json.loads(app.captures_path.read_text(encoding="utf-8"))
                self.assertEqual(len(saved["muestras"]), 20)
                # Los ángulos del agarre quedan guardados para recalcular la mesa si cambia el modelo 3D.
                self.assertEqual(saved["muestras"][0]["angulos"], {str(motor): 180.0 for motor in range(1, 7)})
                app.save_active()
                self.assertIn("Primero calculá", app.status.get())
                app.calculate()
                self.assertIn("mediana", app.result.get())
                app.save_active()
                calibration = CameraCalibration.load(app.output_path)
                self.assertEqual(set(calibration.mappings), {"cubo", "recipiente"})
                app.remove_last()
                self.assertEqual(len(app.samples), 19)
                reloaded = CameraCalibrationApp(root, project_root=Path(temporary), locator=locator)
                reloaded.load_captures(app.captures_path)
                self.assertEqual(len(reloaded.samples), 19)
                self.assertEqual(reloaded.samples[0].pose, {motor: 180.0 for motor in range(1, 7)})
                self.assertEqual(saved["modelo_3d"], app.current_model)
            finally:
                root.destroy()

    def test_captures_from_another_3d_model_are_recomputed_or_dropped(self) -> None:
        root = tk.Tk()
        root.withdraw()
        pixels = {"frontal": [0.4, 0.6], "lateral": [0.5, 0.7]}
        pose = {str(motor): 150.0 + motor for motor in range(1, 7)}
        captures = {"version": 1, "modelo_3d": "viejo", "muestras": [
            {"tipo": "cubo", "pixeles": pixels, "mesa_m": [0.5, 0.5], "origen": "con ángulos", "angulos": pose},
            {"tipo": "cubo", "pixeles": pixels, "mesa_m": [0.5, 0.5], "origen": "sin ángulos"},
        ]}

        def locator(value):
            return value[1] / 1000, -value[2] / 1000

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "capturas.json"
            path.write_text(json.dumps(captures), encoding="utf-8")
            try:
                app = CameraCalibrationApp(root, project_root=Path(temporary), locator=locator, model="nuevo")
                app.load_captures(path)
                self.assertEqual([s.source for s in app.samples], ["con ángulos"])
                self.assertEqual(app.samples[0].table_xy, (0.151, -0.152))
                self.assertIn("1 recalculadas", app.status.get())
                self.assertIn("1 descartadas", app.status.get())
                # Con el mismo modelo se conservan tal cual, aunque no tengan ángulos.
                same = CameraCalibrationApp(root, project_root=Path(temporary), locator=locator, model="viejo")
                same.load_captures(path)
                self.assertEqual([s.table_xy for s in same.samples], [(0.5, 0.5), (0.5, 0.5)])
            finally:
                root.destroy()


if __name__ == "__main__":
    unittest.main()
