"""Calibración cámaras → mesa con datos sintéticos de geometría conocida."""
from __future__ import annotations

import dataclasses
import tempfile
import unittest
from pathlib import Path

import numpy as np

from brazoia.calibracion_camaras import (
    SHARED_MODEL,
    CameraCalibration,
    Sample,
    _fit,
    describe,
    fit_calibration,
    fit_mapping,
    object_pixels,
    settings_from_config,
)
from brazoia.config import load_config
from brazoia.vision import FEATURE_NAMES


# Proyecciones mesa (metros, marco del modelo) → imagen (0 a 1) con perspectiva.
FRONTAL = np.array([[2.4, 0.15, 0.5], [0.05, -1.2, 0.35], [0.1, 0.9, 1.25]])
LATERAL = np.array([[0.3, -2.1, 0.0], [-0.9, 0.2, 0.62], [-0.6, 0.2, 1.0]])
CONFIG = load_config()
SETTINGS = settings_from_config(CONFIG.cameras)


def project(matrix, xy):
    point = matrix @ np.array([xy[0], xy[1], 1.0])
    return float(point[0] / point[2]), float(point[1] / point[2])


def workspace_points(count, seed=0):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(-0.15, 0.15, count), rng.uniform(-0.31, -0.15, count)])


# Cámaras casi al ras de la mesa, ajustadas a la calibración real del 4/11: cada
# una resuelve bien una sola dirección (la frontal, izquierda-derecha; la lateral,
# cerca-lejos de la base).
LOW_FRONTAL = np.array([[-1.826, 0.466, 0.407], [-0.291, 0.536, 0.582], [-0.222, 1.374, 1.0]])
LOW_LATERAL = np.array([[-0.362, -0.956, 0.302], [-0.222, 0.128, 0.648], [-0.878, 0.355, 1.0]])


def samples_for(points, kind="cubo", noise=0.0, seed=1, cameras=None, label_offset=(0.0, 0.0)):
    """Muestras sintéticas; `label_offset` corre la posición anotada respecto de la que se ve."""
    cameras = cameras or {"frontal": FRONTAL, "lateral": LATERAL}
    rng = np.random.default_rng(seed)
    result = []
    for index, xy in enumerate(points):
        pixels = {name: tuple(np.add(project(matrix, xy), rng.normal(0, noise, 2)))
                  for name, matrix in cameras.items()}
        label = (float(xy[0] + label_offset[0]), float(xy[1] + label_offset[1]))
        result.append(Sample(kind, pixels, label, f"prueba {index}"))
    return result


def scene_features(cube_xy, container_xy, missing=()):
    features = np.zeros(len(FEATURE_NAMES))
    for kind, xy in (("cubo", cube_xy), ("recipiente", container_xy)):
        for camera, matrix in (("frontal", FRONTAL), ("lateral", LATERAL)):
            if (camera, kind) in missing:
                continue
            x, y = project(matrix, xy)
            features[FEATURE_NAMES.index(f"{camera}_{kind}_presente")] = 1.0
            features[FEATURE_NAMES.index(f"{camera}_{kind}_x")] = x
            features[FEATURE_NAMES.index(f"{camera}_{kind}_y")] = y
    return features


class CameraCalibrationTests(unittest.TestCase):
    def test_synthetic_projections_stay_inside_the_image(self) -> None:
        for sample in samples_for(workspace_points(200)):
            for x, y in sample.pixels.values():
                self.assertTrue(0.0 < x < 1.0 and 0.0 < y < 1.0, sample.pixels)

    def test_exact_projection_is_recovered(self) -> None:
        mapping = fit_mapping(samples_for(workspace_points(12)))
        self.assertTrue(mapping.model.startswith("homografia"), mapping.model)
        self.assertLess(mapping.validation["maximo_mm"], 0.5)
        target = (0.02, -0.21)
        pixels = {"frontal": project(FRONTAL, target), "lateral": project(LATERAL, target)}
        np.testing.assert_allclose(mapping.locate(pixels, "el cubo rojo"), target, atol=5e-4)

    def test_noisy_detection_reports_its_error(self) -> None:
        mapping = fit_mapping(samples_for(workspace_points(15), noise=0.003))
        self.assertLess(mapping.validation["mediana_mm"], 5.0)
        self.assertIn(mapping.model, mapping.candidates)

    def test_low_cameras_combine_what_each_one_resolves(self) -> None:
        low = {"frontal": LOW_FRONTAL, "lateral": LOW_LATERAL}
        for sample in samples_for(workspace_points(200), cameras=low):
            for x, y in sample.pixels.values():
                self.assertTrue(0.0 < x < 1.0 and 0.0 < y < 1.0, sample.pixels)
        mapping = fit_mapping(samples_for(workspace_points(15), noise=0.004, cameras=low))
        self.assertEqual(mapping.model, "reproyeccion")
        single = mapping.candidates["homografia_frontal"]["mediana_mm"]
        self.assertLess(mapping.validation["mediana_mm"], 0.85 * single)

    def test_shared_mapping_learns_the_offset_between_objects(self) -> None:
        low = {"frontal": LOW_FRONTAL, "lateral": LOW_LATERAL}
        cubes = samples_for(workspace_points(10), cameras=low)
        # El cubo se sostuvo 6 mm a la derecha y 3 mm atrás del centro del recipiente.
        containers = samples_for(workspace_points(10, seed=5), "recipiente", cameras=low,
                                 label_offset=(0.006, -0.003))
        params = _fit(SHARED_MODEL, cubes, containers)
        np.testing.assert_allclose(params["corrimiento_compartidas_m"], (-0.006, 0.003), atol=1e-6)
        # Con ruido y pocas muestras propias, compartir las del otro objeto ayuda.
        noisy_cubes = samples_for(workspace_points(8, seed=2), noise=0.004, cameras=low)
        noisy_containers = samples_for(workspace_points(10, seed=5), "recipiente", noise=0.004, seed=3,
                                       cameras=low, label_offset=(0.006, -0.003))
        mapping = fit_mapping(noisy_cubes, shared=noisy_containers)
        self.assertEqual(mapping.model, SHARED_MODEL)
        self.assertEqual(mapping.shared_samples, 10)
        self.assertLess(mapping.validation["media_mm"], mapping.candidates["reproyeccion"]["media_mm"])
        # Cada objeto se ubica en su propio marco: el corrimiento no se le pasa al otro.
        calibration = fit_calibration(cubes + containers, SETTINGS)
        target = (0.03, -0.24)
        pixels = {name: project(matrix, target) for name, matrix in low.items()}
        np.testing.assert_allclose(calibration.mappings["cubo"].locate(pixels, "el cubo rojo"), target, atol=1e-4)
        np.testing.assert_allclose(calibration.mappings["recipiente"].locate(pixels, "el recipiente"),
                                   (0.036, -0.243), atol=1e-4)

    def test_shared_mapping_needs_enough_samples_of_the_other_object(self) -> None:
        low = {"frontal": LOW_FRONTAL, "lateral": LOW_LATERAL}
        cubes = samples_for(workspace_points(10), noise=0.002, cameras=low)
        few = samples_for(workspace_points(3, seed=5), "recipiente", noise=0.002, cameras=low)
        self.assertNotIn(SHARED_MODEL, fit_mapping(cubes, shared=few).candidates)
        self.assertNotIn(SHARED_MODEL, fit_mapping(cubes).candidates)

    def test_shared_mapping_survives_save_and_load(self) -> None:
        low = {"frontal": LOW_FRONTAL, "lateral": LOW_LATERAL}
        samples = (samples_for(workspace_points(8, seed=2), noise=0.004, cameras=low)
                   + samples_for(workspace_points(10, seed=5), "recipiente", noise=0.004, seed=3, cameras=low))
        calibration = fit_calibration(samples, SETTINGS)
        self.assertEqual(calibration.mappings["cubo"].model, SHARED_MODEL)
        self.assertIn("(más 10 del recipiente)", describe(calibration))
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "camaras.json"
            calibration.save(path)
            loaded = CameraCalibration.load(path)
        self.assertEqual(loaded.mappings["cubo"].shared_samples, 10)
        pixels = {name: project(matrix, (0.0, -0.22)) for name, matrix in low.items()}
        self.assertEqual(loaded.mappings["cubo"].locate(pixels, "el cubo rojo"),
                         calibration.mappings["cubo"].locate(pixels, "el cubo rojo"))

    def test_too_few_samples_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "al menos 8"):
            fit_mapping(samples_for(workspace_points(5)))

    def test_locate_refuses_missing_detection_and_points_outside_the_zone(self) -> None:
        points = workspace_points(12)
        calibration = fit_calibration(samples_for(points) + samples_for(points, "recipiente"), SETTINGS)
        located = calibration.locate(scene_features((0.05, -0.25), (-0.1, -0.2)))
        np.testing.assert_allclose(located["cubo"], (0.05, -0.25), atol=1e-3)
        with self.assertRaisesRegex(ValueError, "No se detecta el cubo rojo en la cámara frontal"):
            calibration.locate(scene_features((0.05, -0.25), (-0.1, -0.2), missing={("frontal", "cubo")}))
        with self.assertRaisesRegex(ValueError, "fuera de la zona calibrada"):
            calibration.locate(scene_features((0.30, -0.25), (-0.1, -0.2)))

    def test_container_uses_the_cube_mapping_without_its_own_samples(self) -> None:
        points = workspace_points(12)
        only_cube = fit_calibration(samples_for(points), SETTINGS)
        self.assertNotIn("recipiente", only_cube.mappings)
        located = only_cube.locate(scene_features((0.05, -0.25), (-0.1, -0.2)))
        np.testing.assert_allclose(located["recipiente"], (-0.1, -0.2), atol=1e-3)
        both = fit_calibration(samples_for(points) + samples_for(workspace_points(9, seed=4), "recipiente"), SETTINGS)
        self.assertIn("recipiente", both.mappings)

    def test_the_3d_model_is_saved_and_checked(self) -> None:
        calibration = fit_calibration(samples_for(workspace_points(12)), SETTINGS, model="aaaa")
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "camaras.json"
            calibration.save(path)
            loaded = CameraCalibration.load(path)
        self.assertEqual(loaded.model, "aaaa")
        self.assertEqual(loaded.model_problems("aaaa"), [])
        self.assertIn("otro modelo 3D", " ".join(loaded.model_problems("bbbb")))
        old = fit_calibration(samples_for(workspace_points(12)), SETTINGS)
        self.assertIn("no registra", " ".join(old.model_problems("aaaa")))

    def test_save_load_and_camera_changes(self) -> None:
        calibration = fit_calibration(samples_for(workspace_points(12)), SETTINGS)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "camaras.json"
            calibration.save(path)
            loaded = CameraCalibration.load(path)
        scene = scene_features((0.05, -0.25), (-0.1, -0.2))
        self.assertEqual(loaded.locate(scene), calibration.locate(scene))
        self.assertEqual(len(loaded.samples), 12)
        self.assertEqual(loaded.camera_problems(CONFIG.cameras), [])
        zoomed = dict(CONFIG.cameras)
        zoomed["frontal"] = dataclasses.replace(zoomed["frontal"], zoom_factor=1.5)
        self.assertIn("frontal: zoom", " ".join(loaded.camera_problems(zoomed)))

    def test_object_pixels_reads_the_feature_vector(self) -> None:
        features = scene_features((0.05, -0.25), (-0.1, -0.2), missing={("lateral", "recipiente")})
        self.assertEqual(set(object_pixels(features, "cubo")), {"frontal", "lateral"})
        self.assertEqual(set(object_pixels(features, "recipiente")), {"frontal"})


if __name__ == "__main__":
    unittest.main()
