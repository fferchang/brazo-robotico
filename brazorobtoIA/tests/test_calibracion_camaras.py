"""Calibración cámaras → mesa con datos sintéticos de geometría conocida."""
from __future__ import annotations

import dataclasses
import tempfile
import unittest
from pathlib import Path

import numpy as np

from brazoia.calibracion_camaras import (
    CameraCalibration,
    Sample,
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


def samples_for(points, kind="cubo", noise=0.0, seed=1):
    rng = np.random.default_rng(seed)
    result = []
    for index, xy in enumerate(points):
        pixels = {name: tuple(np.add(project(matrix, xy), rng.normal(0, noise, 2)))
                  for name, matrix in (("frontal", FRONTAL), ("lateral", LATERAL))}
        result.append(Sample(kind, pixels, (float(xy[0]), float(xy[1])), f"prueba {index}"))
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
