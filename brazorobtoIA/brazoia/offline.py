"""Evaluador de archivos: no posee puerto serie ni implementa una política ejecutable."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .config import KEYFRAME_STAGES
from .policy import CANONICAL_STAGE_INDICES, SCENE_FEATURE_INDICES, _quadratic, scene_problem


class OfflineModel:
    def __init__(self, path: Path):
        self.path = Path(path)
        with np.load(path, allow_pickle=False) as saved:
            self.metadata = json.loads(str(saved["metadata_json"].item()))
            if self.metadata.get("stages") != list(KEYFRAME_STAGES):
                raise ValueError("El modelo no corresponde a las 12 etapas actuales.")
            self.schema = int(self.metadata.get("schema", 1))
            if self.schema >= 2:
                self._load_scene_model(saved)
            else:
                self._load_quadratic_model(saved)
        if self.home.shape != (5,) or self.guard.shape != (5,):
            raise ValueError("Pose canónica inválida.")
        if not np.isfinite(self.home).all() or not np.isfinite(self.guard).all():
            raise ValueError("El modelo contiene valores no finitos.")
        self.limits = {int(k): tuple(map(float, v)) for k, v in self.metadata["limits"].items()}
        for i in range(1, 7):
            low, high = self.limits[i]
            if not np.isfinite([low, high]).all() or low >= high:
                raise ValueError("Límites de motor inválidos.")

    def _load_scene_model(self, saved) -> None:
        """Esquema 2: todas las etapas se predicen desde la escena de Inicio."""
        if not np.array_equal(saved["scene_feature_indices"], SCENE_FEATURE_INDICES):
            raise ValueError("El modelo usa otras características de escena.")
        self.scene_mean = saved["scene_mean"].copy()
        self.scene_min = saved["scene_min"].copy()
        self.scene_max = saved["scene_max"].copy()
        self.scene_margin = saved["scene_margin"].copy()
        self.target_mean = saved["target_mean"].copy()
        self.weights = saved["weights"].copy()
        self.home = saved["canonical_start"].copy()
        self.guard = saved["canonical_guard"].copy()
        count = len(SCENE_FEATURE_INDICES)
        for array, shape in (
            (self.scene_mean, (count,)), (self.scene_min, (count,)), (self.scene_max, (count,)),
            (self.scene_margin, (count,)), (self.target_mean, (12, 5)), (self.weights, (12, count, 5)),
        ):
            if array.shape != shape or not np.isfinite(array).all():
                raise ValueError("Modelo dañado o incompatible.")

    def _load_quadratic_model(self, saved) -> None:
        """Esquema 1 (anterior): cada etapa se predecía con sus propias fotos."""
        self.mean = saved["mean"].copy()
        self.scale = saved["scale"].copy()
        self.feature_min = saved["feature_min"].copy()
        self.feature_max = saved["feature_max"].copy()
        self.weights = [saved[f"weights_{i}"].copy() for i in range(12)]
        self.home = saved["canonical_start"].copy()
        self.guard = saved["canonical_guard"].copy()
        if self.mean.shape != (32,) or self.scale.shape != (32,) or np.any(self.scale <= 0):
            raise ValueError("Normalización incompatible con las 32 características.")
        for array in [self.mean, self.scale, self.feature_min, self.feature_max, *self.weights]:
            if not np.isfinite(array).all():
                raise ValueError("El modelo contiene valores no finitos.")
        if self.feature_min.shape not in ((32,), (12, 32)) or self.feature_max.shape != self.feature_min.shape:
            raise ValueError("Rangos de características inválidos.")
        if any(weight.shape != (561, 5) for weight in self.weights):
            raise ValueError("Pesos incompatibles con el modelo cuadrático.")

    def evaluate(self, index, stage, *, scene_stage=None, demo_name=None):
        if not 0 <= index < 12 or stage.get("stage") != KEYFRAME_STAGES[index]:
            raise ValueError("Etapa fuera de secuencia.")
        features = np.asarray(stage["features"], dtype=float)
        actual = np.asarray([stage["joint_degrees_logical"][str(i)] for i in range(1, 7)], dtype=float)
        if features.shape != (32,) or not np.isfinite(features).all() or not np.isfinite(actual).all():
            raise ValueError("La captura contiene datos inválidos.")
        notes = list(stage.get("capture_warnings", []))
        if self.schema >= 2:
            raw, extra = self._scene_prediction(index, scene_stage, demo_name)
            notes[:0] = extra
            if index in CANONICAL_STAGE_INDICES:
                notes.insert(0, "Inicio y guardado usan la pose canónica del modelo.")
            requested = raw
        else:
            normalized = (features - self.mean) / self.scale
            lower = self.feature_min[index] if self.feature_min.ndim == 2 else self.feature_min
            upper = self.feature_max[index] if self.feature_max.ndim == 2 else self.feature_max
            raw = (_quadratic(normalized.reshape(1, -1)) @ self.weights[index])[0]
            if np.any(normalized < lower - 1.5) or np.any(normalized > upper + 1.5):
                notes.insert(0, "Fuera de distribución: se muestra únicamente para análisis offline.")
            # Inicio/guardado usan la pose canónica en el flujo supervisado; el error
            # del regresor se informa aparte para no ocultarlo detrás de recortes.
            requested = self.home if index == 0 else self.guard if index == 11 else raw
            if index in (0, 11):
                notes.insert(0, "Vista con pose canónica de inicio/guardado; tabla conserva la predicción cruda.")
        if not np.isfinite(raw).all():
            raise ValueError("La predicción no es finita.")
        preview = {i: float(np.clip(requested[i-1], *self.limits[i])) for i in range(1, 6)}
        if any(abs(preview[i] - requested[i-1]) > 1e-8 for i in preview):
            notes.insert(0, "La vista fue recortada a límites; ver valores crudos en la tabla.")
        preview[6] = float(np.clip(actual[5], *self.limits[6]))
        return {"raw": raw, "actual": actual, "preview": preview,
                "mae": float(np.mean(np.abs(raw - actual[:5]))), "notes": notes}

    def _scene_prediction(self, index, scene_stage, demo_name):
        held_out = self.metadata.get("held_out_predictions", {})
        if demo_name is not None and demo_name in held_out:
            raw = np.asarray(held_out[demo_name][index], dtype=float)
            if raw.shape != (5,):
                raise ValueError("Predicción de validación inválida.")
            return raw, ["Validación cruzada: predicción de un modelo entrenado sin esta demostración."]
        if scene_stage is None or scene_stage.get("stage") != KEYFRAME_STAGES[0]:
            raise ValueError("Falta la escena de Inicio de la demostración.")
        scene_features = np.asarray(scene_stage.get("features", []), dtype=float)
        problem = scene_problem(scene_features)
        if problem:
            raise ValueError(problem)
        scene = scene_features[SCENE_FEATURE_INDICES]
        notes = ["Demostración no usada en este entrenamiento: predicción del modelo final."]
        if np.any(scene < self.scene_min - self.scene_margin) or np.any(scene > self.scene_max + self.scene_margin):
            notes.insert(0, "Fuera de distribución: se muestra únicamente para análisis offline.")
        if index == 0:
            return self.home.copy(), notes
        if index == 11:
            return self.guard.copy(), notes
        return self.target_mean[index] + (scene - self.scene_mean) @ self.weights[index], notes
