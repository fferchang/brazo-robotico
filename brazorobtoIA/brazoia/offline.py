"""Evaluador de archivos: no posee puerto serie ni implementa una política ejecutable."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .config import KEYFRAME_STAGES
from .policy import _quadratic


class OfflineModel:
    def __init__(self, path: Path):
        self.path = Path(path)
        with np.load(path, allow_pickle=False) as saved:
            self.metadata = json.loads(str(saved["metadata_json"].item()))
            if self.metadata.get("stages") != list(KEYFRAME_STAGES):
                raise ValueError("El modelo no corresponde a las 12 etapas actuales.")
            self.mean = saved["mean"].copy()
            self.scale = saved["scale"].copy()
            self.feature_min = saved["feature_min"].copy()
            self.feature_max = saved["feature_max"].copy()
            self.weights = [saved[f"weights_{i}"].copy() for i in range(12)]
            self.home = saved["canonical_start"].copy()
            self.guard = saved["canonical_guard"].copy()
        if self.mean.shape != (32,) or self.scale.shape != (32,) or np.any(self.scale <= 0):
            raise ValueError("Normalización incompatible con las 32 características.")
        for array in [self.mean, self.scale, self.feature_min, self.feature_max, self.home, self.guard, *self.weights]:
            if not np.isfinite(array).all():
                raise ValueError("El modelo contiene valores no finitos.")
        if self.home.shape != (5,) or self.guard.shape != (5,):
            raise ValueError("Pose canónica inválida.")
        if self.feature_min.shape not in ((32,), (12, 32)) or self.feature_max.shape != self.feature_min.shape:
            raise ValueError("Rangos de características inválidos.")
        if any(weight.shape != (561, 5) for weight in self.weights):
            raise ValueError("Pesos incompatibles con el modelo cuadrático.")
        self.limits = {int(k): tuple(map(float, v)) for k, v in self.metadata["limits"].items()}
        for i in range(1, 7):
            low, high = self.limits[i]
            if not np.isfinite([low, high]).all() or low >= high:
                raise ValueError("Límites de motor inválidos.")

    def evaluate(self, index, stage):
        if not 0 <= index < 12 or stage.get("stage") != KEYFRAME_STAGES[index]:
            raise ValueError("Etapa fuera de secuencia.")
        features = np.asarray(stage["features"], dtype=float)
        actual = np.asarray([stage["joint_degrees_logical"][str(i)] for i in range(1, 7)], dtype=float)
        if features.shape != (32,) or not np.isfinite(features).all() or not np.isfinite(actual).all():
            raise ValueError("La captura contiene datos inválidos.")
        normalized = (features - self.mean) / self.scale
        lower = self.feature_min[index] if self.feature_min.ndim == 2 else self.feature_min
        upper = self.feature_max[index] if self.feature_max.ndim == 2 else self.feature_max
        raw = (_quadratic(normalized.reshape(1, -1)) @ self.weights[index])[0]
        if not np.isfinite(raw).all():
            raise ValueError("La predicción no es finita.")
        notes = list(stage.get("capture_warnings", []))
        if np.any(normalized < lower - 1.5) or np.any(normalized > upper + 1.5):
            notes.insert(0, "Fuera de distribución: se muestra únicamente para análisis offline.")
        # Inicio/guardado usan la pose canónica en el flujo supervisado; el error
        # del regresor se informa aparte para no ocultarlo detrás de recortes.
        requested = self.home if index == 0 else self.guard if index == 11 else raw
        if index in (0, 11):
            notes.insert(0, "Vista con pose canónica de inicio/guardado; tabla conserva la predicción cruda.")
        preview = {i: float(np.clip(requested[i-1], *self.limits[i])) for i in range(1, 6)}
        if any(abs(preview[i] - requested[i-1]) > 1e-8 for i in preview):
            notes.insert(0, "La vista fue recortada a límites; ver valores crudos en la tabla.")
        preview[6] = float(np.clip(actual[5], *self.limits[6]))
        return {"raw": raw, "actual": actual, "preview": preview,
                "mae": float(np.mean(np.abs(raw - actual[:5]))), "notes": notes}
