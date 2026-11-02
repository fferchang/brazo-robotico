from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .config import KEYFRAME_STAGES
from .dataset import load_successful_demonstrations


MODEL_SCHEMA = 1
# Cinco demostraciones permiten comprobar el circuito completo en modo piloto.
# Sigue siendo recomendable reunir muchas más antes de confiar en la política.
MINIMUM_DEMONSTRATIONS = 5
PILOT_DEMONSTRATION_LIMIT = 20
MAXIMUM_ALLOWED_MEAN_MAE_DEGREES = 20.0
MAXIMUM_ALLOWED_STAGE_MAE_DEGREES = 45.0


def _quadratic(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    parts = [np.ones((len(values), 1)), values]
    products = [
        values[:, i : i + 1] * values[:, j : j + 1]
        for i in range(values.shape[1])
        for j in range(i, values.shape[1])
    ]
    parts.extend(products)
    return np.concatenate(parts, axis=1)


def _ridge_fit(phi: np.ndarray, targets: np.ndarray, regularization: float) -> np.ndarray:
    sample_count, feature_count = phi.shape
    if sample_count <= feature_count:
        system = phi @ phi.T + regularization * np.eye(sample_count)
        return phi.T @ np.linalg.solve(system, targets)
    system = phi.T @ phi + regularization * np.eye(feature_count)
    system[0, 0] -= regularization
    return np.linalg.solve(system, phi.T @ targets)


@dataclass(frozen=True)
class TrainingReport:
    demonstrations: int
    training_demonstrations: int
    validation_demonstrations: int
    mean_absolute_error_degrees: float
    maximum_stage_mae_degrees: float
    model_path: Path
    validation_passed: bool = True


class StagePolicy:
    def __init__(
        self,
        mean: np.ndarray,
        scale: np.ndarray,
        weights: list[np.ndarray],
        feature_min: np.ndarray,
        feature_max: np.ndarray,
        canonical_start: np.ndarray,
        canonical_guard: np.ndarray,
        limits: dict[int, tuple[float, float]],
        metadata: dict[str, object],
    ):
        self.mean = mean
        self.scale = scale
        self.weights = weights
        self.feature_min = feature_min
        self.feature_max = feature_max
        self.canonical_start = canonical_start
        self.canonical_guard = canonical_guard
        self.limits = limits
        self.metadata = metadata

    @classmethod
    def load(cls, path: Path) -> "StagePolicy":
        model = np.load(path, allow_pickle=False)
        metadata = json.loads(str(model["metadata_json"].item()))
        if metadata.get("validation_passed") is False or metadata.get("experimental_only") is True:
            model.close()
            raise ValueError("Modelo experimental no validado: disponible para análisis offline, no para mover el brazo.")
        if metadata.get("stages") != list(KEYFRAME_STAGES):
            model.close()
            raise ValueError("El modelo usa otra secuencia de etapas. Entrená con las demostraciones actuales.")
        limits = {
            int(key): (float(value[0]), float(value[1]))
            for key, value in metadata["limits"].items()
        }
        weights = [model[f"weights_{index}"] for index in range(len(KEYFRAME_STAGES))]
        return cls(
            mean=model["mean"],
            scale=model["scale"],
            weights=weights,
            feature_min=model["feature_min"],
            feature_max=model["feature_max"],
            canonical_start=model["canonical_start"],
            canonical_guard=model["canonical_guard"],
            limits=limits,
            metadata=metadata,
        )

    def predict(self, stage_index: int, features: np.ndarray) -> dict[int, float]:
        if not 0 <= stage_index < len(KEYFRAME_STAGES):
            raise IndexError(stage_index)
        vector = np.asarray(features, dtype=np.float64)
        if vector.shape != self.mean.shape:
            raise ValueError("Cantidad de características incompatible con el modelo.")
        normalized = (vector - self.mean) / self.scale
        # Evitar extrapolar muy lejos de las escenas observadas en esta etapa.
        stage_min = (
            self.feature_min[stage_index]
            if self.feature_min.ndim == 2
            else self.feature_min
        )
        stage_max = (
            self.feature_max[stage_index]
            if self.feature_max.ndim == 2
            else self.feature_max
        )
        if np.any(normalized < stage_min - 1.5) or np.any(
            normalized > stage_max + 1.5
        ):
            raise ValueError(
                "La escena está fuera de la distribución enseñada; agregá demostraciones en esta zona."
            )
        phi = _quadratic(normalized.reshape(1, -1))
        predicted = (phi @ self.weights[stage_index])[0]
        result: dict[int, float] = {}
        for offset, motor_id in enumerate(range(1, 6)):
            low, high = self.limits[motor_id]
            result[motor_id] = float(np.clip(predicted[offset], low, high))
        return result


def train_policy(
    demonstrations_root: Path,
    model_path: Path,
    limits: dict[int, tuple[float, float]],
    *,
    seed: int = 101,
    validation_fraction: float = 0.2,
    regularization: float = 0.08,
    export_unvalidated: bool = False,
) -> TrainingReport:
    """Entrena incluyendo avisos de captura. La exportación experimental no habilita ejecución."""
    demonstrations = load_successful_demonstrations(demonstrations_root)
    if len(demonstrations) < MINIMUM_DEMONSTRATIONS:
        raise ValueError(
            f"Se necesitan al menos {MINIMUM_DEMONSTRATIONS} demostraciones exitosas; "
            f"hay {len(demonstrations)}."
        )

    feature_count = len(demonstrations[0]["stages"][0]["features"])
    features = np.empty((len(demonstrations), len(KEYFRAME_STAGES), feature_count))
    targets = np.empty((len(demonstrations), len(KEYFRAME_STAGES), 5))
    for demo_index, demonstration in enumerate(demonstrations):
        for stage_index, stage in enumerate(demonstration["stages"]):
            features[demo_index, stage_index] = np.asarray(stage["features"], dtype=float)
            joints = stage["joint_degrees_logical"]
            targets[demo_index, stage_index] = [float(joints[str(i)]) for i in range(1, 6)]

    rng = np.random.default_rng(seed)
    order = rng.permutation(len(demonstrations))
    validation_count = max(1, int(round(len(order) * validation_fraction)))
    validation_indices = order[:validation_count]
    training_indices = order[validation_count:]

    training_features = features[training_indices].reshape(-1, feature_count)
    mean = training_features.mean(axis=0)
    scale = training_features.std(axis=0)
    scale[scale < 1e-6] = 1.0
    normalized_all = (features - mean) / scale
    feature_min = normalized_all[training_indices].min(axis=0)
    feature_max = normalized_all[training_indices].max(axis=0)

    weights: list[np.ndarray] = []
    stage_maes: list[float] = []
    for stage_index in range(len(KEYFRAME_STAGES)):
        train_x = normalized_all[training_indices, stage_index]
        train_y = targets[training_indices, stage_index]
        weight = _ridge_fit(_quadratic(train_x), train_y, regularization)
        weights.append(weight)
        validation_x = normalized_all[validation_indices, stage_index]
        predicted = _quadratic(validation_x) @ weight
        stage_maes.append(float(np.mean(np.abs(predicted - targets[validation_indices, stage_index]))))

    # Inicio y guardado representan una única pose canónica. Promediar ambas
    # tolera las pequeñas diferencias de una colocación manual tras una pausa,
    # pero hace que la ejecución automática empiece y termine exactamente igual.
    home_samples = np.concatenate(
        (targets[training_indices, 0], targets[training_indices, -1]), axis=0
    )
    canonical_start = home_samples.mean(axis=0)
    canonical_guard = canonical_start.copy()
    metadata = {
        "schema": MODEL_SCHEMA,
        "stages": list(KEYFRAME_STAGES),
        "demonstrations": len(demonstrations),
        "training_demonstrations": len(training_indices),
        "validation_demonstrations": len(validation_indices),
        "stage_mae_degrees": stage_maes,
        "limits": {str(key): list(value) for key, value in limits.items()},
        "seed": seed,
        "source_demonstrations": [Path(item["_path"]).name for item in demonstrations],
        "training_sources": [Path(demonstrations[i]["_path"]).name for i in training_indices],
        "validation_sources": [Path(demonstrations[i]["_path"]).name for i in validation_indices],
        "capture_warnings_policy": "included_without_filtering",
        "captures_with_warnings": sum(bool(stage.get("capture_warnings"))
                                     for demo in demonstrations for stage in demo["stages"]),
        "regularization": regularization,
    }
    mean_mae = float(np.mean(stage_maes))
    maximum_stage_mae = float(np.max(stage_maes))
    validation_passed = not (
        mean_mae > MAXIMUM_ALLOWED_MEAN_MAE_DEGREES
        or maximum_stage_mae > MAXIMUM_ALLOWED_STAGE_MAE_DEGREES
        or not np.isfinite(stage_maes).all()
    )
    metadata.update(validation_passed=validation_passed,
                    experimental_only=not validation_passed,
                    mean_absolute_error_degrees=mean_mae,
                    maximum_stage_mae_degrees=maximum_stage_mae)
    if not validation_passed and not export_unvalidated:
        raise ValueError(
            "Modelo rechazado por validación: "
            f"MAE promedio {mean_mae:.1f}° (máximo permitido "
            f"{MAXIMUM_ALLOWED_MEAN_MAE_DEGREES:.0f}°), peor etapa "
            f"{maximum_stage_mae:.1f}° (máximo permitido "
            f"{MAXIMUM_ALLOWED_STAGE_MAE_DEGREES:.0f}°). "
            "Revisá las demostraciones o agregá muestras coherentes."
        )
    model_path.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, object] = {
        "mean": mean,
        "scale": scale,
        "feature_min": feature_min,
        "feature_max": feature_max,
        "canonical_start": canonical_start,
        "canonical_guard": canonical_guard,
        "metadata_json": np.asarray(json.dumps(metadata)),
    }
    payload.update({f"weights_{index}": weight for index, weight in enumerate(weights)})
    np.savez(model_path, **payload)
    return TrainingReport(
        demonstrations=len(demonstrations),
        training_demonstrations=len(training_indices),
        validation_demonstrations=len(validation_indices),
        mean_absolute_error_degrees=mean_mae,
        maximum_stage_mae_degrees=maximum_stage_mae,
        model_path=model_path,
        validation_passed=validation_passed,
    )

