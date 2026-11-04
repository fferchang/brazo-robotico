from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .config import KEYFRAME_STAGES
from .dataset import load_successful_demonstrations
from .vision import FEATURE_NAMES


MODEL_SCHEMA = 2
# Cinco demostraciones permiten comprobar el circuito completo en modo piloto.
# Sigue siendo recomendable reunir muchas más antes de confiar en la política.
MINIMUM_DEMONSTRATIONS = 5
PILOT_DEMONSTRATION_LIMIT = 20
MAXIMUM_ALLOWED_MEAN_MAE_DEGREES = 20.0
MAXIMUM_ALLOWED_STAGE_MAE_DEGREES = 45.0

# Todas las etapas se predicen desde la escena de Inicio: es lo que la
# aplicación ve antes de mover el brazo, con el cubo y el recipiente a la vista.
# Las fotos de las etapas siguientes muestran el brazo tapando o llevando el
# cubo, y en la ejecución todavía no existen cuando se predice cada etapa.
SCENE_STAGE_INDEX = 0
# Inicio y guardado usan una única pose canónica; no se predicen.
CANONICAL_STAGE_INDICES = (0, len(KEYFRAME_STAGES) - 1)
SCENE_FEATURE_NAMES = tuple(
    f"{camera}_{kind}_{axis}"
    for camera in ("frontal", "lateral")
    for kind in ("cubo", "recipiente")
    for axis in ("x", "y")
)
SCENE_FEATURE_INDICES = np.asarray([FEATURE_NAMES.index(name) for name in SCENE_FEATURE_NAMES])
_PRESENCE = tuple(
    (camera, kind, FEATURE_NAMES.index(f"{camera}_{kind}_presente"))
    for camera in ("frontal", "lateral")
    for kind in ("cubo", "recipiente")
)
_OBJECT_LABELS = {"cubo": "el cubo rojo", "recipiente": "el recipiente"}
# Penalización ridge, en las unidades de las coordenadas de imagen (0 a 1).
# El extremo alto equivale a predecir la pose media de la etapa.
REGULARIZATION_GRID = tuple(float(value) for value in np.logspace(-6, 1, 15))
# Margen aceptado fuera del rango enseñado: 1,5 desvíos y al menos 3 % de la imagen.
SCENE_MARGIN_STD = 1.5
MINIMUM_SCENE_MARGIN = 0.03


def _quadratic(values: np.ndarray) -> np.ndarray:
    """Expansión del modelo anterior (esquema 1); solo para análisis offline."""
    values = np.asarray(values, dtype=np.float64)
    parts = [np.ones((len(values), 1)), values]
    products = [
        values[:, i : i + 1] * values[:, j : j + 1]
        for i in range(values.shape[1])
        for j in range(i, values.shape[1])
    ]
    parts.extend(products)
    return np.concatenate(parts, axis=1)


def scene_problem(features: object) -> str:
    """Motivo por el que una escena de Inicio no sirve para predecir, o ''."""
    vector = np.asarray(features, dtype=np.float64)
    if vector.shape != (len(FEATURE_NAMES),) or not np.isfinite(vector).all():
        return "Las características de la escena de Inicio son inválidas."
    missing = [
        f"{_OBJECT_LABELS[kind]} en la cámara {camera}"
        for camera, kind, index in _PRESENCE
        if vector[index] < 0.5
    ]
    if missing:
        return "En la escena de Inicio no se detecta " + ", ".join(missing) + "."
    return ""


def _scene_label(index: int) -> str:
    camera, kind, axis = SCENE_FEATURE_NAMES[index].split("_")
    return f"{axis} de {_OBJECT_LABELS[kind]} en la cámara {camera}"


def _loo_mae(scenes: np.ndarray, targets: np.ndarray, alpha: float) -> float:
    """Error de dejar una demostración afuera, exacto para ridge con penalización fija."""
    count = len(scenes)
    centered = scenes - scenes.mean(axis=0)
    system = centered.T @ centered + alpha * np.eye(scenes.shape[1])
    projection = centered @ np.linalg.solve(system, centered.T)
    deviation = targets - targets.mean(axis=0)
    residual = deviation - projection @ deviation
    leverage = 1.0 - (np.diag(projection) + 1.0 / count)
    if np.any(leverage < 1e-9):
        return float("inf")
    return float(np.mean(np.abs(residual / leverage[:, None])))


def _choose_alpha(scenes: np.ndarray, targets: np.ndarray) -> float:
    if len(scenes) < 3:
        return REGULARIZATION_GRID[-1]
    scores = [_loo_mae(scenes, targets, alpha) for alpha in REGULARIZATION_GRID]
    # Ante un empate, la penalización más fuerte.
    best = min(range(len(scores)), key=lambda index: (scores[index], -index))
    return REGULARIZATION_GRID[best]


@dataclass(frozen=True)
class _Fit:
    scene_mean: np.ndarray  # (8,)
    target_mean: np.ndarray  # (12, 5)
    weights: np.ndarray  # (12, 8, 5)
    alphas: np.ndarray  # (12,)
    canonical: np.ndarray  # (5,)


def _fit(scenes: np.ndarray, targets: np.ndarray) -> _Fit:
    """Ridge lineal por etapa sobre ángulos centrados: sin datos, predice la media."""
    stage_count = targets.shape[1]
    scene_mean = scenes.mean(axis=0)
    centered = scenes - scene_mean
    target_mean = targets.mean(axis=0)
    weights = np.zeros((stage_count, scenes.shape[1], targets.shape[2]))
    alphas = np.zeros(stage_count)
    for stage_index in range(stage_count):
        if stage_index in CANONICAL_STAGE_INDICES:
            continue
        stage_targets = targets[:, stage_index]
        alpha = _choose_alpha(scenes, stage_targets)
        system = centered.T @ centered + alpha * np.eye(scenes.shape[1])
        weights[stage_index] = np.linalg.solve(
            system, centered.T @ (stage_targets - target_mean[stage_index])
        )
        alphas[stage_index] = alpha
    # Inicio y guardado representan una única pose canónica. Promediar ambas
    # tolera las pequeñas diferencias de una colocación manual tras una pausa,
    # pero hace que la ejecución automática empiece y termine exactamente igual.
    canonical = np.concatenate(
        [targets[:, index] for index in CANONICAL_STAGE_INDICES], axis=0
    ).mean(axis=0)
    return _Fit(scene_mean, target_mean, weights, alphas, canonical)


def _predict_stages(fit: _Fit, scene: np.ndarray) -> np.ndarray:
    raw = fit.target_mean + np.einsum("f,sfj->sj", scene - fit.scene_mean, fit.weights)
    for index in CANONICAL_STAGE_INDICES:
        raw[index] = fit.canonical
    return raw


@dataclass(frozen=True)
class TrainingReport:
    demonstrations: int
    training_demonstrations: int
    validation_demonstrations: int
    mean_absolute_error_degrees: float
    maximum_stage_mae_degrees: float
    model_path: Path
    validation_passed: bool = True
    excluded_demonstrations: int = 0


class StagePolicy:
    def __init__(
        self,
        *,
        scene_mean: np.ndarray,
        scene_min: np.ndarray,
        scene_max: np.ndarray,
        scene_margin: np.ndarray,
        target_mean: np.ndarray,
        weights: np.ndarray,
        canonical_start: np.ndarray,
        canonical_guard: np.ndarray,
        limits: dict[int, tuple[float, float]],
        metadata: dict[str, object],
    ):
        self.scene_mean = scene_mean
        self.scene_min = scene_min
        self.scene_max = scene_max
        self.scene_margin = scene_margin
        self.target_mean = target_mean
        self.weights = weights
        self.canonical_start = canonical_start
        self.canonical_guard = canonical_guard
        self.limits = limits
        self.metadata = metadata

    @classmethod
    def load(cls, path: Path) -> "StagePolicy":
        with np.load(path, allow_pickle=False) as model:
            metadata = json.loads(str(model["metadata_json"].item()))
            if metadata.get("validation_passed") is False or metadata.get("experimental_only") is True:
                raise ValueError("Modelo experimental no validado: disponible para análisis offline, no para mover el brazo.")
            if metadata.get("stages") != list(KEYFRAME_STAGES):
                raise ValueError("El modelo usa otra secuencia de etapas. Entrená con las demostraciones actuales.")
            if metadata.get("schema") != MODEL_SCHEMA:
                raise ValueError(
                    "Modelo de una versión anterior del entrenamiento: volvé a entrenarlo con las "
                    "demostraciones guardadas."
                )
            arrays = {
                name: model[name].copy()
                for name in (
                    "scene_feature_indices", "scene_mean", "scene_min", "scene_max",
                    "scene_margin", "target_mean", "weights", "canonical_start", "canonical_guard",
                )
            }
        stage_count, scene_count = len(KEYFRAME_STAGES), len(SCENE_FEATURE_INDICES)
        if not np.array_equal(arrays.pop("scene_feature_indices"), SCENE_FEATURE_INDICES):
            raise ValueError("El modelo usa otras características de escena. Volvé a entrenarlo.")
        shapes = {
            "scene_mean": (scene_count,), "scene_min": (scene_count,), "scene_max": (scene_count,),
            "scene_margin": (scene_count,), "target_mean": (stage_count, 5),
            "weights": (stage_count, scene_count, 5), "canonical_start": (5,), "canonical_guard": (5,),
        }
        for name, shape in shapes.items():
            if arrays[name].shape != shape or not np.isfinite(arrays[name]).all():
                raise ValueError(f"Modelo dañado: {name} inválido.")
        limits = {
            int(key): (float(value[0]), float(value[1]))
            for key, value in metadata["limits"].items()
        }
        return cls(**arrays, limits=limits, metadata=metadata)

    def check_scene(self, scene_features: object) -> np.ndarray:
        """Coordenadas de la escena de Inicio, o ValueError si no sirven para predecir."""
        problem = scene_problem(scene_features)
        if problem:
            raise ValueError(problem)
        scene = np.asarray(scene_features, dtype=np.float64)[SCENE_FEATURE_INDICES]
        outside = (scene < self.scene_min - self.scene_margin) | (
            scene > self.scene_max + self.scene_margin
        )
        if np.any(outside):
            labels = ", ".join(_scene_label(index) for index in np.flatnonzero(outside))
            raise ValueError(
                f"La escena de Inicio está fuera de lo enseñado ({labels}); agregá demostraciones "
                "con el cubo y el recipiente en esa zona."
            )
        return scene

    def predict(self, stage_index: int, scene_features: object) -> dict[int, float]:
        """Pose M1–M5 de una etapa a partir de la escena observada en Inicio."""
        if not 0 <= stage_index < len(KEYFRAME_STAGES):
            raise IndexError(stage_index)
        scene = self.check_scene(scene_features)
        if stage_index == CANONICAL_STAGE_INDICES[0]:
            raw = self.canonical_start
        elif stage_index == CANONICAL_STAGE_INDICES[1]:
            raw = self.canonical_guard
        else:
            raw = self.target_mean[stage_index] + (scene - self.scene_mean) @ self.weights[stage_index]
        result: dict[int, float] = {}
        for offset, motor_id in enumerate(range(1, 6)):
            low, high = self.limits[motor_id]
            result[motor_id] = float(np.clip(raw[offset], low, high))
        return result


def _demonstration_arrays(demonstration: dict[str, object]) -> tuple[np.ndarray, np.ndarray, str]:
    """(escena de Inicio, ángulos (12, 5), motivo de exclusión o '')."""
    stages = demonstration["stages"]
    scene = np.asarray(stages[SCENE_STAGE_INDEX].get("features", []), dtype=np.float64)
    problem = scene_problem(scene)
    try:
        joints = np.asarray(
            [[float(stage["joint_degrees_logical"][str(motor)]) for motor in range(1, 6)] for stage in stages],
            dtype=np.float64,
        )
    except (KeyError, TypeError, ValueError):
        return scene, np.empty((0, 5)), problem or "Faltan ángulos M1–M5."
    if not np.isfinite(joints).all():
        problem = problem or "Hay ángulos no finitos."
    return scene, joints, problem


def train_policy(
    demonstrations_root: Path,
    model_path: Path,
    limits: dict[int, tuple[float, float]],
    *,
    export_unvalidated: bool = False,
) -> TrainingReport:
    """Entrena desde la escena de Inicio y valida dejando cada demostración afuera.

    Incluye los avisos de captura. La exportación experimental no habilita ejecución.
    """
    demonstrations = load_successful_demonstrations(demonstrations_root)
    usable, scenes, targets, excluded = [], [], [], []
    for demonstration in demonstrations:
        scene, joints, problem = _demonstration_arrays(demonstration)
        if problem:
            excluded.append({"demostracion": Path(demonstration["_path"]).name, "motivo": problem})
            continue
        usable.append(demonstration)
        scenes.append(scene[SCENE_FEATURE_INDICES])
        targets.append(joints)
    if len(usable) < MINIMUM_DEMONSTRATIONS:
        detail = (
            f" Otras {len(excluded)} no tienen el cubo y el recipiente detectados en Inicio en las "
            "dos cámaras."
            if excluded
            else ""
        )
        raise ValueError(
            f"Se necesitan al menos {MINIMUM_DEMONSTRATIONS} demostraciones exitosas; "
            f"hay {len(usable)} utilizables.{detail}"
        )
    names = [Path(item["_path"]).name for item in usable]
    scene_array = np.stack(scenes)
    target_array = np.stack(targets)

    # Cada demostración se predice con un modelo entrenado sin ella, incluida la
    # elección de la penalización. Estima el error ante una escena nueva.
    held_out = np.empty_like(target_array)
    for index in range(len(usable)):
        keep = np.arange(len(usable)) != index
        held_out[index] = _predict_stages(_fit(scene_array[keep], target_array[keep]), scene_array[index])
    stage_maes = np.abs(held_out - target_array).mean(axis=(0, 2))
    final = _fit(scene_array, target_array)

    metadata: dict[str, object] = {
        "schema": MODEL_SCHEMA,
        "stages": list(KEYFRAME_STAGES),
        "demonstrations": len(usable),
        "training_demonstrations": len(usable),
        "validation_demonstrations": len(usable),
        "validation": "leave_one_demonstration_out",
        "stage_mae_degrees": stage_maes.tolist(),
        "limits": {str(key): list(value) for key, value in limits.items()},
        "scene_stage": KEYFRAME_STAGES[SCENE_STAGE_INDEX],
        "scene_features": list(SCENE_FEATURE_NAMES),
        "regularization": final.alphas.tolist(),
        "source_demonstrations": names,
        "training_sources": names,
        # Todas se validaron una vez, cada una con un modelo entrenado sin ella.
        "validation_sources": names,
        "held_out_predictions": {
            name: np.round(held_out[index], 3).tolist() for index, name in enumerate(names)
        },
        "excluded_demonstrations": excluded,
        "capture_warnings_policy": "included_without_filtering",
        "captures_with_warnings": sum(
            bool(stage.get("capture_warnings")) for demo in usable for stage in demo["stages"]
        ),
    }
    mean_mae = float(np.mean(stage_maes))
    maximum_stage_mae = float(np.max(stage_maes))
    validation_passed = bool(
        np.isfinite(stage_maes).all()
        and mean_mae <= MAXIMUM_ALLOWED_MEAN_MAE_DEGREES
        and maximum_stage_mae <= MAXIMUM_ALLOWED_STAGE_MAE_DEGREES
    )
    metadata.update(
        validation_passed=validation_passed,
        experimental_only=not validation_passed,
        mean_absolute_error_degrees=mean_mae,
        maximum_stage_mae_degrees=maximum_stage_mae,
    )
    if not validation_passed and not export_unvalidated:
        raise ValueError(
            "Modelo rechazado por validación cruzada: "
            f"MAE promedio {mean_mae:.1f}° (máximo permitido "
            f"{MAXIMUM_ALLOWED_MEAN_MAE_DEGREES:.0f}°), peor etapa "
            f"{maximum_stage_mae:.1f}° (máximo permitido "
            f"{MAXIMUM_ALLOWED_STAGE_MAE_DEGREES:.0f}°). "
            "Revisá las demostraciones o agregá muestras coherentes."
        )
    model_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        model_path,
        scene_feature_indices=SCENE_FEATURE_INDICES,
        scene_mean=final.scene_mean,
        scene_min=scene_array.min(axis=0),
        scene_max=scene_array.max(axis=0),
        scene_margin=np.maximum(MINIMUM_SCENE_MARGIN, SCENE_MARGIN_STD * scene_array.std(axis=0)),
        target_mean=final.target_mean,
        weights=final.weights,
        canonical_start=final.canonical,
        canonical_guard=final.canonical.copy(),
        metadata_json=np.asarray(json.dumps(metadata)),
    )
    return TrainingReport(
        demonstrations=len(usable),
        training_demonstrations=len(usable),
        validation_demonstrations=len(usable),
        mean_absolute_error_degrees=mean_mae,
        maximum_stage_mae_degrees=maximum_stage_mae,
        model_path=model_path,
        validation_passed=validation_passed,
        excluded_demonstrations=len(excluded),
    )
