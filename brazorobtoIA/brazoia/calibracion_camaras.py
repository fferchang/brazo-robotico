"""Calibración de cámaras: dónde está un objeto en la mesa según dónde se ve.

Cada muestra une el centro detectado de un objeto en las dos cámaras
(coordenadas de imagen de 0 a 1, con la rotación y el zoom de la aplicación)
con su posición en la mesa, en metros, en el marco del modelo 3D: el mismo que
usan la simulación y el experto de cinemática inversa.

En el brazo real la posición sale de un agarre: el operador agarra el cubo
donde quedó y el modelo 3D dice dónde está la garra. Así la calibración
también absorbe errores suaves del modelo, porque el experto vuelve a pedir
ángulos parecidos a los del agarre enseñado.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from .vision import FEATURE_NAMES


CALIBRATION_VERSION = 1
CAMERAS = ("frontal", "lateral")
KINDS = ("cubo", "recipiente")
KIND_LABELS = {"cubo": "el cubo rojo", "recipiente": "el recipiente"}
MINIMUM_SAMPLES = 8
RECOMMENDED_SAMPLES = 15
# Margen aceptado alrededor de lo calibrado, en la mesa y en la imagen.
TABLE_MARGIN_M = 0.03
PIXEL_MARGIN = 0.05

# Mapeos candidatos: cámaras que usan y muestras mínimas para ajustarlos.
# Los de reproyección ajustan mesa → imagen en cada cámara y ubican el objeto
# donde las dos imágenes coinciden mejor: cada cámara pesa en la dirección que
# resuelve bien (la frontal, izquierda-derecha; la lateral, cerca-lejos de la
# base). El compartido suma las muestras del otro objeto, corridas en la mesa
# por la diferencia media entre los dos.
CANDIDATES = {
    "homografia_frontal": (("frontal",), 4),
    "homografia_lateral": (("lateral",), 4),
    "homografias_promedio": (CAMERAS, 4),
    "cuadratica_frontal": (("frontal",), 7),
    "cuadratica_lateral": (("lateral",), 7),
    "cuadraticas_promedio": (CAMERAS, 7),
    "conjunta_lineal": (CAMERAS, 6),
    "conjunta_cuadratica": (CAMERAS, 19),
    "reproyeccion": (CAMERAS, 4),
    "reproyeccion_compartida": (CAMERAS, 4),
}
SHARED_MODEL = "reproyeccion_compartida"
# Muestras del otro objeto para que el corrimiento entre los dos sea confiable.
MINIMUM_SHARED_SAMPLES = 4


@dataclass(frozen=True)
class Sample:
    """Una muestra. `pose` son los ángulos del agarre: con ellos se puede recalcular
    `table_xy` si después cambia la calibración del modelo 3D (ceros de M3/M4, M1)."""

    kind: str
    pixels: dict[str, tuple[float, float]]
    table_xy: tuple[float, float]
    source: str = ""
    pose: dict[int, float] | None = None

    def to_dict(self) -> dict[str, Any]:
        result = {
            "tipo": self.kind,
            "pixeles": {camera: list(value) for camera, value in self.pixels.items()},
            "mesa_m": list(self.table_xy),
            "origen": self.source,
        }
        if self.pose is not None:
            result["angulos"] = {str(motor): float(value) for motor, value in sorted(self.pose.items())}
        return result

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Sample":
        angles = data.get("angulos")
        return cls(
            kind=str(data["tipo"]),
            pixels={camera: (float(v[0]), float(v[1])) for camera, v in data["pixeles"].items()},
            table_xy=(float(data["mesa_m"][0]), float(data["mesa_m"][1])),
            source=str(data.get("origen", "")),
            pose=None if angles is None else {int(motor): float(value) for motor, value in angles.items()},
        )


def object_pixels(features: Iterable[float], kind: str) -> dict[str, tuple[float, float]]:
    """Centro de `kind` en cada cámara donde se detectó, desde las 32 características."""
    vector = np.asarray(list(features), dtype=np.float64)
    if vector.shape != (len(FEATURE_NAMES),) or not np.isfinite(vector).all():
        raise ValueError("Las características de la escena son inválidas.")
    pixels = {}
    for camera in CAMERAS:
        if vector[FEATURE_NAMES.index(f"{camera}_{kind}_presente")] >= 0.5:
            pixels[camera] = (
                float(vector[FEATURE_NAMES.index(f"{camera}_{kind}_x")]),
                float(vector[FEATURE_NAMES.index(f"{camera}_{kind}_y")]),
            )
    return pixels


# --------------------------------------------------------------------------
# Mapeos


def _monomials(points: np.ndarray, degree: int) -> np.ndarray:
    columns = [np.ones(len(points)), *(points[:, i] for i in range(points.shape[1]))]
    if degree >= 2:
        columns += [
            points[:, i] * points[:, j]
            for i in range(points.shape[1])
            for j in range(i, points.shape[1])
        ]
    return np.column_stack(columns)


def _fit_polynomial(points: np.ndarray, targets: np.ndarray, degree: int) -> np.ndarray:
    phi = _monomials(points, degree)
    if np.linalg.matrix_rank(phi) < phi.shape[1]:
        raise np.linalg.LinAlgError("Muestras insuficientes o alineadas para este mapeo.")
    return np.linalg.solve(phi.T @ phi + 1e-12 * np.eye(phi.shape[1]), phi.T @ targets)


def _fit_homography(points: np.ndarray, targets: np.ndarray) -> np.ndarray:
    """Transformación DLT normalizada imagen → mesa (Hartley)."""

    def normalizer(values):
        center = values.mean(axis=0)
        scale = math.sqrt(2) / max(np.sqrt(((values - center) ** 2).sum(axis=1)).mean(), 1e-12)
        return np.array([[scale, 0, -scale * center[0]], [0, scale, -scale * center[1]], [0, 0, 1]])

    source, destination = normalizer(points), normalizer(targets)
    a = np.c_[points, np.ones(len(points))] @ source.T
    b = np.c_[targets, np.ones(len(targets))] @ destination.T
    rows = []
    for (x, y, _), (u, v, _) in zip(a, b):
        rows.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        rows.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    if len(points) < 4:
        raise np.linalg.LinAlgError("Una homografía necesita al menos 4 muestras.")
    _, singular, vt = np.linalg.svd(np.asarray(rows))
    # Con rango menor que 8 la solución no es única: muestras alineadas.
    if singular[7] < 1e-10 * singular[0]:
        raise np.linalg.LinAlgError("Muestras alineadas: no definen una homografía.")
    matrix = np.linalg.inv(destination) @ vt[-1].reshape(3, 3) @ source
    if not np.isfinite(matrix).all() or abs(matrix[2, 2]) < 1e-12:
        raise np.linalg.LinAlgError("Homografía degenerada.")
    return matrix / matrix[2, 2]


def _apply_homography(matrix: np.ndarray, point: Sequence[float]) -> np.ndarray:
    result = np.asarray(matrix) @ np.array([point[0], point[1], 1.0])
    if abs(result[2]) < 1e-9:
        raise ValueError("El punto queda en el horizonte de la cámara.")
    return result[:2] / result[2]


def _project_with_jacobian(matrix: np.ndarray, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Punto de la imagen para `xy` en la mesa y su derivada respecto de `xy`."""
    numerator = matrix[:2, :2] @ xy + matrix[:2, 2]
    denominator = float(matrix[2, :2] @ xy + matrix[2, 2])
    if abs(denominator) < 1e-9:
        raise ValueError("El punto queda en el horizonte de la cámara.")
    pixel = numerator / denominator
    return pixel, (matrix[:2, :2] - np.outer(pixel, matrix[2, :2])) / denominator


def _reprojection_locate(forward: dict[str, np.ndarray], pixels: dict[str, tuple[float, float]]) -> np.ndarray:
    """Punto de la mesa que mejor explica lo que ve cada cámara (Gauss-Newton)."""
    position = np.mean(
        [_apply_homography(np.linalg.inv(matrix), pixels[camera]) for camera, matrix in forward.items()], axis=0
    )
    for _ in range(20):
        residuals, jacobians = [], []
        for camera, matrix in forward.items():
            pixel, jacobian = _project_with_jacobian(matrix, position)
            residuals.append(pixel - np.asarray(pixels[camera]))
            jacobians.append(jacobian)
        step = np.linalg.lstsq(np.vstack(jacobians), -np.concatenate(residuals), rcond=None)[0]
        position = position + step
        if not np.isfinite(position).all():
            raise ValueError("No converge la ubicación por reproyección.")
        if np.linalg.norm(step) < 1e-7:
            break
    return position


def _fit_forward(samples: Sequence[Sample], targets: np.ndarray) -> dict[str, np.ndarray]:
    return {
        camera: _fit_homography(targets, np.array([sample.pixels[camera] for sample in samples]))
        for camera in CAMERAS
    }


def _fit_shared(samples: Sequence[Sample], shared: Sequence[Sample]) -> tuple[dict[str, np.ndarray], np.ndarray]:
    """Mesa → imagen con las muestras propias y las del otro objeto, corridas en la mesa.

    El corrimiento junta lo que separa a los dos objetos aunque estén en el mismo
    lugar: el centro detectado de un objeto más grande, o dónde se sostuvo el cubo
    sobre el recipiente. Se estima alternando con el ajuste: cada vuelta el ajuste
    absorbe parte del corrimiento, así que converge en unas 20 vueltas.
    """
    group = [*samples, *shared]
    own = np.array([sample.table_xy for sample in samples])
    other = np.array([sample.table_xy for sample in shared])
    offset = np.zeros(2)
    for _ in range(60):
        forward = _fit_forward(group, np.vstack([own, other + offset]))
        located = np.array([_reprojection_locate(forward, sample.pixels) for sample in shared])
        correction = np.mean(located - (other + offset), axis=0)
        offset = offset + correction
        if np.linalg.norm(correction) < 1e-7:
            break
    return _fit_forward(group, np.vstack([own, other + offset])), offset


def _fit(name: str, samples: Sequence[Sample], shared: Sequence[Sample] = ()) -> dict[str, Any]:
    targets = np.array([sample.table_xy for sample in samples])

    def points(camera):
        return np.array([sample.pixels[camera] for sample in samples])

    if name == "reproyeccion":
        return {camera: matrix.tolist() for camera, matrix in _fit_forward(samples, targets).items()}
    if name == SHARED_MODEL:
        forward, offset = _fit_shared(samples, shared)
        return {**{camera: matrix.tolist() for camera, matrix in forward.items()},
                "corrimiento_compartidas_m": offset.tolist()}
    if name.startswith("homografia_"):
        return {"H": _fit_homography(points(name.split("_")[1]), targets).tolist()}
    if name.startswith("cuadratica_"):
        return {"coef": _fit_polynomial(points(name.split("_")[1]), targets, 2).tolist()}
    if name == "homografias_promedio":
        return {camera: _fit_homography(points(camera), targets).tolist() for camera in CAMERAS}
    if name == "cuadraticas_promedio":
        return {camera: _fit_polynomial(points(camera), targets, 2).tolist() for camera in CAMERAS}
    joint = np.hstack([points(camera) for camera in CAMERAS])
    degree = 1 if name == "conjunta_lineal" else 2
    return {"coef": _fit_polynomial(joint, targets, degree).tolist(), "grado": degree}


def _predict(name: str, params: dict[str, Any], pixels: dict[str, tuple[float, float]]) -> np.ndarray:
    if name in ("reproyeccion", SHARED_MODEL):
        return _reprojection_locate({camera: np.asarray(params[camera]) for camera in CAMERAS}, pixels)
    if name.startswith("homografia_"):
        return _apply_homography(params["H"], pixels[name.split("_")[1]])
    if name.startswith("cuadratica_"):
        point = np.array([pixels[name.split("_")[1]]])
        return (_monomials(point, 2) @ np.asarray(params["coef"]))[0]
    if name == "homografias_promedio":
        return np.mean([_apply_homography(params[camera], pixels[camera]) for camera in CAMERAS], axis=0)
    if name == "cuadraticas_promedio":
        return np.mean(
            [(_monomials(np.array([pixels[camera]]), 2) @ np.asarray(params[camera]))[0] for camera in CAMERAS],
            axis=0,
        )
    joint = np.array([[*pixels["frontal"], *pixels["lateral"]]])
    return (_monomials(joint, int(params["grado"])) @ np.asarray(params["coef"]))[0]


def _loo_errors(name: str, samples: Sequence[Sample], shared: Sequence[Sample] = ()) -> np.ndarray:
    """Error de cada muestra propia ajustando sin ella; las compartidas quedan siempre."""
    errors = np.full(len(samples), math.inf)
    for index, sample in enumerate(samples):
        others = [item for position, item in enumerate(samples) if position != index]
        try:
            prediction = _predict(name, _fit(name, others, shared), sample.pixels)
        except (np.linalg.LinAlgError, ValueError):
            continue
        if np.isfinite(prediction).all():
            errors[index] = float(np.linalg.norm(prediction - np.asarray(sample.table_xy)))
    return errors


def _error_summary(errors: np.ndarray) -> dict[str, float]:
    millimeters = errors * 1000.0
    return {
        "media_mm": float(np.mean(millimeters)),
        "mediana_mm": float(np.median(millimeters)),
        "p90_mm": float(np.percentile(millimeters, 90)),
        "maximo_mm": float(np.max(millimeters)),
    }


@dataclass(frozen=True)
class Mapping:
    """Mapeo elegido para un tipo de objeto, con la zona que cubren sus muestras."""

    model: str
    params: dict[str, Any]
    table_min: tuple[float, float]
    table_max: tuple[float, float]
    pixel_min: dict[str, tuple[float, float]]
    pixel_max: dict[str, tuple[float, float]]
    validation: dict[str, float]
    candidates: dict[str, dict[str, float]]
    samples: int
    shared_samples: int = 0

    @property
    def cameras(self) -> tuple[str, ...]:
        return CANDIDATES[self.model][0]

    def to_dict(self) -> dict[str, Any]:
        return {
            "modelo": self.model,
            "parametros": self.params,
            "mesa_min_m": list(self.table_min),
            "mesa_max_m": list(self.table_max),
            "pixeles_min": {camera: list(value) for camera, value in self.pixel_min.items()},
            "pixeles_max": {camera: list(value) for camera, value in self.pixel_max.items()},
            "validacion_cruzada": self.validation,
            "candidatos": self.candidates,
            "muestras": self.samples,
            "muestras_compartidas": self.shared_samples,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Mapping":
        if data["modelo"] not in CANDIDATES:
            raise ValueError(f"Mapeo desconocido: {data['modelo']}.")
        return cls(
            model=str(data["modelo"]),
            params=data["parametros"],
            table_min=tuple(map(float, data["mesa_min_m"])),
            table_max=tuple(map(float, data["mesa_max_m"])),
            pixel_min={camera: tuple(map(float, v)) for camera, v in data["pixeles_min"].items()},
            pixel_max={camera: tuple(map(float, v)) for camera, v in data["pixeles_max"].items()},
            validation=dict(data["validacion_cruzada"]),
            candidates=dict(data["candidatos"]),
            samples=int(data["muestras"]),
            shared_samples=int(data.get("muestras_compartidas", 0)),
        )

    def locate(self, pixels: dict[str, tuple[float, float]], label: str) -> tuple[float, float]:
        missing = [camera for camera in self.cameras if camera not in pixels]
        if missing:
            raise ValueError(f"No se detecta {label} en la cámara {', '.join(missing)}.")
        for camera in self.cameras:
            low, high = self.pixel_min[camera], self.pixel_max[camera]
            if not all(low[i] - PIXEL_MARGIN <= pixels[camera][i] <= high[i] + PIXEL_MARGIN for i in range(2)):
                raise ValueError(
                    f"{label.capitalize()} se ve en la cámara {camera} fuera de la zona calibrada; "
                    "agregá muestras de calibración en esa zona."
                )
        position = _predict(self.model, self.params, pixels)
        if not np.isfinite(position).all():
            raise ValueError(f"No se pudo ubicar {label}.")
        if not all(
            self.table_min[i] - TABLE_MARGIN_M <= position[i] <= self.table_max[i] + TABLE_MARGIN_M
            for i in range(2)
        ):
            raise ValueError(
                f"{label.capitalize()} queda fuera de la zona calibrada de la mesa; agregá muestras "
                "de calibración en esa zona."
            )
        return float(position[0]), float(position[1])


def _complete(samples: Sequence[Sample]) -> list[Sample]:
    return [sample for sample in samples if all(camera in sample.pixels for camera in CAMERAS)]


def fit_mapping(samples: Sequence[Sample], shared: Sequence[Sample] = ()) -> Mapping:
    """Elige el mapeo con menor error medio al dejar cada muestra afuera.

    `shared` son muestras del otro objeto: solo las usa el mapeo compartido, y el
    error se sigue midiendo con las muestras propias.
    """
    usable = _complete(samples)
    if len(usable) < MINIMUM_SAMPLES:
        raise ValueError(
            f"Se necesitan al menos {MINIMUM_SAMPLES} muestras con detección en las dos cámaras; "
            f"hay {len(usable)}."
        )
    shared_usable = _complete(shared)
    candidates: dict[str, dict[str, float]] = {}
    for name, (_cameras, minimum) in CANDIDATES.items():
        if len(usable) - 1 < minimum:
            continue
        if name == SHARED_MODEL and len(shared_usable) < MINIMUM_SHARED_SAMPLES:
            continue
        errors = _loo_errors(name, usable, shared_usable)
        if np.isfinite(errors).all():
            candidates[name] = _error_summary(errors)
    if not candidates:
        raise ValueError("Ningún mapeo pudo ajustarse: repartí las muestras por toda la zona de trabajo.")
    # Ante errores parecidos (menos de 1 mm), el mapeo más simple: el orden de CANDIDATES.
    best_error = min(item["media_mm"] for item in candidates.values())
    model = next(name for name in CANDIDATES if name in candidates and candidates[name]["media_mm"] <= best_error + 1.0)
    params = _fit(model, usable, shared_usable)
    # La zona calibrada es la que cubren las muestras del ajuste; las compartidas,
    # llevadas al marco de este objeto.
    zone = list(usable)
    targets = [sample.table_xy for sample in usable]
    if model == SHARED_MODEL:
        zone += shared_usable
        offset = np.asarray(params["corrimiento_compartidas_m"])
        targets += [tuple(np.asarray(sample.table_xy) + offset) for sample in shared_usable]
    targets = np.array(targets)
    return Mapping(
        model=model,
        params=params,
        table_min=tuple(map(float, targets.min(axis=0))),
        table_max=tuple(map(float, targets.max(axis=0))),
        pixel_min={camera: tuple(map(float, np.min([s.pixels[camera] for s in zone], axis=0))) for camera in CAMERAS},
        pixel_max={camera: tuple(map(float, np.max([s.pixels[camera] for s in zone], axis=0))) for camera in CAMERAS},
        validation=candidates[model],
        candidates=candidates,
        samples=len(usable),
        shared_samples=len(shared_usable) if model == SHARED_MODEL else 0,
    )


@dataclass(frozen=True)
class CameraCalibration:
    mappings: dict[str, Mapping]
    camera_settings: dict[str, dict[str, Any]]
    samples: tuple[Sample, ...]
    created: str

    def locate(self, features: Iterable[float]) -> dict[str, tuple[float, float]]:
        """Posición del cubo y del recipiente (metros, marco del modelo) en una escena."""
        vector = list(features)
        positions = {}
        for kind in KINDS:
            # Sin muestras propias, el recipiente usa el mapeo del cubo: ambos miden 5 cm
            # de alto y su centro detectado queda a una altura parecida.
            mapping = self.mappings.get(kind) or self.mappings["cubo"]
            positions[kind] = mapping.locate(object_pixels(vector, kind), KIND_LABELS[kind])
        return positions

    def camera_problems(self, cameras_config: dict[str, Any]) -> list[str]:
        """Diferencias de rotación o zoom que invalidan la calibración."""
        problems = []
        for camera in CAMERAS:
            saved = self.camera_settings.get(camera, {})
            current = settings_from_config(cameras_config).get(camera, {})
            for key, label in (("nombre", "cámara"), ("rotacion", "rotación"), ("zoom", "zoom")):
                if saved.get(key) != current.get(key):
                    problems.append(f"{camera}: {label} {saved.get(key)} al calibrar, ahora {current.get(key)}")
        return problems

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": CALIBRATION_VERSION,
            "fecha": self.created,
            "camaras": self.camera_settings,
            "mapeos": {kind: mapping.to_dict() for kind, mapping in self.mappings.items()},
            "muestras": [sample.to_dict() for sample in self.samples],
        }

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        temporary.replace(path)

    @classmethod
    def load(cls, path: Path) -> "CameraCalibration":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if data.get("version") != CALIBRATION_VERSION:
            raise ValueError("Calibración de cámaras de otra versión: volvé a calcularla.")
        mappings = {kind: Mapping.from_dict(value) for kind, value in data["mapeos"].items()}
        if "cubo" not in mappings:
            raise ValueError("La calibración de cámaras no tiene el mapeo del cubo.")
        return cls(
            mappings=mappings,
            camera_settings=dict(data["camaras"]),
            samples=tuple(Sample.from_dict(item) for item in data.get("muestras", [])),
            created=str(data.get("fecha", "")),
        )


def settings_from_config(cameras_config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Lo que cambia la geometría de la imagen: qué cámara, rotación y zoom."""
    return {
        camera: {
            "nombre": cameras_config[camera].name,
            "rotacion": int(cameras_config[camera].rotation_degrees),
            "zoom": float(cameras_config[camera].zoom_factor),
        }
        for camera in CAMERAS
        if camera in cameras_config
    }


def fit_calibration(samples: Sequence[Sample], camera_settings: dict[str, dict[str, Any]]) -> CameraCalibration:
    """Ajusta el cubo (obligatorio) y el recipiente si tiene suficientes muestras propias."""
    cubes = [sample for sample in samples if sample.kind == "cubo"]
    containers = [sample for sample in samples if sample.kind == "recipiente"]
    mappings = {"cubo": fit_mapping(cubes, shared=containers)}
    if len(_complete(containers)) >= MINIMUM_SAMPLES:
        mappings["recipiente"] = fit_mapping(containers, shared=cubes)
    return CameraCalibration(
        mappings=mappings,
        camera_settings=camera_settings,
        samples=tuple(samples),
        created=datetime.now().isoformat(timespec="seconds"),
    )


def describe(calibration: CameraCalibration) -> str:
    lines = []
    for kind in KINDS:
        mapping = calibration.mappings.get(kind)
        if mapping is None:
            lines.append(f"{KIND_LABELS[kind].capitalize()}: usa el mapeo del cubo (sin muestras propias).")
            continue
        error = mapping.validation
        shared = ""
        if mapping.shared_samples:
            other = next(item for item in KINDS if item != kind)
            shared = f" (más {mapping.shared_samples} d{KIND_LABELS[other]})"  # «del recipiente»
        lines.append(
            f"{KIND_LABELS[kind].capitalize()}: {mapping.model}, {mapping.samples} muestras{shared}. Error al dejar "
            f"cada una afuera: mediana {error['mediana_mm']:.0f} mm, 90 % ≤ {error['p90_mm']:.0f} mm, "
            f"máximo {error['maximo_mm']:.0f} mm."
        )
    return "\n".join(lines)
