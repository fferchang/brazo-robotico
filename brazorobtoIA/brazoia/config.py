from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


KEYFRAME_STAGES = (
    "inicio",
    "aproximacion_1",
    "aproximacion_2",
    "aproximacion_3",
    "apertura_sobre_cubo",
    "descenso_abierto",
    "agarre",
    "traslado_1",
    "traslado_2",
    "traslado_3",
    "soltado",
    "guardado",
)

STAGE_LABELS = {
    "inicio": "Inicio",
    "aproximacion_1": "Aproximación 1/3",
    "aproximacion_2": "Aproximación 2/3",
    "aproximacion_3": "Aproximación 3/3",
    "apertura_sobre_cubo": "Abrir sobre el cubo",
    "descenso_abierto": "Bajar con garra abierta",
    "agarre": "Agarre",
    "traslado_1": "Traslado 1/3",
    "traslado_2": "Traslado 2/3",
    "traslado_3": "Traslado 3/3",
    "soltado": "Soltado",
    "guardado": "Guardado",
}

STAGE_INSTRUCTIONS = {
    "inicio": "Garra cerrada y brazo en la posición inicial estándar.",
    "aproximacion_1": "Primer punto entre inicio y cubo; garra cerrada.",
    "aproximacion_2": "Más cerca del cubo; abrir la garra antes de la foto.",
    "aproximacion_3": "Garra cerrada y alineada con el cubo; tomar foto.",
    "apertura_sobre_cubo": "Abrir la garra sobre el cubo y tomar foto.",
    "descenso_abierto": "Bajar la garra abierta al lado del cubo y tomar foto.",
    "agarre": "Cerrar por presión y esperar GRIPPER_HOLD.",
    "traslado_1": "Primer punto de traslado; M6 continúa sosteniendo el cubo.",
    "traslado_2": "Continuar hacia el recipiente sosteniendo el cubo.",
    "traslado_3": "Cubo sostenido sobre el centro del recipiente.",
    "soltado": "Abrir la garra y comprobar que el cubo quedó dentro.",
    "guardado": "Volver a la pose de Inicio con la garra cerrada.",
}

GRIPPER_CLOSED_MAX_DEGREES = 8.0
GRIPPER_OPEN_MIN_DEGREES = 135.0
REFERENCE_POSE = {1: 180.0, 2: 90.0, 3: 270.0, 4: 180.0, 5: 270.0}
INITIAL_POSE = {1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0}


def validate_stage_gripper(stage: str, m6_degrees: float, gripper_state: str) -> str:
    """Devuelve un mensaje de corrección si M6 no coincide con la etapa."""
    if stage in {"inicio", "aproximacion_1", "aproximacion_3", "guardado"}:
        if m6_degrees > GRIPPER_CLOSED_MAX_DEGREES:
            return "Cerrá la garra antes de tomar esta foto."
    elif stage in {"aproximacion_2", "apertura_sobre_cubo", "descenso_abierto", "soltado"}:
        if m6_degrees < GRIPPER_OPEN_MIN_DEGREES:
            return "Abrí completamente la garra antes de tomar esta foto."
    elif stage in {"agarre", "traslado_1", "traslado_2", "traslado_3"}:
        if gripper_state != "holding":
            return "La garra debe indicar GRIPPER_HOLD y sostener el cubo."
    return ""


@dataclass(frozen=True)
class CameraConfig:
    name: str
    fallback_index: int
    rotation_degrees: int
    brightness_percent: float
    contrast_percent: float
    saturation_percent: float
    zoom_factor: float


@dataclass(frozen=True)
class CaptureConfig:
    maximum_camera_age_seconds: float
    maximum_camera_skew_seconds: float
    maximum_telemetry_age_seconds: float
    stable_window_seconds: float
    stable_max_range_degrees: float


@dataclass(frozen=True)
class VisionConfig:
    minimum_red_area_ratio: float
    minimum_lime_area_ratio: float
    frontal_lime_hsv_lower: list[int]
    frontal_lime_hsv_upper: list[int]
    frontal_minimum_lime_area_ratio: float
    frontal_minimum_lime_shape_ratio: float
    frontal_minimum_lime_confidence: float


@dataclass(frozen=True)
class AppConfig:
    root: Path
    port: str
    baud: int
    cameras: dict[str, CameraConfig]
    limits: dict[int, tuple[float, float]]
    capture: CaptureConfig
    vision: VisionConfig

    @property
    def demonstrations_dir(self) -> Path:
        return self.root / "datos" / "demostraciones"

    @property
    def models_dir(self) -> Path:
        return self.root / "modelos"


def load_config(path: Path | None = None) -> AppConfig:
    root = Path(__file__).resolve().parent.parent
    path = path or root / "config.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    capture_raw = dict(raw["capture"])
    legacy_contrast = float(capture_raw.pop("contrast_percent", 0.0))
    cameras = {
        key: CameraConfig(
            name=str(value["name"]),
            fallback_index=int(value["fallback_index"]),
            rotation_degrees=int(value["rotation_degrees"]) % 360,
            brightness_percent=float(value.get("brightness_percent", 0.0)),
            contrast_percent=float(value.get("contrast_percent", legacy_contrast)),
            saturation_percent=float(value.get("saturation_percent", 0.0)),
            zoom_factor=max(1.0, float(value.get("zoom_factor", 1.0))),
        )
        for key, value in raw["cameras"].items()
    }
    limits = {
        int(key): (float(value[0]), float(value[1]))
        for key, value in raw["limits"].items()
    }
    return AppConfig(
        root=root,
        port=str(raw["serial"]["port"]),
        baud=int(raw["serial"]["baud"]),
        cameras=cameras,
        limits=limits,
        capture=CaptureConfig(**capture_raw),
        vision=VisionConfig(**raw["vision"]),
    )
