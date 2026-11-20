from __future__ import annotations

import json
from dataclasses import dataclass, field
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


def detection_parameters(vision: VisionConfig, camera_index: int) -> dict:
    """Argumentos de `detect_scene` para la cámara 0 (frontal) o 1 (lateral)."""
    frontal = camera_index == 0
    return dict(
        minimum_red_area_ratio=vision.minimum_red_area_ratio,
        minimum_lime_area_ratio=(
            vision.frontal_minimum_lime_area_ratio if frontal else vision.minimum_lime_area_ratio
        ),
        lime_hsv_lower=vision.frontal_lime_hsv_lower if frontal else (30, 50, 50),
        lime_hsv_upper=vision.frontal_lime_hsv_upper if frontal else (90, 255, 255),
        minimum_lime_shape_ratio=vision.frontal_minimum_lime_shape_ratio if frontal else 0.0,
        minimum_lime_confidence=vision.frontal_minimum_lime_confidence if frontal else 0.0,
    )


@dataclass(frozen=True)
class MotionConfig:
    """Sección `movimiento` de config.json. La app la manda al firmware al conectar, si
    el firmware la anuncia en `caps`; con el firmware de la exposición no se usa."""
    # Pasos/s del motor que más recorre (450 ≈ 40°/s). El firmware no acepta más de 1000.
    max_speed: int = 450
    acceleration: int = 45
    # Pasos/s de M6 al abrir (`gripper speed`; 900 ≈ 79°/s). Antes: 450 por ángulo y 300
    # con «gripper open». Cerrar no cambia: el cierre por presión tiene su propio ritmo.
    gripper_open_speed: int = 900
    # Cierre por presión con tramo rápido hasta 55° (execution.GRIPPER_FAST_CLOSE_UNTIL).
    fast_gripper_close: bool = True
    # Supervisor de la fuente: con la tensión más baja de los servos bajo `slow_volts` o la
    # corriente total sobre `slow_ma`, M1–M5 van más despacio, todos por igual; bajo
    # `hold_volts` o sobre `hold_ma` durante 200 ms, el brazo se frena donde está.
    guard_enabled: bool = True
    guard_slow_volts: float = 9.5
    guard_hold_volts: float = 8.0
    guard_slow_ma: int = 3000
    guard_hold_ma: int = 4500


def load_motion(raw: dict) -> MotionConfig:
    guard = raw.get("guardia_fuente", {})
    defaults = MotionConfig()
    motion = MotionConfig(
        max_speed=int(raw.get("velocidad_max", defaults.max_speed)),
        acceleration=int(raw.get("aceleracion", defaults.acceleration)),
        gripper_open_speed=int(raw.get("velocidad_garra_abrir", defaults.gripper_open_speed)),
        fast_gripper_close=bool(raw.get("cierre_rapido", defaults.fast_gripper_close)),
        guard_enabled=bool(guard.get("activa", defaults.guard_enabled)),
        guard_slow_volts=float(guard.get("lento_v", defaults.guard_slow_volts)),
        guard_hold_volts=float(guard.get("corte_v", defaults.guard_hold_volts)),
        guard_slow_ma=int(guard.get("lento_ma", defaults.guard_slow_ma)),
        guard_hold_ma=int(guard.get("corte_ma", defaults.guard_hold_ma)),
    )
    # Los mismos rangos que acepta el firmware (`speed max` y `power guard`).
    if not 100 <= motion.max_speed <= 1000 or not 10 <= motion.acceleration <= 150 \
            or not 100 <= motion.gripper_open_speed <= 1000:
        raise ValueError("movimiento: velocidad_max y velocidad_garra_abrir entre 100 y 1000, "
                         "aceleracion entre 10 y 150.")
    if not 5.0 <= motion.guard_hold_volts < motion.guard_slow_volts <= 13.0 \
            or not 100 <= motion.guard_slow_ma < motion.guard_hold_ma <= 10000:
        raise ValueError("movimiento.guardia_fuente: 5 <= corte_v < lento_v <= 13 y 100 <= lento_ma < corte_ma <= 10000.")
    return motion


@dataclass(frozen=True)
class AppConfig:
    root: Path
    port: str
    baud: int
    cameras: dict[str, CameraConfig]
    limits: dict[int, tuple[float, float]]
    capture: CaptureConfig
    vision: VisionConfig
    motion: MotionConfig = field(default_factory=MotionConfig)

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
        motion=load_motion(raw.get("movimiento", {})),
    )
