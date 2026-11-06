from __future__ import annotations

from dataclasses import asdict, dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    kind: str
    center_x: float
    center_y: float
    width: float
    height: float
    area: float
    rectangularity: float
    shape_ratio: float
    confidence: float
    box: tuple[int, int, int, int]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class SceneObservation:
    red_cube: Detection | None
    lime_container: Detection | None

    @property
    def complete(self) -> bool:
        return self.red_cube is not None and self.lime_container is not None

    def to_dict(self) -> dict[str, object | None]:
        return {
            "red_cube": None if self.red_cube is None else self.red_cube.to_dict(),
            "lime_container": (
                None if self.lime_container is None else self.lime_container.to_dict()
            ),
        }


def _clean(mask: np.ndarray) -> np.ndarray:
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def _red_mask(hsv: np.ndarray) -> np.ndarray:
    red_low = cv2.inRange(hsv, np.array([0, 90, 55]), np.array([12, 255, 255]))
    red_high = cv2.inRange(hsv, np.array([168, 90, 55]), np.array([180, 255, 255]))
    return _clean(cv2.bitwise_or(red_low, red_high))


def _candidates(
    mask: np.ndarray,
    kind: str,
    minimum_area_ratio: float,
    minimum_rectangularity: float = 0.45,
) -> list[Detection]:
    height, width = mask.shape[:2]
    frame_area = float(height * width)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result: list[Detection] = []
    for contour in contours:
        area_px = float(cv2.contourArea(contour))
        area_ratio = area_px / frame_area
        if area_ratio < minimum_area_ratio or area_ratio > 0.45:
            continue
        x, y, box_width, box_height = cv2.boundingRect(contour)
        if x <= 1 or y <= 1 or x + box_width >= width - 1 or y + box_height >= height - 1:
            continue
        rectangle_area = float(max(1, box_width * box_height))
        rectangularity = min(1.0, area_px / rectangle_area)
        shape_ratio = min(box_width, box_height) / max(box_width, box_height)
        if rectangularity < minimum_rectangularity:
            continue
        if kind == "red_cube":
            shape_score = shape_ratio
        else:
            # El recipiente puede verse ancho o casi cuadrado según la cámara.
            shape_score = min(1.0, shape_ratio / 0.35)
        confidence = min(
            1.0,
            0.45 * rectangularity
            + 0.35 * shape_score
            + 0.20 * min(1.0, area_ratio / max(minimum_area_ratio * 4.0, 1e-6)),
        )
        result.append(
            Detection(
                kind=kind,
                center_x=(x + box_width / 2.0) / width,
                center_y=(y + box_height / 2.0) / height,
                width=box_width / width,
                height=box_height / height,
                area=area_ratio,
                rectangularity=rectangularity,
                shape_ratio=shape_ratio,
                confidence=confidence,
                box=(x, y, box_width, box_height),
            )
        )
    return result


def detect_scene(
    frame: np.ndarray,
    *,
    minimum_red_area_ratio: float = 0.001,
    minimum_lime_area_ratio: float = 0.004,
    lime_hsv_lower: tuple[int, int, int] | list[int] = (30, 50, 50),
    lime_hsv_upper: tuple[int, int, int] | list[int] = (90, 255, 255),
    minimum_lime_shape_ratio: float = 0.0,
    minimum_lime_confidence: float = 0.0,
) -> SceneObservation:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask = _red_mask(hsv)
    # Verde lima: verde amarillento y saturado. El límite superior evita la
    # pared azulada y el inferior excluye la mesa marrón y objetos grises.
    lime_mask = _clean(
        cv2.inRange(
            hsv,
            np.asarray(lime_hsv_lower, dtype=np.uint8),
            np.asarray(lime_hsv_upper, dtype=np.uint8),
        )
    )

    red = _candidates(red_mask, "red_cube", minimum_red_area_ratio)
    lime = _candidates(lime_mask, "lime_container", minimum_lime_area_ratio)
    lime = [
        item
        for item in lime
        if item.shape_ratio >= minimum_lime_shape_ratio
        and item.confidence >= minimum_lime_confidence
    ]
    red.sort(key=lambda item: (item.confidence, item.area), reverse=True)
    lime.sort(key=lambda item: (item.confidence, item.area), reverse=True)
    return SceneObservation(
        red_cube=red[0] if red else None,
        lime_container=lime[0] if lime else None,
    )


def draw_observation(frame: np.ndarray, observation: SceneObservation) -> np.ndarray:
    output = frame.copy()
    for detection, color, label in (
        (observation.red_cube, (0, 0, 255), "CUBO ROJO"),
        (observation.lime_container, (0, 255, 80), "RECIPIENTE VERDE LIMA"),
    ):
        if detection is None:
            continue
        x, y, width, height = detection.box
        cv2.rectangle(output, (x, y), (x + width, y + height), color, 3)
        cv2.putText(
            output,
            f"{label} {detection.confidence:.2f}",
            (x, max(24, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            output,
            f"{label} {detection.confidence:.2f}",
            (x, max(24, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )
    return output


FEATURE_NAMES = tuple(
    f"{camera}_{kind}_{value}"
    for camera in ("frontal", "lateral")
    for kind in ("cubo", "recipiente")
    for value in ("presente", "x", "y", "w", "h", "area", "forma", "confianza")
)


def observation_features(observations: list[SceneObservation]) -> np.ndarray:
    if len(observations) != 2:
        raise ValueError("Se necesitan las observaciones de ambas cámaras.")
    values: list[float] = []
    for observation in observations:
        for detection in (observation.red_cube, observation.lime_container):
            if detection is None:
                values.extend([0.0] * 8)
                continue
            values.extend(
                [
                    1.0,
                    detection.center_x,
                    detection.center_y,
                    detection.width,
                    detection.height,
                    detection.area,
                    detection.shape_ratio,
                    detection.confidence,
                ]
            )
    return np.asarray(values, dtype=np.float64)


# Dentro del recipiente el cubo asoma solo por arriba de las paredes: en la cámara
# lateral real queda en 0,08–0,36 % de la imagen y en la del gemelo en ~0,044 %,
# menos que el mínimo para buscarlo en la mesa (0,1 %), y a veces recortado.
INSIDE_MINIMUM_RED_AREA_RATIO = 0.0002
INSIDE_MINIMUM_RECTANGULARITY = 0.3
INSIDE_MARGIN = 0.12


@dataclass(frozen=True)
class ContainerCheck:
    """Lo que ve una cámara al final: «dentro», «vacio» o «sin_recipiente»."""

    state: str
    container: Detection | None
    cube: Detection | None

    def to_dict(self) -> dict[str, object | None]:
        return {
            "estado": self.state,
            "recipiente": None if self.container is None else self.container.to_dict(),
            "rojo_dentro": None if self.cube is None else self.cube.to_dict(),
        }


def _inside(item: Detection, container: Detection, margin: float) -> bool:
    return (
        abs(item.center_x - container.center_x) <= container.width * (0.5 + margin)
        and abs(item.center_y - container.center_y) <= container.height * (0.5 + margin)
    )


def check_container(
    frame: np.ndarray, observation: SceneObservation, margin: float = INSIDE_MARGIN
) -> ContainerCheck:
    """Busca rojo dentro del recipiente detectado, no el rojo más grande de la imagen.

    El «cubo» de `detect_scene` es el rojo con más confianza en toda la imagen; al
    final suele ser otra cosa (algo rojo detrás de la mesa) y el cubo, ya dentro,
    queda chico y tapado por las paredes.
    """
    container = observation.lime_container
    if container is None:
        return ContainerCheck("sin_recipiente", None, None)
    red = _candidates(
        _red_mask(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)),
        "red_cube",
        INSIDE_MINIMUM_RED_AREA_RATIO,
        minimum_rectangularity=INSIDE_MINIMUM_RECTANGULARITY,
    )
    inside = sorted(
        (item for item in red if _inside(item, container, margin)),
        key=lambda item: item.area,
        reverse=True,
    )
    if inside:
        return ContainerCheck("dentro", container, inside[0])
    return ContainerCheck("vacio", container, None)


def verify_cube_in_container(
    frames: list[np.ndarray], observations: list[SceneObservation]
) -> tuple[bool, list[ContainerCheck]]:
    """Verificación visual final: alguna cámara ve rojo dentro del recipiente y
    ninguna lo ve vacío. Una cámara que no encuentra el recipiente no decide.

    Con las 15 demos de octubre: 14 de 15 Guardado aprobados (antes 8) y ninguna de
    las 90 fotos con el cubo todavía en la mesa (Inicio a Descenso abierto).
    """
    if len(frames) != 2 or len(observations) != 2:
        return False, []
    checks = [check_container(frame, item) for frame, item in zip(frames, observations)]
    states = [check.state for check in checks]
    return "dentro" in states and "vacio" not in states, checks


