"""Ejecución sin aprendizaje: cámaras calibradas y cinemática inversa del modelo 3D.

GeometricPolicy tiene la misma interfaz que StagePolicy para SupervisedExecution.
Con la escena de Inicio estima dónde están el cubo y el recipiente y le pide las
doce poses a PickPlaceExpert, el experto de la simulación MuJoCo. Cada etapa
sigue pidiendo confirmación y pasando por el planificador y el firmware.
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .calibracion_camaras import KIND_LABELS, CameraCalibration
from .config import INITIAL_POSE, KEYFRAME_STAGES, AppConfig
from .gripper_geometry import model_fingerprint
from .mujoco_sim import PickPlaceExpert, Placement, PlanningError, VirtualArm


CALIBRATION_FILE = Path("datos") / "calibracion" / "camaras.json"
# Sostenido, el cubo queda apretado contra la cara interna del dedo fijo, donde
# está el punto «pinza» del modelo; su centro queda medio cubo hacia el dedo móvil.
CUBE_HALF_SIZE_M = 0.025


def object_center(arm: VirtualArm, pose: dict[int, float], offset_m: float = CUBE_HALF_SIZE_M) -> tuple[float, float]:
    """Centro (x, y), en metros, de lo que sostiene la garra según el modelo 3D.

    Sirve para el cubo agarrado y para el recipiente: con el cubo sostenido sobre
    su centro, como en Soltado, el experto ubica la garra con el mismo corrimiento.
    """
    data = mujoco.MjData(arm.model)
    for motor in range(1, 7):
        data.qpos[arm.qadr[motor]] = arm.to_qpos(motor, float(pose[motor]))
    mujoco.mj_kinematics(arm.model, data)
    axis = data.site_xmat[arm.site_id].reshape(3, 3)[:2, 0]
    length = float(np.linalg.norm(axis))
    if length < 0.2:
        raise ValueError("La garra no apunta hacia la mesa: no se puede ubicar el objeto.")
    center = data.site_xpos[arm.site_id][:2] - offset_m * axis / length
    return float(center[0]), float(center[1])


class GeometricPolicy:
    def __init__(
        self,
        calibration: CameraCalibration,
        limits: dict[int, tuple[float, float]],
        *,
        arm: VirtualArm | None = None,
        expert: PickPlaceExpert | None = None,
        cube_yaw: float = 0.0,
        container_yaw: float = 0.0,
    ):
        self.calibration = calibration
        self.limits = limits
        self.arm = arm if arm is not None else VirtualArm(limits=limits)
        self.expert = expert if expert is not None else PickPlaceExpert(self.arm)
        home = np.array([INITIAL_POSE[motor] for motor in range(1, 6)], dtype=np.float64)
        self.canonical_start = home
        self.canonical_guard = home.copy()
        # Sin estimar la rotación: el cubo y el recipiente van alineados con la base.
        self.cube_yaw = cube_yaw
        self.container_yaw = container_yaw
        self.metadata = {"tipo": "geometrico", "calibracion": calibration.created}
        self._key: bytes | None = None
        self._placement: Placement | None = None
        self._plan = None

    def placement(self, scene_features) -> Placement:
        positions = self.calibration.locate(scene_features)
        return Placement(positions["cubo"], self.cube_yaw, positions["recipiente"], self.container_yaw)

    def plan(self, scene_features):
        """(ubicación estimada, plan del experto); se calcula una vez por escena."""
        key = np.asarray(scene_features, dtype=np.float64).tobytes()
        if key != self._key:
            placement = self.placement(scene_features)
            try:
                plan = self.expert.plan(placement)
            except PlanningError as exc:
                raise ValueError(f"No hay poses alcanzables para esa ubicación: {exc}") from exc
            self._key, self._placement, self._plan = key, placement, plan
        return self._placement, self._plan

    def predict(self, stage_index: int, scene_features) -> dict[int, float]:
        if not 0 <= stage_index < len(KEYFRAME_STAGES):
            raise IndexError(stage_index)
        _, plan = self.plan(scene_features)
        return self._clip(plan.keyframes[stage_index].pose)

    def via(self, stage_index: int, scene_features) -> list[dict[int, float]]:
        """Puntos previos del experto: descenso y retiro verticales."""
        _, plan = self.plan(scene_features)
        return [self._clip(pose) for pose in plan.keyframes[stage_index].via]

    def describe(self, scene_features) -> str:
        placement, plan = self.plan(scene_features)
        text = (
            f"Según la calibración, {KIND_LABELS['cubo']} está en "
            f"({placement.cube_xy[0] * 1000:.0f}, {placement.cube_xy[1] * 1000:.0f}) mm y "
            f"{KIND_LABELS['recipiente']} en ({placement.container_xy[0] * 1000:.0f}, "
            f"{placement.container_xy[1] * 1000:.0f}) mm."
        )
        if plan.warnings:
            text += " Avisos del plan: " + "; ".join(plan.warnings)
        return text

    def _clip(self, pose: dict[int, float]) -> dict[int, float]:
        return {motor: float(np.clip(pose[motor], *self.limits[motor])) for motor in range(1, 6)}


def calibration_path(config: AppConfig) -> Path:
    return config.root / CALIBRATION_FILE


def load_geometric_policy(config: AppConfig, **kwargs) -> GeometricPolicy:
    path = calibration_path(config)
    if not path.is_file():
        raise ValueError("Falta la calibración de cámaras: hacela con «Calibración · cámaras».")
    calibration = CameraCalibration.load(path)
    problems = calibration.camera_problems(config.cameras)
    if problems:
        raise ValueError(
            "La calibración de cámaras no coincide con la configuración actual ("
            + "; ".join(problems)
            + "). Volvé a calibrar."
        )
    arm = kwargs.get("arm")
    model = model_fingerprint(arm.assets_root if arm is not None else config.root / "assets")
    problems = calibration.model_problems(model)
    if problems:
        raise ValueError(
            "La calibración de cámaras no sirve con el modelo 3D actual (" + "; ".join(problems) + "). "
            "En «Calibración · cámaras», «Cargar capturas…» recalcula solas las capturas con ángulos; "
            "las que no los tienen hay que repetirlas."
        )
    return GeometricPolicy(calibration, config.limits, **kwargs)
