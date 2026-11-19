"""Demo sin recipiente: el brazo lleva el cubo de un punto de la mesa a otro, en loop.

Cada vuelta es la tarea de siempre (las doce etapas, con el planificador, el
ejecutor y las reglas del firmware) con dos cambios:

- La política ubica solo el cubo con las cámaras calibradas; no busca el recipiente.
- El destino es el punto de DEMO_SPOTS más lejano al cubo, y en Soltado la garra
  baja hasta casi apoyar el cubo en la mesa antes de abrir (TablePlaceExpert).

Así el cubo va y viene entre los dos puntos. Si alguien lo mueve, la vuelta
siguiente lo busca donde esté y lo lleva al punto más lejano.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from .calibracion_camaras import KIND_LABELS, CameraCalibration, object_pixels
from .config import AppConfig
from .mujoco_sim import (
    CLASSIC_EXPERT,
    GRIPPER_HOLD_ESTIMATE,
    TRANSIT_TILTS_DEGREES,
    PickPlaceExpert,
    Placement,
    PlanningError,
    VirtualArm,
    _sweep,
)
from .planner import gripper_sweep_problem
from .politica_geometrica import GeometricPolicy

# Puntos de la mesa entre los que va y viene el cubo (marco base del modelo, metros).
# Están dentro de la zona calibrada de las cámaras reales (cubo: x −93..162 mm,
# y −301..−113 mm) y de las virtuales, a 22 y 25 cm de la base: la tarea real anda
# bien entre 18 y 29 cm. Están 19 cm separados para que se note el traslado.
DEMO_SPOTS = ((-0.060, -0.210), (0.130, -0.210))
# Lo que baja el cubo sostenido antes de soltarlo, sobre la altura del agarre: la base
# del cubo queda a esta altura de la mesa. Se prueba de menor a mayor hasta que abrir
# la garra ahí no barra la mesa (planificador 2D, el mismo que revisa la ejecución).
RELEASE_DROPS = (0.008, 0.015, 0.025, 0.035)
# Al volver a Inicio, el cubo cuenta como «llegó» si se lo ve a menos de esto del destino.
ARRIVAL_RADIUS_MM = 40.0
# Recuperación tras una vuelta fallida: antes de girar M1/M5 hacia Inicio, la garra sube en
# vertical hasta esta altura del punto pinza sobre la mesa (el retiro que hace el experto).
RECOVERY_LIFT_HEIGHT = 0.10


def lift_waypoints(expert, current: dict[int, float], m6: float,
                   top: float = RECOVERY_LIFT_HEIGHT) -> list[dict[int, float]]:
    """Poses para subir la garra en vertical desde `current` hasta `top` (m sobre la mesa),
    con el mismo rumbo y casi sin mover M1 ni M5; [] si ya está a esa altura o más.

    Con la garra abajo (un agarre fallido, el soltado), girar la base primero arrastra lo que
    haya al lado de los dedos. Lanza PlanningError si no hay subida libre de choques."""
    arm = expert.arm
    data = mujoco.MjData(arm.model)
    for motor in range(1, 7):
        data.qpos[arm.qadr[motor]] = arm.to_qpos(motor, current.get(motor, m6 if motor == 6 else 0.0))
    mujoco.mj_kinematics(arm.model, data)
    position = data.site_xpos[arm.site_id].copy()
    rotation = data.site_xmat[arm.site_id].reshape(3, 3)
    height = float(position[2] - arm.floor_z)
    if height >= top - 1e-3:
        return []
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    xy = (float(position[0]), float(position[1]))

    def above(_yaw):
        return xy

    previous = {motor: float(current[motor]) for motor in range(1, 6)}
    via = expert._vertical(above, yaw, height, top, previous, (m6,))
    _, final, _ = expert._solve("subida vertical", above, [top], [yaw], via[-1] if via else previous,
                                grippers=(m6,))
    return [*via, final]
# Recipiente del gemelo, fuera de la mesa y de la vista de las cámaras virtuales.
TWIN_CONTAINER_AWAY = (2.5, 2.5)


def farther_spot(cube_xy, spots=DEMO_SPOTS) -> tuple[float, float]:
    """El destino: el punto más lejano al cubo (si está en uno, el otro)."""
    return max(spots, key=lambda spot: math.hypot(cube_xy[0] - spot[0], cube_xy[1] - spot[1]))


def locate_cube(calibration: CameraCalibration, features) -> tuple[float, float]:
    """Solo el cubo: en la demo no hay recipiente que detectar."""
    mapping = calibration.mappings["cubo"]
    return mapping.locate(object_pixels(list(features), "cubo"), KIND_LABELS["cubo"])


class TablePlaceExpert(PickPlaceExpert):
    """El experto de la tarea, pero suelta el cubo sobre la mesa en `container_xy`.

    Agarre igual (empujando el cubo contra el dedo fijo). Después: Traslado 3 sobre
    el destino a la altura de siempre, Soltado bajando en vertical hasta que la base
    del cubo queda a RELEASE_DROPS de la mesa y Guardado subiendo en vertical con la
    garra abierta antes de volver a Inicio. No hay recipiente que esquivar.
    """

    def plan(self, placement: Placement):
        self.arm.obstacles = False
        return self._plan(placement)

    def _pushes_into_container(self, placement: Placement, yaw: float) -> bool:
        return False

    def _place(self, placement: Placement, lift_xy, yaw, lift, clear):
        dims = self.arm.dimensions
        holding = (GRIPPER_HOLD_ESTIMATE,)
        releasing = _sweep(GRIPPER_HOLD_ESTIMATE, self.open_degrees)
        target = placement.container_xy

        def at_target(release_yaw):
            # Sostenido, el centro del cubo queda medio cubo hacia el dedo móvil.
            return (target[0] + dims.cube_size / 2 * math.cos(release_yaw),
                    target[1] + dims.cube_size / 2 * math.sin(release_yaw))

        reason = "sin rumbo para soltar"
        # Rumbos alineados con la base, empezando por el del agarre: menos giro de M5.
        yaws = sorted((placement.container_yaw + k * math.pi / 2 for k in range(4)),
                      key=lambda value: abs(math.remainder(value - yaw, 2 * math.pi)))
        for release_yaw in yaws:
            try:
                _, above, above_height = self._solve("traslado_3", at_target, [clear, clear + 0.015, clear + 0.03],
                                                     [release_yaw], lift, grippers=holding)
                release, release_height = self._release(at_target, release_yaw, above, releasing)
                down = self._vertical(at_target, release_yaw, above_height, release_height, above, holding)
                up = self._vertical(at_target, release_yaw, release_height, above_height, release,
                                    (self.open_degrees,))
                _, retreat, _ = self._solve("retiro", at_target, [above_height, above_height + 0.03],
                                            [release_yaw], up[-1] if up else release,
                                            tilts=TRANSIT_TILTS_DEGREES, grippers=(self.open_degrees,))
            except PlanningError as exc:
                reason = str(exc)
                continue
            release_xy = at_target(release_yaw)
            middle = ((lift_xy[0] + release_xy[0]) / 2, (lift_xy[1] + release_xy[1]) / 2)
            _, carry, _ = self._solve("traslado_2", lambda _: middle, [0.17, 0.15, 0.13, clear + 0.01],
                                      [yaw, release_yaw], lift, tilts=TRANSIT_TILTS_DEGREES, grippers=holding)
            stages = {"traslado_2": (carry, ()), "traslado_3": (above, ()), "soltado": (release, down),
                      "guardado": (*up, retreat)}
            return stages, []
        raise PlanningError(f"soltado en la mesa: {reason}")

    def _release(self, at_target, release_yaw, above, releasing):
        """La pose más baja de RELEASE_DROPS donde abrir la garra no barre la mesa."""
        problem = ""
        for drop in RELEASE_DROPS:
            height = self.grasp_height + drop
            try:
                _, pose, _ = self._solve("soltado", at_target, [height], [release_yaw], above, grippers=releasing)
            except PlanningError as exc:
                problem = str(exc)
                continue
            # Sin M6 conocido revisa toda la apertura: lo más conservador.
            problem = gripper_sweep_problem((pose[2], pose[3], pose[4]), None, self.open_degrees) or ""
            if not problem:
                return pose, height
        raise PlanningError(f"soltado: {problem}")


class DemoPolicy(GeometricPolicy):
    """Política geométrica de la demo: ubica el cubo y elige el destino solo."""

    def __init__(self, calibration: CameraCalibration, limits, *, arm: VirtualArm | None = None,
                 expert: TablePlaceExpert | None = None, spots=DEMO_SPOTS):
        arm = arm if arm is not None else VirtualArm(limits=limits)
        super().__init__(calibration, limits, arm=arm, expert=expert or TablePlaceExpert(arm))
        self.spots = tuple(spots)
        self.metadata = {"tipo": "demo", "calibracion": calibration.created}
        # `container_xy` es el destino en la mesa, no un recipiente que esquivar.
        self.container_obstacle = False

    def locate_cube(self, scene_features) -> tuple[float, float]:
        return locate_cube(self.calibration, scene_features)

    def placement(self, scene_features) -> Placement:
        cube = self.locate_cube(scene_features)
        return Placement(cube, self.cube_yaw, farther_spot(cube, self.spots), 0.0)

    def describe(self, scene_features) -> str:
        placement, plan = self.plan(scene_features)
        cube, target = placement.cube_xy, placement.container_xy
        text = (f"Demo: {KIND_LABELS['cubo']} está en ({cube[0] * 1000:.0f}, {cube[1] * 1000:.0f}) mm; "
                f"lo llevo a ({target[0] * 1000:.0f}, {target[1] * 1000:.0f}) mm.")
        if plan.warnings:
            text += " Avisos del plan: " + "; ".join(plan.warnings)
        return text

    def summary(self, scene_features) -> dict[str, object]:
        placement, plan = self.plan(scene_features)
        cube, target = placement.cube_xy, placement.container_xy
        return {
            "tipo": "demo",
            "calibracion": self.calibration.created,
            "cubo_mm": [round(cube[0] * 1000, 1), round(cube[1] * 1000, 1)],
            "destino_mm": [round(target[0] * 1000, 1), round(target[1] * 1000, 1)],
            "cubo_a_la_base_cm": round(float(np.hypot(*cube)) * 100, 1),
            "traslado_cm": round(float(np.hypot(cube[0] - target[0], cube[1] - target[1])) * 100, 1),
            "avisos_del_plan": list(plan.warnings),
        }


def arrival_error_mm(policy: DemoPolicy, scene_features, target_xy) -> float:
    """Distancia, según las cámaras, entre el cubo y el destino de la vuelta."""
    cube = policy.locate_cube(scene_features)
    return math.hypot(cube[0] - target_xy[0], cube[1] - target_xy[1]) * 1000.0


def load_demo_policy(config: AppConfig, *, arm: VirtualArm, twin: bool = False, classic: bool = False) -> DemoPolicy:
    """Con la misma calibración (y los mismos controles) que la tarea geométrica.

    `classic`: el experto de antes de las optimizaciones (mujoco_sim.CLASSIC_EXPERT)."""
    if twin:
        from .gemelo import load_twin_policy

        geometric = load_twin_policy(config, arm=arm, expert=PickPlaceExpert(arm))
    else:
        from .politica_geometrica import load_geometric_policy

        geometric = load_geometric_policy(config, arm=arm, expert=PickPlaceExpert(arm))
    expert = TablePlaceExpert(arm, **CLASSIC_EXPERT) if classic else None
    return DemoPolicy(geometric.calibration, config.limits, arm=arm, expert=expert)


def twin_demo_placement(rng: np.random.Generator, spots=DEMO_SPOTS) -> Placement:
    """Escena del gemelo para la demo: el cubo cerca de un punto y sin recipiente a la vista."""
    spot = spots[int(rng.integers(len(spots)))]
    cube = (spot[0] + float(rng.uniform(-0.012, 0.012)), spot[1] + float(rng.uniform(-0.012, 0.012)))
    return Placement(cube, float(rng.uniform(-math.radians(5), math.radians(5))), TWIN_CONTAINER_AWAY, 0.0)
