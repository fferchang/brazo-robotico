"""Escena física en MuJoCo y experto por cinemática inversa; solo virtual.

No importa el controlador serie, no abre puertos ni usa cámaras reales.
Unidades internas: metros y radianes de MuJoCo. Hacia afuera se usan los
grados lógicos M1–M6 de la aplicación, con la conversión de calibracion.json
que usa Rerun. El marco del mundo es el marco `base` del URDF; la mesa está
en `GripperGeometry.floor_z` (altura del eje M2 calibrada con toques de mesa),
igual que el diagnóstico 3D.

Restricciones duras: que las piezas no se atraviesen y que el brazo no
atraviese la mesa. Los límites de config.json y el planificador 2D están mal
ajustados en algunos motores; acá solo se informan como avisos.
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from .config import INITIAL_POSE, KEYFRAME_STAGES, load_config
from .gripper_geometry import GripperGeometry, joint_angle, origin_transform, stl_vertices
from .planner import classify_pose


ROOT = Path(__file__).resolve().parent.parent

# Bits de contacto. Eslabones y garra chocan entre sí (MuJoCo excluye las piezas
# contiguas) y con la mesa. El hombro se excluye de la base, con la que comparte
# el eje de M1, y por lo tanto también de la mesa, que está en el mismo cuerpo.
ENV, ARM, GRIPPER = 1, 2, 4
ROBOT = ARM | GRIPPER

GRIPPER_OPEN = 137.0        # Apertura medida en las demostraciones reales.
GRIPPER_CLOSED = 0.0
GRIPPER_HOLD_ESTIMATE = 40.0  # Solo para revisar trayectos; la física decide el valor real.
GRIPPER_SWEEP_STEP = 8.0    # Cada cuántos grados de M6 se revisa un barrido de la garra al planificar.
GRIPPER_STEP = 1.5          # Mismo paso y período que el firmware.
GRIPPER_STEP_SECONDS = 0.04
GRIPPER_INRUSH_SECONDS = 0.25
GRIPPER_TIMEOUT_SECONDS = 15.0
GRIPPER_CONTACT_TORQUE = 0.30  # N·m; sustituye al umbral de corriente real.
GRIPPER_CONTACT_SAMPLES = 3

# Margen garra-mesa de las poses planificadas; con la calibración de toques de
# mesa el modelo le erra a la altura de la garra por ~2 mm RMS.
FLOOR_MARGIN_MM = 5.0
# Separación mínima entre piezas al planificar. Se revisa en una copia del modelo
# con `margin`: en la física ese margen altera el agarre (el dedo móvil cerraba
# ~6° menos), así que la física usa el modelo sin margen.
PART_CLEARANCE = 0.003
# Separación mínima entre el brazo y el recipiente al planificar. La ubicación del
# recipiente sale de las cámaras, con ~8 mm de error típico (docs/CALIBRACION_CAMARAS.md).
CONTAINER_CLEARANCE = 0.012

# Inclinaciones de la garra hacia afuera que prueba la IK, en orden de preferencia.
# Cerca del cubo y del recipiente se exige garra casi vertical; en tránsito, no.
IK_LIMIT_MARGIN_DEGREES = 1.0
IK_TILTS_DEGREES = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, -5.0, -10.0)
TRANSIT_TILTS_DEGREES = (0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, -10.0, -20.0)

# Punto de agarre: cara interna del dedo fijo, 6 mm por encima de su punta.
FINGER_Y = -0.100

STAGE_GRIPPER_ACTION = {
    "aproximacion_2": "open",
    "aproximacion_3": "close",
    "apertura_sobre_cubo": "open",
    "agarre": "close",
    "soltado": "open",
    "guardado": "close",
}

PART_NAMES = {"world": "base", "shoulder": "hombro", "upper_arm": "brazo", "lower_arm": "antebrazo",
              "wrist": "muñeca", "gripper": "garra", "jaw": "dedo móvil"}


def joint_to_logical(config, motor_id, radians):
    """Inversa exacta de `joint_angle`."""
    servo = (math.degrees((radians - float(config["urdf_zero_radians"])
                           - math.radians(float(config.get("visual_offset_degrees", 0.0))))
                          / float(config["direction"]))
             + float(config["servo_zero_degrees"]))
    return servo - (10.0 if motor_id == 6 else 0.0)


def physical_limits(geometry: GripperGeometry, limits):
    """Rango simulado: lo que permitan config.json o el URDF, dentro del recorrido del servo.

    Los topes del URDF aproximan dónde chocan piezas contiguas; el resto de las
    autocolisiones se detecta con envolventes convexas.
    """
    result = {}
    for motor, config in geometry.calibration.items():
        joint = geometry.joints[str(config["joint"])]
        urdf = sorted(joint_to_logical(config, motor, value) for value in (joint["lower"], joint["upper"]))
        result[motor] = (max(0.0, min(limits[motor][0], urdf[0])), min(360.0, max(limits[motor][1], urdf[1])))
    return result


def current_rule_warnings(pose, limits):
    """Reglas actuales del brazo real que rechazarían la pose; solo informativo."""
    warnings = [f"M{motor}={pose[motor]:.1f}° fuera de {low:g}–{high:g}° (config.json)"
                for motor, (low, high) in sorted(limits.items())
                if motor in pose and not low <= pose[motor] <= high]
    allowed, reason = classify_pose(tuple(pose[motor] for motor in (2, 3, 4)), pose.get(6))
    if not allowed:
        warnings.append(f"planificador 2D: {reason}")
    return warnings


def _quat(rotation_matrix):
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.asarray(rotation_matrix, dtype=float).flatten())
    return quat


def _look_at(position, target):
    """Cuaternión de cámara MuJoCo (mira hacia -z, y hacia arriba)."""
    forward = np.asarray(target, float) - np.asarray(position, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    back = -forward
    return _quat(np.column_stack([right, np.cross(back, right), back]))


def _yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def _sweep(start, end, step=GRIPPER_SWEEP_STEP):
    """Valores de M6 de `start` a `end`, ambos incluidos, a no más de `step` grados."""
    count = max(1, math.ceil(abs(end - start) / step))
    return tuple(start + (end - start) * index / count for index in range(count + 1))


@dataclass(frozen=True)
class SceneDimensions:
    """Medidas reales: cubo de 5 cm; caja de 10 × 10 cm por fuera, 5 cm de alto y paredes
    de 5 mm. La masa del cubo y el espesor del fondo son estimados."""
    cube_size: float = 0.050
    cube_mass: float = 0.060
    container_inner: tuple[float, float] = (0.090, 0.090)
    container_wall_height: float = 0.050
    container_wall_thickness: float = 0.005
    container_floor_thickness: float = 0.005

    @property
    def container_outer(self):
        return tuple(size + 2 * self.container_wall_thickness for size in self.container_inner)


@dataclass(frozen=True)
class Placement:
    cube_xy: tuple[float, float]
    cube_yaw: float
    container_xy: tuple[float, float]
    container_yaw: float


@dataclass(frozen=True)
class IkSolution:
    pose: dict[int, float]
    position_error_mm: float
    orientation_error_deg: float
    tilt_deg: float = 0.0
    problems: tuple[str, ...] = ()  # Piezas que se atraviesan o garra bajo el margen.

    @property
    def converged(self):
        return self.position_error_mm <= 1.0 and self.orientation_error_deg <= 1.0

    @property
    def ok(self):
        return self.converged and not self.problems


@dataclass(frozen=True)
class Keyframe:
    stage: str
    pose: dict[int, float]      # M1–M5; M6 la decide la acción de garra.
    gripper_action: str | None
    via: tuple[dict[int, float], ...] = ()  # Puntos previos: descenso y retiro verticales.


@dataclass(frozen=True)
class Plan:
    keyframes: list[Keyframe]
    warnings: list[str]


class PlanningError(ValueError):
    pass


@dataclass
class StageRecord:
    stage: str
    commanded: dict[int, float]
    measured: dict[int, float]
    gripper_state: str
    cube_position: list[float]
    rule_warnings: list[str]


@dataclass
class EpisodeResult:
    placement: Placement
    success: bool
    reasons: list[str] = field(default_factory=list)
    records: list[StageRecord] = field(default_factory=list)
    plan_warnings: list[str] = field(default_factory=list)
    holding_m6: float | None = None
    contacts: dict[str, int] = field(default_factory=dict)
    minimum_keyframe_clearance_mm: float = math.inf
    simulated_seconds: float = 0.0


def _robot_from_urdf(spec, assets_root, calibration, joint_limits, clearance):
    """Agrega cuerpos, articulaciones y mallas del URDF; devuelve x de la cara del dedo fijo."""
    model_path = assets_root / "modelo" / "so100.urdf"
    root = ET.parse(model_path).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    by_parent = {}
    for joint in root.findall("joint"):
        by_parent.setdefault(joint.find("parent").get("link"), []).append(joint)
    colors = {"3d_printed": [0.93, 0.93, 0.9, 1.0], "sts3215": [0.1, 0.1, 0.1, 1.0]}
    mesh_root = model_path.parent
    motor_by_joint = {str(config["joint"]): motor for motor, config in calibration.items()}

    def add_mesh_geoms(body, link_name):
        for index, visual in enumerate(links[link_name].findall("visual")):
            filename = visual.find("geometry/mesh").get("filename")
            name = f"{link_name}_{index}"
            spec.add_mesh(name=name, file=str((mesh_root / filename).resolve()))
            material = visual.find("material")
            rgba = colors.get(material.get("name") if material is not None else "", [0.8] * 3 + [1])
            # Cada pieza colisiona con su envolvente convexa; la garra usa dedos recortados.
            collides = link_name not in ("gripper", "jaw")
            body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name, rgba=rgba,
                          group=1 if collides else 2, contype=ARM if collides else 0,
                          conaffinity=ROBOT if collides else 0, margin=clearance, gap=clearance)

    def add_hull(body, name, vertices):
        spec.add_mesh(name=name, uservert=np.asarray(vertices, float).flatten().tolist())
        body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name,
                      group=3, rgba=[0.2, 0.6, 1.0, 0.25], contype=GRIPPER, conaffinity=ARM,
                      condim=4, friction=[1.0, 0.01, 0.001], margin=clearance, gap=clearance)

    def build(parent_body, link_name):
        for joint in by_parent.get(link_name, []):
            child = joint.find("child").get("link")
            transform = origin_transform(joint.find("origin"))
            body = parent_body.add_body(name=child, pos=transform[:3, 3], quat=_quat(transform[:3, :3]))
            inertial = links[child].find("inertial")
            body.mass = float(inertial.find("mass").get("value"))
            body.ipos = np.fromstring(inertial.find("origin").get("xyz"), sep=" ")
            inertia = inertial.find("inertia")
            body.inertia = [float(inertia.get(key)) for key in ("ixx", "iyy", "izz")]
            body.explicitinertial = True
            motor = motor_by_joint[joint.get("name")]
            low, high = sorted(joint_angle(calibration[motor], motor, value) for value in joint_limits[motor])
            body.add_joint(name=f"M{motor}", type=mujoco.mjtJoint.mjJNT_HINGE,
                           axis=np.fromstring(joint.find("axis").get("xyz"), sep=" "),
                           range=[low, high], damping=0.1 if motor == 6 else 1.5,
                           armature=0.002 if motor == 6 else 0.02)
            add_mesh_geoms(body, child)
            build(body, child)

    add_mesh_geoms(spec.worldbody, "base")
    build(spec.worldbody, "base")
    # El hombro es hijo directo del mundo: MuJoCo no lo excluye automáticamente.
    spec.add_exclude(bodyname1="world", bodyname2="shoulder")

    # Colisión de garra: envolventes de la parte de dedo de cada malla real.
    fixed = stl_vertices(mesh_root / "assets" / "Fixed_Jaw.stl")
    motor = stl_vertices(mesh_root / "assets" / "Fixed_Jaw_Motor.stl")
    moving = stl_vertices(mesh_root / "assets" / "Moving_Jaw.stl")
    gripper, jaw = spec.body("gripper"), spec.body("jaw")
    add_hull(gripper, "dedo_fijo", fixed[fixed[:, 1] < -0.045])
    add_hull(gripper, "cuerpo_garra", np.vstack([fixed[fixed[:, 1] >= -0.045], motor]))
    add_hull(jaw, "dedo_movil", moving[moving[:, 1] < -0.035])
    add_hull(jaw, "raiz_dedo_movil", moving[moving[:, 1] >= -0.035])
    band = fixed[(fixed[:, 1] < FINGER_Y + 0.006) & (fixed[:, 1] > FINGER_Y - 0.006)]
    inner_x = float(band[:, 0].min())
    gripper.add_site(name="pinza", pos=[inner_x, FINGER_Y, 0.0], size=[0.003, 0.003, 0.003],
                     rgba=[1, 0, 1, 1], group=4)
    return inner_x


class VirtualArm:
    """Brazo SO-100, mesa, cubo y recipiente en MuJoCo. No envía órdenes reales."""

    def __init__(self, assets_root: Path | None = None, *, limits=None,
                 dimensions: SceneDimensions | None = None, timestep=0.002,
                 floor_margin_mm=FLOOR_MARGIN_MM):
        self.assets_root = Path(assets_root or ROOT / "assets")
        self.limits = limits or load_config().limits  # Reglas actuales: solo avisos.
        self.dimensions = dimensions or SceneDimensions()
        self.floor_margin_mm = floor_margin_mm
        self.geometry = GripperGeometry(self.assets_root)
        self.calibration = self.geometry.calibration
        self.joint_limits = physical_limits(self.geometry, self.limits)
        self.floor_z = self.geometry.floor_z
        self.model = self._build(timestep, 0.0)
        self.data = mujoco.MjData(self.model)
        # Misma escena con margen de contacto, solo para IK y revisión de poses.
        self.check_model = self._build(timestep, PART_CLEARANCE)
        self._ik_data = mujoco.MjData(self.check_model)
        self._check_data = mujoco.MjData(self.check_model)
        self.joint_ids = {i: self.model.joint(f"M{i}").id for i in range(1, 7)}
        self.qadr = {i: int(self.model.jnt_qposadr[j]) for i, j in self.joint_ids.items()}
        self.dofadr = {i: int(self.model.jnt_dofadr[j]) for i, j in self.joint_ids.items()}
        self.actuator_ids = {i: self.model.actuator(f"M{i}").id for i in range(1, 7)}
        self.site_id = self.model.site("pinza").id
        self.cube_body = self.model.body("cubo").id
        self.cube_qadr = int(self.model.jnt_qposadr[self.model.body_jntadr[self.cube_body]])
        self.container_mocap = int(self.model.body_mocapid[self.model.body("recipiente").id])
        self.container_geoms = frozenset(index for index in range(self.model.ngeom)
                                         if self.model.geom(index).name.startswith("recipiente_"))
        # La IK y la revisión de tramos evitan el recipiente una vez ubicado con set_obstacles.
        self.obstacles = False
        self.floor_geom = self.model.geom("mesa").id
        self.m1_axis_xy = np.array(self.model.body("shoulder").pos[:2])
        self.ranges = {i: tuple(self.model.jnt_range[j]) for i, j in self.joint_ids.items()}
        # La IK planifica dentro de los límites del brazo real para que el plan sea
        # ejecutable; la física conserva los topes físicos.
        # Con 1° de margen en M1–M5: un servo ordenado justo al límite queda unas décimas
        # afuera con la carga, y el firmware rechaza los `pair` siguientes (lo mostró el
        # gemelo digital con M4 = 278°).
        self.ik_ranges = {}
        for motor, (low, high) in self.ranges.items():
            margin = IK_LIMIT_MARGIN_DEGREES if motor != 6 else 0.0
            low_limit, high_limit = self.limits[motor]
            wanted = sorted(self.to_qpos(motor, value) for value in (low_limit + margin, high_limit - margin))
            self.ik_ranges[motor] = (max(low, wanted[0]), min(high, wanted[1]))
        self.commanded = {**INITIAL_POSE, 6: GRIPPER_CLOSED}
        self.observer = None  # callable(arm) llamado cada 20 ms simulados.
        self.contacts = Counter()

    # ----- construcción -----
    def _build(self, timestep, clearance):
        spec = mujoco.MjSpec()
        spec.modelname = "brazo_ia_simulacion"
        spec.compiler.degree = False
        spec.compiler.autolimits = True
        spec.option.timestep = timestep
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        spec.option.impratio = 10
        spec.visual.global_.offwidth = 640
        spec.visual.global_.offheight = 480
        dims = self.dimensions

        spec.add_texture(name="fondo", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                         builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                         rgb1=[0.85, 0.87, 0.9], rgb2=[0.45, 0.47, 0.5], width=256, height=256)
        spec.worldbody.add_light(pos=[0.3, -0.6, 1.2], dir=[-0.25, 0.45, -1.0],
                                 diffuse=[0.7, 0.7, 0.7], castshadow=True)
        spec.worldbody.add_light(pos=[-0.4, 0.2, 0.9], dir=[0.4, -0.3, -1.0],
                                 diffuse=[0.35, 0.35, 0.35], castshadow=False)
        spec.worldbody.add_geom(name="mesa", type=mujoco.mjtGeom.mjGEOM_PLANE,
                                pos=[0, -0.15, self.floor_z], size=[0.45, 0.35, 0.01],
                                rgba=[0.72, 0.6, 0.45, 1], contype=ENV, conaffinity=ENV | ROBOT)
        self.finger_inner_x = _robot_from_urdf(spec, self.assets_root, self.calibration, self.joint_limits,
                                               clearance)

        cube = spec.worldbody.add_body(name="cubo", pos=[0, -0.25, self.floor_z + dims.cube_size / 2])
        cube.add_freejoint(name="cubo")
        cube.add_geom(name="cubo", type=mujoco.mjtGeom.mjGEOM_BOX, size=[dims.cube_size / 2] * 3,
                      mass=dims.cube_mass, rgba=[0.85, 0.22, 0.1, 1], contype=ENV,
                      conaffinity=ENV | ROBOT, condim=4, friction=[1.0, 0.01, 0.001])

        container = spec.worldbody.add_body(name="recipiente", pos=[0.12, -0.22, self.floor_z], mocap=True)
        (inner_x, inner_y), (outer_x, outer_y) = dims.container_inner, dims.container_outer
        wall, height, bottom = dims.container_wall_thickness, dims.container_wall_height, dims.container_floor_thickness
        boxes = [("fondo", [0, 0, bottom / 2], [outer_x / 2, outer_y / 2, bottom / 2])]
        for sign in (-1, 1):
            boxes.append((f"pared_x{sign:+d}", [sign * (inner_x + wall) / 2, 0, height / 2],
                          [wall / 2, outer_y / 2, height / 2]))
            boxes.append((f"pared_y{sign:+d}", [0, sign * (inner_y + wall) / 2, height / 2],
                          [inner_x / 2, wall / 2, height / 2]))
        # En el modelo de revisión el recipiente detecta el brazo desde CONTAINER_CLEARANCE.
        container_margin = CONTAINER_CLEARANCE if clearance > 0 else 0.0
        for name, pos, size in boxes:
            container.add_geom(name=f"recipiente_{name}", type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=size,
                               rgba=[0.62, 0.85, 0.16, 1], contype=ENV, conaffinity=ENV | ROBOT,
                               margin=container_margin, gap=container_margin)

        # Posiciones aproximadas de las cámaras reales; falta calibrarlas.
        for name, position, target in (("frontal", [0.0, -0.62, self.floor_z + 0.13], [0.0, -0.15, self.floor_z + 0.12]),
                                       ("lateral", [-0.62, -0.20, self.floor_z + 0.12], [0.0, -0.20, self.floor_z + 0.12])):
            spec.worldbody.add_camera(name=name, pos=position, quat=_look_at(position, target), fovy=58)

        for motor in range(1, 7):
            actuator = spec.add_actuator(name=f"M{motor}", target=f"M{motor}",
                                         trntype=mujoco.mjtTrn.mjTRN_JOINT)
            actuator.set_to_position(kp=8.0 if motor == 6 else 40.0)
            actuator.inheritrange = 1.0
            limit = 1.0 if motor == 6 else 2.9  # STS3215 ~2,9 N·m a 12 V.
            actuator.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
            actuator.forcerange = [-limit, limit]
        return spec.compile()

    # ----- estado -----
    def to_qpos(self, motor_id, degrees):
        return joint_angle(self.calibration[motor_id], motor_id, degrees)

    def to_logical(self, motor_id, radians):
        return joint_to_logical(self.calibration[motor_id], motor_id, radians)

    def logical_pose(self):
        return {i: self.to_logical(i, float(self.data.qpos[self.qadr[i]])) for i in range(1, 7)}

    def m1_toward(self, xy):
        """M1 lógico que orienta el plano del brazo hacia `xy` (semilla de la IK).

        El eje de M1 es +z del mundo y con M1 = 180° el brazo apunta a −y: el
        ángulo articular es el azimut desde −y hacia +x. El sentido de M1 sale de
        calibracion.json, no se supone.
        """
        dx, dy = float(xy[0]) - self.m1_axis_xy[0], float(xy[1]) - self.m1_axis_xy[1]
        return self.to_logical(1, self.to_qpos(1, 180.0) + math.atan2(dx, -dy))

    def cube_position(self):
        return self.data.xpos[self.cube_body].copy()

    def reset(self, placement: Placement, pose=None):
        mujoco.mj_resetData(self.model, self.data)
        pose = dict(pose or {**INITIAL_POSE, 6: GRIPPER_CLOSED})
        for motor, value in pose.items():
            self.data.qpos[self.qadr[motor]] = self.to_qpos(motor, value)
        x, y = placement.cube_xy
        self.data.qpos[self.cube_qadr:self.cube_qadr + 3] = [x, y, self.floor_z + self.dimensions.cube_size / 2 + 1e-4]
        self.data.qpos[self.cube_qadr + 3:self.cube_qadr + 7] = _yaw_quat(placement.cube_yaw)
        self.data.mocap_pos[self.container_mocap] = [*placement.container_xy, self.floor_z]
        self.data.mocap_quat[self.container_mocap] = _yaw_quat(placement.container_yaw)
        self.commanded = dict(pose)
        self._apply_commanded()
        mujoco.mj_forward(self.model, self.data)
        self.contacts = Counter()
        self.step(0.3)

    def _apply_commanded(self):
        for motor, value in self.commanded.items():
            self.data.ctrl[self.actuator_ids[motor]] = self.to_qpos(motor, value)

    def step(self, seconds):
        steps = max(1, round(seconds / self.model.opt.timestep))
        observe_every = max(1, round(0.02 / self.model.opt.timestep))
        for index in range(steps):
            mujoco.mj_step(self.model, self.data)
            if index % observe_every == 0:
                self.contacts.update(self._robot_contacts(self.data))
                if self.observer is not None:
                    self.observer(self)

    # ----- colisiones -----
    def _part(self, geom):
        return PART_NAMES.get(self.model.body(int(self.model.geom_bodyid[geom])).name, "?")

    def _robot_contacts(self, data, clearance=0.0, container_clearance=0.0):
        """Piezas del brazo a menos de `clearance` metros entre sí o de la mesa, y a menos
        de `container_clearance` del recipiente (None: no se revisa el recipiente)."""
        found = set()
        for contact in data.contact[:data.ncon]:
            first, second = int(contact.geom1), int(contact.geom2)
            robot_first = bool(self.model.geom_contype[first] & ROBOT)
            robot_second = bool(self.model.geom_contype[second] & ROBOT)
            if robot_first and robot_second:
                if contact.dist < clearance:
                    found.add("autocolisión " + " ↔ ".join(sorted((self._part(first), self._part(second)))))
            elif robot_first or robot_second:
                robot, other = (first, second) if robot_first else (second, first)
                if other == self.floor_geom and contact.dist < clearance:
                    found.add(f"{self._part(robot)} ↔ mesa")
                elif (other in self.container_geoms and container_clearance is not None
                      and contact.dist < container_clearance):
                    found.add(f"{self._part(robot)} ↔ recipiente")
        return found

    def set_obstacles(self, placement: Placement):
        """Ubica el recipiente en el modelo de revisión: hasta que `obstacles` vuelva a
        False, la IK y la revisión de tramos lo evitan con CONTAINER_CLEARANCE de margen.
        PickPlaceExpert.plan lo activa solo mientras planifica."""
        for data in (self._ik_data, self._check_data):
            data.mocap_pos[self.container_mocap] = [*placement.container_xy, self.floor_z]
            data.mocap_quat[self.container_mocap] = _yaw_quat(placement.container_yaw)
        self.obstacles = True

    def pose_problems(self, pose):
        """Revisa una pose estática completa (M1–M6) sin tocar la simulación en curso."""
        data = self._check_data
        for motor, value in pose.items():
            data.qpos[self.qadr[motor]] = self.to_qpos(motor, value)
        mujoco.mj_fwdPosition(self.check_model, data)
        problems = sorted(self._robot_contacts(data, PART_CLEARANCE,
                                               CONTAINER_CLEARANCE if self.obstacles else None))
        clearance = self.geometry.clearance(pose)["height_mm"]
        if clearance <= self.floor_margin_mm:
            problems.append(f"garra a {clearance:.1f} mm de la mesa")
        return problems

    def segment_problems(self, start, end, samples=12):
        """Revisa la interpolación articular entre dos poses completas."""
        for index in range(1, samples + 1):
            amount = index / samples
            problems = self.pose_problems({i: start[i] + (end[i] - start[i]) * amount for i in start})
            if problems:
                return problems
        return []

    # ----- movimiento -----
    def move_arm(self, target, speed_deg_s=45.0, settle_seconds=2.0):
        """Interpolación articular sincronizada de M1–M5 y espera de llegada."""
        start = {i: self.commanded[i] for i in range(1, 6)}
        goal = {i: float(target[i]) for i in range(1, 6)}
        duration = max(abs(goal[i] - start[i]) for i in goal) / speed_deg_s
        ticks = max(1, math.ceil(duration / 0.02))
        for tick in range(1, ticks + 1):
            amount = tick / ticks
            self.commanded.update({i: start[i] + (goal[i] - start[i]) * amount for i in goal})
            self._apply_commanded()
            self.step(0.02)
        waited = 0.0
        while waited < settle_seconds:
            pose = self.logical_pose()
            if max(abs(pose[i] - goal[i]) for i in goal) <= 1.2:
                break
            self.step(0.02)
            waited += 0.02

    def open_gripper(self, speed_deg_s=150.0):
        start = self.commanded[6]
        ticks = max(1, math.ceil(abs(GRIPPER_OPEN - start) / speed_deg_s / 0.02))
        for tick in range(1, ticks + 1):
            self.commanded[6] = start + (GRIPPER_OPEN - start) * tick / ticks
            self._apply_commanded()
            self.step(0.02)
        self.step(0.3)
        return "open"

    def close_gripper(self):
        """Cierre por presión como el firmware: pasos de 1,5° cada 40 ms."""
        self.commanded[6] = min(self.commanded[6], self.logical_pose()[6] + 2.0)
        elapsed, samples = 0.0, 0
        while elapsed < GRIPPER_TIMEOUT_SECONDS:
            self.commanded[6] = max(GRIPPER_CLOSED, self.commanded[6] - GRIPPER_STEP)
            self._apply_commanded()
            self.step(GRIPPER_STEP_SECONDS)
            elapsed += GRIPPER_STEP_SECONDS
            torque = abs(float(self.data.actuator_force[self.actuator_ids[6]]))
            if elapsed > GRIPPER_INRUSH_SECONDS and torque > GRIPPER_CONTACT_TORQUE:
                samples += 1
                if samples >= GRIPPER_CONTACT_SAMPLES:
                    self.step(0.3)  # Mantiene la consigna adelantada: sostiene con presión.
                    return "holding"
            else:
                samples = 0
            if self.commanded[6] <= GRIPPER_CLOSED and self.logical_pose()[6] <= GRIPPER_CLOSED + 2.0:
                self.step(0.2)
                return "closed_limit"
        return "timeout"

    # ----- cinemática inversa -----
    def target_rotation(self, position, yaw, tilt):
        """Dedos hacia abajo, inclinados `tilt` rad hacia afuera dentro del plano del brazo.

        El plano vertical del brazo pasa por el eje de M1 y por el eje de los dedos;
        el punto pinza está corrido `finger_inner_x` sobre el eje x de la garra.
        Solo una inclinación dentro de ese plano es alcanzable con cinco motores.
        """
        finger_axis = np.asarray(position[:2], float)
        for _ in range(4):
            radial = finger_axis - self.m1_axis_xy
            radial = radial / np.linalg.norm(radial) if np.linalg.norm(radial) > 1e-6 else np.array([0.0, -1.0])
            y_axis = math.cos(tilt) * np.array([0.0, 0.0, 1.0]) - math.sin(tilt) * np.array([*radial, 0.0])
            x_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            x_axis -= x_axis.dot(y_axis) * y_axis
            x_axis /= np.linalg.norm(x_axis)
            finger_axis = np.asarray(position[:2], float) - self.finger_inner_x * x_axis[:2]
        return np.column_stack([x_axis, y_axis, np.cross(x_axis, y_axis)])

    def solve_ik(self, position, yaw, seeds, *, grippers=(GRIPPER_CLOSED,), tilts=IK_TILTS_DEGREES, iterations=150):
        """Punto `pinza` en `position`, apertura según `yaw` y la menor inclinación posible.

        Solo acepta soluciones sin piezas que se atraviesen y con la garra sobre
        el margen de la mesa, con cada valor de M6 de `grippers` (la mandíbula
        también barre al abrirse o cerrarse en esa pose).
        """
        target = np.asarray(position, float)
        dofs = [self.dofadr[i] for i in range(1, 6)]
        qadr = [self.qadr[i] for i in range(1, 6)]
        low = np.array([self.ik_ranges[i][0] for i in range(1, 6)])
        high = np.array([self.ik_ranges[i][1] for i in range(1, 6)])
        model = self.check_model
        jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        data, weight, best = self._ik_data, 0.1, None
        quat, error_r = np.zeros(4), np.zeros(3)
        for tilt in tilts:
            rotation = self.target_rotation(target, yaw, math.radians(tilt))
            for seed in seeds:
                q = np.clip([self.to_qpos(i, seed[i]) for i in range(1, 6)], low, high)
                # M5 gira sobre el eje de los dedos: se lo orienta antes de iterar.
                data.qpos[qadr] = q
                mujoco.mj_kinematics(model, data)
                current = data.site_xmat[self.site_id].reshape(3, 3)
                turn = math.atan2(np.cross(current[:, 0], rotation[:, 0]).dot(current[:, 1]),
                                  current[:, 0].dot(rotation[:, 0]))
                wrist = min((q[4] + turn + k * 2 * math.pi for k in (-1, 0, 1)),
                            key=lambda value: max(low[4] - value, value - high[4], 0.0))
                q[4] = min(high[4], max(low[4], wrist))
                for iteration in range(iterations):
                    data.qpos[qadr] = q
                    mujoco.mj_kinematics(model, data)
                    mujoco.mj_comPos(model, data)
                    current = data.site_xmat[self.site_id].reshape(3, 3)
                    error_p = target - data.site_xpos[self.site_id]
                    # Logaritmo de la rotación residual: válido también cerca de 180°.
                    mujoco.mju_mat2Quat(quat, (rotation @ current.T).flatten())
                    mujoco.mju_quat2Vel(error_r, quat, 1.0)
                    if np.linalg.norm(error_p) < 5e-5 and np.linalg.norm(error_r) < 5e-4:
                        break
                    if iteration == 40 and np.linalg.norm(error_p) > 0.005:
                        break  # No converge desde esta semilla; probar otra.
                    mujoco.mj_jacSite(model, data, jacp, jacr, self.site_id)
                    jacobian = np.vstack([jacp[:, dofs], weight * jacr[:, dofs]])
                    error = np.concatenate([error_p, weight * error_r])
                    delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-4 * np.eye(6), error)
                    q = np.clip(q + np.clip(delta, -0.2, 0.2), low, high)
                data.qpos[qadr] = q
                mujoco.mj_kinematics(model, data)
                current = data.site_xmat[self.site_id].reshape(3, 3)
                position_error = float(np.linalg.norm(target - data.site_xpos[self.site_id])) * 1000
                cosine = np.clip((np.trace(current.T @ rotation) - 1) / 2, -1.0, 1.0)
                pose = {i: self.to_logical(i, float(q[k])) for k, i in enumerate(range(1, 6))}
                solution = IkSolution(pose, position_error, math.degrees(math.acos(cosine)), float(tilt))
                if solution.converged:
                    problems = tuple(dict.fromkeys(problem for gripper in grippers
                                                   for problem in self.pose_problems({**pose, 6: gripper})))
                    solution = IkSolution(pose, position_error, solution.orientation_error_deg, float(tilt), problems)
                    if solution.ok:
                        return solution
                if best is None or (solution.position_error_mm + solution.orientation_error_deg
                                    < best.position_error_mm + best.orientation_error_deg):
                    best = solution
        return best


class PickPlaceExpert:
    """Genera las doce poses clave con IK. Alturas del punto pinza sobre la mesa, en metros."""
    # Semillas tomadas de poses de las demostraciones reales (cerca de la mesa y en tránsito).
    SEEDS = ({1: 180.0, 2: 205.0, 3: 160.0, 4: 255.0, 5: 263.0},
             {1: 180.0, 2: 175.0, 3: 195.0, 4: 245.0, 5: 263.0},
             {1: 180.0, 2: 150.0, 3: 215.0, 4: 245.0, 5: 263.0},
             {1: 180.0, 2: 225.0, 3: 130.0, 4: 270.0, 5: 263.0})
    TRANSIT_SEEDS = SEEDS + ({1: 180.0, 2: 134.0, 3: 200.0, 4: 255.0, 5: 263.0},
                             {1: 180.0, 2: 154.0, 3: 174.0, 4: 250.0, 5: 263.0},
                             {1: 180.0, 2: 179.0, 3: 169.0, 4: 242.0, 5: 263.0},
                             {1: 180.0, 2: 190.0, 3: 122.0, 4: 242.0, 5: 263.0},
                             {1: 180.0, 2: 212.0, 3: 90.0, 4: 224.0, 5: 263.0})

    def __init__(self, arm: VirtualArm, *, grasp_height=0.028, side_gap=0.003, release_margin=0.03,
                 push_clearance=0.018, push_extra=0.015, push_step=0.004):
        self.arm = arm
        # El dedo móvil cierra en arco y toca primero cerca de la arista superior
        # del cubo. Con la punta ~22 mm sobre la mesa y 3 mm de holgura contra el
        # dedo fijo, toca el costado ~2 mm por debajo de la arista; con 6 mm de
        # holgura la tocaba justo en la arista y a veces el cierre quedaba apoyado.
        self.grasp_height = grasp_height
        self.side_gap = side_gap
        self.release_margin = release_margin    # Base del cubo sobre el borde del recipiente.
        # Agarre empujando: el dedo fijo baja a `push_clearance` del costado del cubo
        # (si el cubo está más cerca, no cae encima) y avanza a la misma altura hasta
        # `push_extra` más allá de donde estaría ese costado (si está más lejos, lo
        # arrastra). Así el cubo queda apoyado contra el dedo fijo al cerrar, aunque
        # la ubicación erre unos `push_clearance` hacia un lado o `push_extra` hacia
        # el otro. Con push_clearance=None baja directo a `side_gap`, sin empujar.
        # 18/15 mm: en la simulación aguanta 20 mm de error en cualquier dirección.
        self.push_clearance = push_clearance
        self.push_extra = push_extra
        self.push_step = push_step

    def _solve(self, label, target_xy, heights, yaws, previous, *, tilts=IK_TILTS_DEGREES, grippers=(GRIPPER_CLOSED,)):
        """Primera altura alcanzable; entre `yaws`, menos inclinación y luego menos giro de M5."""
        reference = (previous or INITIAL_POSE)[5]
        pool = self.TRANSIT_SEEDS if tilts is TRANSIT_TILTS_DEGREES else self.SEEDS
        for height in heights:
            best = None
            for yaw in yaws:
                xy = target_xy(yaw)
                m1 = self.arm.m1_toward(xy)
                seeds = ([previous] if previous else []) + [{**seed, 1: m1} for seed in pool]
                solution = self.arm.solve_ik([xy[0], xy[1], self.arm.floor_z + height], yaw, seeds,
                                             grippers=grippers, tilts=tilts)
                if not solution.ok:
                    continue
                cost = (abs(solution.tilt_deg), abs(solution.pose[5] - reference))
                if best is None or cost < best[0]:
                    best = (cost, yaw, solution.pose)
            if best is not None:
                return best[1], best[2], height
        raise PlanningError(f"{label}: sin pose alcanzable y libre de choques entre "
                            f"{min(heights) * 1000:.0f} y {max(heights) * 1000:.0f} mm sobre la mesa")

    def _vertical(self, target_xy, yaw, start, end, previous, grippers, step=0.015):
        """Puntos intermedios para bajar o subir en línea casi recta."""
        count = max(0, math.ceil(abs(end - start) / step) - 1)
        heights = [start + (end - start) * (index + 1) / (count + 1) for index in range(count)]
        via = []
        for height in heights:
            _, previous, _ = self._solve("tramo vertical", target_xy, [height], [yaw], previous, grippers=grippers)
            via.append(previous)
        return tuple(via)

    def _horizontal(self, start_xy, end_xy, yaw, height, previous, grippers):
        """Puntos para correr la garra en línea recta a una altura: el inicio y los intermedios."""
        start, end = np.asarray(start_xy(yaw), float), np.asarray(end_xy(yaw), float)
        count = max(1, math.ceil(float(np.linalg.norm(end - start)) / self.push_step))
        via = []
        for index in range(count):
            point = tuple(start + (end - start) * index / count)
            _, previous, _ = self._solve("empuje", lambda _: point, [height], [yaw], previous, grippers=grippers)
            via.append(previous)
        return tuple(via)

    def _pushes_into_container(self, placement: Placement, yaw: float) -> bool:
        """Si con este rumbo el empuje acerca el cubo al recipiente hasta menos de
        CONTAINER_CLEARANCE (con el cubo girado como peor caso)."""
        dims = self.arm.dimensions
        c, s = math.cos(-placement.container_yaw), math.sin(-placement.container_yaw)
        half_x, half_y = (size / 2 for size in dims.container_outer)

        def gap(point):
            dx, dy = point[0] - placement.container_xy[0], point[1] - placement.container_xy[1]
            local = (c * dx - s * dy, s * dx + c * dy)
            return math.hypot(max(abs(local[0]) - half_x, 0.0), max(abs(local[1]) - half_y, 0.0))

        start = np.asarray(placement.cube_xy, float)
        end = start - self.push_extra * np.array([math.cos(yaw), math.sin(yaw)])
        reach = dims.cube_size / 2 * math.sqrt(2) + CONTAINER_CLEARANCE
        return gap(end) < gap(start) and gap(end) < reach

    def plan(self, placement: Placement) -> Plan:
        # Mientras se planifica, el recipiente es un obstáculo: ningún tramo debe tocarlo.
        self.arm.set_obstacles(placement)
        try:
            return self._plan(placement)
        finally:
            self.arm.obstacles = False

    def _plan(self, placement: Placement) -> Plan:
        dims = self.arm.dimensions

        def beside(center, distance):
            # El punto pinza está en la cara interna del dedo fijo (eje x de la garra);
            # el cubo queda del lado del dedo móvil.
            return lambda yaw: (center[0] + distance * math.cos(yaw), center[1] + distance * math.sin(yaw))

        half = dims.cube_size / 2
        pushing = self.push_clearance is not None
        # Baja con el dedo fijo en `at_start` y cierra en `at_cube`; sin empuje, son el mismo punto.
        at_start = beside(placement.cube_xy, half + (self.push_clearance if pushing else self.side_gap))
        at_cube = beside(placement.cube_xy, half - self.push_extra) if pushing else at_start
        cube_yaws = [placement.cube_yaw + k * math.pi / 2 for k in range(4)]
        if pushing:
            cube_yaws = [yaw for yaw in cube_yaws if not self._pushes_into_container(placement, yaw)]
            if not cube_yaws:
                raise PlanningError("agarre: con cualquier rumbo, el empuje llevaría el cubo contra el "
                                    "recipiente; separalos")
        # M6 con que se revisa cada pose: incluye el barrido de abrir o cerrar ahí. El dedo
        # móvil cierra en arco; con solo tres valores se escapaba un choque con la pared del
        # recipiente que pasaba entre 57° y 73°, así que se revisa cada GRIPPER_SWEEP_STEP.
        opening = _sweep(GRIPPER_CLOSED, GRIPPER_OPEN)
        gripping = _sweep(GRIPPER_OPEN, GRIPPER_HOLD_ESTIMATE)
        releasing = tuple(reversed(gripping))
        # Al subir, la base del cubo queda ~grasp_height por debajo del punto pinza.
        holding = (GRIPPER_HOLD_ESTIMATE,)
        clear = dims.container_wall_height + self.grasp_height + self.release_margin
        # El rumbo se elige por la pose de agarre; si con él el dedo fijo no puede bajar o
        # subir (por ejemplo, porque al costado del cubo está el recipiente), prueba otro.
        remaining = list(cube_yaws)
        while True:
            yaw, grasp, _ = self._solve("agarre", at_cube, [self.grasp_height], remaining, None, grippers=gripping)
            try:
                _, hover, hover_height = self._solve("aproximacion_3", at_start,
                                                     [dims.cube_size + h for h in (0.035, 0.025, 0.018)], [yaw],
                                                     grasp, grippers=opening)
                _, approach_2, _ = self._solve("aproximacion_2", at_start, [0.14, 0.12, 0.10], [yaw], hover,
                                               tilts=TRANSIT_TILTS_DEGREES, grippers=opening)
                _, approach_1, _ = self._solve("aproximacion_1", at_start, [0.20, 0.17, 0.14], [yaw], approach_2,
                                               tilts=TRANSIT_TILTS_DEGREES)
                descent = self._vertical(at_start, yaw, hover_height, self.grasp_height, hover, (GRIPPER_OPEN,))
                if pushing:
                    # Abajo, al costado del cubo; después, a la misma altura, hacia el dedo móvil.
                    descent += self._horizontal(at_start, at_cube, yaw, self.grasp_height,
                                                descent[-1] if descent else hover, (GRIPPER_OPEN,))
                _, lift, lift_height = self._solve("traslado_1", at_cube, [0.15, 0.13, 0.11, clear], [yaw], grasp,
                                                   tilts=TRANSIT_TILTS_DEGREES, grippers=holding)
                lift_via = self._vertical(at_cube, yaw, self.grasp_height, min(lift_height, hover_height), grasp,
                                          holding)
                break
            except PlanningError:
                remaining.remove(yaw)
                if not remaining:
                    raise
        # Sostenido, el cubo queda apoyado contra el dedo fijo.
        at_container = beside(placement.container_xy, dims.cube_size / 2)
        container_yaws = [placement.container_yaw + k * math.pi / 2 for k in range(4)]
        release_yaw, release, release_height = self._solve(
            "traslado_3", at_container, [clear, clear + 0.015, clear + 0.03], container_yaws, lift,
            grippers=releasing)
        lift_xy, release_xy = at_cube(yaw), at_container(release_yaw)
        middle = ((lift_xy[0] + release_xy[0]) / 2, (lift_xy[1] + release_xy[1]) / 2)
        _, carry, _ = self._solve("traslado_2", lambda _: middle, [0.17, 0.15, 0.13, clear + 0.01],
                                  [yaw, release_yaw], lift, tilts=TRANSIT_TILTS_DEGREES, grippers=holding)
        warnings = []
        try:
            _, retreat, _ = self._solve("retiro", at_container,
                                        [release_height + h for h in (0.08, 0.05, 0.03, 0.015)],
                                        [release_yaw], release, tilts=TRANSIT_TILTS_DEGREES,
                                        grippers=(GRIPPER_OPEN,))
            guard_via = (retreat,)
        except PlanningError:
            guard_via = ()  # Sin retiro vertical: el tramo directo se revisa abajo.
            warnings.append("Sin retiro vertical sobre el recipiente; vuelve directo al inicio.")

        start = dict(INITIAL_POSE)
        stages = {
            "inicio": (start, ()), "aproximacion_1": (approach_1, ()), "aproximacion_2": (approach_2, ()),
            "aproximacion_3": (hover, ()), "apertura_sobre_cubo": (hover, ()),
            "descenso_abierto": (grasp, descent), "agarre": (grasp, ()), "traslado_1": (lift, lift_via),
            "traslado_2": (carry, ()), "traslado_3": (release, ()), "soltado": (release, ()),
            "guardado": (start, guard_via),
        }
        keyframes = [Keyframe(stage, stages[stage][0], STAGE_GRIPPER_ACTION.get(stage), stages[stage][1])
                     for stage in KEYFRAME_STAGES]
        return Plan(keyframes, warnings + self._check_path(keyframes))

    def _check_path(self, keyframes):
        """Revisa cada tramo con el M6 que tendrá la garra durante el movimiento."""
        warnings, current, gripper = [], {**INITIAL_POSE, 6: GRIPPER_CLOSED}, GRIPPER_CLOSED
        for keyframe in keyframes:
            for target in (*keyframe.via, keyframe.pose):
                pose = {**target, 6: gripper}
                problems = self.arm.segment_problems(current, pose)
                if problems:
                    warnings.append(f"Tramo hacia {keyframe.stage}: " + ", ".join(problems))
                current = pose
            gripper = {"open": GRIPPER_OPEN, "close": GRIPPER_HOLD_ESTIMATE if keyframe.stage == "agarre"
                       else GRIPPER_CLOSED}.get(keyframe.gripper_action, gripper)
            if gripper != current[6]:
                problems = self.arm.segment_problems(current, {**current, 6: gripper})
                if problems:
                    warnings.append(f"Garra en {keyframe.stage}: " + ", ".join(problems))
            current = {**current, 6: gripper}
        return warnings


def sample_placement(rng: np.random.Generator, dims: SceneDimensions | None = None):
    """Zona de trabajo tomada de las 15 demostraciones reales (marco base, metros)."""
    dims = dims or SceneDimensions()
    side = rng.choice([-1.0, 1.0])
    for _ in range(200):
        cube = (side * rng.uniform(0.03, 0.15), rng.uniform(-0.31, -0.15))
        container = (-side * rng.uniform(0.10, 0.15), rng.uniform(-0.27, -0.19))
        half = max(dims.container_outer) / 2 + dims.cube_size * 0.75
        if abs(cube[0] - container[0]) > half and math.hypot(*cube) > 0.17:
            return Placement(cube, float(rng.uniform(-math.pi / 4, math.pi / 4)),
                             container, float(rng.uniform(-0.35, 0.35)))
    raise RuntimeError("No se pudo ubicar cubo y recipiente sin superposición")


def cube_in_container(arm: VirtualArm, placement: Placement):
    dims = arm.dimensions
    cube = arm.cube_position()
    dx, dy = cube[0] - placement.container_xy[0], cube[1] - placement.container_xy[1]
    c, s = math.cos(-placement.container_yaw), math.sin(-placement.container_yaw)
    local = (c * dx - s * dy, s * dx + c * dy)
    inside = all(abs(value) <= size / 2 for value, size in zip(local, dims.container_inner))
    return inside and cube[2] - arm.floor_z < dims.container_wall_height + dims.cube_size / 2


def run_episode(arm: VirtualArm, expert: PickPlaceExpert, placement: Placement, *,
                speed_deg_s=45.0, plan: Plan | None = None, on_stage=None) -> EpisodeResult:
    """Ejecuta las doce etapas en la física. `on_stage(record)` permite registrar fotos."""
    plan = plan or expert.plan(placement)
    arm.reset(placement)
    result = EpisodeResult(placement, success=False, plan_warnings=list(plan.warnings))
    start_time = arm.data.time
    start_cube = arm.cube_position()
    gripper_state = "closed_limit"
    for keyframe in plan.keyframes:
        for waypoint in keyframe.via:
            arm.move_arm(waypoint, speed_deg_s)
        arm.move_arm(keyframe.pose, speed_deg_s)
        if keyframe.gripper_action == "open":
            gripper_state = arm.open_gripper()
        elif keyframe.gripper_action == "close":
            gripper_state = arm.close_gripper()
        if keyframe.stage == "agarre" and gripper_state == "holding":
            result.holding_m6 = arm.logical_pose()[6]
        pose = {**keyframe.pose, 6: arm.commanded[6]}
        clearance = arm.geometry.clearance(pose)["height_mm"]
        result.minimum_keyframe_clearance_mm = min(result.minimum_keyframe_clearance_mm, clearance)
        record = StageRecord(keyframe.stage, pose, arm.logical_pose(), gripper_state,
                             arm.cube_position().round(4).tolist(), current_rule_warnings(pose, arm.limits))
        result.records.append(record)
        if on_stage is not None:
            on_stage(record)
        if keyframe.stage == "agarre" and gripper_state != "holding":
            result.reasons.append(f"La garra no detectó el cubo ({gripper_state}).")
            break
        if keyframe.stage == "traslado_1" and arm.cube_position()[2] < start_cube[2] + 0.03:
            result.reasons.append("El cubo no se levantó con la garra.")
            break
        if keyframe.stage == "soltado":
            arm.step(0.4)
    arm.step(0.5)
    result.contacts = dict(arm.contacts)
    result.simulated_seconds = arm.data.time - start_time
    if not result.reasons and not cube_in_container(arm, placement):
        result.reasons.append("El cubo no quedó dentro del recipiente.")
    for name, samples in sorted(result.contacts.items()):
        result.reasons.append(f"{name} ({samples} muestras de 20 ms)")
    result.success = not result.reasons
    return result


def episode_metadata(result: EpisodeResult, dims: SceneDimensions):
    return {
        "schema": 1,
        "origen": "simulacion_mujoco",
        "aviso": "Datos sintéticos; no son demostraciones reales ni entran al entrenamiento.",
        "result": "success" if result.success else "failure",
        "reasons": result.reasons,
        "plan_warnings": result.plan_warnings,
        "dimensions": {"cube_size": dims.cube_size, "container_inner": list(dims.container_inner),
                       "container_wall_height": dims.container_wall_height},
        "placement": {"cube_xy": list(result.placement.cube_xy), "cube_yaw": result.placement.cube_yaw,
                      "container_xy": list(result.placement.container_xy),
                      "container_yaw": result.placement.container_yaw},
        "holding_m6": result.holding_m6,
        "minimum_keyframe_clearance_mm": result.minimum_keyframe_clearance_mm,
        "stages": [{"stage": r.stage, "joint_degrees_commanded": {str(k): v for k, v in r.commanded.items()},
                    "joint_degrees_measured": {str(k): v for k, v in r.measured.items()},
                    "gripper_state": r.gripper_state, "cube_position_m": r.cube_position,
                    "rule_warnings": r.rule_warnings}
                   for r in result.records],
    }


def save_metadata(path: Path, payload: dict):
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
