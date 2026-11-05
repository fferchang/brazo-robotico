"""Gemelo digital: el brazo de MuJoCo detrás del mismo puerto serie y las mismas cámaras.

`DigitalTwin` corre la física en tiempo real e imita `firmware/firmware_secuencias.ino`:
mismos comandos (`<id> <grados>`, `pair`, `arm torque on`, `gripper close`…), mismas
líneas de respuesta (`POS ID`, `PAIR_MOVE_OK`, `GRIPPER_HOLD`…) y mismas reglas de
geometría, límites y cierre por presión. `open_port` devuelve un objeto con la interfaz
de `serial.Serial` que `RobotSerial` usa sin saber que no hay un ESP32.

`VirtualCameraStream` tiene la interfaz de `CameraStream` y entrega las vistas de las
cámaras virtuales `frontal` y `lateral`. Con las dos, la aplicación, el ejecutor y la
política geométrica corren el mismo código que con el brazo real.

Diferencias con el brazo real (además de las de `docs/SIMULACION_MUJOCO.md`):
- Los servos siguen un perfil trapezoidal con la velocidad y aceleración de
  `WritePosEx`; no hay juego mecánico ni error de encoder.
- Al congelar la garra, el firmware ordena la posición leída y la elasticidad del
  cubo y de los dedos sigue apretando. MuJoCo no tiene esa elasticidad: el gemelo
  conserva la consigna del momento, igual que `VirtualArm.close_gripper`.
- La corriente de M6 sale del torque del actuador: el umbral de contacto de la
  simulación (0,30 N·m) equivale al de corriente del firmware (92).

Nada de este módulo abre puertos ni cámaras reales.
"""
from __future__ import annotations

import collections
import dataclasses
import math
import threading
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np

from .calibracion_camaras import CAMERAS, CameraCalibration, Sample, fit_calibration, object_pixels
from .capture import CameraStream
from .config import INITIAL_POSE, AppConfig, CameraConfig, load_config
from .gripper_geometry import model_fingerprint
from .mujoco_sim import (
    GRIPPER_CONTACT_TORQUE,
    Placement,
    VirtualArm,
    cube_in_container,
    sample_placement,
)
from .planner import (
    ESCAPE_TOLERANCE,
    FLOOR_TOUCH,
    _collides_base,
    _collides_link,
    compute_geometry,
    dynamic_m4_max,
    floor_clearance,
    floor_escape,
)
from .vision import detect_scene, observation_features


VIRTUAL_PORT = "SIMULADO"
TWIN_DATA_DIR = Path("datos") / "gemelo"
TWIN_CALIBRATION_FILE = TWIN_DATA_DIR / "camaras_virtuales.json"

# Las cámaras virtuales están ubicadas a ojo y ven el recipiente más chico que la
# frontal real; los umbrales estrictos de esa cámara no aplican aquí.
SIM_DETECTION = dict(minimum_red_area_ratio=0.001, minimum_lime_area_ratio=0.004,
                     lime_hsv_lower=(30, 50, 50), lime_hsv_upper=(90, 255, 255))

# ----- constantes del firmware (firmware_secuencias.ino) -----
STEPS_PER_TURN = 4095.0
MOVE_SPEED = 450            # pasos/s del servo
MOVE_ACC = 45               # unidades de 100 pasos/s²
MOVE_COOLDOWN_S = 0.050
AUTO_POS_INTERVAL_S = 0.050
POST_MOVE_REPORT_S = 0.200
GRIPPER_OPEN_DEGREES = 140.0
GRIPPER_CLOSED_DEGREES = 0.0
GRIPPER_STEP_DEGREES = 1.5
GRIPPER_STEP_INTERVAL_S = 0.040
GRIPPER_INRUSH_IGNORE_S = 0.250
GRIPPER_CLOSE_TIMEOUT_S = 15.0
GRIPPER_CLOSED_TOLERANCE_DEGREES = 2.0
GRIPPER_CONTACT_CURRENT_RAW = 92
GRIPPER_HARD_CURRENT_RAW = 200
GRIPPER_CONTACT_SAMPLES = 3
USER_LIMITS = {1: (60.0, 300.0), 2: (90.0, 270.0), 3: (90.0, 276.0),
               4: (75.0, 278.0), 5: (0.0, 340.0), 6: (0.0, 140.0)}
TICK_S = 0.010              # paso del bucle: firmware + física


def firmware_pose_reason(pose, m6, escape_from) -> str:
    """Réplica de `classifyPoseInvalid` del firmware: piso y autocolisión, nada más.

    `planner.classify_pose` agrega límites por motor y la regla de M4, que el
    firmware revisa aparte (o no revisa) según el comando."""
    points, corners = compute_geometry(pose)
    clearance = floor_clearance(pose, m6)
    if clearance <= FLOOR_TOUCH and (escape_from is None or clearance < escape_from - ESCAPE_TOLERANCE):
        return "PISO"
    if _collides_base(corners):
        return "COLISION_BASE"
    if _collides_link(corners, points[0], points[1]):
        return "COLISION_L1"
    if _collides_link(corners, points[2], points[3]):
        return "COLISION_L2"
    return "OK"


def deg_to_pos(degrees: float) -> int:
    return int(round(degrees / 360.0 * STEPS_PER_TURN))


def pos_to_deg(position: int) -> float:
    return position * 360.0 / STEPS_PER_TURN


def speed_to_deg_s(speed: float) -> float:
    return speed * 360.0 / STEPS_PER_TURN


def acc_to_deg_s2(acc: float) -> float:
    return acc * 100.0 * 360.0 / STEPS_PER_TURN


def to_real(motor_id: int, logical: float) -> float:
    return logical + (10.0 if motor_id == 6 else 0.0)


def current_raw_from_torque(torque_nm: float) -> int:
    """Torque del actuador de M6 en unidades de corriente del firmware."""
    return int(round(abs(torque_nm) / GRIPPER_CONTACT_TORQUE * GRIPPER_CONTACT_CURRENT_RAW))


@dataclasses.dataclass
class ServoProfile:
    """Consigna de un STS3215 tras `WritePosEx`: trapezoide de velocidad y aceleración."""
    setpoint: float
    target: float
    velocity: float = 0.0
    speed: float = speed_to_deg_s(MOVE_SPEED)
    acceleration: float = acc_to_deg_s2(MOVE_ACC)

    def command(self, target: float, speed: float, acc: float) -> None:
        self.target = float(target)
        self.speed = max(1e-3, speed_to_deg_s(speed))
        self.acceleration = max(1e-3, acc_to_deg_s2(acc))

    def hold(self, position: float) -> None:
        self.setpoint = self.target = float(position)
        self.velocity = 0.0

    def advance(self, dt: float) -> None:
        remaining = self.target - self.setpoint
        if abs(remaining) < 1e-6 and abs(self.velocity) < 1e-6:
            return
        direction = math.copysign(1.0, remaining) if remaining else -math.copysign(1.0, self.velocity)
        braking = self.velocity ** 2 / (2.0 * self.acceleration)
        moving_toward = self.velocity * direction > 0
        if moving_toward and braking >= abs(remaining):
            wanted = 0.0
        else:
            wanted = direction * self.speed
        change = self.acceleration * dt
        self.velocity = min(max(wanted, self.velocity - change), self.velocity + change)
        step = self.velocity * dt
        if (remaining > 0 and step >= remaining) or (remaining < 0 and step <= remaining) \
                or (abs(remaining) < 0.05 and abs(self.velocity) <= change):
            self.setpoint, self.velocity = self.target, 0.0
        else:
            self.setpoint += step


class VirtualSerialPort:
    """Lo mínimo de `serial.Serial` que usa `RobotSerial`, conectado al gemelo."""

    def __init__(self, twin: "DigitalTwin", port: str = VIRTUAL_PORT, baudrate: int = 115200,
                 timeout: float | None = 0.1, write_timeout: float | None = None):
        self.twin = twin
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.dtr = False
        self.rts = False
        self._buffer = bytearray()
        self._condition = threading.Condition()
        self._open = True
        twin._attach(self)

    @property
    def is_open(self) -> bool:
        return self._open

    @property
    def in_waiting(self) -> int:
        with self._condition:
            return len(self._buffer)

    def _feed(self, data: bytes) -> None:
        with self._condition:
            if not self._open:
                return
            self._buffer.extend(data)
            if len(self._buffer) > 1_000_000:  # nadie lee: no crecer sin fin
                del self._buffer[:-200_000]
            self._condition.notify_all()

    def read(self, size: int = 1) -> bytes:
        deadline = None if self.timeout is None else time.monotonic() + self.timeout
        with self._condition:
            while not self._buffer and self._open:
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    break
                self._condition.wait(remaining)
            data = bytes(self._buffer[:size])
            del self._buffer[:size]
            return data

    def write(self, data: bytes) -> int:
        if not self._open:
            raise OSError("Puerto virtual cerrado.")
        self.twin._receive(bytes(data))
        return len(data)

    def flush(self) -> None:
        return None

    def reset_input_buffer(self) -> None:
        with self._condition:
            self._buffer.clear()

    def close(self) -> None:
        with self._condition:
            self._open = False
            self._condition.notify_all()
        self.twin._detach(self)


class DigitalTwin:
    """Brazo virtual con el comportamiento del firmware, corriendo en tiempo real.

    `pace`: velocidad respecto del tiempo real (1 = real; 0 = lo más rápido posible).
    Todo acceso a `arm.data` desde otro hilo va bajo `lock`.
    """

    def __init__(self, config: AppConfig | None = None, *, placement: Placement | None = None,
                 seed: int | None = None, pace: float = 1.0, max_yaw_degrees: float = 5.0,
                 assets_root: Path | None = None):
        self.config = config or load_config()
        self.arm = VirtualArm(assets_root, limits=self.config.limits)
        self.lock = threading.RLock()
        self.rng = np.random.default_rng(seed)
        self.pace = float(pace)
        self.max_yaw_degrees = float(max_yaw_degrees)
        self.viewer = None
        self._ports: list[VirtualSerialPort] = []
        self._ports_lock = threading.Lock()
        self._input = bytearray()
        self._input_lock = threading.Lock()
        self._line = ""
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._kp = {motor: float(self.arm.model.actuator_gainprm[aid, 0])
                    for motor, aid in self.arm.actuator_ids.items()}
        # Lo que respondió el firmware virtual, sin la telemetría: (número, línea).
        self.log: collections.deque[tuple[int, str]] = collections.deque(maxlen=2000)
        self._log_count = 0
        self.placement = placement or self.random_placement()
        self._reset_state(self.placement)

    # ----- escena -----
    def random_placement(self) -> Placement:
        """Cubo y recipiente al azar, alineados a mano con la base (giro chico)."""
        limit = math.radians(self.max_yaw_degrees)
        base = sample_placement(self.rng)
        return dataclasses.replace(base, cube_yaw=float(self.rng.uniform(-limit, limit)),
                                   container_yaw=float(self.rng.uniform(-limit, limit)))

    def new_scene(self, placement: Placement | None = None) -> Placement:
        """Escena nueva con el brazo en la pose inicial, garra cerrada y torque ON."""
        with self.lock:
            self.placement = placement or self.random_placement()
            self._reset_state(self.placement)
            return self.placement

    def _reset_state(self, placement: Placement) -> None:
        for motor, aid in self.arm.actuator_ids.items():
            self._set_gain(motor, self._kp[motor])
        pose = {**INITIAL_POSE, 6: GRIPPER_CLOSED_DEGREES}
        self.arm.reset(placement, pose)
        self.torque = {motor: True for motor in range(1, 7)}
        self.profiles = {motor: ServoProfile(pose[motor], pose[motor]) for motor in range(1, 7)}
        self.last_move = -math.inf
        self.last_telemetry = -math.inf
        self.pending_report: tuple[float, int] | None = None
        self.gripper_closing = False
        self.gripper_command = GRIPPER_CLOSED_DEGREES
        self.gripper_started = 0.0
        self.gripper_last_step = -math.inf
        self.gripper_samples = 0
        self.contact_threshold = GRIPPER_CONTACT_CURRENT_RAW
        self.arm.contacts.clear()

    @property
    def now(self) -> float:
        return float(self.arm.data.time)

    def log_since(self, number: int) -> list[str]:
        """Líneas del firmware virtual (sin telemetría) posteriores a la número `number`."""
        return [text for count, text in list(self.log) if count > number]

    @property
    def log_count(self) -> int:
        return self._log_count

    def cube_in_container(self) -> bool:
        with self.lock:
            return bool(cube_in_container(self.arm, self.placement))

    def cube_position(self) -> np.ndarray:
        with self.lock:
            return self.arm.cube_position()

    def contacts(self) -> dict[str, int]:
        with self.lock:
            return dict(self.arm.contacts)

    def logical_pose(self) -> dict[int, float]:
        with self.lock:
            return self.arm.logical_pose()

    # ----- hilo -----
    def start(self) -> "DigitalTwin":
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="gemelo-digital", daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for port in list(self._ports):
            port.close()

    def _run(self) -> None:
        wall_start, sim_start = time.perf_counter(), None
        last_viewer = 0.0
        while not self._stop.is_set():
            with self.lock:
                if sim_start is None:
                    sim_start = self.now
                self.tick()
                simulated = self.now - sim_start
                viewer = self.viewer
                if viewer is not None and time.perf_counter() - last_viewer > 1 / 30:
                    if viewer.is_running():
                        viewer.sync()
                    last_viewer = time.perf_counter()
            if self.pace > 0:
                ahead = wall_start + simulated / self.pace - time.perf_counter()
                if ahead > 0:
                    self._stop.wait(ahead)
                elif ahead < -0.5:  # la PC no da abasto: seguir sin acumular atraso
                    wall_start, sim_start = time.perf_counter(), self.now
            elif int(simulated / TICK_S) % 20 == 0:
                time.sleep(0)  # dejar correr a los otros hilos

    def tick(self) -> None:
        """Un paso de 10 ms: comandos recibidos, firmware, servos y física. Bajo `lock`."""
        with self._input_lock:
            data, self._input = bytes(self._input), bytearray()
        for char in data.decode("ascii", errors="replace"):
            if char in "\r\n":
                if self._line:
                    line, self._line = self._line, ""
                    self.handle_line(line)
            else:
                self._line += char
        self._process_gripper_closing()
        self._auto_print_positions()
        self._process_pending_report()
        for motor, profile in self.profiles.items():
            if self.torque[motor]:
                profile.advance(TICK_S)
                self.arm.commanded[motor] = profile.setpoint
            else:
                profile.hold(self._actual(motor))
                self.arm.commanded[motor] = profile.setpoint
        self.arm._apply_commanded()
        self.arm.step(TICK_S)

    # ----- puerto -----
    def open_port(self, port: str = VIRTUAL_PORT, baudrate: int = 115200, timeout: float | None = 0.1,
                  write_timeout: float | None = None, **_ignored) -> VirtualSerialPort:
        """Misma firma que `serial.Serial`: pasala a `RobotSerial(opener=...)`."""
        self.start()
        return VirtualSerialPort(self, port, baudrate, timeout, write_timeout)

    def _attach(self, port: VirtualSerialPort) -> None:
        with self._ports_lock:
            self._ports.append(port)

    def _detach(self, port: VirtualSerialPort) -> None:
        with self._ports_lock:
            if port in self._ports:
                self._ports.remove(port)

    def _receive(self, data: bytes) -> None:
        with self._input_lock:
            self._input.extend(data)

    def println(self, text: str = "") -> None:
        if not text.startswith("POS ID"):
            self._log_count += 1
            self.log.append((self._log_count, text))
        payload = (text + "\r\n").encode("utf-8")
        with self._ports_lock:
            ports = list(self._ports)
        for port in ports:
            port._feed(payload)

    # ----- servos -----
    def _actual(self, motor: int) -> float:
        return float(self.arm.to_logical(motor, float(self.arm.data.qpos[self.arm.qadr[motor]])))

    def _read_pos(self, motor: int) -> int:
        return min(4095, max(0, deg_to_pos(to_real(motor, self._actual(motor)))))

    def _logical_from_encoder(self, motor: int) -> float:
        return pos_to_deg(self._read_pos(motor)) - (10.0 if motor == 6 else 0.0)

    def _write_pos(self, motor: int, logical: float, speed: float, acc: float) -> None:
        profile = self.profiles[motor]
        if not self.torque[motor]:
            return
        profile.command(logical, speed, acc)

    def _set_gain(self, motor: int, kp: float) -> None:
        aid = self.arm.actuator_ids[motor]
        self.arm.model.actuator_gainprm[aid, 0] = kp
        self.arm.model.actuator_biasprm[aid, 1] = -kp

    def _enable_torque(self, motor: int, enable: bool) -> int:
        if enable and not self.torque[motor]:
            self.profiles[motor].hold(self._actual(motor))
        self.torque[motor] = enable
        self._set_gain(motor, self._kp[motor] if enable else 0.0)
        return 1

    # ----- firmware: impresión -----
    def _divider(self) -> None:
        self.println("-" * 50)

    def _read_one(self, motor: int) -> None:
        if not 1 <= motor <= 6:
            self.println(f"ID {motor}: sin respuesta")
            return
        position = self._read_pos(motor)
        self.println(f"ID {motor}: pos={position} (~{pos_to_deg(position):.1f} deg) | V=120 | T=30")

    def _print_all_positions(self) -> None:
        parts = []
        for motor in range(1, 7):
            position = self._read_pos(motor)
            parts.append(f"M{motor}={position}({pos_to_deg(position):.1f}deg)")
        self.println(" | ".join(parts))
        self._divider()

    def _auto_print_positions(self) -> None:
        if self.now - self.last_telemetry < AUTO_POS_INTERVAL_S - 1e-9:
            return
        self.last_telemetry = self.now
        for motor in range(1, 7):
            position = self._read_pos(motor)
            self.println(f"POS ID {motor}: {position} (~{pos_to_deg(position):.1f} deg)")

    def _process_pending_report(self) -> None:
        if self.pending_report is None or self.now < self.pending_report[0]:
            return
        _, motor = self.pending_report
        self.pending_report = None
        self._read_one(motor)
        self._divider()

    def _help(self) -> None:
        self._divider()
        self.println("Comandos disponibles (gemelo digital del firmware):")
        for text in ("  <id> <grados>", "  pair <id1> <grados1> <id2> <grados2>", "  read <id>", "  scan",
                     "  posall", "  arm torque off  (solamente M1..M5)", "  arm torque on   (solamente M1..M5)",
                     "  torque off      (emergencia: M1..M6)", "  torque on       (todos: M1..M6)",
                     "  gripper open", "  gripper close", "  gripper stop",
                     "  gripper release  (torque OFF solamente en M6)", "  gripper current 92  (aprox. 600 mA)",
                     "  help"):
            self.println(text)
        self.println("Limites configurados:")
        for motor, (low, high) in USER_LIMITS.items():
            self.println(f"  ID {motor}:    {low:g}..{high:g}")

    # ----- firmware: reglas -----
    def _planar_now(self) -> tuple[float, float, float]:
        return tuple(self._logical_from_encoder(motor) for motor in (2, 3, 4))

    def _pose_allowed(self, motor: int, degrees: float, current, m6, escape) -> bool:
        target = list(current)
        if motor in (2, 3, 4):
            target[motor - 2] = degrees
        if motor == 4 and degrees > dynamic_m4_max(current[0], current[1]):
            return False
        if motor in (2, 3, 4):
            return firmware_pose_reason(tuple(target), m6, escape) == "OK"
        return True

    def _validate_dynamic(self, motor: int, degrees: float) -> float | None:
        current = self._planar_now()
        m6 = self._logical_from_encoder(6)
        escape = floor_escape(current, m6)
        if self._pose_allowed(motor, degrees, current, m6, escape):
            return degrees
        low, high = USER_LIMITS[motor]
        for step in range(1, math.ceil((high - low) / 0.5) + 1):
            for candidate in (degrees - step * 0.5, degrees + step * 0.5):
                if low <= candidate <= high and self._pose_allowed(motor, candidate, current, m6, escape):
                    self.println(f"AJUSTE SEGURO ID {motor}: {degrees:.2f} -> {candidate:.2f}")
                    return candidate
        self.println(f"Bloqueado: no existe correccion segura cercana para ID {motor}")
        return None

    def _validate_pair(self, targets: dict[int, float]) -> str:
        current = self._planar_now()
        m6 = self._logical_from_encoder(6)
        goal = list(current)
        goal_m6 = targets.get(6, m6)
        for motor, value in targets.items():
            if motor in (2, 3, 4):
                goal[motor - 2] = value
        escape = floor_escape(current, m6)
        for step in range(1, 21):
            amount = step / 20
            pose = tuple(a + (b - a) * amount for a, b in zip(current, goal))
            if pose[2] > dynamic_m4_max(pose[0], pose[1]):
                return "REGLA_DINAMICA_M4"
            reason = firmware_pose_reason(pose, m6 + (goal_m6 - m6) * amount, escape)
            if reason != "OK":
                return reason
        return "OK"

    # ----- firmware: movimientos -----
    def _move_one(self, motor: int, degrees: float) -> None:
        if not 1 <= motor <= 6:
            self.println("ID invalido. Usa 1..6.")
            return
        if motor == 6:
            if self.gripper_closing:
                self.println("GRIPPER_STOP reason=manual_position_command detail=se_recibio_un_angulo_para_M6")
            self.gripper_closing = False
            self.gripper_samples = 0
        if self.now - self.last_move < MOVE_COOLDOWN_S:
            wait_ms = int((MOVE_COOLDOWN_S - (self.now - self.last_move)) * 1000)
            self.println(f"Cooldown activo. Espera {wait_ms} ms antes del siguiente movimiento.")
            return
        low, high = USER_LIMITS[motor]
        if not low <= degrees <= high:
            original, degrees = degrees, min(max(degrees, low), high)
            self.println(f"AJUSTE LIMITE ID {motor}: {original:.2f} -> {degrees:.2f}")
        validated = self._validate_dynamic(motor, degrees)
        if validated is None:
            self._divider()
            return
        degrees = validated
        real = to_real(motor, degrees)
        extra = f" (real {real:.1f} deg)" if motor == 6 else ""
        self.println(f"Moviendo ID {motor} a {degrees:.1f} deg{extra} -> pos {deg_to_pos(real)}")
        self.last_move = self.now
        self._write_pos(motor, degrees, MOVE_SPEED, MOVE_ACC)
        self.pending_report = (self.now + POST_MOVE_REPORT_S, motor)

    def _move_pair(self, first: int, first_deg: float, second: int, second_deg: float) -> None:
        if not (1 <= first <= 6 and 1 <= second <= 6):
            self.println("PAIR_REJECTED reason=ID_invalido usa_1_a_6")
            return
        if first == second:
            self.println("PAIR_REJECTED reason=IDs_repetidos")
            return
        for motor, value in ((first, first_deg), (second, second_deg)):
            low, high = USER_LIMITS[motor]
            if not low <= value <= high:
                self.println(f"PAIR_REJECTED reason=fuera_de_rango ID={motor}")
                return
        if self.now - self.last_move < MOVE_COOLDOWN_S:
            self.println("PAIR_REJECTED reason=cooldown")
            return
        reason = self._validate_pair({first: first_deg, second: second_deg})
        if reason != "OK":
            self.println(f"PAIR_REJECTED reason=trayectoria_insegura detail={reason}")
            return
        if 6 in (first, second) and self.gripper_closing:
            self.println("GRIPPER_STOP reason=manual_pair_command detail=se_recibio_un_angulo_para_M6")
            self.gripper_closing = False
            self.gripper_samples = 0
        deltas = [abs(first_deg - self._logical_from_encoder(first)),
                  abs(second_deg - self._logical_from_encoder(second))]
        largest = max(deltas)
        speeds = [MOVE_SPEED, MOVE_SPEED]
        if largest > 0.1:
            speeds = [int(max(80.0, MOVE_SPEED * delta / largest)) for delta in deltas]
        self._write_pos(first, first_deg, speeds[0], MOVE_ACC)
        self._write_pos(second, second_deg, speeds[1], MOVE_ACC)
        self.last_move = self.now
        self.pending_report = None
        self.println(f"PAIR_MOVE_OK ID1={first} target1={first_deg:.2f} speed1={speeds[0]} "
                     f"ID2={second} target2={second_deg:.2f} speed2={speeds[1]}")

    # ----- firmware: garra -----
    def _gripper_current_raw(self) -> int:
        return current_raw_from_torque(float(self.arm.data.actuator_force[self.arm.actuator_ids[6]]))

    def _freeze_gripper(self, reason: str, current_raw: int) -> None:
        position = self._read_pos(6)
        # El firmware ordena la posición leída; ver la nota del módulo sobre la elasticidad.
        self.profiles[6].target = self.profiles[6].setpoint
        self.profiles[6].velocity = 0.0
        self.gripper_command = pos_to_deg(position) - 10.0
        self.gripper_closing = False
        self.gripper_samples = 0
        self.println(f"GRIPPER_HOLD reason={reason} current_raw={current_raw} "
                     f"current_ma_aprox={current_raw * 6.5:.1f} position_raw={position} "
                     f"logical_deg={self.gripper_command:.1f}")

    def _open_gripper(self) -> None:
        if self.gripper_closing:
            self.println("GRIPPER_STOP reason=open_command detail=se_cancelo_el_cierre_para_abrir")
        self.gripper_closing = False
        self.gripper_samples = 0
        self._enable_torque(6, True)
        self._write_pos(6, GRIPPER_OPEN_DEGREES, 300, 25)
        self.gripper_command = GRIPPER_OPEN_DEGREES
        self.println("GRIPPER_OPEN_OK target=140.0")

    def _start_closing(self) -> None:
        self._enable_torque(6, True)
        self.gripper_command = min(max(self._logical_from_encoder(6), GRIPPER_CLOSED_DEGREES), GRIPPER_OPEN_DEGREES)
        self.gripper_closing = True
        self.gripper_started = self.now
        self.gripper_last_step = -math.inf
        self.gripper_samples = 0
        self.println(f"GRIPPER_CLOSE_START from={self.gripper_command:.1f} contact_raw={self.contact_threshold}")

    def _process_gripper_closing(self) -> None:
        if not self.gripper_closing:
            return
        if self.now - self.gripper_last_step < GRIPPER_STEP_INTERVAL_S - 1e-9:
            return
        self.gripper_last_step = self.now
        current_raw = self._gripper_current_raw()
        elapsed = self.now - self.gripper_started
        if elapsed >= GRIPPER_CLOSE_TIMEOUT_S:
            self.println(f"GRIPPER_STOP_DETAIL reason=timeout elapsed_ms={int(elapsed * 1000)}")
            self._freeze_gripper("timeout", current_raw)
            return
        if current_raw >= GRIPPER_HARD_CURRENT_RAW:
            self.println(f"GRIPPER_STOP_DETAIL reason=hard_current threshold_raw={GRIPPER_HARD_CURRENT_RAW} "
                         f"threshold_ma_aprox={GRIPPER_HARD_CURRENT_RAW * 6.5:.1f}")
            self._freeze_gripper("hard_current", current_raw)
            self._enable_torque(6, False)
            self.println("GRIPPER_TORQUE_OFF proteccion de sobrecorriente")
            return
        if elapsed >= GRIPPER_INRUSH_IGNORE_S and current_raw >= self.contact_threshold:
            self.gripper_samples += 1
        else:
            self.gripper_samples = 0
        if self.gripper_samples >= GRIPPER_CONTACT_SAMPLES:
            self.println(f"GRIPPER_STOP_DETAIL reason=contact samples={self.gripper_samples} "
                         f"threshold_raw={self.contact_threshold} "
                         f"threshold_ma_aprox={self.contact_threshold * 6.5:.1f}")
            self._freeze_gripper("contact", current_raw)
            return
        if self.gripper_command <= GRIPPER_CLOSED_DEGREES:
            actual = min(max(self._logical_from_encoder(6), GRIPPER_CLOSED_DEGREES), GRIPPER_OPEN_DEGREES)
            if actual <= GRIPPER_CLOSED_DEGREES + GRIPPER_CLOSED_TOLERANCE_DEGREES:
                self.println(f"GRIPPER_STOP_DETAIL reason=closed_limit actual_logical_deg={actual:.1f}")
                self._freeze_gripper("closed_limit", current_raw)
                return
            self._write_pos(6, GRIPPER_CLOSED_DEGREES, 220, 18)
            return
        self.gripper_command = max(GRIPPER_CLOSED_DEGREES, self.gripper_command - GRIPPER_STEP_DEGREES)
        self._write_pos(6, self.gripper_command, 220, 18)

    # ----- firmware: comandos -----
    def handle_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        lowered = line.lower()
        if lowered == "help":
            self._help()
        elif lowered == "scan":
            for motor in range(1, 7):
                self._read_one(motor)
            self._divider()
        elif lowered == "posall":
            self._print_all_positions()
        elif lowered in ("gripper open", "abrir garra"):
            self._open_gripper()
        elif lowered in ("gripper close", "cerrar garra"):
            self._start_closing()
        elif lowered in ("gripper stop", "parar garra"):
            self._freeze_gripper("manual", self._gripper_current_raw())
        elif lowered in ("gripper release", "garra libre"):
            if self.gripper_closing:
                self.println("GRIPPER_STOP reason=torque_release detail=se_cancelo_el_cierre")
            self.gripper_closing = False
            self.gripper_samples = 0
            result = self._enable_torque(6, False)
            self.println(f"GRIPPER_TORQUE_OFF reason=manual_release result={result}")
        elif line.startswith("gripper current "):
            requested = _to_int(line[16:])
            if not 20 <= requested <= 180:
                self.println("Umbral invalido. Usa gripper current 20..180.")
            else:
                self.contact_threshold = requested
                self.println(f"GRIPPER_CURRENT_THRESHOLD={requested}")
        elif lowered in ("arm torque on", "arm torque off"):
            enable = lowered == "arm torque on"
            self.println("Activando torque del brazo M1..M5..." if enable
                         else "Desactivando torque del brazo M1..M5... M6 no cambia.")
            for motor in range(1, 6):
                result = self._enable_torque(motor, enable)
                self.println(f"ID {motor}: torque {'ON' if enable else 'OFF'} resultado={result}")
            self.println("ARM_TORQUE_ON_OK" if enable else "ARM_TORQUE_OFF_OK")
        elif lowered in ("torque on", "torque off"):
            enable = lowered == "torque on"
            self.println("Activando torque global M1..M6..." if enable else "Desactivando torque global M1..M6...")
            for motor in range(1, 7):
                result = self._enable_torque(motor, enable)
                self.println(f"ID {motor}: torque {'ON' if enable else 'OFF'} resultado={result}")
            self.println("TORQUE_ON_OK" if enable else "TORQUE_OFF_OK")
        elif line.startswith("read "):
            self._read_one(_to_int(line[5:]))
            self._divider()
        elif line.startswith(("bpair ", "pair ")):
            values = line.split()[1:]
            if len(values) != 4:
                self.println("PAIR_REJECTED reason=formato_bpair" if line.startswith("bpair ")
                             else "PAIR_REJECTED reason=formato usa: pair 1 90 2 80")
                return
            try:
                first, first_deg = int(values[0]), float(values[1])
                second, second_deg = int(values[2]), float(values[3])
            except ValueError:
                self.println("PAIR_REJECTED reason=formato usa: pair 1 90 2 80")
                return
            self._move_pair(first, first_deg, second, second_deg)
        elif line.startswith("bmove "):
            values = line.split()[1:]
            try:
                self._move_one(int(values[0]), float(values[1]))
            except (IndexError, ValueError):
                self.println("Bloqueado: formato_bmove")
        elif line.startswith("motor"):
            if " " not in line:
                self.println("Formato: motor2 90")
                return
            head, tail = line.split(" ", 1)
            self._move_one(_to_int(head[5:]), _to_float(tail))
        else:
            if " " not in line:
                self.println("Comando invalido. Usa 'help'.")
                return
            head, tail = line.split(" ", 1)
            self._move_one(_to_int(head), _to_float(tail))


def _to_int(text: str) -> int:
    """Como `String.toInt()` de Arduino: lee el entero inicial, 0 si no hay."""
    text = text.strip()
    digits = ""
    for index, char in enumerate(text):
        if char.isdigit() or (index == 0 and char in "+-"):
            digits += char
        else:
            break
    try:
        return int(digits)
    except ValueError:
        return 0


def _to_float(text: str) -> float:
    """Como `String.toFloat()` de Arduino: lee el número inicial, 0 si no hay."""
    text = text.strip()
    best = 0.0
    for end in range(1, len(text) + 1):
        try:
            best = float(text[:end])
        except ValueError:
            if end > 2:
                break
    return best


# ----- cámaras virtuales -----
# GLFW no tolera crear contextos de OpenGL a la vez desde dos hilos («la clase ya existe»).
_RENDERER_LOCK = threading.Lock()


def make_renderer(model, height: int, width: int) -> mujoco.Renderer:
    with _RENDERER_LOCK:
        return mujoco.Renderer(model, height=height, width=width)


class VirtualCameraStream(CameraStream):
    """`CameraStream` que muestra una cámara virtual del gemelo en vez de DirectShow."""

    def __init__(self, twin: DigitalTwin, camera: str, index: int = -1, rotation_degrees: int = 0,
                 *, fps: float = 12.0, height: int = 480, width: int = 640, **image_parameters):
        if camera not in CAMERAS:
            raise ValueError(f"Cámara virtual desconocida: {camera}")
        self.twin = twin
        self.camera_name = camera
        self.period = 1.0 / max(1.0, fps)
        self.frame_size = (height, width)
        super().__init__(index, rotation_degrees, **image_parameters)
        self.status = "Gemelo digital: preparando la cámara virtual..."

    def _run(self) -> None:
        renderer = None
        try:
            renderer = make_renderer(self.twin.arm.model, *self.frame_size)
            self.status = f"Gemelo digital · cámara virtual {self.camera_name}"
            while not self.stop_event.is_set():
                with self.twin.lock:
                    renderer.update_scene(self.twin.arm.data, camera=self.camera_name)
                frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
                self._publish(frame)
                self.stop_event.wait(self.period)
        except Exception as exc:  # sin OpenGL, por ejemplo
            self.status = f"Cámara virtual sin imagen: {exc}"
        finally:
            if renderer is not None:
                renderer.close()


def virtual_camera_config(base: AppConfig) -> dict[str, CameraConfig]:
    """Cámaras del gemelo: sin rotación ni ajustes, con nombre propio para la calibración."""
    return {
        camera: CameraConfig(name=f"virtual {camera}", fallback_index=-1, rotation_degrees=0,
                             brightness_percent=0.0, contrast_percent=0.0, saturation_percent=0.0,
                             zoom_factor=1.0)
        for camera in CAMERAS if camera in base.cameras
    }


def twin_config(base: AppConfig | None = None) -> AppConfig:
    """Configuración de la aplicación para el gemelo: puerto y cámaras virtuales.

    Los límites y la detección son los del brazo real salvo los umbrales del
    recipiente, que en las cámaras virtuales se ve más chico (`SIM_DETECTION`)."""
    base = base or load_config()
    vision = dataclasses.replace(
        base.vision,
        minimum_red_area_ratio=SIM_DETECTION["minimum_red_area_ratio"],
        minimum_lime_area_ratio=SIM_DETECTION["minimum_lime_area_ratio"],
        frontal_lime_hsv_lower=list(SIM_DETECTION["lime_hsv_lower"]),
        frontal_lime_hsv_upper=list(SIM_DETECTION["lime_hsv_upper"]),
        frontal_minimum_lime_area_ratio=SIM_DETECTION["minimum_lime_area_ratio"],
        frontal_minimum_lime_shape_ratio=0.0,
        frontal_minimum_lime_confidence=0.0,
    )
    return dataclasses.replace(base, port=VIRTUAL_PORT, cameras=virtual_camera_config(base), vision=vision)


# ----- calibración de las cámaras virtuales -----
class VirtualCameras:
    """Fotos de la escena de Inicio con las cámaras virtuales, sobre un brazo propio."""

    def __init__(self, arm: VirtualArm, height=480, width=640):
        self.arm = arm
        self.renderer = make_renderer(arm.model, height, width)

    def frames(self) -> list[np.ndarray]:
        frames = []
        for camera in CAMERAS:
            self.renderer.update_scene(self.arm.data, camera=camera)
            frames.append(cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR))
        return frames

    def features(self, placement) -> np.ndarray:
        """Las 32 características de la escena de Inicio, como las calcula la aplicación."""
        self.arm.reset(placement)
        return observation_features([detect_scene(frame, **SIM_DETECTION) for frame in self.frames()])

    def close(self) -> None:
        self.renderer.close()


def placement_by_hand(rng, max_yaw_degrees: float) -> Placement:
    """Escena al azar con el cubo y el recipiente alineados a mano con la base."""
    limit = math.radians(max_yaw_degrees)
    return dataclasses.replace(sample_placement(rng), cube_yaw=float(rng.uniform(-limit, limit)),
                               container_yaw=float(rng.uniform(-limit, limit)))


def calibration_samples(cameras: VirtualCameras, rng, count: int, *, containers: bool,
                        max_yaw_degrees: float) -> list[Sample]:
    samples = []
    for index in range(count):
        placement = placement_by_hand(rng, max_yaw_degrees)
        features = cameras.features(placement)
        objects = [("cubo", placement.cube_xy)] + ([("recipiente", placement.container_xy)] if containers else [])
        for kind, xy in objects:
            pixels = object_pixels(features, kind)
            if all(camera in pixels for camera in CAMERAS):
                samples.append(Sample(kind, pixels, (float(xy[0]), float(xy[1])), f"simulación {index + 1}"))
    return samples


_CALIBRATION_LOCK = threading.Lock()


def _virtual_cameras_fingerprint(arm: VirtualArm, scenes: int, seed: int, max_yaw_degrees: float) -> str:
    """Lo que cambia el mapeo píxel → mesa de las cámaras virtuales, además del modelo 3D."""
    import hashlib
    import json

    model = arm.model
    payload = {
        "camaras": [[round(float(value), 6) for value in (*model.cam_pos[index], *model.cam_quat[index],
                                                          model.cam_fovy[index])]
                    for index in range(model.ncam)],
        "deteccion": {key: list(value) if isinstance(value, tuple) else value for key, value in SIM_DETECTION.items()},
        "escenas": scenes, "semilla": seed, "giro_max": max_yaw_degrees,
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode("utf-8")).hexdigest()[:8]


TWIN_CALIBRATION_SCENES = 25
TWIN_CALIBRATION_SEED = 7
TWIN_CALIBRATION_MAX_YAW = 5.0


def twin_model_suffix(arm: VirtualArm, scenes: int = TWIN_CALIBRATION_SCENES, seed: int = TWIN_CALIBRATION_SEED,
                      max_yaw_degrees: float = TWIN_CALIBRATION_MAX_YAW) -> str:
    """Se agrega a la huella del modelo 3D en la calibración virtual."""
    return "+camaras-virtuales-" + _virtual_cameras_fingerprint(arm, scenes, seed, max_yaw_degrees)


def load_twin_policy(config: AppConfig, *, arm: VirtualArm, expert=None):
    """Política geométrica con la calibración de las cámaras virtuales."""
    from .politica_geometrica import load_geometric_policy

    return load_geometric_policy(config, path=twin_calibration_path(config), model_suffix=twin_model_suffix(arm),
                                 arm=arm, expert=expert)


def twin_calibration_path(config: AppConfig) -> Path:
    return config.root / TWIN_CALIBRATION_FILE


def ensure_twin_calibration(config: AppConfig, *, scenes: int = TWIN_CALIBRATION_SCENES,
                            seed: int = TWIN_CALIBRATION_SEED,
                            max_yaw_degrees: float = TWIN_CALIBRATION_MAX_YAW, force: bool = False,
                            progress=None) -> CameraCalibration:
    """Calibración de las cámaras virtuales; se recalcula si falta o cambió el modelo 3D.

    Es como la del brazo real (cubo y recipiente en lugares al azar, una foto de cada
    cámara y la posición verdadera), pero automática. Nunca toca `camaras.json`."""
    config = twin_config(config) if config.port != VIRTUAL_PORT else config
    with _CALIBRATION_LOCK:  # la app la pide al arrancar y al iniciar una tarea
        return _ensure_twin_calibration(config, scenes, seed, max_yaw_degrees, force, progress)


def _ensure_twin_calibration(config, scenes, seed, max_yaw_degrees, force, progress) -> CameraCalibration:
    path = twin_calibration_path(config)
    arm = VirtualArm(config.root / "assets", limits=config.limits)
    # La huella del modelo 3D más la de las cámaras virtuales, que model_problems compara entera.
    fingerprint = model_fingerprint(config.root / "assets") + twin_model_suffix(arm, scenes, seed, max_yaw_degrees)
    if path.is_file() and not force:
        try:
            calibration = CameraCalibration.load(path)
            if not calibration.model_problems(fingerprint) and not calibration.camera_problems(config.cameras):
                return calibration
        except (OSError, ValueError, KeyError):
            pass
    if progress is not None:
        progress(f"Calibrando las cámaras virtuales con {scenes} escenas...")
    cameras = VirtualCameras(arm)
    try:
        samples = calibration_samples(cameras, np.random.default_rng(seed), scenes, containers=True,
                                      max_yaw_degrees=max_yaw_degrees)
    finally:
        cameras.close()
    settings = {camera: {"nombre": item.name, "rotacion": int(item.rotation_degrees), "zoom": float(item.zoom_factor)}
                for camera, item in config.cameras.items()}
    calibration = fit_calibration(samples, settings, model=fingerprint)
    calibration.save(path)
    return calibration
