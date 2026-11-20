from __future__ import annotations

import csv
import queue
import re
import secrets
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import serial


COMPACT_POSITION = re.compile(
    r"M([1-6])=(NA|-?\d+)(?:\(([-+]?\d+(?:\.\d+)?)deg\))?"
)
SINGLE_POSITION = re.compile(
    r"POS ID ([1-6]):\s*(-?\d+)\s*\(~([-+]?\d+(?:\.\d+)?) deg\)"
)

WAYPOINT_REPLY = re.compile(r"WAYPOINT_(ACCEPTED|DONE|CANCELLED|REJECTED) id=(\d+)(?: reason=(\w+))?")


FAULT_MARKERS = (
    "pair_rejected",
    "sync_rejected",
    "power_guard_hold",
    "power_guard_rejected",
    "arm_speed_rejected",
    "gripper_speed_rejected",
    "bloqueado",
    "fuera de rango",
    "sin respuesta",
    "no responde",
    "no se pudo leer",
    "cooldown activo",
    "gripper_error",
    "reason=communication_error",
    "reason=hard_current",
    "reason=timeout",
    "ajuste seguro",
    "ajuste limite",
)


@dataclass(frozen=True)
class RobotEvent:
    timestamp: float
    kind: str
    payload: str


@dataclass(frozen=True)
class RobotSnapshot:
    positions: dict[int, float]
    received_at: dict[int, float]
    torque_enabled: bool | None
    gripper_state: str
    fault: str
    # Acuses de órdenes de movimiento («Moviendo ID…», «PAIR_MOVE_OK…») y GRIPPER_HOLD
    # recibidos desde la conexión: sirven para esperar la respuesta a una orden nueva y no
    # tomar la de una anterior.
    move_acks: int = 0
    gripper_holds: int = 0
    gripper_hold_reason: str = ""
    # Escala de velocidad de M1–M5 del supervisor de la fuente (1.0 = sin limitar).
    speed_scale: float = 1.0


POWER_TELEMETRY = re.compile(
    r"POWER_TELEM ms=(\d+) I=(-?\d+(?:,-?\d+){5}) total_ma=(-?\d+) v_min_dv=(-?\d+) v_id=(\d+)"
)
CURRENT_MA_PER_UNIT = 6.5  # registro de corriente del STS3215, como en el firmware
POWER_LOG_COLUMNS = (
    ["hora", "t_s", "firmware_ms"] + [f"I{motor}_mA" for motor in range(1, 7)]
    + ["total_mA", "v_min_V", "v_id"] + [f"M{motor}_deg" for motor in range(1, 7)] + ["etapa"]
)


@dataclass(frozen=True)
class PowerSample:
    """Una línea de `power telem on`: corriente de M1..M6 (mA; None sin lectura) y el
    voltaje más bajo del bus (V; None sin lectura)."""

    firmware_ms: int
    currents_ma: tuple[float | None, ...]
    total_ma: float
    v_min: float | None
    v_id: int


def parse_power(line: str) -> PowerSample | None:
    match = POWER_TELEMETRY.match(line.strip())
    if match is None:
        return None
    raw = [int(value) for value in match.group(2).split(",")]
    voltage = int(match.group(4))
    return PowerSample(
        firmware_ms=int(match.group(1)),
        currents_ma=tuple(None if value < 0 else value * CURRENT_MA_PER_UNIT for value in raw),
        total_ma=float(match.group(3)),
        v_min=None if voltage < 0 else voltage / 10.0,
        v_id=int(match.group(5)),
    )


def parse_positions(line: str) -> dict[int, float | None]:
    """Convierte la telemetria del firmware a grados logicos."""
    values: dict[int, float | None] = {}
    for match in COMPACT_POSITION.finditer(line):
        motor_id = int(match.group(1))
        raw_text = match.group(2)
        if raw_text == "NA" or int(raw_text) < 0:
            values[motor_id] = None
            continue
        raw_degrees = (
            float(match.group(3))
            if match.group(3) is not None
            else int(raw_text) * 360.0 / 4095.0
        )
        values[motor_id] = raw_degrees - (10.0 if motor_id == 6 else 0.0)

    single = SINGLE_POSITION.search(line)
    if single:
        motor_id = int(single.group(1))
        if int(single.group(2)) < 0:
            values[motor_id] = None
        else:
            values[motor_id] = float(single.group(3)) - (
                10.0 if motor_id == 6 else 0.0
            )
    return values


def motion_commands(motion) -> list[str]:
    """Órdenes que dejan el firmware como pide `config.movimiento` (MotionConfig)."""
    commands = [f"speed max {int(motion.max_speed)} {int(motion.acceleration)}"]
    if motion.guard_enabled:
        commands.append(f"power guard {round(motion.guard_slow_volts * 10)} {round(motion.guard_hold_volts * 10)} "
                        f"{int(motion.guard_slow_ma)} {int(motion.guard_hold_ma)}")
        commands.append("power guard on")
    else:
        commands.append("power guard off")
    return commands


def configure_motion(robot: "RobotSerial", motion, timeout: float = 1.5) -> list[str]:
    """Después de conectar: espera la respuesta a `caps` y, si el firmware tiene `speed max`
    y `power guard`, le manda la configuración. Con el firmware de la exposición no manda
    nada. Devuelve las órdenes enviadas."""
    deadline = time.monotonic() + timeout
    while not robot.guard_supported and time.monotonic() < deadline:
        time.sleep(0.02)
    if not robot.guard_supported:
        return []
    commands = motion_commands(motion)
    if robot.gripper_fast_supported:
        commands.append(f"gripper speed {int(motion.gripper_open_speed)}")
    for command in commands:
        robot.send(command)
    return commands


def is_fault_line(line: str) -> bool:
    lowered = line.strip().lower()
    if line.startswith("GRABARB_OVERRIDE"):
        return False
    return any(marker in lowered for marker in FAULT_MARKERS)


def discard_partial_line(connection: serial.Serial) -> None:
    """Descarta el resto de la linea que reset_input_buffer() dejo cortada.

    Con la telemetria a 20 Hz casi siempre hay una linea en camino; sin esto
    su final llega como una linea invalida. Si no habia ninguna cortada se
    pierde una de telemetria.
    """
    for _ in range(4096):
        byte = connection.read(1)
        if not byte or byte == b"\n":
            return


class RobotSerial:
    """Un solo hilo posee el puerto y publica telemetria inmutable."""

    def __init__(self, port: str, baud: int, history_seconds: float = 3.0, opener=None):
        """`opener`: reemplazo de `serial.Serial` con su misma firma; el gemelo
        digital pasa `DigitalTwin.open_port`. Sin él se abre el puerto real."""
        self.port = port
        self.opener = opener or serial.Serial
        self.baud = baud
        self.history_seconds = history_seconds
        self.connection: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.events: queue.Queue[RobotEvent] = queue.Queue()
        self.positions: dict[int, float] = {}
        self.received_at: dict[int, float] = {}
        self.history: dict[int, deque[tuple[float, float]]] = {
            motor_id: deque() for motor_id in range(1, 7)
        }
        self.torque_enabled: bool | None = None
        self.gripper_state = "idle"
        self.fault = ""
        self.move_acks = 0
        self._waypoint_id = secrets.randbelow(2147483647)
        self._waypoint_reply: tuple[int, str, str] | None = None
        self.gripper_holds = 0
        self.gripper_hold_reason = ""
        # El firmware con la orden `sync` lo anuncia al responder `caps`; el cargado
        # el 6/11 no responde y queda en False: se sigue usando pair como siempre.
        self.sync_supported = False
        # `caps` con `guard=1`: el firmware tiene `power guard` y `speed max`.
        self.guard_supported = False
        # `caps` con `gripper_fast=1`: `gripper close <hasta>` y `gripper speed`.
        self.gripper_fast_supported = False
        self.speed_scale = 1.0
        self.arm_max_speed: int | None = None
        # Medición de la fuente: última muestra, CSV abierto y la etapa que la app anota.
        self.power_last: PowerSample | None = None
        self.power_last_at = 0.0
        self.power_label = ""
        self._power_file = None
        self._power_writer = None
        self._power_started = 0.0

    @property
    def connected(self) -> bool:
        return self.connection is not None and self.connection.is_open

    def connect(self) -> None:
        if self.connected:
            return
        self.stop_event.clear()
        connection = self.opener(
            self.port, self.baud, timeout=0.1, write_timeout=0.5
        )
        connection.dtr = False
        connection.rts = False
        time.sleep(2.0)
        if self.stop_event.is_set():
            connection.close()
            raise RuntimeError("Conexión cancelada.")
        connection.reset_input_buffer()
        discard_partial_line(connection)
        self.connection = connection
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.events.put(RobotEvent(time.monotonic(), "connected", self.port))
        # Lectura solamente: conectar nunca ordena movimiento ni cambia torque.
        self.send("posall")
        self.send("caps")

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.5)
        connection, self.connection = self.connection, None
        if connection is not None:
            connection.close()
        self.events.put(RobotEvent(time.monotonic(), "disconnected", self.port))

    def send(self, command: str) -> None:
        connection = self.connection
        if connection is None or not connection.is_open:
            raise RuntimeError("El ESP32 no está conectado.")
        clean = command.strip()
        if not clean:
            return
        with self.write_lock:
            connection.write((clean + "\n").encode("ascii"))
            connection.flush()
        self.events.put(RobotEvent(time.monotonic(), "tx", clean))

    def power_reading(self) -> tuple[PowerSample | None, float]:
        """La última muestra de `power telem` y cuándo llegó (time.monotonic)."""
        with self.state_lock:
            return self.power_last, self.power_last_at

    def start_power_log(self, path: Path) -> None:
        """Graba cada muestra de `power telem on` en `path` (CSV), con los ángulos y la etapa."""
        self.stop_power_log()
        path.parent.mkdir(parents=True, exist_ok=True)
        handle = path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(handle)
        writer.writerow(POWER_LOG_COLUMNS)
        with self.state_lock:
            self._power_file, self._power_writer = handle, writer
            self._power_started = time.monotonic()

    def stop_power_log(self) -> None:
        with self.state_lock:
            handle, self._power_file, self._power_writer = self._power_file, None, None
        if handle is not None:
            handle.close()

    def clear_fault(self) -> None:
        with self.state_lock:
            self.fault = ""

    def next_waypoint_id(self) -> int:
        with self.state_lock:
            self._waypoint_id = self._waypoint_id % 2147483647 + 1
            self._waypoint_reply = None
            return self._waypoint_id

    def waypoint_reply(self, waypoint_id: int) -> tuple[str, str] | None:
        with self.state_lock:
            reply = self._waypoint_reply
            if reply is not None and reply[0] == waypoint_id:
                return reply[1:]
            return None

    def snapshot(self) -> RobotSnapshot:
        with self.state_lock:
            return RobotSnapshot(
                positions=dict(self.positions),
                received_at=dict(self.received_at),
                torque_enabled=self.torque_enabled,
                gripper_state=self.gripper_state,
                fault=self.fault,
                move_acks=self.move_acks,
                speed_scale=self.speed_scale,
                gripper_holds=self.gripper_holds,
                gripper_hold_reason=self.gripper_hold_reason,
            )

    def stable_pose(
        self,
        now: float,
        *,
        maximum_age: float,
        window_seconds: float,
        maximum_range: float,
    ) -> tuple[dict[int, float] | None, str]:
        with self.state_lock:
            if any(now - self.received_at.get(i, 0.0) > maximum_age for i in range(1, 7)):
                return None, "Falta telemetría reciente de los seis motores."
            result = dict(self.positions)
            for motor_id in range(1, 7):
                samples = [
                    value
                    for stamp, value in self.history[motor_id]
                    if now - stamp <= window_seconds
                ]
                if len(samples) < 3:
                    return None, f"M{motor_id} todavía no tiene suficientes lecturas."
                if max(samples) - min(samples) > maximum_range:
                    return None, f"M{motor_id} todavía se está moviendo."
            return result, ""

    def _reader_loop(self) -> None:
        connection = self.connection
        assert connection is not None
        buffer = b""
        try:
            while not self.stop_event.is_set():
                chunk = connection.read(max(1, min(connection.in_waiting, 4096)))
                if chunk:
                    buffer += chunk
                while b"\n" in buffer:
                    raw_line, buffer = buffer.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if line:
                        self._consume_line(line, time.monotonic())
                if len(buffer) > 16384:
                    raise RuntimeError("Respuesta serie sin fin de línea.")
        except Exception as exc:
            with self.state_lock:
                self.fault = f"Conexión serie: {exc}"
            self.events.put(RobotEvent(time.monotonic(), "error", str(exc)))

    def _write_power(self, sample: PowerSample, stamp: float) -> None:
        """Bajo `state_lock`. Un error de disco corta la grabación, no la conexión."""
        def number(value, digits=0):
            return "" if value is None else f"{value:.{digits}f}"

        try:
            self._power_writer.writerow(
                [datetime.now().isoformat(timespec="milliseconds"), f"{stamp - self._power_started:.3f}",
                 sample.firmware_ms]
                + [number(value) for value in sample.currents_ma]
                + [number(sample.total_ma), number(sample.v_min, 1), sample.v_id]
                + [number(self.positions.get(motor), 1) for motor in range(1, 7)]
                + [self.power_label]
            )
            self._power_file.flush()
        except (OSError, ValueError):
            self._power_file, self._power_writer = None, None

    def _consume_line(self, line: str, stamp: float) -> None:
        values = parse_positions(line)
        with self.state_lock:
            for motor_id, value in values.items():
                if value is None:
                    self.received_at.pop(motor_id, None)
                    continue
                self.positions[motor_id] = value
                self.received_at[motor_id] = stamp
                history = self.history[motor_id]
                history.append((stamp, value))
                while history and stamp - history[0][0] > self.history_seconds:
                    history.popleft()

            if line in ("TORQUE_ON_OK", "ARM_TORQUE_ON_OK"):
                self.torque_enabled = True
            elif line in ("TORQUE_OFF_OK", "ARM_TORQUE_OFF_OK"):
                self.torque_enabled = False
            elif line.startswith("GRIPPER_CLOSE_START"):
                self.gripper_state = "closing"
            elif line.startswith("GRIPPER_OPEN_OK"):
                self.gripper_state = "opening"
            elif line.startswith("GRIPPER_HOLD"):
                self.gripper_state = "holding"
                self.gripper_holds += 1
                reason = next((part[7:] for part in line.split() if part.startswith("reason=")), "")
                self.gripper_hold_reason = reason
            elif line.startswith(("Moviendo ID", "PAIR_MOVE_OK", "SYNC_MOVE_OK")):
                self.move_acks += 1
            elif line.startswith("FIRMWARE_CAPS "):
                fields = line.split()
                self.sync_supported = "sync=1" in fields
                self.guard_supported = "guard=1" in fields
                self.gripper_fast_supported = "gripper_fast=1" in fields
            elif line.startswith("POWER_GUARD scale="):
                try:
                    self.speed_scale = int(line.split()[1].split("=", 1)[1]) / 100.0
                except (IndexError, ValueError):
                    pass
            elif line.startswith("POWER_GUARD="):
                self.speed_scale = 1.0
            elif line.startswith("ARM_SPEED max="):
                try:
                    self.arm_max_speed = int(line.split()[1].split("=", 1)[1])
                except (IndexError, ValueError):
                    pass
            elif line.startswith("POWER_TELEM "):
                sample = parse_power(line)
                if sample is not None:
                    self.power_last, self.power_last_at = sample, stamp
                    if self._power_writer is not None:
                        self._write_power(sample, stamp)
            elif line.startswith("GRIPPER_TORQUE_OFF"):
                self.gripper_state = "released"

            if reply := WAYPOINT_REPLY.fullmatch(line):
                reply_id = int(reply[2])
                if reply_id == self._waypoint_id:
                    self._waypoint_reply = (reply_id, reply[1], reply[3] or "")
            if is_fault_line(line):
                self.fault = line
        self.events.put(RobotEvent(stamp, "rx", line))

