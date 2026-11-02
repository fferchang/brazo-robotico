from __future__ import annotations

import queue
import re
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
REASON = re.compile(r"reason=(\w+)")
GRIPPER_CURRENT = re.compile(r"^GRIPPER_I raw=(-?\d+)")

# Estado de la garra según el motivo de GRIPPER_HOLD. Solo «contact» significa
# que la garra se detuvo sobre un objeto; «closed_limit» es un cierre sin objeto.
HOLD_STATES = {
    "contact": "holding",
    "closed_limit": "closed",
    "manual": "stopped",
    "timeout": "stopped",
}

FAULT_MARKERS = (
    "pair_rejected",
    "bloqueado",
    "fuera de rango",
    "sin respuesta",
    "no responde",
    "no se pudo leer",
    "cooldown activo",
    "gripper_error",
    "gripper_fault",
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
    # Motivo y momento (time.monotonic) del último cambio de estado de la garra.
    gripper_reason: str = ""
    gripper_updated_at: float = 0.0
    gripper_current_raw: int | None = None


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


def is_fault_line(line: str) -> bool:
    lowered = line.strip().lower()
    if line.startswith("GRABARB_OVERRIDE"):
        return False
    return any(marker in lowered for marker in FAULT_MARKERS)


class RobotSerial:
    """Un solo hilo posee el puerto y publica telemetria inmutable."""

    def __init__(
        self,
        port: str,
        baud: int,
        history_seconds: float = 3.0,
        log_folder: Path | None = None,
    ):
        self.port = port
        self.baud = baud
        self.history_seconds = history_seconds
        # Registro persistente de todo lo enviado y recibido, un archivo por conexión.
        self.log_folder = log_folder
        self.log_path: Path | None = None
        self.log_lock = threading.Lock()
        self._log_file = None
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
        self.gripper_reason = ""
        self.gripper_updated_at = 0.0
        self.gripper_current_raw: int | None = None
        self.fault = ""

    @property
    def connected(self) -> bool:
        return self.connection is not None and self.connection.is_open

    def connect(self) -> None:
        if self.connected:
            return
        self.stop_event.clear()
        connection = serial.Serial(
            self.port, self.baud, timeout=0.1, write_timeout=0.5
        )
        connection.dtr = False
        connection.rts = False
        time.sleep(2.0)
        if self.stop_event.is_set():
            connection.close()
            raise RuntimeError("Conexión cancelada.")
        connection.reset_input_buffer()
        self._open_log()
        self.connection = connection
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.events.put(RobotEvent(time.monotonic(), "connected", self.port))
        # Lectura solamente: conectar nunca ordena movimiento ni cambia torque.
        self.send("posall")

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.5)
        connection, self.connection = self.connection, None
        if connection is not None:
            connection.close()
        self.events.put(RobotEvent(time.monotonic(), "disconnected", self.port))
        with self.log_lock:
            if self._log_file is not None:
                self._log_file.close()
                self._log_file = None

    def _open_log(self) -> None:
        if self.log_folder is None:
            return
        self.log_folder.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime("serie_%Y%m%d_%H%M%S_%f.log")
        with self.log_lock:
            self.log_path = self.log_folder / name
            self._log_file = self.log_path.open("a", encoding="utf-8", buffering=1)

    def _log(self, stamp: float, kind: str, payload: str) -> None:
        with self.log_lock:
            if self._log_file is not None:
                self._log_file.write(f"{stamp:.3f}\t{kind}\t{payload}\n")

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
        stamp = time.monotonic()
        self._log(stamp, "tx", clean)
        self.events.put(RobotEvent(stamp, "tx", clean))

    def clear_fault(self) -> None:
        with self.state_lock:
            self.fault = ""

    def snapshot(self) -> RobotSnapshot:
        with self.state_lock:
            return RobotSnapshot(
                positions=dict(self.positions),
                received_at=dict(self.received_at),
                torque_enabled=self.torque_enabled,
                gripper_state=self.gripper_state,
                fault=self.fault,
                gripper_reason=self.gripper_reason,
                gripper_updated_at=self.gripper_updated_at,
                gripper_current_raw=self.gripper_current_raw,
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

            reason_match = REASON.search(line)
            reason = reason_match.group(1) if reason_match else ""
            current = GRIPPER_CURRENT.match(line)
            gripper_state: str | None = None
            if current:
                self.gripper_current_raw = int(current.group(1))
            elif line in ("TORQUE_ON_OK", "ARM_TORQUE_ON_OK"):
                self.torque_enabled = True
            elif line == "TORQUE_OFF_OK":
                self.torque_enabled = False
                gripper_state = "released"  # El apagado global también desenergiza M6.
            elif line == "ARM_TORQUE_OFF_OK":
                self.torque_enabled = False
            elif line.startswith("GRIPPER_CLOSE_START"):
                gripper_state = "closing"
            elif line.startswith("GRIPPER_OPEN_OK"):
                gripper_state = "opening"
            elif line.startswith("GRIPPER_HOLD"):
                gripper_state = HOLD_STATES.get(reason, "stopped")
            elif line.startswith("GRIPPER_TORQUE_OFF"):
                gripper_state = "released"
            elif line.startswith("GRIPPER_FAULT") or (
                line.startswith("GRIPPER_STOP ") and reason == "communication_error"
            ):
                gripper_state = "error"
            if gripper_state is not None:
                self.gripper_state = gripper_state
                self.gripper_reason = reason
                self.gripper_updated_at = stamp

            if is_fault_line(line):
                self.fault = line
        self._log(stamp, "rx", line)
        self.events.put(RobotEvent(stamp, "rx", line))

