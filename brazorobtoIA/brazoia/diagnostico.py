"""Prueba de hardware de solo lectura: las dos cámaras y el ESP32.

Usa las mismas clases y criterios que la aplicación (CameraStream,
RobotSerial y config.json). Al ESP32 solamente le envía `help`, `posall` y
`scan`: nunca ordena movimiento ni cambia el torque.
"""
from __future__ import annotations

import json
import platform
import queue
import re
import statistics
import threading
import time
from collections import Counter
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import serial
from serial.tools import list_ports

from .capture import CameraStream, FrameSnapshot, discover_camera_index, validate_frame_pair
from .config import AppConfig
from .robot import RobotEvent, RobotSerial, is_fault_line, parse_positions
from .vision import detect_scene, draw_observation


READ_ONLY_COMMANDS = ("help", "posall", "scan")

OK = "OK"
AVISO = "AVISO"
FALLA = "FALLA"
REVISAR = "REVISAR"
OMITIDA = "OMITIDA"
_SEVERITY = {OK: 0, REVISAR: 0, OMITIDA: 0, AVISO: 1, FALLA: 2}
GROUP_ORDER = ("entorno", "firmware", "cámaras", "esp32", "medición", "integración")

MOTORS = tuple(range(1, 7))
# Los mismos marcadores que App.refresh_ports usa para elegir el adaptador.
USB_MARKERS = ("usb", "ch340", "cp210", "ftdi")
# Mensajes de la ROM del ESP32 y de setup() en firmware_secuencias.ino.
BOOT_MARKERS = ("rst:", "ets ", "Control sincronizado de hasta dos motores")
SCAN_LINE = re.compile(
    r"^ID ([1-6]): pos=(-?\d+) \(~(-?\d+(?:\.\d+)?) deg\) \| V=(-?\d+) \| T=(-?\d+)$"
)
SCAN_MISSING = re.compile(r"^ID ([1-6]): sin respuesta$")
HELP_TIMEOUT_S = 1.5
POSALL_TIMEOUT_S = 1.5
SCAN_TIMEOUT_S = 3.0
TICK_S = 0.05
MINIMUM_VOLTAGE_V = 4.5
MAXIMUM_VOLTAGE_SPREAD_V = 0.5
HOT_SERVO_C = 50


# --------------------------------------------------------------------------
# Reporte


@dataclass
class Check:
    group: str
    name: str
    status: str
    detail: str
    data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "grupo": self.group,
            "prueba": self.name,
            "estado": self.status,
            "detalle": self.detail,
            "datos": self.data,
        }


class Report:
    def __init__(self) -> None:
        self.checks: list[Check] = []
        self.info: dict[str, Any] = {}
        self.serial_log: list[str] = []

    def add(self, group: str, name: str, status: str, detail: str, **data: Any) -> Check:
        check = Check(group, name, status, detail, data)
        self.checks.append(check)
        return check

    def find(self, group: str, name: str) -> Check | None:
        return next(
            (check for check in self.checks if check.group == group and check.name == name),
            None,
        )

    def counts(self) -> Counter:
        return Counter(check.status for check in self.checks)

    @property
    def failed(self) -> bool:
        return any(check.status == FALLA for check in self.checks)

    def ordered(self) -> list[Check]:
        """Pruebas agrupadas; dentro de cada grupo, en el orden en que se hicieron."""
        def position(check: Check) -> int:
            return GROUP_ORDER.index(check.group) if check.group in GROUP_ORDER else len(GROUP_ORDER)

        return sorted(self.checks, key=position)

    def to_dict(self) -> dict[str, Any]:
        return {
            "info": self.info,
            "resumen": dict(self.counts()),
            "pruebas": [check.to_dict() for check in self.ordered()],
        }

    def to_text(self) -> str:
        lines = [
            f"PRUEBA DE HARDWARE  {self.info.get('fecha', '')}",
            "Solo lectura: al ESP32 solo se le enviaron help, posall y scan.",
        ]
        group = None
        for check in self.ordered():
            if check.group != group:
                group = check.group
                lines += ["", group.upper()]
            lines.append(f"  {'[' + check.status + ']':<10}{check.name}: {check.detail}")
        counts = self.counts()
        summary = ", ".join(
            f"{counts[status]} {status}"
            for status in (OK, AVISO, FALLA, REVISAR, OMITIDA)
            if counts[status]
        )
        lines += ["", f"Resumen: {summary}.", "No prueba movimiento, torque ni garra."]
        return "\n".join(lines)


def save_report(report: Report, output_dir: Path) -> None:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "reporte.json").write_text(
        json.dumps(report.to_dict(), ensure_ascii=False, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )
    (output_dir / "reporte.txt").write_text(report.to_text() + "\n", encoding="utf-8")
    (output_dir / "serial.log").write_text(
        "".join(line + "\n" for line in report.serial_log), encoding="utf-8"
    )


def _json_default(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    raise TypeError(f"No se puede guardar {type(value).__name__} en JSON.")


def _worst(statuses: Iterable[str]) -> str:
    return max(statuses, key=lambda status: _SEVERITY[status], default=OK)


def _with_problems(summary: str, problems: list[tuple[str, str]]) -> tuple[str, str]:
    status = _worst(status for status, _ in problems)
    return status, summary + "".join(f"; {text}" for _, text in problems) + "."


def _motors(motors: Iterable[int]) -> str:
    return ", ".join(f"M{motor}" for motor in motors)


def _range(limits: tuple[float, float] | None) -> str:
    return "sin dato" if limits is None else f"{limits[0]:g}..{limits[1]:g}"


# --------------------------------------------------------------------------
# Firmware del repositorio


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    in_string = escaped = False
    for position in range(opening, len(source)):
        char = source[position]
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
        elif char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1 : position]
    raise ValueError(f"No se encontró el final de {signature}.")


_C_STRING = r'"((?:[^"\\]|\\.)*)"'
_HELP_STATEMENT = re.compile(
    r"printDivider\(\)\s*;"
    r"|Serial\.(?P<fn>println|print)\(\s*"
    r'(?:F\(\s*"(?P<text>(?:[^"\\]|\\.)*)"\s*\)|(?P<expr>[^;]*?))\s*\)\s*;'
)
_LIMIT_CASE = re.compile(
    r"case\s+(\d+)\s*:\s*minDeg\s*=\s*(-?[\d.]+)f?\s*;\s*maxDeg\s*=\s*(-?[\d.]+)f?\s*;"
)
_HELP_LIMIT = re.compile(r"^ID (\d+):\s+(-?\d+(?:\.\d+)?)\.\.(-?\d+(?:\.\d+)?)")


def _unescape(text: str) -> str:
    return re.sub(r"\\(.)", r"\1", text)


def _clean_template(parts: Sequence[str | None]) -> tuple[str | None, ...]:
    merged: list[str | None] = []
    for part in parts:
        if part == "":
            continue
        if part is not None and merged and merged[-1] is not None:
            merged[-1] += part
        else:
            merged.append(part)
    if merged and merged[0] is not None:
        merged[0] = merged[0].lstrip()
    if merged and merged[-1] is not None:
        merged[-1] = merged[-1].rstrip()
    cleaned = tuple(part for part in merged if part != "")
    # Una línea formada solo por valores variables no identifica nada.
    return cleaned if any(part is not None for part in cleaned) else ()


def help_templates(source: str) -> list[tuple[str | None, ...]]:
    """Líneas que imprime printHelp(); None marca un valor variable.

    RobotSerial quita los espacios de los extremos de cada línea, así que
    las plantillas también los quitan.
    """
    divider_match = re.search(r"F\(\s*" + _C_STRING, _function_body(source, "void printDivider()"))
    divider = _unescape(divider_match.group(1)) if divider_match else ""
    templates: list[tuple[str | None, ...]] = []
    current: list[str | None] = []
    for match in _HELP_STATEMENT.finditer(_function_body(source, "void printHelp()")):
        if match.group("fn") is None:  # printDivider() hace println de la línea.
            current.append(divider)
            templates.append(tuple(current))
            current = []
            continue
        if match.group("text") is not None:
            current.append(_unescape(match.group("text")))
        elif match.group("expr").strip():
            current.append(None)
        if match.group("fn") == "println":
            templates.append(tuple(current))
            current = []
    if current:
        templates.append(tuple(current))
    return [template for template in map(_clean_template, templates) if template]


def render_template(template: Sequence[str | None], fill: str = "0") -> str:
    return "".join(fill if part is None else part for part in template)


def _template_pattern(template: Sequence[str | None]) -> re.Pattern[str]:
    return re.compile("".join(".*?" if part is None else re.escape(part) for part in template))


def firmware_limits(source: str) -> dict[int, tuple[float, float]]:
    body = _function_body(source, "bool getUserAngleLimits(")
    return {
        int(match.group(1)): (float(match.group(2)), float(match.group(3)))
        for match in _LIMIT_CASE.finditer(body)
    }


def help_limits(templates: Iterable[Sequence[str | None]]) -> dict[int, tuple[float, float]]:
    limits: dict[int, tuple[float, float]] = {}
    for template in templates:
        match = _HELP_LIMIT.match(render_template(template))
        if match:
            limits[int(match.group(1))] = (float(match.group(2)), float(match.group(3)))
    return limits


# --------------------------------------------------------------------------
# Dispositivos


def list_directshow_cameras() -> list[dict[str, Any]]:
    from cv2_enumerate_cameras import enumerate_cameras

    return [
        {"indice": int(camera.index), "nombre": str(camera.name), "vid": camera.vid, "pid": camera.pid}
        for camera in enumerate_cameras(cv2.CAP_DSHOW)
    ]


def list_serial_ports() -> list[dict[str, Any]]:
    return [
        {
            "puerto": port.device,
            "descripcion": port.description or "",
            "fabricante": port.manufacturer or "",
            "vid": port.vid,
            "pid": port.pid,
        }
        for port in list_ports.comports()
    ]


def resolve_port(
    preferred: str, ports: Sequence[dict[str, Any]], override: str | None = None
) -> tuple[str | None, str]:
    """Elige el puerto con el mismo criterio que App.refresh_ports."""
    if override:
        return override, "indicado con --puerto"
    if preferred in [port["puerto"] for port in ports]:
        return preferred, "config.json"
    usb = [
        port["puerto"]
        for port in ports
        if any(
            marker in f"{port.get('descripcion', '')} {port.get('fabricante', '')}".lower()
            for marker in USB_MARKERS
        )
    ]
    if len(usb) == 1:
        return usb[0], "autodetectado"
    return None, "no encontrado"


def detection_parameters(config: AppConfig, camera_key: str) -> dict[str, Any]:
    """Los mismos parámetros que App usa al detectar en cada cámara (app.py)."""
    vision = config.vision
    if camera_key == "frontal":
        return {
            "minimum_red_area_ratio": vision.minimum_red_area_ratio,
            "minimum_lime_area_ratio": vision.frontal_minimum_lime_area_ratio,
            "lime_hsv_lower": vision.frontal_lime_hsv_lower,
            "lime_hsv_upper": vision.frontal_lime_hsv_upper,
            "minimum_lime_shape_ratio": vision.frontal_minimum_lime_shape_ratio,
            "minimum_lime_confidence": vision.frontal_minimum_lime_confidence,
        }
    return {
        "minimum_red_area_ratio": vision.minimum_red_area_ratio,
        "minimum_lime_area_ratio": vision.minimum_lime_area_ratio,
        "lime_hsv_lower": (30, 50, 50),
        "lime_hsv_upper": (90, 255, 255),
        "minimum_lime_shape_ratio": 0.0,
        "minimum_lime_confidence": 0.0,
    }


class DiagnosticRobotSerial(RobotSerial):
    """RobotSerial que solo acepta órdenes de lectura.

    connect() abre el puerto igual que la aplicación, pero durante la espera
    inicial guarda lo recibido para saber si el ESP32 se reinició.
    """

    def __init__(
        self,
        port: str,
        baud: int,
        *,
        serial_factory: Callable[..., Any] | None = None,
        boot_wait: float = 2.0,
    ):
        super().__init__(port, baud)
        self.serial_factory = serial_factory or serial.Serial
        self.boot_wait = boot_wait
        self.boot_lines: list[str] = []

    def connect(self) -> None:
        if self.connected:
            return
        self.stop_event.clear()
        connection = self.serial_factory(self.port, self.baud, timeout=0.1, write_timeout=0.5)
        try:
            connection.dtr = False
            connection.rts = False
            raw = bytearray()
            deadline = time.monotonic() + self.boot_wait
            while time.monotonic() < deadline:
                raw += connection.read(max(1, min(connection.in_waiting, 4096)))
            connection.reset_input_buffer()
        except BaseException:
            connection.close()
            raise
        decoded = (line.decode("utf-8", errors="replace").strip() for line in bytes(raw).split(b"\n"))
        self.boot_lines = [line for line in decoded if line]
        self.connection = connection
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.events.put(RobotEvent(time.monotonic(), "connected", self.port))
        self.send("posall")

    def send(self, command: str) -> None:
        clean = command.strip()
        if clean.lower() not in READ_ONLY_COMMANDS:
            raise ValueError(
                f"La prueba de hardware no envía '{clean}': solo {', '.join(READ_ONLY_COMMANDS)}."
            )
        super().send(clean)


class Transcript:
    """Todo lo enviado y recibido por el puerto serie, en orden."""

    def __init__(self) -> None:
        self.start = time.monotonic()
        self.events: list[RobotEvent] = []

    def drain(self, robot: RobotSerial | None) -> None:
        if robot is None:
            return
        while True:
            try:
                self.events.append(robot.events.get_nowait())
            except queue.Empty:
                return

    def lines(self) -> list[str]:
        return [
            f"{event.timestamp - self.start:9.3f}  {event.kind:<10} {event.payload}"
            for event in self.events
        ]


def _request(
    robot: RobotSerial,
    transcript: Transcript,
    command: str,
    done: Callable[[list[str]], bool],
    timeout: float,
) -> tuple[list[RobotEvent], float]:
    transcript.drain(robot)
    mark = len(transcript.events)
    sent_at = time.monotonic()
    robot.send(command)
    while True:
        transcript.drain(robot)
        received = [event for event in transcript.events[mark:] if event.kind == "rx"]
        if done([event.payload for event in received]) or time.monotonic() - sent_at >= timeout:
            return received, sent_at
        time.sleep(0.02)


def parse_scan(lines: Iterable[str]) -> dict[int, dict[str, Any] | None]:
    """Respuesta de `scan`; None indica `ID n: sin respuesta`."""
    result: dict[int, dict[str, Any] | None] = {}
    for line in lines:
        match = SCAN_LINE.match(line)
        if match:
            result[int(match.group(1))] = {
                "pos": int(match.group(2)),
                "voltaje_v": int(match.group(4)) / 10.0,
                "temperatura_c": int(match.group(5)),
            }
            continue
        match = SCAN_MISSING.match(line)
        if match:
            result[int(match.group(1))] = None
    return result


def _max_window_range(items: Sequence[tuple[float, float]], window_seconds: float) -> float:
    """Mayor variación dentro de cualquier ventana de `window_seconds`."""
    best = 0.0
    first = 0
    for last in range(len(items)):
        while items[last][0] - items[first][0] > window_seconds:
            first += 1
        values = [value for _, value in items[first : last + 1]]
        best = max(best, max(values) - min(values))
    return best


def telemetry_stats(
    events: Iterable[RobotEvent], started: float, ended: float, window_seconds: float
) -> dict[int, dict[str, Any]]:
    """Lecturas automáticas `POS ID` de cada motor durante la medición.

    `rango_ventana_deg` usa la misma ventana que RobotSerial.stable_pose.
    """
    samples: dict[int, list[tuple[float, float]]] = {motor: [] for motor in MOTORS}
    for event in events:
        if event.kind != "rx" or not started <= event.timestamp <= ended:
            continue
        if not event.payload.startswith("POS ID"):
            continue
        for motor, value in parse_positions(event.payload).items():
            if value is not None:
                samples[motor].append((event.timestamp, value))
    duration = max(ended - started, 1e-6)
    stats: dict[int, dict[str, Any]] = {}
    for motor, items in samples.items():
        stamps = [started, *(stamp for stamp, _ in items), ended]
        values = [value for _, value in items]
        stats[motor] = {
            "muestras": len(items),
            "hz": len(items) / duration,
            "max_hueco_s": max(after - before for before, after in zip(stamps, stamps[1:])),
            "rango_deg": max(values) - min(values) if values else None,
            "rango_ventana_deg": _max_window_range(items, window_seconds) if items else None,
        }
    return stats


def capture_conditions(
    config: AppConfig, snapshots: Sequence[FrameSnapshot], robot: RobotSerial, now: float
) -> tuple[list[str], list[str]]:
    """Comprobaciones de App al pulsar Tomar foto, salvo la del torque.

    Devuelve (motivos de rechazo, avisos), con los mismos textos de la app.
    """
    capture = config.capture
    blocking: list[str] = []
    if len(snapshots) != 2 or any(item.frame is None for item in snapshots):
        blocking.append("Falta imagen de una cámara.")
    state = robot.snapshot()
    missing = [motor for motor in MOTORS if motor not in state.positions]
    if missing:
        blocking.append(f"Faltan posiciones de {_motors(missing)}.")
    warnings: list[str] = []
    problem = validate_frame_pair(
        list(snapshots),
        now,
        maximum_age=capture.maximum_camera_age_seconds,
        maximum_skew=capture.maximum_camera_skew_seconds,
    )
    if problem:
        warnings.append(problem)
    if state.fault:
        warnings.append(f"Fallo activo: {state.fault}")
    _pose, stability = robot.stable_pose(
        now,
        maximum_age=capture.maximum_telemetry_age_seconds,
        window_seconds=capture.stable_window_seconds,
        maximum_range=capture.stable_max_range_degrees,
    )
    if stability:
        warnings.append(stability)
    for motor, value in sorted(state.positions.items()):
        low, high = config.limits[motor]
        if not low <= value <= high:
            warnings.append(f"M{motor} fuera de límites.")
    return blocking, warnings


# --------------------------------------------------------------------------
# Ejecución


@dataclass
class Options:
    output_dir: Path
    cameras: bool = True
    esp32: bool = True
    port: str | None = None
    duration: float = 5.0
    open_timeout: float = 15.0
    boot_wait: float = 2.0
    firmware_path: Path | None = None


@dataclass
class CameraRun:
    key: str
    index: int
    rotation: int
    stream: Any = None
    first_frame_s: float | None = None


@dataclass
class Window:
    started: float
    ended: float = 0.0
    first: dict[str, FrameSnapshot] = field(default_factory=dict)
    last: dict[str, FrameSnapshot] = field(default_factory=dict)
    max_age: dict[str, float] = field(default_factory=dict)
    pair_total: int = 0
    pair_problems: Counter = field(default_factory=Counter)
    skews: list[float] = field(default_factory=list)
    capture_total: int = 0
    capture_blocked: int = 0
    capture_warned: int = 0
    capture_blocking: Counter = field(default_factory=Counter)
    capture_warnings: Counter = field(default_factory=Counter)


def run_diagnostic(
    config: AppConfig,
    options: Options,
    *,
    stream_factory: Callable[..., Any] = CameraStream,
    index_resolver: Callable[[str, int], int] = discover_camera_index,
    camera_enumerator: Callable[[], list[dict[str, Any]]] = list_directshow_cameras,
    port_lister: Callable[[], list[dict[str, Any]]] = list_serial_ports,
    serial_factory: Callable[..., Any] | None = None,
    progress: Callable[[str], None] = print,
) -> Report:
    report = Report()
    output = Path(options.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    report.info.update(
        fecha=datetime.now().isoformat(timespec="seconds"),
        carpeta=str(output),
        camaras=options.cameras,
        esp32=options.esp32,
        segundos=options.duration,
    )
    firmware_path = Path(options.firmware_path or config.root / "firmware" / "firmware_secuencias.ino")
    source = firmware_path.read_text(encoding="utf-8") if firmware_path.is_file() else None

    _guarded(report, "entorno", "software", lambda: _check_environment(report, config))
    _guarded(
        report, "firmware", "límites del repositorio",
        lambda: _check_firmware_source(report, config, source),
    )
    transcript = Transcript()
    runs: list[CameraRun] = []
    robots: list[DiagnosticRobotSerial] = []
    robot: DiagnosticRobotSerial | None = None
    try:
        if options.cameras:
            progress("Abriendo las cámaras (puede tardar unos segundos)...")
            _guarded(
                report, "cámaras", "apertura",
                lambda: _open_cameras(
                    report, config, options, runs,
                    stream_factory=stream_factory,
                    index_resolver=index_resolver,
                    camera_enumerator=camera_enumerator,
                ),
            )
        if options.esp32:
            progress("Conectando al ESP32...")
            robot = _guarded(
                report, "esp32", "conexión",
                lambda: _connect_esp32(
                    report, config, options, robots, transcript,
                    port_lister=port_lister, serial_factory=serial_factory,
                ),
            )
            if robot is not None:
                progress("Consultando help, posall y scan...")
                _guarded(
                    report, "firmware", "versión cargada (help)",
                    lambda: _check_help(report, robot, transcript, source),
                )
                _guarded(
                    report, "esp32", "posición de los seis servos (posall)",
                    lambda: _check_posall(report, robot, transcript),
                )
                _guarded(
                    report, "esp32", "voltaje (scan)",
                    lambda: _check_scan(report, robot, transcript),
                )
        active = [run for run in runs if run.stream is not None]
        if active or robot is not None:
            progress(f"Midiendo durante {options.duration:g} s: no toques el brazo...")
            window = _guarded(
                report, "medición", "muestreo",
                lambda: _sample_window(config, active, robot, transcript, options.duration),
            )
            if window is not None:
                if active:
                    _guarded(
                        report, "cámaras", "imágenes",
                        lambda: _report_cameras(report, config, active, window, output),
                    )
                if robot is not None:
                    _guarded(
                        report, "esp32", "telemetría",
                        lambda: _report_telemetry(report, config, robot, transcript, window),
                    )
                if len(active) == 2 and robot is not None:
                    _guarded(
                        report, "integración", "condiciones de Tomar foto",
                        lambda: _report_integration(report, window),
                    )
        if options.cameras and options.esp32 and not report.find(
            "integración", "condiciones de Tomar foto"
        ):
            report.add(
                "integración", "condiciones de Tomar foto", OMITIDA,
                "Necesita las dos cámaras y el ESP32 funcionando a la vez.",
            )
    finally:
        _close(runs, robots, transcript)
        report.serial_log = transcript.lines()
    return report


def _guarded(report: Report, group: str, name: str, action: Callable[[], Any]) -> Any:
    try:
        return action()
    except Exception as exc:  # El reporte tiene que generarse igual.
        report.add(group, name, FALLA, f"Error inesperado: {type(exc).__name__}: {exc}")
        return None


def _close(
    runs: Sequence[CameraRun], robots: Sequence[DiagnosticRobotSerial], transcript: Transcript
) -> None:
    for run in runs:
        if run.stream is not None:
            run.stream.close()
    for run in runs:
        thread = getattr(run.stream, "thread", None)
        if thread is not None:
            thread.join(timeout=3.0)
    for robot in robots:
        robot.disconnect()
        transcript.drain(robot)


def _check_environment(report: Report, config: AppConfig) -> None:
    versions = (
        f"Python {platform.python_version()}, {platform.platform()}, OpenCV {cv2.__version__}, "
        f"numpy {np.__version__}, pyserial {serial.__version__}"
    )
    report.info["entorno"] = versions
    if platform.system() == "Windows":
        report.add("entorno", "software", OK, versions + ".")
    else:
        report.add(
            "entorno", "software", AVISO,
            versions + ". Las cámaras se abren con DirectShow, que solo existe en Windows.",
        )
    cameras = "; ".join(
        f"{key} '{camera.name}' (índice de respaldo {camera.fallback_index}, "
        f"rotación {camera.rotation_degrees}°, brillo {camera.brightness_percent:+g}%, "
        f"contraste {camera.contrast_percent:+g}%, saturación {camera.saturation_percent:+g}%, "
        f"zoom {camera.zoom_factor:g}x)"
        for key, camera in config.cameras.items()
    )
    report.add("entorno", "config.json", OK, f"ESP32 en {config.port} a {config.baud} baud; {cameras}.")


def _check_firmware_source(report: Report, config: AppConfig, source: str | None) -> None:
    group, name = "firmware", "límites del repositorio"
    if source is None:
        report.add(group, name, OMITIDA, "No se encontró firmware/firmware_secuencias.ino.")
        return
    in_code = firmware_limits(source)
    in_help = help_limits(help_templates(source))
    differences = []
    for motor in MOTORS:
        expected = config.limits.get(motor)
        if in_code.get(motor) != expected:
            differences.append(
                f"M{motor}: config.json {_range(expected)}, getUserAngleLimits {_range(in_code.get(motor))}"
            )
        if in_help.get(motor) != expected:
            differences.append(
                f"M{motor}: config.json {_range(expected)}, help {_range(in_help.get(motor))}"
            )
    if differences:
        report.add(group, name, AVISO, "Límites distintos: " + "; ".join(differences) + ".")
    else:
        report.add(
            group, name, OK,
            "config.json, getUserAngleLimits() y el texto de help del firmware tienen los mismos "
            "límites de M1–M6.",
        )


def _describe_camera(camera: dict[str, Any]) -> str:
    text = f"{camera['indice']}: {camera['nombre']}"
    if camera.get("vid") is not None and camera.get("pid") is not None:
        text += f" ({camera['vid']:04X}:{camera['pid']:04X})"
    return text


def _open_cameras(
    report: Report,
    config: AppConfig,
    options: Options,
    runs: list[CameraRun],
    *,
    stream_factory: Callable[..., Any],
    index_resolver: Callable[[str, int], int],
    camera_enumerator: Callable[[], list[dict[str, Any]]],
) -> None:
    group = "cámaras"
    try:
        cameras: list[dict[str, Any]] | None = list(camera_enumerator())
        error = ""
    except (ImportError, OSError, RuntimeError) as exc:
        cameras, error = None, f"{type(exc).__name__}: {exc}"
    report.info["camaras_detectadas"] = cameras
    if cameras is None:
        report.add(
            group, "lista de cámaras", AVISO,
            f"No se pudo listar las cámaras DirectShow ({error}); la app usa los índices de "
            "respaldo de config.json.",
        )
    elif not cameras:
        report.add(
            group, "lista de cámaras", FALLA,
            "Windows no informa ninguna cámara DirectShow. Revisá los cables USB.",
        )
    else:
        listed = "; ".join(_describe_camera(camera) for camera in cameras)
        report.add(group, "lista de cámaras", OK, f"{len(cameras)} cámara(s): {listed}.")

    resolved = []
    for key in ("frontal", "lateral"):
        camera = config.cameras[key]
        index = int(index_resolver(camera.name, camera.fallback_index))
        matches = [item for item in cameras or () if item["nombre"] == camera.name]
        if len(matches) == 1:
            status, detail = OK, f"'{camera.name}' encontrada por nombre en el índice {index}."
        elif matches:
            status, detail = AVISO, (
                f"Hay {len(matches)} cámaras llamadas '{camera.name}'; la app usa el índice de "
                f"respaldo {index}. Verificá en la foto que sea la correcta."
            )
        elif cameras is None:
            status, detail = AVISO, (
                f"Sin lista de cámaras; la app usa el índice de respaldo {index}. Verificá en la "
                "foto que sea la correcta."
            )
        else:
            status, detail = AVISO, (
                f"'{camera.name}' no aparece en la lista; la app usa el índice de respaldo "
                f"{index}. Si la cámara cambió de nombre, corregilo en config.json."
            )
        report.add(group, f"{key} · identificación", status, detail, indice=index)
        resolved.append((key, camera, index))

    if resolved[0][2] == resolved[1][2]:
        report.add(
            group, "índices", FALLA,
            f"Frontal y lateral resolvieron al mismo índice ({resolved[0][2]}); la app no las "
            "abre así. Revisá nombres e índices de respaldo en config.json.",
        )
        return

    for key, camera, index in resolved:
        run = CameraRun(key, index, camera.rotation_degrees)
        runs.append(run)
        run.stream = stream_factory(
            index,
            rotation_degrees=camera.rotation_degrees,
            contrast_percent=camera.contrast_percent,
            brightness_percent=camera.brightness_percent,
            saturation_percent=camera.saturation_percent,
            zoom_factor=camera.zoom_factor,
        )

    started = time.monotonic()
    while time.monotonic() - started < options.open_timeout:
        for run in runs:
            if run.first_frame_s is None and run.stream.snapshot().frame is not None:
                run.first_frame_s = time.monotonic() - started
        if all(run.first_frame_s is not None for run in runs):
            break
        time.sleep(TICK_S)

    for run in runs:
        if run.first_frame_s is None:
            report.add(
                group, f"{run.key} · apertura", FALLA,
                f"Sin imagen después de {options.open_timeout:g} s (estado: "
                f"{run.stream.snapshot().status}). Cerrá la app principal u otro programa que use "
                "la cámara y revisá el cable USB.",
            )
        else:
            report.add(
                group, f"{run.key} · apertura", OK,
                f"Primera imagen en {run.first_frame_s:.1f} s.",
            )


def _connect_esp32(
    report: Report,
    config: AppConfig,
    options: Options,
    robots: list[DiagnosticRobotSerial],
    transcript: Transcript,
    *,
    port_lister: Callable[[], list[dict[str, Any]]],
    serial_factory: Callable[..., Any] | None,
) -> DiagnosticRobotSerial | None:
    group = "esp32"
    ports = list(port_lister())
    report.info["puertos"] = ports
    port, origin = resolve_port(config.port, ports, options.port)
    listed = ", ".join(f"{item['puerto']} ({item['descripcion']})" for item in ports) or "ninguno"
    if port is None:
        report.add(
            group, "puerto", FALLA,
            f"No está {config.port} y no hay un único adaptador USB serie. Puertos: {listed}. "
            "Revisá el cable USB y el driver (CH340/CP210x) o indicá --puerto.",
        )
        return None
    if origin == "autodetectado":
        report.add(
            group, "puerto", AVISO,
            f"{config.port} no existe; se usa {port}, como hace la app con un único adaptador "
            f"USB. Conviene corregir config.json. Puertos: {listed}.",
        )
    else:
        report.add(group, "puerto", OK, f"{port} ({origin}). Puertos: {listed}.")

    robot = DiagnosticRobotSerial(
        port, config.baud, serial_factory=serial_factory, boot_wait=options.boot_wait
    )
    robots.append(robot)
    opened_at = time.monotonic()
    try:
        robot.connect()
    except (serial.SerialException, OSError) as exc:
        text = str(exc)
        busy = any(marker in text.lower() for marker in ("denied", "denegado", "permission", "busy", "ocupado"))
        hint = (
            "El puerto está en uso: cerrá la app principal, el monitor serie de Arduino u otro programa."
            if busy
            else "Revisá el cable USB y que el ESP32 esté encendido."
        )
        report.add(group, "conexión", FALLA, f"No se pudo abrir {port}: {text}. {hint}")
        return None

    boot = robot.boot_lines
    transcript.events.extend(RobotEvent(opened_at, "arranque", line) for line in boot)
    if any(marker in line for line in boot for marker in BOOT_MARKERS):
        status, detail = OK, (
            "El ESP32 se reinició al abrir el puerto (normal en placas con auto-reset). setup() no "
            "mueve servos ni cambia torque, pero reconectar reinicia el estado del firmware, por "
            "ejemplo el de la garra."
        )
    elif any(line.startswith("POS ID") for line in boot):
        status, detail = OK, "El firmware ya estaba enviando posiciones; abrir el puerto no lo reinició."
    elif boot:
        status, detail = AVISO, (
            f"Llegaron {len(boot)} líneas que no se reconocen (p. ej. '{boot[0][:60]}'). Revisá la "
            f"velocidad ({config.baud}) y el firmware cargado."
        )
    else:
        status, detail = AVISO, (
            f"No llegó nada en {options.boot_wait:g} s después de abrir el puerto. ¿Está cargado "
            "el firmware?"
        )
    report.add(group, "conexión", status, f"{port} abierto a {config.baud} baud. {detail}")
    return robot


def _check_help(
    report: Report, robot: RobotSerial, transcript: Transcript, source: str | None
) -> None:
    group, name = "firmware", "versión cargada (help)"
    templates = help_templates(source) if source else []
    patterns = [_template_pattern(template) for template in templates]

    def candidates(lines: list[str]) -> list[str]:
        return [line for line in lines if not parse_positions(line)]

    def complete(lines: list[str]) -> bool:
        found = candidates(lines)
        return bool(patterns) and all(
            any(pattern.fullmatch(line) for line in found) for pattern in patterns
        )

    received, _ = _request(robot, transcript, "help", complete, HELP_TIMEOUT_S)
    lines = candidates([event.payload for event in received])
    if not lines:
        report.add(group, name, AVISO, "Sin respuesta a help: el ESP32 puede tener otro firmware.")
        return
    if not templates:
        report.add(group, name, OMITIDA, "No se encontró firmware/firmware_secuencias.ino para comparar.")
        return
    missing = [
        render_template(template, "<valor>")
        for template, pattern in zip(templates, patterns)
        if not any(pattern.fullmatch(line) for line in lines)
    ]
    extra = [line for line in lines if not any(pattern.fullmatch(line) for pattern in patterns)]
    if missing or extra:
        parts = []
        if missing:
            parts.append(f"faltan {len(missing)} línea(s), p. ej. '{missing[0]}'")
        if extra:
            parts.append(f"sobran {len(extra)}, p. ej. '{extra[0]}'")
        report.add(
            group, name, AVISO,
            "El texto de help no coincide con firmware/firmware_secuencias.ino ("
            + "; ".join(parts)
            + "). Probablemente el ESP32 tiene otra versión: volvé a cargar ese firmware.",
            faltan=missing,
            sobran=extra,
        )
    else:
        report.add(
            group, name, OK,
            f"El texto de help coincide con firmware/firmware_secuencias.ino ({len(templates)} "
            "líneas). Compara solo ese texto, no toda la lógica.",
        )


def _check_posall(report: Report, robot: RobotSerial, transcript: Transcript) -> None:
    group, name = "esp32", "posición de los seis servos (posall)"

    def is_answer(line: str) -> bool:
        return line.startswith("M1=") and "M6=" in line

    received, sent_at = _request(
        robot, transcript, "posall", lambda lines: any(map(is_answer, lines)), POSALL_TIMEOUT_S
    )
    answer = next((event for event in received if is_answer(event.payload)), None)
    if answer is None:
        report.add(
            group, name, FALLA,
            "Sin respuesta a posall. Revisá que el ESP32 tenga firmware_secuencias.ino y use "
            "115200 baud.",
        )
        return
    values = parse_positions(answer.payload)
    missing = [motor for motor in MOTORS if values.get(motor) is None]
    angles = ", ".join(
        f"M{motor} {values[motor]:.1f}°" for motor in MOTORS if values.get(motor) is not None
    )
    latency_ms = (answer.timestamp - sent_at) * 1000.0
    if missing:
        report.add(
            group, name, FALLA,
            f"{_motors(missing)} sin respuesta (NA): revisá el cable del bus, el ID y la "
            f"alimentación de ese servo. Leídos: {angles or 'ninguno'}.",
            grados=values,
        )
    else:
        report.add(
            group, name, OK,
            f"Responden los seis en {latency_ms:.0f} ms: {angles}.",
            grados=values,
            latencia_ms=round(latency_ms, 1),
        )


def _check_scan(report: Report, robot: RobotSerial, transcript: Transcript) -> None:
    group = "esp32"
    received, _ = _request(
        robot, transcript, "scan",
        lambda lines: set(parse_scan(lines)) >= set(MOTORS),
        SCAN_TIMEOUT_S,
    )
    scan = parse_scan(event.payload for event in received)
    if not scan:
        report.add(group, "voltaje (scan)", FALLA, "Sin respuesta a scan.")
        report.add(group, "temperatura (scan)", FALLA, "Sin respuesta a scan.")
        return
    absent = [motor for motor in MOTORS if scan.get(motor) is None]
    read = {motor: data for motor, data in sorted(scan.items()) if data is not None}

    volts = {motor: data["voltaje_v"] for motor, data in read.items() if data["voltaje_v"] > 0}
    invalid = [motor for motor, data in read.items() if data["voltaje_v"] <= 0]
    problems: list[tuple[str, str]] = []
    if absent:
        problems.append((FALLA, f"no respondieron {_motors(absent)}"))
    if invalid:
        problems.append((AVISO, f"lectura inválida en {_motors(invalid)}"))
    if volts:
        low, high = min(volts.values()), max(volts.values())
        if low < MINIMUM_VOLTAGE_V:
            problems.append((AVISO, f"tensión baja ({low:.1f} V)"))
        if high - low > MAXIMUM_VOLTAGE_SPREAD_V:
            problems.append((
                AVISO,
                f"{high - low:.1f} V de diferencia entre servos (revisá cables y conectores del bus)",
            ))
    listed = ", ".join(f"M{motor} {value:.1f} V" for motor, value in volts.items()) or "sin lecturas"
    status, detail = _with_problems(listed, problems)
    report.add(group, "voltaje (scan)", status, detail, voltaje_v=volts)

    temperatures = {
        motor: data["temperatura_c"] for motor, data in read.items() if data["temperatura_c"] >= 0
    }
    hot = [motor for motor, value in temperatures.items() if value >= HOT_SERVO_C]
    listed = ", ".join(f"M{motor} {value} °C" for motor, value in temperatures.items())
    if not temperatures:
        report.add(group, "temperatura (scan)", AVISO, "Sin lecturas de temperatura.")
    elif hot:
        report.add(
            group, "temperatura (scan)", AVISO,
            f"{listed}; {_motors(hot)} a {HOT_SERVO_C} °C o más: dejalos enfriar antes de usar el brazo.",
            temperatura_c=temperatures,
        )
    else:
        report.add(group, "temperatura (scan)", OK, f"{listed}.", temperatura_c=temperatures)


def _sample_window(
    config: AppConfig,
    runs: Sequence[CameraRun],
    robot: RobotSerial | None,
    transcript: Transcript,
    duration: float,
) -> Window:
    capture = config.capture
    window = Window(started=time.monotonic())
    window.first = {run.key: run.stream.snapshot() for run in runs}
    while time.monotonic() - window.started < duration:
        snapshots = [run.stream.snapshot() for run in runs]
        now = time.monotonic()
        for run, snapshot in zip(runs, snapshots):
            if snapshot.frame is not None:
                age = now - snapshot.timestamp
                window.max_age[run.key] = max(window.max_age.get(run.key, 0.0), age)
        if len(snapshots) == 2:
            window.pair_total += 1
            problem = validate_frame_pair(
                snapshots,
                now,
                maximum_age=capture.maximum_camera_age_seconds,
                maximum_skew=capture.maximum_camera_skew_seconds,
            )
            if problem:
                window.pair_problems[problem] += 1
            if all(snapshot.frame is not None for snapshot in snapshots):
                window.skews.append(abs(snapshots[0].timestamp - snapshots[1].timestamp))
        if robot is not None:
            transcript.drain(robot)
            if len(snapshots) == 2:
                blocking, warnings = capture_conditions(config, snapshots, robot, now)
                window.capture_total += 1
                if blocking:
                    window.capture_blocked += 1
                    window.capture_blocking.update(blocking)
                if warnings:
                    window.capture_warned += 1
                    window.capture_warnings.update(warnings)
        time.sleep(TICK_S)
    window.last = {run.key: run.stream.snapshot() for run in runs}
    window.ended = time.monotonic()
    transcript.drain(robot)
    return window


def _write_png(path: Path, frame: np.ndarray) -> None:
    ok, encoded = cv2.imencode(".png", frame)
    if not ok:
        raise RuntimeError(f"No se pudo codificar {path.name}.")
    path.write_bytes(encoded.tobytes())


def _report_cameras(
    report: Report, config: AppConfig, runs: Sequence[CameraRun], window: Window, output: Path
) -> None:
    group = "cámaras"
    duration = max(window.ended - window.started, 1e-6)
    skew_limit_ms = config.capture.maximum_camera_skew_seconds * 1000.0
    for run in runs:
        first, last = window.first[run.key], window.last[run.key]
        if run.first_frame_s is None or last.frame is None:
            continue  # Ya informado en la apertura.
        frames = last.sequence - first.sequence
        fps = frames / duration
        frame = last.frame
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean, spread = float(gray.mean()), float(gray.std())
        frozen = bool(
            frames > 0
            and first.frame is not None
            and first.frame.shape == frame.shape
            and np.array_equal(first.frame, frame)
        )
        max_age = window.max_age.get(run.key, 0.0)
        problems: list[tuple[str, str]] = []
        if frames <= 0:
            problems.append((FALLA, "la imagen no se actualizó durante la medición"))
        if mean < 10:
            problems.append((FALLA, "imagen negra: tapa puesta, sin luz o cámara equivocada"))
        elif mean < 40:
            problems.append((AVISO, "imagen oscura; puede fallar la detección"))
        elif mean > 235:
            problems.append((AVISO, "imagen sobreexpuesta"))
        if mean >= 10 and spread < 4:
            problems.append((AVISO, "imagen casi uniforme (¿lente tapado?)"))
        if frozen:
            problems.append((AVISO, "el primer y el último cuadro son idénticos (¿imagen congelada?)"))
        if 0 < fps < 10:
            problems.append((
                AVISO,
                f"pocos cuadros por segundo; el desfase entre cámaras puede superar {skew_limit_ms:.0f} ms",
            ))
        if max_age > config.capture.maximum_camera_age_seconds:
            problems.append((AVISO, f"cortes de imagen de hasta {max_age:.1f} s"))
        height, width = frame.shape[:2]
        status, detail = _with_problems(
            f"{width}x{height}, {fps:.1f} cuadros/s, brillo medio {mean:.0f}/255", problems
        )
        report.add(
            group, f"{run.key} · imagen", status, detail,
            ancho=width, alto=height, cuadros_por_segundo=round(fps, 1),
            brillo_medio=round(mean, 1), desvio=round(spread, 1),
            max_antiguedad_s=round(max_age, 3),
        )

    if len(runs) == 2:
        _report_pair(report, config, window)

    for run in runs:
        last = window.last[run.key]
        if last.frame is None:
            continue
        image = output / f"{run.key}.png"
        _write_png(image, last.frame)
        observation = detect_scene(last.frame, **detection_parameters(config, run.key))
        _write_png(output / f"{run.key}_deteccion.png", draw_observation(last.frame, observation))
        found, missing = [], []
        for label, detection in (
            ("cubo rojo", observation.red_cube),
            ("recipiente verde lima", observation.lime_container),
        ):
            if detection is None:
                missing.append(label)
            else:
                found.append(f"{label} (confianza {detection.confidence:.2f})")
        parts = []
        if found:
            parts.append("Detectado: " + ", ".join(found) + ".")
        if missing:
            parts.append(
                "No se detectó: " + ", ".join(missing) + ". Si estaba a la vista, revisá la "
                "iluminación y los ajustes de imagen."
            )
        report.add(
            group, f"{run.key} · detección", AVISO if missing else OK, " ".join(parts),
            observacion=observation.to_dict(),
        )
        report.add(
            group, f"{run.key} · revisión visual", REVISAR,
            f"Abrí {image.name}: tiene que ser la cámara {run.key}, no estar invertida (rotación "
            f"{run.rotation}°) y mostrar el cubo y el recipiente.",
        )


def _report_pair(report: Report, config: AppConfig, window: Window) -> None:
    group, name = "cámaras", "par de imágenes"
    if not window.skews:
        report.add(
            group, name, FALLA,
            "Nunca hubo imagen de las dos cámaras a la vez; Tomar foto se rechazaría.",
        )
        return
    limit_ms = config.capture.maximum_camera_skew_seconds * 1000.0
    valid = window.pair_total - sum(window.pair_problems.values())
    ratio = valid / window.pair_total
    skew = (
        f"desfase mediano {statistics.median(window.skews) * 1000:.0f} ms, máximo "
        f"{max(window.skews) * 1000:.0f} ms (límite {limit_ms:.0f} ms)"
    )
    if ratio >= 0.95:
        report.add(group, name, OK, f"El {ratio:.0%} de las muestras cumple antigüedad y desfase; {skew}.")
    else:
        reason = window.pair_problems.most_common(1)[0][0]
        report.add(
            group, name, AVISO,
            f"Solo el {ratio:.0%} de las muestras cumple; motivo más frecuente: '{reason}' "
            f"({skew}). Tomar foto lo registrará como aviso.",
        )


def _report_telemetry(
    report: Report, config: AppConfig, robot: RobotSerial, transcript: Transcript, window: Window
) -> None:
    group = "esp32"
    capture = config.capture
    duration = window.ended - window.started
    stats = telemetry_stats(
        transcript.events, window.started, window.ended, capture.stable_window_seconds
    )

    # stable_pose necesita al menos tres lecturas dentro de su ventana.
    minimum_hz = 3.0 / capture.stable_window_seconds
    silent = [motor for motor in MOTORS if stats[motor]["muestras"] == 0]
    heard = [motor for motor in MOTORS if motor not in silent]
    slow = [motor for motor in heard if stats[motor]["hz"] < minimum_hz]
    weak = [motor for motor in heard if minimum_hz <= stats[motor]["hz"] < 2 * minimum_hz]
    gaps = [
        motor for motor in heard
        if stats[motor]["max_hueco_s"] > capture.maximum_telemetry_age_seconds
    ]
    problems: list[tuple[str, str]] = []
    if silent:
        problems.append((
            FALLA,
            f"sin lecturas de {_motors(silent)}: el servo no responde (cable del bus, ID o alimentación)",
        ))
    if slow:
        problems.append((
            FALLA,
            f"{_motors(slow)} por debajo de {minimum_hz:.0f} Hz; Tomar foto no podrá confirmar estabilidad",
        ))
    if weak:
        problems.append((AVISO, f"{_motors(weak)} por debajo de {2 * minimum_hz:.0f} Hz"))
    if gaps:
        problems.append((
            AVISO,
            f"huecos de más de {capture.maximum_telemetry_age_seconds:g} s en {_motors(gaps)}",
        ))
    rates = [f"{stats[motor]['hz']:.1f}" for motor in heard]
    if not rates:
        summary = "Sin lecturas automáticas"
    else:
        low, high = min(rates, key=float), max(rates, key=float)
        span = low if low == high else f"{low} a {high}"
        summary = f"{span} Hz por motor (el firmware envía unos 20 Hz)"
    status, detail = _with_problems(summary, problems)
    report.add(
        group, "telemetría automática", status, detail,
        hz={motor: round(stats[motor]["hz"], 1) for motor in MOTORS},
    )

    limit = capture.stable_max_range_degrees
    window_s = capture.stable_window_seconds
    _pose, problem = robot.stable_pose(
        time.monotonic(),
        maximum_age=capture.maximum_telemetry_age_seconds,
        window_seconds=window_s,
        maximum_range=limit,
    )
    ranges = {motor: round(stats[motor]["rango_ventana_deg"], 3) for motor in heard}
    totals = {motor: round(stats[motor]["rango_deg"], 3) for motor in heard}
    listed = ", ".join(f"M{motor} {value:.2f}°" for motor, value in ranges.items())
    measured = f"Mayor variación en {window_s:g} s: {listed} (límite {limit:g}°)."
    noisy = [motor for motor, value in ranges.items() if value > limit]
    if not ranges:
        report.add(group, "estabilidad en reposo", AVISO, "Sin lecturas para evaluarla.")
    elif silent:
        report.add(
            group, "estabilidad en reposo", AVISO,
            f"No se puede confirmar con {_motors(silent)} sin lecturas. {measured}",
            rango_ventana_deg=ranges, rango_total_deg=totals,
        )
    elif noisy:
        report.add(
            group, "estabilidad en reposo", AVISO,
            f"{measured} {_motors(noisy)} supera el límite: si nadie tocó el brazo, es ruido de "
            "lectura y Tomar foto avisará que se está moviendo.",
            rango_ventana_deg=ranges, rango_total_deg=totals,
        )
    elif problem:
        report.add(
            group, "estabilidad en reposo", AVISO, f"{problem} {measured}",
            rango_ventana_deg=ranges, rango_total_deg=totals,
        )
    else:
        report.add(
            group, "estabilidad en reposo", OK,
            f"Mayor variación en {window_s:g} s: {max(ranges.values()):.2f}° (límite {limit:g}°); "
            f"en los {duration:.0f} s, {max(totals.values()):.2f}°.",
            rango_ventana_deg=ranges, rango_total_deg=totals,
        )

    state = robot.snapshot()
    outside = []
    for motor, value in sorted(state.positions.items()):
        low, high = config.limits[motor]
        if not low <= value <= high:
            outside.append(f"M{motor} {value:.1f}° (límites {low:g}..{high:g})")
    angles = ", ".join(f"M{motor} {value:.1f}°" for motor, value in sorted(state.positions.items()))
    if not state.positions:
        report.add(group, "ángulos dentro de límites", AVISO, "Sin ángulos para comparar.")
    elif outside:
        report.add(
            group, "ángulos dentro de límites", AVISO,
            "Fuera de los límites de config.json: " + ", ".join(outside)
            + ". Tomar foto lo registrará como aviso.",
        )
    else:
        report.add(group, "ángulos dentro de límites", OK, f"{angles}.")

    received = [event for event in transcript.events if event.kind == "rx"]
    garbled = [event for event in received if "�" in event.payload]
    errors = [event.payload for event in transcript.events if event.kind == "error"]
    if errors:
        report.add(group, "línea serie", FALLA, f"Se cortó la conexión serie: {errors[0]}.")
    elif garbled:
        report.add(
            group, "línea serie", AVISO,
            f"{len(garbled)} de {len(received)} líneas con bytes inválidos (ruido en el cable USB "
            f"o velocidad distinta de {config.baud}).",
        )
    else:
        report.add(group, "línea serie", OK, f"{len(received)} líneas recibidas sin bytes inválidos.")

    faults = sorted({event.payload for event in received if is_fault_line(event.payload)})
    if faults:
        report.add(
            group, "mensajes de falla", AVISO,
            f"{len(faults)} distinto(s): " + " | ".join(faults[:5]),
        )
    else:
        report.add(group, "mensajes de falla", OK, "Ninguno.")


def _report_integration(report: Report, window: Window) -> None:
    group, name = "integración", "condiciones de Tomar foto"
    total = window.capture_total
    if not total:
        report.add(group, name, OMITIDA, "No hubo muestras.")
        return
    blocked = window.capture_blocked / total
    warned = window.capture_warned / total
    note = "El aviso de torque no se evalúa porque esta prueba no lo cambia."
    if blocked:
        reason = window.capture_blocking.most_common(1)[0][0]
        report.add(
            group, name, AVISO if blocked <= 0.1 else FALLA,
            f"Tomar foto se rechazaría en el {blocked:.0%} de las muestras: '{reason}'",
        )
    elif warned <= 0.1:
        report.add(
            group, name, OK,
            f"Tomar foto no se rechazaría y el {1 - warned:.0%} de las muestras no tendría avisos. {note}",
        )
    else:
        reason = window.capture_warnings.most_common(1)[0][0]
        report.add(
            group, name, AVISO,
            f"Tomar foto no se rechazaría, pero el {warned:.0%} de las muestras tendría avisos; el "
            f"más frecuente: '{reason}' {note}",
        )
