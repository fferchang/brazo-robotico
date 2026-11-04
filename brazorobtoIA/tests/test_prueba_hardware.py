"""Prueba de hardware (brazoia.diagnostico) con un ESP32 y cámaras simulados.

El ESP32 simulado responde como firmware_secuencias.ino: telemetría a 20 Hz,
help, posall y scan. Que estas pruebas pasen no certifica el hardware real.
"""
from __future__ import annotations

import json
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path

import cv2
import numpy as np

from brazoia.capture import FrameSnapshot
from brazoia.config import load_config
from brazoia.diagnostico import (
    AVISO,
    FALLA,
    OK,
    READ_ONLY_COMMANDS,
    REVISAR,
    DiagnosticRobotSerial,
    Options,
    firmware_limits,
    help_limits,
    help_templates,
    render_template,
    resolve_port,
    run_diagnostic,
    save_report,
)


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = (ROOT / "firmware" / "firmware_secuencias.ino").read_text(encoding="utf-8")
FIRMWARE_HELP = [render_template(template, "50") for template in help_templates(FIRMWARE)]
DIVIDER = FIRMWARE_HELP[0]
CONFIG = load_config()
POSE = {1: 180.0, 2: 180.0, 3: 120.0, 4: 180.0, 5: 263.0, 6: 2.0}
CAMERAS = [
    {"indice": 1, "nombre": "GENERAL WEBCAM", "vid": 0x1908, "pid": 0x2311},
    {"indice": 2, "nombre": "USB CAMERA", "vid": 0x0C45, "pid": 0x6366},
]
PORTS = [
    {"puerto": "COM4", "descripcion": "USB-SERIAL CH340 (COM4)", "fabricante": "wch.cn",
     "vid": 0x1A86, "pid": 0x7523},
]


def raw_position(motor: int) -> int:
    real = POSE[motor] + (10.0 if motor == 6 else 0.0)
    return round(real * 4095 / 360)


def raw_degrees(motor: int) -> float:
    return raw_position(motor) * 360 / 4095


class FakeESP32Serial:
    """Puerto serie con el comportamiento de firmware_secuencias.ino."""

    def __init__(self, port, baud, timeout=0.1, write_timeout=0.5, *, missing=(),
                 help_lines=None, reset_on_open=True, voltage=74, temperature=31):
        self.port, self.baudrate, self.timeout = port, baud, timeout
        self.is_open = True
        self.missing = set(missing)
        self.help_lines = FIRMWARE_HELP if help_lines is None else help_lines
        self.reset_on_open = reset_on_open
        self.voltage, self.temperature = voltage, temperature
        self.written: list[str] = []
        self._buffer = bytearray()
        self._lock = threading.RLock()
        self._dtr = self._rts = True
        self._next = time.monotonic()

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        # Auto-reset habitual: DTR inactivo con RTS activo baja EN y reinicia.
        if self._dtr and not value and self._rts and self.reset_on_open:
            self._emit(
                "ets Jun  8 2016 00:22:57",
                "rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)",
                DIVIDER,
                "Control sincronizado de hasta dos motores - TEST REGLAS",
                *self.help_lines,
            )
        self._dtr = value

    @property
    def rts(self):
        return self._rts

    @rts.setter
    def rts(self, value):
        self._rts = value

    @property
    def in_waiting(self):
        with self._lock:
            self._pump()
            return len(self._buffer)

    def read(self, size=1):
        deadline = time.monotonic() + self.timeout
        while True:
            with self._lock:
                self._pump()
                if self._buffer:
                    data = bytes(self._buffer[:size])
                    del self._buffer[:size]
                    return data
            if time.monotonic() >= deadline:
                return b""
            time.sleep(0.005)

    def write(self, data):
        for command in data.decode("ascii").splitlines():
            self.written.append(command)
            if command == "help":
                self._emit(*self.help_lines)
            elif command == "posall":
                self._emit(" | ".join(
                    f"M{m}=NA" if m in self.missing
                    else f"M{m}={raw_position(m)}({raw_degrees(m):.1f}deg)"
                    for m in range(1, 7)
                ), DIVIDER)
            elif command == "scan":
                self._emit(*(
                    f"ID {m}: sin respuesta" if m in self.missing
                    else f"ID {m}: pos={raw_position(m)} (~{raw_degrees(m):.1f} deg) | "
                         f"V={self.voltage} | T={self.temperature}"
                    for m in range(1, 7)
                ), DIVIDER)
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        with self._lock:
            self._buffer.clear()

    def close(self):
        self.is_open = False

    def _emit(self, *lines):
        with self._lock:
            for line in lines:
                self._buffer += (line + "\r\n").encode("utf-8")

    def _pump(self):
        now = time.monotonic()
        while self._next <= now:
            self._emit(*(
                f"POS ID {m}: {raw_position(m)} (~{raw_degrees(m):.1f} deg)"
                for m in range(1, 7) if m not in self.missing
            ))
            self._next += 0.05


def _scene() -> np.ndarray:
    frame = np.full((480, 640, 3), (90, 95, 100), np.uint8)
    lime = cv2.cvtColor(np.uint8([[[40, 200, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
    frame[260:420, 300:560] = lime
    frame[150:210, 120:180] = (30, 30, 200)
    return frame


SCENE = _scene()


def scene(sequence: int) -> np.ndarray:
    frame = SCENE.copy()
    frame[0, 0] = (sequence % 256, sequence // 256 % 256, 0)  # cuadros distintos
    return frame


def black(_sequence: int) -> np.ndarray:
    return np.zeros((480, 640, 3), np.uint8)


class FakeStream:
    """Sustituto de CameraStream con cuadros sintéticos a 30 cuadros/s."""

    def __init__(self, index, frame_factory, *, rotation_degrees=180, fps=30.0, **_params):
        self.index = index
        self.frame_factory = frame_factory
        self.rotation_degrees = rotation_degrees
        self.fps = fps
        self.started = time.monotonic()
        self.closed = False
        self.thread = types.SimpleNamespace(join=lambda timeout=None: None)

    def snapshot(self) -> FrameSnapshot:
        sequence = int((time.monotonic() - self.started) * self.fps) + 1
        frame = self.frame_factory(sequence)
        return FrameSnapshot(
            frame=frame,
            timestamp=self.started + (sequence - 1) / self.fps,
            sequence=sequence,
            camera_index=self.index,
            rotation_degrees=self.rotation_degrees,
            status="DirectShow" if frame is not None else "Sin imagen; cámara ocupada o no disponible.",
        )

    def close(self):
        self.closed = True


def run_fake(*, serial_options=None, frames=None, same_index=False, cameras=True, esp32=True):
    factories = {"frontal": scene, "lateral": scene, **(frames or {})}
    by_index = {CONFIG.cameras[key].fallback_index: factory for key, factory in factories.items()}
    devices: list[FakeESP32Serial] = []

    def serial_factory(port, baud, **kwargs):
        device = FakeESP32Serial(port, baud, **kwargs, **(serial_options or {}))
        devices.append(device)
        return device

    with tempfile.TemporaryDirectory() as folder:
        output = Path(folder)
        report = run_diagnostic(
            CONFIG,
            Options(output_dir=output, cameras=cameras, esp32=esp32, duration=1.0,
                    open_timeout=1.0, boot_wait=0.3),
            stream_factory=lambda index, **params: FakeStream(index, by_index[index], **params),
            index_resolver=lambda _name, fallback: 2 if same_index else fallback,
            camera_enumerator=lambda: CAMERAS,
            port_lister=lambda: PORTS,
            serial_factory=serial_factory,
            progress=lambda _message: None,
        )
        save_report(report, output)
        files = {path.name for path in output.iterdir()}
        saved = json.loads((output / "reporte.json").read_text(encoding="utf-8"))
    return report, devices, files, saved


class FirmwareSourceTests(unittest.TestCase):
    def test_help_templates_follow_print_help(self) -> None:
        self.assertEqual(set(DIVIDER), {"-"})
        self.assertEqual(FIRMWARE_HELP[-1], DIVIDER)
        for line in (
            "Comandos disponibles:",
            "posall",
            "arm torque off  (solamente M1..M5)",
            "Cooldown entre movimientos: 50 ms",
            "ID 6:    0..140 logico -> 10..150 real",
        ):
            self.assertIn(line, FIRMWARE_HELP)

    def test_firmware_limits_match_config_json(self) -> None:
        self.assertEqual(firmware_limits(FIRMWARE), CONFIG.limits)
        self.assertEqual(help_limits(help_templates(FIRMWARE)), CONFIG.limits)


class PortTests(unittest.TestCase):
    def test_configured_port_is_used_when_present(self) -> None:
        self.assertEqual(resolve_port("COM4", PORTS), ("COM4", "config.json"))

    def test_single_usb_adapter_is_chosen_like_the_app(self) -> None:
        ports = [
            {"puerto": "COM1", "descripcion": "Puerto de comunicaciones (COM1)",
             "fabricante": "(Tipos de puerto estándar)"},
            {"puerto": "COM7", "descripcion": "Silicon Labs CP210x USB to UART Bridge (COM7)",
             "fabricante": "Silicon Labs"},
        ]
        self.assertEqual(resolve_port("COM4", ports), ("COM7", "autodetectado"))

    def test_argument_overrides_configuration(self) -> None:
        self.assertEqual(resolve_port("COM4", PORTS, "COM9"), ("COM9", "indicado con --puerto"))

    def test_no_port_found(self) -> None:
        self.assertEqual(resolve_port("COM4", []), (None, "no encontrado"))


class ReadOnlyTests(unittest.TestCase):
    def test_motion_and_torque_commands_are_refused(self) -> None:
        robot = DiagnosticRobotSerial("COM4", 115200, serial_factory=FakeESP32Serial, boot_wait=0.1)
        robot.connect()
        try:
            for command in ("torque on", "arm torque off", "1 180", "pair 2 90 3 120",
                            "gripper close", "gripper release"):
                with self.subTest(command=command), self.assertRaises(ValueError):
                    robot.send(command)
            self.assertEqual(robot.connection.written, ["posall"])
        finally:
            robot.disconnect()


class HealthyHardwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report, cls.devices, cls.files, cls.saved = run_fake()

    def check(self, group: str, name: str):
        check = self.report.find(group, name)
        self.assertIsNotNone(check, f"Falta {group} · {name}")
        return check

    def test_every_check_passes(self) -> None:
        self.assertFalse(self.report.failed, self.report.to_text())
        for group, name in (
            ("firmware", "límites del repositorio"),
            ("firmware", "versión cargada (help)"),
            ("esp32", "puerto"),
            ("esp32", "conexión"),
            ("esp32", "posición de los seis servos (posall)"),
            ("esp32", "voltaje (scan)"),
            ("esp32", "temperatura (scan)"),
            ("esp32", "telemetría automática"),
            ("esp32", "estabilidad en reposo"),
            ("esp32", "ángulos dentro de límites"),
            ("esp32", "línea serie"),
            ("esp32", "mensajes de falla"),
            ("cámaras", "lista de cámaras"),
            ("cámaras", "frontal · identificación"),
            ("cámaras", "lateral · apertura"),
            ("cámaras", "frontal · imagen"),
            ("cámaras", "lateral · imagen"),
            ("cámaras", "par de imágenes"),
            ("cámaras", "frontal · detección"),
            ("cámaras", "lateral · detección"),
            ("integración", "condiciones de Tomar foto"),
        ):
            with self.subTest(prueba=f"{group} · {name}"):
                check = self.check(group, name)
                self.assertEqual(check.status, OK, check.detail)

    def test_only_read_only_commands_reach_the_esp32(self) -> None:
        written = [command for device in self.devices for command in device.written]
        self.assertEqual(set(written), set(READ_ONLY_COMMANDS))

    def test_reset_on_connect_is_reported(self) -> None:
        self.assertIn("se reinició", self.check("esp32", "conexión").detail)

    def test_visual_checks_are_left_to_the_operator(self) -> None:
        for key in ("frontal", "lateral"):
            self.assertEqual(self.check("cámaras", f"{key} · revisión visual").status, REVISAR)

    def test_results_folder_has_images_and_reports(self) -> None:
        expected = {
            "frontal.png", "frontal_deteccion.png", "lateral.png", "lateral_deteccion.png",
            "reporte.json", "reporte.txt", "serial.log",
        }
        self.assertLessEqual(expected, self.files)
        self.assertNotIn(FALLA, self.saved["resumen"])


class FaultyHardwareTests(unittest.TestCase):
    def test_missing_servo_fails(self) -> None:
        report, *_ = run_fake(serial_options={"missing": (3,)})
        self.assertTrue(report.failed)
        for name in ("posición de los seis servos (posall)", "telemetría automática", "voltaje (scan)"):
            with self.subTest(prueba=name):
                check = report.find("esp32", name)
                self.assertEqual(check.status, FALLA)
                self.assertIn("M3", check.detail)
        self.assertEqual(report.find("integración", "condiciones de Tomar foto").status, FALLA)

    def test_other_firmware_version_warns(self) -> None:
        report, *_ = run_fake(serial_options={"help_lines": ["Comandos disponibles:", "  posall"]},
                              cameras=False)
        self.assertEqual(report.find("firmware", "versión cargada (help)").status, AVISO)

    def test_port_opened_without_reset(self) -> None:
        report, *_ = run_fake(serial_options={"reset_on_open": False}, cameras=False)
        check = report.find("esp32", "conexión")
        self.assertEqual(check.status, OK)
        self.assertIn("no lo reinició", check.detail)

    def test_black_camera_fails(self) -> None:
        report, *_ = run_fake(frames={"frontal": black}, esp32=False)
        self.assertEqual(report.find("cámaras", "frontal · imagen").status, FALLA)

    def test_camera_without_image_fails(self) -> None:
        report, *_ = run_fake(frames={"lateral": lambda _sequence: None})
        self.assertEqual(report.find("cámaras", "lateral · apertura").status, FALLA)
        self.assertEqual(report.find("cámaras", "par de imágenes").status, FALLA)
        self.assertEqual(report.find("integración", "condiciones de Tomar foto").status, FALLA)

    def test_both_cameras_on_same_index_fail_before_opening(self) -> None:
        report, *_ = run_fake(same_index=True, esp32=False)
        self.assertEqual(report.find("cámaras", "índices").status, FALLA)
        self.assertIsNone(report.find("cámaras", "frontal · apertura"))


if __name__ == "__main__":
    unittest.main()
