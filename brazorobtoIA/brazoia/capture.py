from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class FrameSnapshot:
    frame: object | None
    timestamp: float
    sequence: int
    camera_index: int
    rotation_degrees: int
    status: str
    brightness_percent: float = 0.0
    contrast_percent: float = 0.0
    saturation_percent: float = 0.0
    zoom_factor: float = 1.0


def discover_camera_index(device_name: str, fallback_index: int) -> int:
    try:
        from cv2_enumerate_cameras import enumerate_cameras

        matches = [
            camera.index
            for camera in enumerate_cameras(cv2.CAP_DSHOW)
            if camera.name == device_name
        ]
        if len(matches) == 1:
            return int(matches[0])
    except (ImportError, OSError, RuntimeError):
        pass
    return fallback_index


def adjust_contrast(frame: np.ndarray, percent: float) -> np.ndarray:
    """Aumenta contraste alrededor del gris medio sin cambiar el formato."""
    factor = 1.0 + float(percent) / 100.0
    adjusted = (frame.astype(np.float32) - 127.5) * factor + 127.5
    return np.clip(adjusted, 0, 255).astype(np.uint8)


def adjust_brightness(frame: np.ndarray, percent: float) -> np.ndarray:
    """Escala el brillo; -25 equivale a conservar el 75% de intensidad."""
    factor = max(0.0, 1.0 + float(percent) / 100.0)
    adjusted = frame.astype(np.float32) * factor
    return np.clip(adjusted, 0, 255).astype(np.uint8)


def adjust_saturation(frame: np.ndarray, percent: float) -> np.ndarray:
    """Escala el canal de saturación HSV conservando brillo y resolución."""
    factor = max(0.0, 1.0 + float(percent) / 100.0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * factor, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def apply_zoom(frame: np.ndarray, factor: float) -> np.ndarray:
    """Zoom digital centrado que mantiene el tamaño original del cuadro."""
    factor = max(1.0, float(factor))
    if factor <= 1.001:
        return frame
    height, width = frame.shape[:2]
    crop_width = max(2, int(round(width / factor)))
    crop_height = max(2, int(round(height / factor)))
    left = (width - crop_width) // 2
    top = (height - crop_height) // 2
    crop = frame[top : top + crop_height, left : left + crop_width]
    return cv2.resize(crop, (width, height), interpolation=cv2.INTER_LINEAR)


class CameraStream:
    """Captura el cuadro más reciente sin acumular latencia."""

    open_lock = threading.Lock()

    def __init__(
        self,
        index: int,
        rotation_degrees: int = 180,
        contrast_percent: float = 0.0,
        brightness_percent: float = 0.0,
        saturation_percent: float = 0.0,
        zoom_factor: float = 1.0,
    ):
        self.index = index
        self.rotation_degrees = rotation_degrees % 360
        self.contrast_percent = float(contrast_percent)
        self.brightness_percent = float(brightness_percent)
        self.saturation_percent = float(saturation_percent)
        self.zoom_factor = max(1.0, float(zoom_factor))
        self.lock = threading.Lock()
        self.capture_lock = threading.Lock()
        self.capture = None
        self.stop_event = threading.Event()
        self.frame = None
        self.timestamp = 0.0
        self.sequence = 0
        self.status = "Abriendo DirectShow..."
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set_rotation(self, degrees: int) -> None:
        with self.lock:
            self.rotation_degrees = degrees % 360
            self.frame = None
            self.timestamp = 0.0

    def toggle_180(self) -> int:
        value = (self.rotation_degrees + 180) % 360
        self.set_rotation(value)
        return value

    def set_image_parameters(
        self,
        *,
        brightness_percent: float,
        contrast_percent: float,
        saturation_percent: float,
        zoom_factor: float,
    ) -> None:
        with self.lock:
            self.brightness_percent = float(brightness_percent)
            self.contrast_percent = float(contrast_percent)
            self.saturation_percent = float(saturation_percent)
            self.zoom_factor = max(1.0, float(zoom_factor))

    def snapshot(self) -> FrameSnapshot:
        with self.lock:
            return FrameSnapshot(
                frame=None if self.frame is None else self.frame.copy(),
                timestamp=self.timestamp,
                sequence=self.sequence,
                camera_index=self.index,
                rotation_degrees=self.rotation_degrees,
                status=self.status,
                brightness_percent=self.brightness_percent,
                contrast_percent=self.contrast_percent,
                saturation_percent=self.saturation_percent,
                zoom_factor=self.zoom_factor,
            )

    def close(self) -> None:
        self.stop_event.set()

    def _remember_capture(self, capture) -> None:
        with self.capture_lock:
            self.capture = capture

    def _forget_capture(self, capture) -> None:
        with self.capture_lock:
            if self.capture is capture:
                self.capture = None

    def _open_directshow(self):
        if self.stop_event.is_set():
            return None, None
        capture = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        self._remember_capture(capture)
        first = None
        if capture.isOpened():
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            for _ in range(20):
                if self.stop_event.is_set():
                    break
                ok, candidate = capture.read()
                if ok and candidate is not None:
                    first = candidate
                    break
                self.stop_event.wait(0.04)
        if first is not None:
            return capture, first
        capture.release()
        self._forget_capture(capture)
        return None, None

    def _publish(self, frame) -> None:
        codes = {
            90: cv2.ROTATE_90_CLOCKWISE,
            180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_COUNTERCLOCKWISE,
        }
        with self.lock:
            if self.rotation_degrees in codes:
                frame = cv2.rotate(frame, codes[self.rotation_degrees])
            frame = apply_zoom(frame, self.zoom_factor)
            if self.contrast_percent:
                frame = adjust_contrast(frame, self.contrast_percent)
            if self.brightness_percent:
                frame = adjust_brightness(frame, self.brightness_percent)
            if self.saturation_percent:
                frame = adjust_saturation(frame, self.saturation_percent)
            self.frame = frame
            self.timestamp = time.monotonic()
            self.sequence += 1

    def _run(self) -> None:
        capture = None
        try:
            while not self.stop_event.is_set():
                with self.open_lock:
                    if self.stop_event.is_set():
                        break
                    capture, first = self._open_directshow()
                if first is None:
                    if capture is not None:
                        capture.release()
                        self._forget_capture(capture)
                    capture = None
                    self.status = "Sin imagen; cámara ocupada o no disponible. Reintentando..."
                    self.stop_event.wait(3.0)
                    continue

                self._publish(first)
                self.status = "DirectShow"
                last_good = time.monotonic()
                while not self.stop_event.is_set():
                    ok, candidate = capture.read()
                    if ok and candidate is not None:
                        self._publish(candidate)
                        last_good = time.monotonic()
                    elif time.monotonic() - last_good > 2.0:
                        break
                    else:
                        self.stop_event.wait(0.03)
                capture.release()
                self._forget_capture(capture)
                capture = None
                self.status = "Se perdió la señal; reintentando..."
                self.stop_event.wait(3.0)
        except Exception as exc:
            self.status = str(exc)
        finally:
            if capture is not None:
                capture.release()
                self._forget_capture(capture)


def validate_frame_pair(
    snapshots: list[FrameSnapshot],
    now: float,
    *,
    maximum_age: float,
    maximum_skew: float,
) -> str:
    if len(snapshots) != 2 or any(item.frame is None for item in snapshots):
        return "Falta imagen de una cámara."
    if any(now - item.timestamp > maximum_age for item in snapshots):
        return "Falta una imagen reciente."
    if abs(snapshots[0].timestamp - snapshots[1].timestamp) > maximum_skew:
        return "Las cámaras tienen más de 100 ms de desfase de recepción."
    return ""

