"""Calibración de cámaras para el modo geométrico. Solo lee cámaras y telemetría.

Se abre desde la aplicación principal («Calibración · cámaras»), que le pasa la
escena detectada y la telemetría que ya recibe. No abre puertos, no cambia
torque ni envía movimientos: la garra se abre y se cierra desde la aplicación
principal. Ver CALIBRACION_CAMARAS.md.
"""
from __future__ import annotations

import json
import math
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, ttk

from brazoia.calibracion_camaras import (
    CAMERAS,
    KIND_LABELS,
    KINDS,
    MINIMUM_SAMPLES,
    RECOMMENDED_SAMPLES,
    Sample,
    describe,
    fit_calibration,
    object_pixels,
)


ROOT = Path(__file__).resolve().parent
INSTRUCTIONS = (
    "1. Con el brazo en la pose inicial, poné el cubo y el recipiente en lugares nuevos, alineados con la "
    "base, y pulsá «Foto de la escena».\n"
    "2. Con «Torque brazo OFF (M1–M5)», llevá a mano la garra abierta alrededor del cubo sin moverlo. "
    "Cerrala con «Cerrar garra por presión» y pulsá «Registrar agarre del cubo».\n"
    "3. Sin soltarlo, llevá el cubo sobre el centro del recipiente, como en Soltado, y pulsá «Registrar "
    "cubo sobre el recipiente».\n"
    f"4. Abrí la garra, volvé a la pose inicial y repetí desde 1. Repartí al menos {RECOMMENDED_SAMPLES} "
    "lugares por toda la zona donde van a ir el cubo y el recipiente."
)


class CameraCalibrationApp:
    def __init__(self, root, *, project_root=ROOT, scene_provider=None, pose_provider=None,
                 camera_settings=None, locator=None, captures_dir=None, output_path=None):
        self.root = root
        self.project_root = Path(project_root)
        self.scene_provider = scene_provider
        self.pose_provider = pose_provider
        self.camera_settings = dict(camera_settings or {})
        self._locator = locator  # pose → (x, y) en metros; por defecto, el modelo 3D
        folder = Path(captures_dir) if captures_dir else self.project_root / "datos" / "calibracion"
        self.captures_path = folder / f"camaras_capturas_{datetime.now():%Y%m%d_%H%M%S}.json"
        self.output_path = Path(output_path) if output_path else folder / "camaras.json"
        self.samples: list[Sample] = []
        self.pending: dict[str, dict] | None = None
        self.used: set[str] = set()
        self.calibration = None

        root.title("Brazo IA · CALIBRACIÓN DE CÁMARAS · SOLO LECTURA")
        root.geometry("1000x700")
        root.minsize(900, 600)
        content = ttk.Frame(root, padding=12)
        content.pack(fill="both", expand=True)
        ttk.Label(content, text="CALIBRACIÓN DE CÁMARAS · SOLO LECTURA", foreground="#1266ad",
                  font=("Segoe UI", 16, "bold")).pack(anchor="w")
        ttk.Label(content, wraplength=950, text=(
            "Esta ventana solo lee la escena detectada y la telemetría que ya recibe la aplicación "
            "principal: nunca mueve el brazo ni cambia el torque. Las capturas se guardan solas en "
            "datos/calibracion/. Guía completa: CALIBRACION_CAMARAS.md.")).pack(anchor="w", pady=(4, 8))
        capture = ttk.LabelFrame(content, text="Captura", padding=10)
        capture.pack(fill="both", expand=True)
        ttk.Label(capture, wraplength=930, justify="left", text=INSTRUCTIONS).pack(anchor="w")
        buttons = ttk.Frame(capture)
        buttons.pack(fill="x", pady=6)
        scene_state = "normal" if scene_provider else "disabled"
        pose_state = "normal" if pose_provider else "disabled"
        ttk.Button(buttons, text="Foto de la escena", state=scene_state, command=self.capture_scene).pack(side="left")
        ttk.Button(buttons, text="Registrar agarre del cubo", state=pose_state,
                   command=lambda: self.register("cubo")).pack(side="left", padx=6)
        ttk.Button(buttons, text="Registrar cubo sobre el recipiente", state=pose_state,
                   command=lambda: self.register("recipiente")).pack(side="left")
        ttk.Button(buttons, text="Borrar última", command=self.remove_last).pack(side="left", padx=6)
        ttk.Button(buttons, text="Cargar capturas…", command=self.choose_captures).pack(side="left")
        self.count = tk.StringVar()
        ttk.Label(capture, textvariable=self.count).pack(anchor="w")
        listing = ttk.Frame(capture)
        listing.pack(fill="both", expand=True, pady=4)
        columns = (("numero", "#", 40), ("tipo", "Objeto", 110), ("frontal", "Frontal (x, y)", 150),
                   ("lateral", "Lateral (x, y)", 150), ("mesa", "Mesa según el modelo (mm)", 230))
        self.table = ttk.Treeview(listing, columns=[c[0] for c in columns], show="headings", height=8)
        for column, title, width in columns:
            self.table.heading(column, text=title)
            self.table.column(column, width=width, anchor="center")
        scroll = ttk.Scrollbar(listing, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=scroll.set)
        self.table.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

        fit = ttk.LabelFrame(content, text="Resultado", padding=10)
        fit.pack(fill="x", pady=(8, 0))
        row = ttk.Frame(fit)
        row.pack(fill="x")
        ttk.Button(row, text="Calcular calibración", command=self.calculate).pack(side="left")
        ttk.Button(row, text="Guardar como calibración activa", command=self.save_active).pack(side="left", padx=6)
        self.result = tk.StringVar(value="Todavía no hay calibración calculada.")
        ttk.Label(fit, textvariable=self.result, wraplength=930, justify="left",
                  font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(6, 0))
        self.status = tk.StringVar(value=(
            "Listo para capturar." if scene_provider and pose_provider else
            "Abrí esta ventana desde la aplicación principal, con las cámaras abiertas y el ESP32 conectado."))
        ttk.Label(content, textvariable=self.status, wraplength=950).pack(anchor="w", pady=8)
        self.refresh_count()

    # ----- captura -----
    def capture_scene(self):
        try:
            if self.scene_provider is None:
                raise ValueError("Abrí esta ventana desde la aplicación principal con las cámaras abiertas.")
            features = self.scene_provider()
            pending = {kind: object_pixels(features, kind) for kind in KINDS}
        except Exception as exc:
            self.status.set(f"No se pudo tomar la foto: {exc}")
            return
        self.pending, self.used = pending, set()
        parts = []
        for kind in KINDS:
            missing = [camera for camera in CAMERAS if camera not in pending[kind]]
            parts.append(f"{KIND_LABELS[kind]}: " + ("en las dos cámaras" if not missing else f"no se ve en la {', '.join(missing)}"))
        self.status.set("Foto tomada. " + "; ".join(parts) + ". Ahora agarrá el cubo donde está.")

    def read_pose(self):
        if self.pose_provider is None:
            raise ValueError("Abrí esta ventana desde la aplicación principal con el ESP32 conectado.")
        pose = {int(motor): float(value) for motor, value in dict(self.pose_provider()).items()}
        if any(motor not in pose or not math.isfinite(pose[motor]) for motor in range(1, 7)):
            raise ValueError("La telemetría no trae los seis ángulos.")
        return pose

    def locate(self, pose):
        if self._locator is None:
            from brazoia.mujoco_sim import VirtualArm
            from brazoia.politica_geometrica import object_center

            arm = VirtualArm()
            self._locator = lambda value: object_center(arm, value)
        return self._locator(pose)

    def register(self, kind):
        label = KIND_LABELS[kind]
        if self.pending is None:
            self.status.set("Primero tomá la foto de la escena.")
            return
        if kind in self.used:
            self.status.set(f"Esa foto ya se usó para {label}: volvé a la pose inicial y tomá una nueva.")
            return
        missing = [camera for camera in CAMERAS if camera not in self.pending[kind]]
        if missing:
            self.status.set(f"En la foto no se ve {label} en la cámara {', '.join(missing)}: "
                            "acomodá la escena y tomá otra foto.")
            return
        try:
            pose = self.read_pose()
            x, y = self.locate(pose)
        except Exception as exc:
            self.status.set(f"No se pudo registrar: {exc}")
            return
        sample = Sample(kind, dict(self.pending[kind]), (float(x), float(y)), f"captura {len(self.samples) + 1}",
                        pose=pose)
        self.samples.append(sample)
        self.used.add(kind)
        self._insert_row(sample)
        self.save_captures()
        self.refresh_count()
        following = ("Llevá el cubo sobre el centro del recipiente y registralo, o abrí la garra y volvé "
                     "a la pose inicial." if kind == "cubo" else "Abrí la garra y volvé a la pose inicial.")
        self.status.set(f"{label.capitalize()} registrado en ({x * 1000:.0f}, {y * 1000:.0f}) mm. {following}")

    def remove_last(self):
        if not self.samples:
            return
        self.samples.pop()
        self.table.delete(self.table.get_children()[-1])
        self.save_captures()
        self.refresh_count()
        self.status.set("Última muestra borrada.")

    def choose_captures(self):
        path = filedialog.askopenfilename(parent=self.root, initialdir=self.captures_path.parent,
                                          filetypes=[("Capturas de cámaras", "*.json")])
        if path:
            self.load_captures(Path(path))

    def load_captures(self, path):
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            loaded = [Sample.from_dict(item) for item in data["muestras"]]
        except Exception as exc:
            self.status.set(f"No se pudieron cargar las capturas: {exc}")
            return
        for sample in loaded:
            self.samples.append(sample)
            self._insert_row(sample)
        self.save_captures()
        self.refresh_count()
        self.status.set(f"{len(loaded)} muestras cargadas de {Path(path).name}.")

    # ----- cálculo -----
    def calculate(self):
        try:
            self.calibration = fit_calibration(self.samples, self.camera_settings)
        except ValueError as exc:
            self.calibration = None
            self.result.set(str(exc))
            return
        self.result.set(describe(self.calibration))
        self.status.set("Calibración calculada. Si el error te parece aceptable, guardala como calibración activa.")

    def save_active(self):
        if self.calibration is None:
            self.status.set("Primero calculá la calibración.")
            return
        self.calibration.save(self.output_path)
        self.status.set(f"Guardada en {self.output_path}. El modo geométrico de la ejecución automática la usa.")

    def save_captures(self):
        payload = {"version": 1, "camaras": self.camera_settings,
                   "muestras": [sample.to_dict() for sample in self.samples]}
        self.captures_path.parent.mkdir(parents=True, exist_ok=True)
        self.captures_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def refresh_count(self):
        counts = {kind: sum(sample.kind == kind for sample in self.samples) for kind in KINDS}
        self.count.set(f"{counts['cubo']} muestras del cubo y {counts['recipiente']} del recipiente "
                       f"(mínimo {MINIMUM_SAMPLES}; recomendado {RECOMMENDED_SAMPLES} o más de cada uno).")

    def _insert_row(self, sample):
        def text(camera):
            x, y = sample.pixels[camera]
            return f"{x:.3f}, {y:.3f}"

        self.table.insert("", "end", values=(
            len(self.table.get_children()) + 1, sample.kind, text("frontal"), text("lateral"),
            f"{sample.table_xy[0] * 1000:.0f}, {sample.table_xy[1] * 1000:.0f}"))
