"""Calibración modelo-brazo: toques de mesa y garra cerrada. Solo lee telemetría.

Se abre desde la aplicación principal («Calibración · toques de mesa» o
lanzadores/CALIBRACION.bat), que le pasa una copia de la telemetría ya recibida. No abre
puertos, no cambia torque, no envía movimientos y no modifica calibracion.json.
Ver docs/CALIBRACION.md.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import tkinter as tk
from tkinter import ttk

from brazoia.calibration import (describe_fit, degrees_text, fit_touches, gripper_offset,
                                 model_m6_for_gap, touch_heights)
from brazoia.gripper_geometry import GripperGeometry


ROOT = Path(__file__).resolve().parent
RECOMMENDED_TOUCHES = 12


def gripper_text(geometry, m6_reading, gap_mm):
    delta = gripper_offset(geometry, m6_reading, gap_mm)
    current = float(geometry.calibration[6].get("visual_offset_degrees", 0.0))
    return (f"Lectura M6 = {m6_reading:.1f}° con {gap_mm:g} mm entre las puntas. El modelo deja esa "
            f"separación en M6 = {m6_reading + delta:.1f}°: hay que sumar {delta:+.1f}° a la lectura "
            f"(visual_offset_degrees de M6: {current:g} → {current + delta:.1f}).")


class CalibrationApp:
    def __init__(self, root, *, project_root=ROOT, real_pose_provider=None, data_dir=None):
        self.root = root
        self.project_root = Path(project_root)
        self.geometry = GripperGeometry(self.project_root / "assets")
        self.provider = real_pose_provider
        self.touches = []
        self.gripper = None
        folder = Path(data_dir) if data_dir else self.project_root / "datos" / "calibracion"
        self.path = folder / f"calibracion_{datetime.now():%Y%m%d_%H%M%S}.json"
        closing = model_m6_for_gap(self.geometry)
        root.title("Brazo IA · CALIBRACIÓN · SOLO LECTURA")
        root.geometry("1000x680")
        root.minsize(900, 600)
        content = ttk.Frame(root, padding=12)
        content.pack(fill="both", expand=True)
        ttk.Label(content, text="CALIBRACIÓN MODELO-BRAZO · SOLO LECTURA", foreground="#1266ad",
                  font=("Segoe UI", 16, "bold")).pack(anchor="w")
        ttk.Label(content, wraplength=950, text=(
            "Esta ventana solo copia la telemetría que ya recibe la aplicación principal: no cambia "
            "torque ni envía movimientos. Los toques se guardan solos en datos/calibracion/. "
            "Guía completa: docs/CALIBRACION.md.")).pack(anchor="w", pady=(4, 10))

        touches = ttk.LabelFrame(content, text="1 · Toques de mesa", padding=10)
        touches.pack(fill="both", expand=True)
        ttk.Label(touches, wraplength=930, text=(
            "Con «Torque brazo OFF (M1–M5)», apoyá suavemente la punta de la garra cerrada sobre la mesa, "
            "activá torque y pulsá «Capturar toque». Repetí 12 veces o más, variando: cerca y lejos de "
            "la base, a la izquierda y a la derecha, con la garra vertical e inclinada.")).pack(anchor="w")
        buttons = ttk.Frame(touches)
        buttons.pack(fill="x", pady=6)
        state = "normal" if real_pose_provider else "disabled"
        ttk.Button(buttons, text="Capturar toque", state=state, command=self.capture_touch).pack(side="left")
        ttk.Button(buttons, text="Borrar último", command=self.remove_last).pack(side="left", padx=6)
        ttk.Button(buttons, text="Calcular ajuste", command=self.calculate).pack(side="left", padx=6)
        self.count = tk.StringVar()
        ttk.Label(buttons, textvariable=self.count).pack(side="left", padx=12)
        listing = ttk.Frame(touches)
        listing.pack(fill="both", expand=True)
        self.table = ttk.Treeview(listing, columns=("numero", "angulos", "altura"), show="headings", height=6)
        for column, title, width in (("numero", "#", 40), ("angulos", "Telemetría (°)", 620),
                                     ("altura", "Garra-mesa según el modelo actual", 230)):
            self.table.heading(column, text=title)
            self.table.column(column, width=width, anchor="center" if column != "angulos" else "w")
        scroll = ttk.Scrollbar(listing, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=scroll.set)
        self.table.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.result = tk.StringVar(value="Todavía no hay ajuste.")
        ttk.Label(touches, textvariable=self.result, wraplength=930, justify="left",
                  font=("Segoe UI", 10, "bold")).pack(anchor="w", pady=(8, 0))

        gripper = ttk.LabelFrame(content, text="2 · Garra cerrada sin nada", padding=10)
        gripper.pack(fill="x", pady=(10, 0))
        ttk.Label(gripper, wraplength=930, text=(
            "Cerrá la garra por presión sin nada entre los dedos. Cuando se detenga, mirá si las puntas "
            "se tocan; si no, medí la separación con una regla. Anotala y pulsá «Capturar garra cerrada».")
                  ).pack(anchor="w")
        row = ttk.Frame(gripper)
        row.pack(fill="x", pady=6)
        ttk.Label(row, text="Separación entre puntas (mm):").pack(side="left")
        self.gap = tk.StringVar(value="0")
        ttk.Entry(row, textvariable=self.gap, width=6, justify="right").pack(side="left", padx=6)
        ttk.Button(row, text="Capturar garra cerrada", state=state, command=self.capture_gripper).pack(side="left")
        self.gripper_result = tk.StringVar(value=f"En el modelo actual las puntas se juntan en M6 = {closing:.1f}°.")
        ttk.Label(gripper, textvariable=self.gripper_result, wraplength=930, justify="left").pack(anchor="w")

        self.status = tk.StringVar(value=(
            "Listo para capturar." if real_pose_provider else
            "Abrí esta ventana desde la aplicación principal con el ESP32 conectado para capturar."))
        ttk.Label(content, textvariable=self.status, wraplength=950).pack(anchor="w", pady=8)
        self.refresh_count()

    def read_pose(self):
        if self.provider is None:
            raise ValueError("Abrí esta ventana desde la aplicación principal con el ESP32 conectado.")
        pose = {int(motor): float(value) for motor, value in dict(self.provider()).items()}
        if any(motor not in pose or not math.isfinite(pose[motor]) for motor in range(1, 7)):
            raise ValueError("La telemetría no trae los seis ángulos.")
        return pose

    def capture_touch(self):
        try:
            pose = self.read_pose()
        except Exception as exc:
            self.status.set(f"No se pudo capturar: {exc}")
            return
        self.touches.append(pose)
        height = float(touch_heights(self.geometry, [pose])[0])
        self.table.insert("", "end", values=(len(self.touches), degrees_text(pose), f"{height:+.1f} mm"))
        self.save()
        self.refresh_count()
        self.status.set(f"Toque {len(self.touches)} capturado: según el modelo actual la garra está a "
                        f"{height:+.1f} mm de la mesa (en la realidad, 0).")

    def remove_last(self):
        if not self.touches:
            return
        self.touches.pop()
        self.table.delete(self.table.get_children()[-1])
        self.save()
        self.refresh_count()
        self.status.set("Último toque borrado.")

    def calculate(self):
        try:
            fit = fit_touches(self.geometry, self.touches)
        except ValueError as exc:
            self.result.set(str(exc))
            return
        self.result.set(describe_fit(self.geometry, fit))
        self.save(fit=fit)

    def capture_gripper(self):
        try:
            gap = float(self.gap.get().strip().replace(",", "."))
            if not math.isfinite(gap) or gap < 0:
                raise ValueError("La separación debe ser 0 o más milímetros.")
            pose = self.read_pose()
            text = gripper_text(self.geometry, pose[6], gap)
        except Exception as exc:
            self.status.set(f"No se pudo capturar la garra: {exc}")
            return
        self.gripper = {"m6": pose[6], "gap_mm": gap, "pose": pose}
        self.gripper_result.set(text)
        self.save()
        self.status.set("Garra cerrada capturada.")

    def refresh_count(self):
        missing = max(0, RECOMMENDED_TOUCHES - len(self.touches))
        advice = f" · falta{'n' if missing > 1 else ''} {missing} para el mínimo recomendado" if missing else ""
        self.count.set(f"{len(self.touches)} toques{advice}")

    def save(self, fit=None):
        payload = {"schema": 1, "created": self.path.stem.removeprefix("calibracion_"),
                   "touches": [{str(motor): value for motor, value in pose.items()} for pose in self.touches],
                   "gripper_closed": self.gripper}
        if fit is not None:
            payload["fit"] = {"table_mm": fit.table_mm, "offsets": {str(m): v for m, v in fit.offsets.items()},
                              "identifiable": fit.identifiable, "rms_before_mm": fit.rms_before_mm,
                              "rms_after_mm": fit.rms_after_mm}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def analyze(path: Path, project_root=ROOT):
    """Recalcula un archivo guardado, sin ventana ni hardware."""
    geometry = GripperGeometry(Path(project_root) / "assets")
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    poses = [{int(motor): float(value) for motor, value in touch.items()} for touch in data["touches"]]
    lines = [f"{len(poses)} toques en {Path(path).name}."]
    if poses:
        heights = touch_heights(geometry, poses)
        lines += [f"  #{index}: {degrees_text(pose)} → modelo {height:+.1f} mm"
                  for index, (pose, height) in enumerate(zip(poses, heights), start=1)]
        try:
            lines.append(describe_fit(geometry, fit_touches(geometry, poses)))
        except ValueError as exc:
            lines.append(str(exc))
    if data.get("gripper_closed"):
        lines.append(gripper_text(geometry, data["gripper_closed"]["m6"], data["gripper_closed"]["gap_mm"]))
    return "\n".join(lines)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analiza un archivo de calibración guardado.")
    parser.add_argument("archivo", type=Path)
    arguments = parser.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print(analyze(arguments.archivo))
