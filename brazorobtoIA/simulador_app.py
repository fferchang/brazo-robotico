"""Controles exclusivamente virtuales. No importa ni conecta el controlador serie."""
from __future__ import annotations

import math
from pathlib import Path
import tkinter as tk
from tkinter import ttk

from brazoia.config import INITIAL_POSE, load_config
from brazoia.gripper_geometry import GripperGeometry
from brazoia.rerun_view import RerunArmView


ROOT = Path(__file__).resolve().parent


class SimulatorApp:
    def __init__(self, root, *, project_root=ROOT, auto_open=True):
        self.root = root
        self.project_root = Path(project_root)
        self.limits = load_config(self.project_root / "config.json").limits
        self.pose = {**INITIAL_POSE, 6: 0.0}
        self.geometry = GripperGeometry(self.project_root / "assets")
        self.view = RerunArmView(self.project_root / "assets",
            recording_name="brazo_ia_SIMULADOR_MANUAL",
            view_name="SIMULADOR MANUAL · SOLO VIRTUAL", clamp_joint_limits=False)
        self.timer = None
        self.open_timer = None
        root.title("Brazo IA · SIMULADOR MANUAL · SIN ESP32")
        root.geometry("860x690")
        root.minsize(760, 650)
        root.protocol("WM_DELETE_WINDOW", self.close)
        content = ttk.Frame(root, padding=18)
        content.pack(fill="both", expand=True)
        ttk.Label(content, text="SIMULACIÓN · SOLO RERUN", font=("Segoe UI", 19, "bold"),
                  foreground="#1266ad").pack(anchor="w")
        ttk.Label(content, text="Estos controles mueven únicamente el modelo 3D. No hay conexión al ESP32.",
                  wraplength=780).pack(anchor="w", pady=(4, 12))
        toolbar = ttk.Frame(content)
        toolbar.pack(fill="x", pady=(0, 12))
        for text, command in (("Abrir Rerun", self.open_view), ("Pose inicial", self.reset_pose),
                              ("Abrir garra virtual", lambda: self.set_angle(6, self.limits[6][1])),
                              ("Cerrar garra virtual", lambda: self.set_angle(6, 0)),
                              ("Copiar ángulos", self.copy_pose)):
            ttk.Button(toolbar, text=text, command=command).pack(side="left", padx=(0, 5))
        ttk.Button(content, text="Planificar trayecto Inicio → Final", command=self.open_trajectory).pack(anchor="w", pady=(0, 6))
        controls = ttk.LabelFrame(content, text="Ángulos lógicos · arrastrá o escribí y pulsá Aplicar", padding=12)
        controls.pack(fill="x")
        controls.columnconfigure(1, weight=1)
        self.sliders, self.entries, self.values = {}, {}, {}
        for motor in range(1, 7):
            low, high = self.limits[motor]
            row = motor - 1
            ttk.Label(controls, text=f"M{motor}", font=("Segoe UI", 12, "bold")).grid(row=row, column=0, padx=(0, 12))
            self.values[motor] = tk.DoubleVar(value=self.pose[motor])
            self.entries[motor] = tk.StringVar(value=f"{self.pose[motor]:.1f}")
            slider = ttk.Scale(controls, from_=low, to=high, variable=self.values[motor],
                               command=lambda value, m=motor: self.slider_changed(m, value))
            slider.grid(row=row, column=1, sticky="ew", pady=12)
            self.sliders[motor] = slider
            entry = ttk.Entry(controls, textvariable=self.entries[motor], width=8, justify="right")
            entry.grid(row=row, column=2, padx=(12, 3))
            entry.bind("<Return>", lambda event, m=motor: self.apply_entry(m))
            ttk.Label(controls, text="°").grid(row=row, column=3)
            ttk.Button(controls, text="Aplicar", command=lambda m=motor: self.apply_entry(m)).grid(row=row, column=4, padx=8)
            ttk.Label(controls, text=f"{low:g}–{high:g}°", width=10).grid(row=row, column=5)
        floor = ttk.LabelFrame(content, text="Distancia virtual de la garra al piso", padding=12)
        floor.pack(fill="x", pady=12)
        self.height_text = tk.StringVar()
        self.height_label = ttk.Label(floor, textvariable=self.height_text, font=("Segoe UI", 17, "bold"))
        self.height_label.pack(anchor="w")
        self.detail = tk.StringVar()
        ttk.Label(floor, textvariable=self.detail, wraplength=760).pack(anchor="w", pady=4)
        ttk.Label(floor, text="Piso a 100 mm debajo del eje M2. Se consideran ambas mordazas, M5 y M6.",
                  wraplength=760).pack(anchor="w")
        ttk.Label(content, text="La simulación permite atravesar el piso para estudiar la geometría. "
                  "No simula presión ni garantiza que una pose sea físicamente válida.",
                  wraplength=780).pack(anchor="w")
        self.status = tk.StringVar(value="Controles listos. Rerun muestra los ángulos sin recortes visuales.")
        ttk.Label(content, textvariable=self.status, wraplength=780).pack(anchor="w", pady=10)
        self.publish()
        if auto_open:
            self.open_timer = root.after(250, self.open_view)

    def set_angle(self, motor, value):
        value = float(value)
        low, high = self.limits[motor]
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"M{motor}: ingresá un ángulo entre {low:g}° y {high:g}°.")
        self.pose[motor] = value
        self.values[motor].set(value)
        self.entries[motor].set(f"{value:.1f}")
        self.schedule_publish()

    def slider_changed(self, motor, value):
        self.set_angle(motor, round(float(value), 1))

    def apply_entry(self, motor):
        try:
            self.set_angle(motor, float(self.entries[motor].get().strip().replace(",", ".")))
            self.status.set(f"M{motor} virtual a {self.pose[motor]:.1f}°.")
        except (ValueError, OverflowError) as exc:
            self.status.set(f"Ángulo inválido: {exc}")

    def schedule_publish(self):
        if self.timer is None:
            self.timer = self.root.after(40, self.publish)

    def publish(self):
        if self.timer is not None:
            self.root.after_cancel(self.timer)
            self.timer = None
        result = self.geometry.clearance(self.pose)
        height = result["height_mm"]
        label = "CRUZA / TOCA EL PISO" if height <= 0 else "CERCA DEL PISO" if height <= 5 else "SOBRE EL PISO"
        self.height_text.set(f"{height:.1f} mm · {label}")
        self.height_label.configure(foreground="#b42318" if height <= 5 else "#168044")
        detail = "Distancia del punto más bajo de la garra al plano del piso."
        if result["clipped_motors"]:
            detail += " Fuera del rango del URDF: " + ", ".join(f"M{i}" for i in result["clipped_motors"])
            detail += ". Se dibuja el ángulo solicitado, sin recortarlo."
        self.detail.set(detail)
        if self.view.available:
            try:
                self.view.log_pose(dict(self.pose))
            except Exception as exc:
                self.status.set(f"No se pudo actualizar Rerun: {exc}")

    def open_view(self):
        if self.open_timer is not None:
            self.root.after_cancel(self.open_timer)
            self.open_timer = None
        if self.view.available:
            try:
                self.view.recording.spawn(hide_welcome_screen=True)
            except Exception as exc:
                self.status.set(f"No se pudo abrir Rerun: {exc}")
                return
        else:
            self.view.open()
        if self.view.available:
            self.status.set("Rerun conectado al simulador. Arrastrá cualquier deslizador.")
            self.publish()
        else:
            self.status.set(f"No se pudo abrir Rerun: {self.view.error}. Podés reintentar con Abrir Rerun.")

    def reset_pose(self):
        for motor, value in {**INITIAL_POSE, 6: 0.0}.items():
            self.set_angle(motor, value)
        self.status.set("Pose inicial virtual; garra cerrada.")

    def copy_pose(self):
        text = " · ".join(f"M{i}: {self.pose[i]:.1f}°" for i in range(1, 7))
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.status.set("Ángulos virtuales copiados.")

    def open_trajectory(self):
        from trayecto_app import TrajectoryApp
        window = tk.Toplevel(self.root)
        window.trajectory = TrajectoryApp(window, project_root=self.project_root,
                                         virtual_pose_provider=lambda: dict(self.pose))

    def close(self):
        for timer in (self.timer, self.open_timer):
            if timer is not None:
                self.root.after_cancel(timer)
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    SimulatorApp(root)
    root.mainloop()
