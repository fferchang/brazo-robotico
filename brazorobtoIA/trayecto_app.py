"""Captura/edición de dos poses y reproducción 3D. Sin controlador de motores."""
from __future__ import annotations

from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

from brazoia.config import INITIAL_POSE, load_config
from brazoia.gripper_geometry import GripperGeometry
from brazoia.rerun_view import RerunArmView
from brazoia.simulation_path import FloorPathPlanner, validate_pose


ROOT = Path(__file__).resolve().parent


class TrajectoryApp:
    def __init__(self, root, *, project_root=ROOT, real_pose_provider=None,
                 virtual_pose_provider=None, auto_open=True):
        self.root = root
        self.limits = load_config(Path(project_root) / "config.json").limits
        self.geometry = GripperGeometry(Path(project_root) / "assets")
        self.view = RerunArmView(Path(project_root) / "assets",
            recording_name="brazo_ia_TRAYECTO_SIMULADO",
            view_name="INICIO → FINAL · SOLO SIMULACIÓN", clamp_joint_limits=False)
        self.real_pose_provider, self.virtual_pose_provider = real_pose_provider, virtual_pose_provider
        self.plan = None
        self.revision = 0
        self.cancel = threading.Event()
        self.results = queue.Queue()
        self.play_timer = self.poll_timer = self.open_timer = None
        self.closed = False
        self.play_speed = 20.0
        self.elapsed = 0.0
        self.last_pose = None
        root.title("Brazo IA · TRAYECTO SIMULADO · SIN MOVIMIENTOS REALES")
        root.geometry("950x700")
        root.minsize(900, 650)
        root.protocol("WM_DELETE_WINDOW", self.close)
        content = ttk.Frame(root, padding=15)
        content.pack(fill="both", expand=True)
        ttk.Label(content, text="INICIO → FINAL · SOLO SIMULACIÓN", foreground="#1266ad",
                  font=("Segoe UI", 18, "bold")).pack(anchor="w")
        ttk.Label(content, text="1. Capturá el inicio.  2. Capturá el final.  3. Calculá y mirá el recorrido en Rerun.",
                  wraplength=900).pack(anchor="w", pady=(5, 10))
        if real_pose_provider:
            explanation = ("Capturar del brazo copia la telemetría de la aplicación principal. "
                           "Esta ventana no cambia torque ni envía movimientos.")
        else:
            explanation = ("Para poses reales: abrí esta ventana desde «Trayecto desde brazo · simulación» "
                           "en la aplicación principal. Aquí también podés escribir los ángulos.")
        ttk.Label(content, text=explanation, wraplength=900).pack(anchor="w", pady=(0, 8))
        self.fields = {}
        poses = ttk.Frame(content)
        poses.pack(fill="x")
        poses.columnconfigure((0, 1), weight=1)
        for column, (key, title) in enumerate((("start", "1 · Inicio"), ("goal", "2 · Final"))):
            box = ttk.LabelFrame(poses, text=title, padding=10)
            box.grid(row=0, column=column, sticky="nsew", padx=5)
            self.fields[key] = {}
            for i in range(1, 7):
                low, high = self.limits[i]
                ttk.Label(box, text=f"M{i}").grid(row=i-1, column=0, padx=8, pady=3)
                value = tk.StringVar()
                ttk.Entry(box, textvariable=value, width=12, justify="right").grid(row=i-1, column=1, padx=8, pady=3)
                ttk.Label(box, text=f"°   ({low:g}–{high:g})").grid(row=i-1, column=2, padx=8)
                value.trace_add("write", lambda *_: self.invalidate())
                self.fields[key][i] = value
            actions = ttk.Frame(box)
            actions.grid(row=6, column=0, columnspan=3, pady=8)
            ttk.Button(actions, text="Capturar del brazo", state="normal" if real_pose_provider else "disabled",
                       command=lambda k=key: self.capture(k, "real")).pack(side="left", padx=2)
            ttk.Button(actions, text="Usar pose virtual", state="normal" if virtual_pose_provider else "disabled",
                       command=lambda k=key: self.capture(k, "virtual")).pack(side="left", padx=2)
            ttk.Button(box, text="Cargar pose inicial estándar",
                       command=lambda k=key: self.fill_pose(k, {**INITIAL_POSE, 6: 0})).grid(row=7, column=0, columnspan=3)
        controls = ttk.Frame(content)
        controls.pack(fill="x", pady=12)
        ttk.Button(controls, text="3 · Calcular y simular", command=self.calculate).pack(side="left", padx=4)
        ttk.Button(controls, text="Reproducir otra vez", command=self.play).pack(side="left", padx=4)
        ttk.Button(controls, text="Detener", command=self.stop).pack(side="left", padx=4)
        ttk.Button(controls, text="Abrir Rerun", command=lambda: self.open_view(reopen=True)).pack(side="left", padx=4)
        ttk.Label(controls, text="Velocidad virtual:").pack(side="left", padx=(15, 4))
        self.speed = tk.StringVar(value="20")
        ttk.Spinbox(controls, from_=1, to=60, textvariable=self.speed, width=5).pack(side="left")
        ttk.Label(controls, text="°/s").pack(side="left")
        self.progress = tk.DoubleVar(value=0)
        ttk.Scale(content, from_=0, to=100, variable=self.progress, command=self.seek).pack(fill="x", pady=5)
        self.summary = tk.StringVar(value="Faltan las dos poses. Margen mínimo: más de 5 mm sobre el piso virtual.")
        ttk.Label(content, textvariable=self.summary, wraplength=900, font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=5)
        self.height = tk.StringVar(value="Altura de la garra: —")
        ttk.Label(content, textvariable=self.height, font=("Segoe UI", 15, "bold")).pack(anchor="w", pady=5)
        self.current = tk.StringVar(value="Ángulos virtuales: —")
        ttk.Label(content, textvariable=self.current, wraplength=900).pack(anchor="w")
        self.status = tk.StringVar(value="La captura no activa torque. Acomodá el brazo con los controles habituales.")
        ttk.Label(content, textvariable=self.status, wraplength=900).pack(anchor="w", pady=10)
        ttk.Label(content, text="Esta prueba verifica garra contra piso en el modelo 3D. No comprueba choques con "
                  "el propio brazo, el cubo, el recipiente u otros objetos. No habilita ejecución física.",
                  wraplength=900, foreground="#705000").pack(anchor="w", pady=5)
        self.poll_timer = root.after(50, self.poll)
        if auto_open:
            self.open_timer = root.after(250, self.open_view)

    def stop_playback(self):
        if self.play_timer is not None:
            self.root.after_cancel(self.play_timer)
            self.play_timer = None

    def invalidate(self):
        self.stop_playback()
        self.cancel.set()
        self.revision += 1
        self.plan = None
        if hasattr(self, "summary"):
            self.summary.set("Poses modificadas: calculá nuevamente. Margen: más de 5 mm.")

    def fill_pose(self, key, pose):
        pose = validate_pose(pose, self.limits)
        for i, value in pose.items():
            self.fields[key][i].set(format(value, ".10g"))
        self.render(pose)

    def capture(self, key, source):
        try:
            provider = self.real_pose_provider if source == "real" else self.virtual_pose_provider
            if provider is None:
                raise ValueError("No hay una fuente de poses disponible.")
            self.fill_pose(key, dict(provider()))
            self.status.set(f"{'Inicio' if key == 'start' else 'Final'} capturado "
                            f"{'del brazo (solo lectura)' if source == 'real' else 'del simulador'}.")
        except Exception as exc:
            self.status.set(f"No se pudo capturar: {exc}")

    def read_pose(self, key):
        try:
            return validate_pose({i: float(v.get().strip().replace(",", "."))
                                  for i, v in self.fields[key].items()}, self.limits)
        except ValueError as exc:
            raise ValueError(f"{'Inicio' if key == 'start' else 'Final'}: completá los seis ángulos. {exc}") from exc

    def calculate(self):
        self.invalidate()
        try:
            start, goal = self.read_pose("start"), self.read_pose("goal")
        except ValueError as exc:
            self.status.set(str(exc))
            return
        self.cancel = threading.Event()
        revision, cancel = self.revision, self.cancel
        self.summary.set("Calculando recorrido y comprobando la altura entre poses…")
        self.status.set("Se probará el recorrido directo y, si hace falta, una elevación intermedia.")

        def work():
            try:
                result = FloorPathPlanner(self.geometry, self.limits, cancel=cancel).plan(start, goal)
                self.results.put((revision, result, None))
            except Exception as exc:
                self.results.put((revision, None, str(exc)))
        threading.Thread(target=work, daemon=True).start()

    def poll(self):
        self.poll_timer = None
        while not self.results.empty():
            revision, result, error = self.results.get_nowait()
            if revision != self.revision:
                continue
            if error:
                self.summary.set("No hay recorrido verificado para reproducir.")
                self.status.set(error)
            else:
                self.plan = result
                self.summary.set(f"{result.kind} · {max(0, len(result.waypoints)-1)} tramos · "
                                 f"altura garantizada por el cálculo: ≥ {result.minimum_bound_mm:.2f} mm.")
                self.play()
        if not self.closed:
            self.poll_timer = self.root.after(50, self.poll)

    def open_view(self, *, reopen=False):
        if self.open_timer is not None:
            self.root.after_cancel(self.open_timer)
            self.open_timer = None
        if reopen and self.view.available:
            try:
                self.view.recording.spawn(hide_welcome_screen=True)
            except Exception as exc:
                self.status.set(f"No se pudo abrir Rerun: {exc}")
                return
        else:
            self.view.open()
        if not self.view.available:
            self.status.set(f"No se pudo abrir Rerun: {self.view.error}")
            return
        if self.last_pose is not None:
            self.render(self.last_pose)

    def play(self):
        self.stop_playback()
        if self.plan is None:
            self.status.set("Primero capturá las dos poses y calculá el recorrido.")
            return
        try:
            speed = float(self.speed.get().replace(",", "."))
            if not 1 <= speed <= 60:
                raise ValueError()
        except ValueError:
            self.status.set("Elegí una velocidad virtual entre 1 y 60 °/s.")
            return
        self.open_view()
        if not self.view.available:
            return
        self.play_speed = speed
        self.started_at = time.monotonic()
        self.status.set("Reproduciendo únicamente en Rerun. El brazo real no recibe órdenes.")
        self.animate()

    def animate(self):
        self.play_timer = None
        if self.plan is None or self.closed:
            return
        duration = self.plan.duration(self.play_speed)
        self.elapsed = min(time.monotonic() - self.started_at, duration)
        if not self.render(self.plan.pose_at(self.elapsed, self.play_speed)):
            return
        self.progress.set(100 * self.elapsed / duration if duration else 100)
        if self.elapsed < duration:
            self.play_timer = self.root.after(40, self.animate)
        else:
            self.status.set("Simulación finalizada en la pose final. Ningún movimiento enviado al brazo.")

    def seek(self, value):
        if self.plan is None:
            return
        self.stop_playback()
        self.elapsed = self.plan.duration(self.play_speed) * float(value) / 100
        self.render(self.plan.pose_at(self.elapsed, self.play_speed))
        self.status.set("Recorrido virtual pausado en el punto seleccionado.")

    def render(self, pose):
        self.last_pose = dict(pose)
        result = self.geometry.clearance(pose)
        self.height.set(f"Altura de la garra: {result['height_mm']:.1f} mm")
        self.current.set(" · ".join(f"M{i}: {pose[i]:.1f}°" for i in range(1, 7)))
        if self.view.available:
            try:
                self.view.log_pose(pose)
            except Exception as exc:
                self.stop_playback()
                self.status.set(f"Error de visualización: {exc}")
                return False
        return True

    def stop(self):
        self.stop_playback()
        self.cancel.set()
        self.revision += 1
        self.status.set("Simulación/cálculo detenido. No se envió ninguna orden al robot.")

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop()
        for timer in (self.poll_timer, self.open_timer):
            if timer is not None:
                self.root.after_cancel(timer)
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    TrajectoryApp(root)
    root.mainloop()
