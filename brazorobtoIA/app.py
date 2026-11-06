from __future__ import annotations

import json
import math
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import cv2
from PIL import Image, ImageTk
from serial.tools import list_ports

from brazoia.capture import CameraStream, discover_camera_index, validate_frame_pair
from brazoia.config import (
    INITIAL_POSE,
    KEYFRAME_STAGES,
    REFERENCE_POSE,
    STAGE_INSTRUCTIONS,
    STAGE_LABELS,
    detection_parameters,
    load_config,
    validate_stage_gripper,
)
from brazoia.dataset import DemonstrationSession, load_successful_demonstrations
from brazoia.execution import MotionExecutor, MotionProposal, SupervisedExecution
from brazoia.planner import MotionGroup, build_motion_plan, edge_safe, floor_escape, planning_start
from brazoia.policy import PILOT_DEMONSTRATION_LIMIT, StagePolicy, train_policy
from brazoia.rerun_view import RerunArmView
from brazoia.robot import RobotEvent, RobotSerial
from brazoia.runlog import ExecutionLog
from brazoia.vision import detect_scene, draw_observation, observation_features, verify_cube_in_container

# Avisos del ejecutor que se guardan en el registro de la ejecución (el resto es progreso).
RUN_LOG_WARNINGS = ("Llegada con desvío", "Garra abierta a")
# Del modelo aprendido solo lo que lo identifica; su metadata trae las predicciones de validación.
# Vista previa de cada cámara: las dos lado a lado y las pestañas debajo, sin desplazar.
PREVIEW_SIZE = (480, 272)
POLICY_SUMMARY_KEYS = ("tipo", "calibracion", "demonstrations", "mean_absolute_error_degrees", "validation_passed")


def build_reference_pose_plan(
    current: dict[int, float], limits: dict[int, tuple[float, float]],
    target: dict[int, float] | None = None,
):
    """Planifica M1–M5 y tolera pequeños desvíos del encoder en los límites."""
    target = dict(REFERENCE_POSE if target is None else target)
    if set(target) != set(range(1, 6)):
        raise ValueError("La pose debe contener únicamente M1–M5.")
    missing = [motor_id for motor_id in REFERENCE_POSE if motor_id not in current]
    if missing:
        raise ValueError("Falta telemetría de " + ", ".join(f"M{i}" for i in missing) + ".")

    planning_current = planning_start({motor_id: current[motor_id] for motor_id in range(1, 6)}, limits)
    planning_current.update({key: value for key, value in current.items() if key not in planning_current})
    return tuple(build_motion_plan(planning_current, target, limits))


def manual_motor_proposal(snapshot, motor_id, target, limits, maximum_age, now):
    """Movimiento directo de un solo motor, sin desvíos de otros ejes."""
    if motor_id not in limits:
        raise ValueError("Motor inválido; usá M1–M6.")
    target = float(str(target).replace(",", "."))
    low, high = limits[motor_id]
    if not math.isfinite(target) or not low <= target <= high:
        raise ValueError(f"M{motor_id} admite {low:g}..{high:g}°.")
    if snapshot.fault:
        raise ValueError(f"Fallo activo: {snapshot.fault}")
    if snapshot.torque_enabled is not True:
        raise ValueError("Activá Torque ON antes de mover manualmente.")
    if snapshot.gripper_state == "closing":
        raise ValueError("Esperá a que termine el cierre por presión de la garra.")
    required = {motor_id} | ({2, 3, 4} if motor_id in (2, 3, 4) else set())
    for motor in required:
        value = snapshot.positions.get(motor)
        age = now - snapshot.received_at.get(motor, -math.inf)
        if value is None or not math.isfinite(value) or age < 0 or age > maximum_age:
            raise ValueError(f"Falta telemetría reciente de M{motor}.")
    if motor_id in (2, 3, 4):
        start = tuple(snapshot.positions[motor] for motor in (2, 3, 4))
        end = tuple(target if motor == motor_id else snapshot.positions[motor] for motor in (2, 3, 4))
        m6 = snapshot.positions.get(6)
        if not edge_safe(start, end, m6=m6, escape_from=floor_escape(start, m6)):
            raise ValueError("El movimiento directo no pasa las reglas de geometría. No se moverán otros motores para compensar.")
    return MotionProposal(None, "angulo_manual", f"Mover M{motor_id}", {motor_id: target},
                          (MotionGroup(((motor_id, target),)),), None)


class _Rejected(RuntimeError):
    """La tarea autónoma no arranca o no sigue: no se movió nada."""


class App:
    def __init__(self, root: tk.Tk, *, gemelo: bool = False):
        self.root = root
        self.config = load_config()
        # Gemelo digital: el brazo y las cámaras de MuJoCo detrás del mismo puerto serie
        # y la misma interfaz de cámara (brazoia/gemelo.py). Nunca abre COM ni DirectShow.
        self.twin = None
        self.twin_viewer = None
        self.auto_stop = threading.Event()        # Detener/Emergencia para la tarea sin confirmación
        self.twin_scene_used = False              # la tarea ya se hizo en esta escena
        self.twin_calibration_ready = threading.Event()
        if gemelo:
            from brazoia.gemelo import DigitalTwin, twin_config

            self.config = twin_config(self.config)
            self.twin = DigitalTwin(self.config)
        self.cameras: list[CameraStream] = []
        self.snapshots = []
        self.observations = []
        self.latest_scene = ([], [])               # (cuadros, detecciones) del mismo tick
        self.emergency_active = False
        self.preview_images = [None, None]
        self.robot: RobotSerial | None = None
        self.demonstration: DemonstrationSession | None = None
        self.policy: StagePolicy | None = None
        self.execution: SupervisedExecution | None = None
        self.executor: MotionExecutor | None = None
        self.manual_executor: MotionExecutor | None = None
        self.execution_log: ExecutionLog | None = None
        self.logged_scene = None                  # escena de Inicio ya guardada en el registro
        self.automatic_verification: bool | None = None
        self.execution_busy = False
        self.camera_settings_locked = True
        self.rerun = RerunArmView(self.config.root / "assets")
        self.closing = False
        self.last_rerun_update = 0.0
        self.worker_messages: queue.Queue[tuple[str, object]] = queue.Queue()

        self.status = tk.StringVar(value="Abrí las cámaras y conectá el ESP32.")
        self.connection_status = tk.StringVar(value="Robot desconectado")
        self.torque_status = tk.StringVar(value="Torque: desconocido")
        self.gripper_status = tk.StringVar(value="Garra: sin estado")
        self.demo_status = tk.StringVar(value="Sin demostración activa")
        self.training_status = tk.StringVar(value="Modelo no cargado")
        self.auto_status = tk.StringVar(value="Modo automático detenido")
        # "aprendido": modelo entrenado con demostraciones; "geometrico": cámaras calibradas
        # y cinemática inversa del modelo 3D (brazoia/politica_geometrica.py).
        self.automatic_mode = tk.StringVar(value="geometrico" if self.twin is not None else "aprendido")
        self.geometric_model = None  # (VirtualArm, PickPlaceExpert): armarlo tarda ~1 s
        self.notes = tk.StringVar()
        self.port = tk.StringVar(value=self.config.port)
        self.rotation_vars = [
            tk.StringVar(value=f"Frontal: {self.config.cameras['frontal'].rotation_degrees}°"),
            tk.StringVar(value=f"Lateral: {self.config.cameras['lateral'].rotation_degrees}°"),
        ]
        self.camera_rotation_values = [
            self.config.cameras["frontal"].rotation_degrees,
            self.config.cameras["lateral"].rotation_degrees,
        ]
        self.camera_parameter_vars = []
        for key in ("frontal", "lateral"):
            camera = self.config.cameras[key]
            self.camera_parameter_vars.append(
                {
                    "brightness_percent": tk.DoubleVar(value=camera.brightness_percent),
                    "contrast_percent": tk.DoubleVar(value=camera.contrast_percent),
                    "saturation_percent": tk.DoubleVar(value=camera.saturation_percent),
                    "zoom_factor": tk.DoubleVar(value=camera.zoom_factor),
                }
            )
        self.camera_setting_widgets = []
        self.angle_vars = {i: tk.StringVar(value="—") for i in range(1, 7)}
        self.manual_angle_vars = {i: tk.StringVar(value="") for i in range(1, 7)}
        self.twin_status = tk.StringVar(value="")
        self.build()
        self.refresh_ports()
        if self.twin is not None:
            self.status.set("GEMELO DIGITAL: abrí las cámaras virtuales y conectá SIMULADO. El brazo real no se usa.")
            self.describe_twin_scene()
            self.run_async(self.prepare_twin_calibration)
        self.load_model_if_present()
        self.update_demo_count()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.tick)

    def build(self) -> None:
        title = "Brazo Robot IA · cubo rojo → recipiente verde lima"
        self.root.title(("GEMELO DIGITAL · " + title) if self.twin is not None else title)
        self.root.geometry("1400x920")
        try:
            self.root.state("zoomed")  # en una notebook la ventana fija no entra entera
        except tk.TclError:
            pass

        footer = ttk.Frame(self.root, padding=6)
        footer.pack(side="bottom", fill="x")
        ttk.Label(footer, textvariable=self.status).pack(side="left", fill="x", expand=True)
        tk.Button(
            footer,
            text="EMERGENCIA · TORQUE OFF",
            bg="#b42332",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            command=self.emergency,
        ).pack(side="right")

        area = ttk.Frame(self.root)
        area.pack(fill="both", expand=True)
        area.rowconfigure(0, weight=1)
        area.columnconfigure(0, weight=1)
        self.page_canvas = tk.Canvas(area, highlightthickness=0)
        self.page_canvas.grid(row=0, column=0, sticky="nsew")
        vertical = ttk.Scrollbar(area, orient="vertical", command=self.page_canvas.yview)
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(area, orient="horizontal", command=self.page_canvas.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        self.page_canvas.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        self.content = ttk.Frame(self.page_canvas)
        self.content_window = self.page_canvas.create_window(0, 0, window=self.content, anchor="nw")
        self.content.bind(
            "<Configure>",
            lambda _event: self.page_canvas.configure(scrollregion=self.page_canvas.bbox("all")),
        )
        self.page_canvas.bind(
            "<Configure>",
            lambda event: self.page_canvas.itemconfigure(
                self.content_window, width=max(event.width, self.content.winfo_reqwidth())
            ),
        )
        self.root.bind("<MouseWheel>", self.scroll_page)

        if self.twin is not None:
            twin = tk.LabelFrame(self.content, text="GEMELO DIGITAL · brazo y cámaras virtuales (MuJoCo)",
                                 padx=8, pady=6, fg="#1d4ed8")
            twin.pack(fill="x", padx=8, pady=4)
            ttk.Button(twin, text="Nueva escena", command=self.new_twin_scene).pack(side="left", padx=3)
            ttk.Button(twin, text="Ver en 3D (MuJoCo)", command=self.open_twin_viewer).pack(side="left", padx=3)
            ttk.Label(twin, textvariable=self.twin_status, wraplength=900).pack(side="left", padx=10)

        self._build_connection_bar(self.content)
        self._build_cameras(self.content)

        notebook = ttk.Notebook(self.content)
        notebook.pack(fill="x", padx=8, pady=4)
        self.notebook = notebook
        for title, builder in (
            ("Tarea automática", self._build_task_tab),
            ("Mover a mano", self._build_manual_tab),
            ("Enseñar y entrenar", self._build_teaching_tab),
            ("Cámaras y calibración", self._build_calibration_tab),
        ):
            tab = ttk.Frame(notebook, padding=8)
            notebook.add(tab, text=f"  {title}  ")
            builder(tab)

        self.log_box = tk.Text(self.content, height=8, state="disabled", font=("Consolas", 9))
        self.log_box.pack(fill="both", expand=True, padx=8, pady=4)

    def _build_connection_bar(self, parent) -> None:
        """Conexiones y telemetría: siempre a la vista."""
        top = ttk.Frame(parent, padding=(8, 6, 8, 2))
        top.pack(fill="x")
        ttk.Button(top, text="Abrir / reabrir cámaras", command=self.open_cameras).pack(side="left")
        ttk.Label(top, text="Puerto:").pack(side="left", padx=(14, 2))
        self.port_combo = ttk.Combobox(top, textvariable=self.port, width=10)
        self.port_combo.pack(side="left")
        ttk.Button(top, text="Actualizar", command=self.refresh_ports).pack(side="left", padx=3)
        ttk.Button(top, text="Conectar robot", command=self.connect_robot).pack(side="left", padx=6)
        ttk.Button(top, text="Desconectar", command=self.disconnect_robot).pack(side="left")
        ttk.Button(top, text="Abrir Rerun", command=self.open_rerun).pack(side="left", padx=8)
        ttk.Label(top, textvariable=self.connection_status).pack(side="left", padx=8)

        readings = ttk.Frame(parent, padding=(8, 2, 8, 4))
        readings.pack(fill="x")
        for motor_id in range(1, 7):
            low, high = self.config.limits[motor_id]
            cell = ttk.Frame(readings)
            cell.pack(side="left", padx=(0, 18))
            ttk.Label(cell, text=f"M{motor_id}", font=("Segoe UI", 9, "bold")).pack(side="left")
            ttk.Label(cell, textvariable=self.angle_vars[motor_id], width=7, anchor="e",
                      font=("Segoe UI", 14, "bold")).pack(side="left", padx=(4, 2))
            ttk.Label(cell, text=f"{low:.0f}..{high:.0f}°", foreground="#666").pack(side="left")
        ttk.Label(readings, textvariable=self.gripper_status).pack(side="right")
        ttk.Label(readings, textvariable=self.torque_status).pack(side="right", padx=14)

    def _build_cameras(self, parent) -> None:
        views = ttk.Frame(parent, padding=(8, 2))
        views.pack(fill="x")
        self.camera_labels = []
        for index, title in enumerate(("FRONTAL", "LATERAL")):
            panel = ttk.Frame(views)
            panel.pack(side="left", fill="both", expand=True, padx=3)
            heading = ttk.Frame(panel)
            heading.pack()
            ttk.Label(heading, text=title, font=("Segoe UI", 10, "bold")).pack(side="left", padx=6)
            ttk.Button(
                heading,
                textvariable=self.rotation_vars[index],
                command=lambda camera_index=index: self.rotate_camera(camera_index),
            ).pack(side="left")
            label = ttk.Label(panel, text="Cámara cerrada", anchor="center")
            label.pack(fill="both", expand=True)
            self.camera_labels.append(label)

    def _build_task_tab(self, tab) -> None:
        main = ttk.Frame(tab)
        main.pack(fill="x")
        modes = ttk.Frame(main)
        modes.pack(side="left", padx=(0, 14))
        ttk.Label(modes, text="Cómo ubica el cubo:").pack(anchor="w")
        ttk.Radiobutton(modes, text="Geométrico (cámaras calibradas)", value="geometrico",
                        variable=self.automatic_mode).pack(anchor="w")
        ttk.Radiobutton(modes, text="Modelo aprendido", value="aprendido",
                        variable=self.automatic_mode).pack(anchor="w")
        tk.Button(
            main, text="▶  HACER LA TAREA SOLA", command=self.run_autonomous_task,
            bg="#15803d", fg="white", activebackground="#166534", activeforeground="white",
            font=("Segoe UI", 13, "bold"), padx=18, pady=8,
        ).pack(side="left", padx=4)
        tk.Button(
            main, text="■  Detener", command=self.stop_automatic,
            font=("Segoe UI", 12, "bold"), padx=14, pady=8,
        ).pack(side="left", padx=8)
        ttk.Label(
            main,
            text="Las 12 etapas seguidas, sin confirmar cada una.\nDetener o EMERGENCIA la cortan.",
            foreground="#555",
        ).pack(side="left", padx=8)

        ttk.Label(tab, textvariable=self.auto_status, wraplength=1250,
                  font=("Segoe UI", 10, "bold")).pack(fill="x", pady=(8, 6))

        lower = ttk.Frame(tab)
        lower.pack(fill="x")
        step = ttk.LabelFrame(lower, text="Paso a paso · confirmás cada etapa", padding=6)
        step.pack(side="left", padx=(0, 8))
        ttk.Button(step, text="Iniciar", command=self.start_automatic).pack(side="left", padx=3)
        ttk.Button(step, text="Preparar etapa", command=self.prepare_next_stage).pack(side="left", padx=3)
        ttk.Button(step, text="CONFIRMAR Y MOVER", command=self.confirm_stage).pack(side="left", padx=3)

        results = ttk.LabelFrame(lower, text="Resultado real (lo marcás vos al final)", padding=6)
        results.pack(side="left", fill="x", expand=True)
        for label, result in (
            ("Correcto", "success"),
            ("Falló agarre", "failed_grasp"),
            ("Falló depósito", "failed_place"),
            ("Detenido", "interrupted"),
        ):
            ttk.Button(results, text=label, command=lambda value=result: self.finish_execution(value)).pack(side="left", padx=3)
        ttk.Label(results, text="Nota:").pack(side="left", padx=(12, 3))
        ttk.Entry(results, textvariable=self.notes).pack(side="left", fill="x", expand=True)
        ttk.Button(results, text="Abrir carpeta de datos", command=self.open_data_folder).pack(side="left", padx=(8, 0))

    def _build_manual_tab(self, tab) -> None:
        poses = ttk.Frame(tab)
        poses.pack(fill="x")
        ttk.Button(
            poses,
            text="Pose inicial: 180°, 180°, 90°, 180°, 263° (M6 sin cambios)",
            command=lambda: self.move_to_reference_pose(INITIAL_POSE, "Pose inicial"),
        ).pack(side="left")
        ttk.Button(
            poses,
            text="Llevar M1–M5 a 180°, 90°, 270°, 180°, 270° (M6 sin cambios)",
            command=self.move_to_reference_pose,
        ).pack(side="left", padx=(8, 0))

        manual = ttk.LabelFrame(tab, text="Ángulos manuales · grados lógicos", padding=6)
        manual.pack(fill="x", pady=6)
        for motor_id in range(1, 7):
            panel = ttk.Frame(manual)
            panel.grid(row=0, column=motor_id - 1, padx=8, sticky="ew")
            manual.columnconfigure(motor_id - 1, weight=1)
            low, high = self.config.limits[motor_id]
            ttk.Label(panel, text=f"M{motor_id} · {low:g}..{high:g}°").pack()
            ttk.Spinbox(panel, from_=low, to=high, increment=1, width=9,
                        textvariable=self.manual_angle_vars[motor_id]).pack(pady=3)
            ttk.Button(panel, text=f"Mover M{motor_id}",
                       command=lambda motor_id=motor_id: self.move_manual_motor(motor_id)).pack()
        ttk.Button(manual, text="Copiar ángulos actuales", command=self.copy_manual_angles).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(manual, text="Escribir no mueve el brazo: pulsá Mover y confirmá. M6 por ángulo NO es cierre por presión.").grid(
            row=1, column=2, columnspan=4, sticky="w", pady=(6, 0))

        controls = ttk.LabelFrame(tab, text="Torque y garra", padding=6)
        controls.pack(fill="x")
        ttk.Button(
            controls,
            text="Torque ON",
            # Compatible con firmware anterior y nuevo. Encender M6 no altera
            # su posición; la separación crítica es al apagar para enseñanza.
            command=lambda: self.send_robot("torque on"),
        ).pack(side="left", padx=3)
        ttk.Button(
            controls,
            text="Torque brazo OFF (M1–M5)",
            command=lambda: self.send_robot("arm torque off"),
        ).pack(side="left", padx=3)
        ttk.Button(controls, text="Abrir garra", command=lambda: self.send_robot("gripper open")).pack(side="left", padx=(12, 3))
        ttk.Button(controls, text="Cerrar garra por presión", command=lambda: self.send_robot("gripper close")).pack(side="left", padx=3)
        ttk.Button(controls, text="Parar garra", command=lambda: self.send_robot("gripper stop")).pack(side="left", padx=3)
        ttk.Button(controls, text="Leer posiciones", command=lambda: self.send_robot("posall")).pack(side="left", padx=(12, 3))

    def _build_teaching_tab(self, tab) -> None:
        teaching = ttk.LabelFrame(tab, text=f"Demostración · {len(KEYFRAME_STAGES)} poses", padding=6)
        teaching.pack(fill="x")
        ttk.Button(teaching, text="Nueva demostración", command=self.start_demonstration).pack(side="left", padx=3)
        ttk.Button(teaching, text="Tomar foto", command=self.take_keyframe).pack(side="left", padx=3)
        ttk.Button(teaching, text="Reintentar última foto", command=self.retry_last_keyframe).pack(side="left", padx=3)
        ttk.Button(teaching, text="Guardar como exitosa", command=lambda: self.finish_demonstration("success")).pack(side="left", padx=3)
        ttk.Button(teaching, text="Interrumpir", command=lambda: self.finish_demonstration("interrupted")).pack(side="left", padx=3)
        ttk.Label(teaching, textvariable=self.demo_status, wraplength=650).pack(side="left", padx=10)

        training = ttk.LabelFrame(tab, text="Modelo aprendido", padding=6)
        training.pack(fill="x", pady=6)
        self.demo_count = tk.StringVar(value="0 demostraciones exitosas")
        ttk.Label(training, textvariable=self.demo_count).pack(side="left")
        ttk.Button(training, text="Entrenar / actualizar modelo", command=self.train_model).pack(side="left", padx=12)
        ttk.Button(training, text="Probar modelo OFFLINE", command=self.open_offline_test).pack(side="left", padx=4)
        ttk.Button(training, text="Simulador manual · Rerun", command=self.open_manual_simulator).pack(side="left", padx=4)
        ttk.Label(training, textvariable=self.training_status).pack(side="left", padx=8)

    def _build_calibration_tab(self, tab) -> None:
        tools = ttk.LabelFrame(tab, text="Calibración y simulación", padding=6)
        tools.pack(fill="x")
        ttk.Button(tools, text="Calibración · cámaras", command=self.open_camera_calibration).pack(side="left", padx=3)
        ttk.Button(tools, text="Calibración · toques de mesa", command=self.open_calibration).pack(side="left", padx=3)
        ttk.Button(tools, text="Trayecto desde brazo · simulación", command=self.open_trajectory_simulator).pack(side="left", padx=3)

        settings = ttk.LabelFrame(tab, text="Ajustes de imagen en vivo", padding=6)
        settings.pack(fill="x", pady=6)
        parameter_definitions = (
            ("Brillo %", "brightness_percent", -100.0, 100.0, 5.0),
            ("Contraste %", "contrast_percent", -100.0, 200.0, 5.0),
            ("Saturación %", "saturation_percent", -100.0, 200.0, 5.0),
            ("Zoom", "zoom_factor", 1.0, 3.0, 0.1),
        )
        for camera_index, title in enumerate(("Primera · frontal", "Segunda · lateral")):
            camera_panel = ttk.LabelFrame(settings, text=title, padding=4)
            camera_panel.pack(side="left", fill="x", expand=True, padx=4)
            for label, key, minimum, maximum, resolution in parameter_definitions:
                control = tk.Scale(
                    camera_panel,
                    label=label,
                    variable=self.camera_parameter_vars[camera_index][key],
                    from_=minimum,
                    to=maximum,
                    resolution=resolution,
                    orient="horizontal",
                    length=130,
                    command=lambda _value, index=camera_index: self.apply_camera_settings(index),
                )
                control.pack(side="left", fill="x", expand=True, padx=2)
                self.camera_setting_widgets.append(control)
        actions = ttk.Frame(settings)
        actions.pack(side="left", padx=6)
        neutral_button = ttk.Button(
            actions, text="Valores neutros", command=self.neutral_camera_settings
        )
        neutral_button.pack(fill="x", pady=2)
        save_button = ttk.Button(
            actions, text="Guardar ajustes", command=self.save_camera_settings
        )
        save_button.pack(fill="x", pady=2)
        self.camera_setting_widgets.extend((neutral_button, save_button))
        self.camera_edit_button = ttk.Button(
            actions, text="Editar ajustes", command=self.toggle_camera_settings_lock
        )
        self.camera_edit_button.pack(fill="x", pady=(8, 2))
        self.set_camera_controls_enabled(True)

    def scroll_page(self, event):
        if event.widget.winfo_class() == "Text":
            return
        if event.delta:
            self.page_canvas.yview_scroll(-int(event.delta / 120) * 3, "units")
            return "break"

    def log(self, text: str) -> None:
        self.log_box.configure(state="normal")
        self.log_box.insert("end", text + "\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def run_async(self, operation, *, success=None, failure=None) -> None:
        def worker():
            try:
                result = operation()
            except Exception as exc:
                self.worker_messages.put(("failure", (failure, exc)))
            else:
                self.worker_messages.put(("success", (success, result)))

        threading.Thread(target=worker, daemon=True).start()

    def refresh_ports(self) -> None:
        if self.twin is not None:
            from brazoia.gemelo import VIRTUAL_PORT

            self.port_combo["values"] = [VIRTUAL_PORT]
            self.port.set(VIRTUAL_PORT)
            self.log(f"Gemelo digital: único puerto {VIRTUAL_PORT} (firmware virtual).")
            return
        discovered = list(list_ports.comports())
        ports = [item.device for item in discovered]
        self.port_combo["values"] = ports
        preferred = self.port.get().strip()
        if preferred not in ports:
            usb_ports = [
                item.device
                for item in discovered
                if any(
                    marker in f"{item.description} {item.manufacturer or ''}".lower()
                    for marker in ("usb", "ch340", "cp210", "ftdi")
                )
            ]
            if len(usb_ports) == 1:
                self.port.set(usb_ports[0])
                self.log(f"Puerto configurado {preferred} ausente; seleccionado {usb_ports[0]}.")
        self.log("Puertos: " + (", ".join(ports) if ports else "ninguno"))

    def camera_parameters(self, index: int) -> dict[str, float]:
        return {
            key: float(variable.get())
            for key, variable in self.camera_parameter_vars[index].items()
        }

    def apply_camera_settings(self, index: int) -> None:
        if self.demonstration is not None or self.execution is not None:
            return
        if len(self.cameras) == 2:
            self.cameras[index].set_image_parameters(**self.camera_parameters(index))

    def set_camera_controls_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled and not self.camera_settings_locked else "disabled"
        for widget in self.camera_setting_widgets:
            widget.configure(state=state)
        self.camera_edit_button.configure(state="normal" if enabled else "disabled")

    def toggle_camera_settings_lock(self) -> None:
        if self.demonstration is not None or self.execution is not None:
            self.status.set("Terminá la demostración o tarea antes de editar las cámaras.")
            return
        self.camera_settings_locked = not self.camera_settings_locked
        self.camera_edit_button.configure(
            text="Editar ajustes" if self.camera_settings_locked else "Bloquear ajustes"
        )
        self.set_camera_controls_enabled(True)
        self.status.set(
            "Ajustes bloqueados."
            if self.camera_settings_locked
            else "Edición de cámaras habilitada."
        )

    def neutral_camera_settings(self) -> None:
        if self.demonstration is not None or self.execution is not None:
            self.status.set("Terminá la demostración o tarea antes de cambiar las cámaras.")
            return
        for camera_index in range(2):
            values = self.camera_parameter_vars[camera_index]
            values["brightness_percent"].set(0.0)
            values["contrast_percent"].set(0.0)
            values["saturation_percent"].set(0.0)
            values["zoom_factor"].set(1.0)
            self.apply_camera_settings(camera_index)
        self.status.set("Ajustes neutros aplicados. Pulsá Guardar si querés conservarlos.")

    def save_camera_settings(self) -> None:
        if self.twin is not None:
            self.status.set("En el gemelo digital los ajustes de cámara no se guardan en config.json.")
            return
        if self.demonstration is not None or self.execution is not None:
            self.status.set("Terminá la demostración o tarea antes de guardar ajustes.")
            return
        path = self.config.root / "config.json"
        raw = json.loads(path.read_text(encoding="utf-8"))
        for index, key in enumerate(("frontal", "lateral")):
            raw["cameras"][key].update(self.camera_parameters(index))
            raw["cameras"][key]["rotation_degrees"] = self.camera_rotation_values[index]
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(raw, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        temporary.replace(path)
        self.camera_settings_locked = True
        self.camera_edit_button.configure(text="Editar ajustes")
        self.set_camera_controls_enabled(True)
        self.status.set("Ajustes de ambas cámaras guardados en config.json.")
        self.log("Ajustes de cámaras guardados.")

    def open_cameras(self) -> None:
        if self.demonstration is not None or self.execution is not None:
            self.status.set("Terminá la demostración o tarea antes de reabrir cámaras.")
            return
        previous_cameras = list(self.cameras)
        for camera in previous_cameras:
            camera.close()
        for camera in previous_cameras:
            camera.thread.join(timeout=3.0)
        if any(camera.thread.is_alive() for camera in previous_cameras):
            self.status.set("Una cámara anterior no se liberó. Cerrá otra aplicación que pueda estar usándola.")
            return
        self.cameras = []
        if self.twin is not None:
            from brazoia.gemelo import VirtualCameraStream

            self.cameras = [
                VirtualCameraStream(self.twin, "frontal", 0, self.camera_rotation_values[0], **self.camera_parameters(0)),
                VirtualCameraStream(self.twin, "lateral", 1, self.camera_rotation_values[1], **self.camera_parameters(1)),
            ]
            self.status.set("Cámaras virtuales del gemelo abiertas.")
            return
        front_config = self.config.cameras["frontal"]
        side_config = self.config.cameras["lateral"]
        front_index = discover_camera_index(front_config.name, front_config.fallback_index)
        side_index = discover_camera_index(side_config.name, side_config.fallback_index)
        if front_index == side_index:
            self.status.set("Las dos cámaras resolvieron al mismo índice.")
            return
        self.cameras = [
            CameraStream(
                front_index,
                self.camera_rotation_values[0],
                **self.camera_parameters(0),
            ),
            CameraStream(
                side_index,
                self.camera_rotation_values[1],
                **self.camera_parameters(1),
            ),
        ]
        self.status.set(f"Abriendo frontal {front_index} y lateral {side_index}...")

    def rotate_camera(self, index: int) -> None:
        if self.demonstration is not None or self.execution is not None:
            self.status.set("No se puede girar una cámara durante una demostración o tarea.")
            return
        if len(self.cameras) != 2:
            self.status.set("Abrí primero las cámaras.")
            return
        value = self.cameras[index].toggle_180()
        self.camera_rotation_values[index] = value
        name = "Frontal" if index == 0 else "Lateral"
        self.rotation_vars[index].set(f"{name}: {value}°")

    def connect_robot(self) -> None:
        if self.robot is not None and self.robot.connected:
            return
        if self.twin is not None:
            robot = RobotSerial(self.port.get().strip(), self.config.baud, opener=self.twin.open_port)
        else:
            robot = RobotSerial(self.port.get().strip(), self.config.baud)
        self.robot = robot
        self.connection_status.set("Conectando...")

        def connected(_result):
            if self.robot is robot:
                self.connection_status.set(f"Conectado · {robot.port}")

        def connection_failed(exc):
            if self.robot is robot:
                self.connection_status.set(f"Error: {exc}")

        self.run_async(
            robot.connect,
            success=connected,
            failure=connection_failed,
        )

    def disconnect_robot(self) -> None:
        if self.execution_busy:
            self.status.set("Usá Emergencia y esperá la parada antes de desconectar.")
            return
        robot, self.robot = self.robot, None
        if robot is not None:
            self.run_async(robot.disconnect)
        self.connection_status.set("Robot desconectado")

    def send_robot(self, command: str) -> None:
        if self.robot is None:
            self.status.set("Conectá primero el robot.")
            return
        if self.execution_busy:
            self.status.set("Hay un movimiento automático en curso; usá Emergencia si necesitás detenerlo.")
            return
        try:
            self.robot.send(command)
        except Exception as exc:
            self.status.set(str(exc))

    def copy_manual_angles(self) -> None:
        if self.robot is None or not self.robot.connected:
            self.status.set("Conectá primero el robot.")
            return
        snapshot = self.robot.snapshot()
        now = time.monotonic()
        for motor_id, variable in self.manual_angle_vars.items():
            value = snapshot.positions.get(motor_id)
            fresh = now - snapshot.received_at.get(motor_id, -math.inf) <= self.config.capture.maximum_telemetry_age_seconds
            variable.set(f"{value:.1f}" if value is not None and math.isfinite(value) and fresh else "")
        self.status.set("Ángulos recientes copiados; no se envió ninguna orden.")

    def move_manual_motor(self, motor_id: int) -> None:
        robot = self.robot
        if robot is None or not robot.connected:
            self.status.set("Conectá primero el robot.")
            return
        if self.execution_busy or self.execution is not None:
            self.status.set("Terminá el movimiento o la tarea automática antes de mover manualmente.")
            return
        try:
            target = float(self.manual_angle_vars[motor_id].get().replace(",", "."))
            def prepare():
                return manual_motor_proposal(robot.snapshot(), motor_id, target, self.config.limits,
                                             self.config.capture.maximum_telemetry_age_seconds, time.monotonic())
            prepare()
        except (ValueError, KeyError) as exc:
            self.status.set(f"Ángulo manual rechazado: {exc}")
            return
        warning = "\nM6 irá por posición, sin cierre por presión. No lo uses para apretar un objeto." if motor_id == 6 else ""
        if not messagebox.askyesno("Confirmar movimiento manual",
                                  f"Mover solamente M{motor_id} a {target:g}°. Los otros motores no recibirán órdenes."
                                  f"{warning}\n\n¿Continuar?"):
            self.status.set("Movimiento cancelado; no se envió ninguna orden.")
            return
        # La confirmación puede permanecer abierta: validar otra vez la pose real.
        if self.robot is not robot or not robot.connected or self.execution_busy or self.execution is not None:
            self.status.set("Cambió el estado del robot; volvé a solicitar el movimiento.")
            return
        executor = MotionExecutor(robot, status=lambda text: self.worker_messages.put(("log", text)))
        self.manual_executor = executor
        self.execution_busy = True
        self.status.set(f"Moviendo M{motor_id} a {target:g}°…")

        def operation():
            if executor.stop_event.is_set():
                raise RuntimeError("Movimiento cancelado.")
            executor.execute(prepare())

        def finished(_result):
            self.execution_busy = False
            self.manual_executor = None
            self.status.set(f"M{motor_id} llegó a {target:g}°.")

        def failed(exc):
            self.execution_busy = False
            self.manual_executor = None
            self.status.set(f"Movimiento manual detenido: {exc}")

        self.run_async(operation, success=finished, failure=failed)

    def move_to_reference_pose(
        self, target: dict[int, float] | None = None, label: str = "Pose fija M1–M5"
    ) -> None:
        target = dict(REFERENCE_POSE if target is None else target)
        robot = self.robot
        if robot is None or not robot.connected:
            self.status.set("Conectá primero el robot.")
            return
        if self.execution_busy:
            self.status.set("Ya hay un movimiento en curso.")
            return
        if self.execution is not None:
            self.status.set("Terminá primero la tarea automática activa.")
            return

        snapshot = robot.snapshot()
        if snapshot.fault:
            self.status.set(f"No se puede mover por fallo activo: {snapshot.fault}")
            return

        # El encoder puede quedar unas décimas fuera del límite mecánico. Para
        # planificar, admitir hasta 5° de diferencia. Los objetivos enviados
        # siguen siendo los cinco valores exactos solicitados.
        current = dict(snapshot.positions)
        try:
            groups = build_reference_pose_plan(current, self.config.limits, target)
        except Exception as exc:
            self.status.set(f"No se encontró una ruta segura: {exc}")
            return

        if not groups:
            self.status.set("M1–M5 ya están en la pose solicitada; M6 no se modificó.")
            return
        target_text = " · ".join(f"M{i}={v:.0f}°" for i, v in target.items())
        if not messagebox.askyesno(
            f"Confirmar: {label}",
            f"El brazo se moverá a:\n{target_text}\n\n"
            "M6 no recibirá órdenes de posición ni de torque.\n\n¿Confirmás el movimiento?",
        ):
            self.status.set("Movimiento cancelado; no se envió ninguna orden.")
            return

        proposal = MotionProposal(
            stage_index=None,
            stage="pose_referencia",
            label=label,
            target=target,
            groups=groups,
            gripper_action=None,
            recovery=True,
        )
        executor = MotionExecutor(
            robot, status=lambda text: self.worker_messages.put(("log", text))
        )
        self.manual_executor = executor
        self.execution_busy = True
        self.status.set("Moviendo M1–M5 a la pose solicitada; M6 permanece sin cambios.")

        def operation() -> None:
            if robot.snapshot().torque_enabled is not True:
                robot.send("arm torque on")
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    if executor.stop_event.wait(0.05):
                        raise RuntimeError("Movimiento detenido por el operador.")
                    current_snapshot = robot.snapshot()
                    if current_snapshot.fault:
                        raise RuntimeError(current_snapshot.fault)
                    if current_snapshot.torque_enabled is True:
                        break
                else:
                    raise RuntimeError("M1–M5 no confirmaron torque ON.")
            executor.execute(proposal)

        def finished(_result) -> None:
            self.execution_busy = False
            self.manual_executor = None
            self.status.set("Pose lista: M1–M5 ajustados; M6 no se modificó.")

        def failed(exc) -> None:
            self.execution_busy = False
            self.manual_executor = None
            self.status.set(f"No se completó la pose: {exc}")

        self.run_async(operation, success=finished, failure=failed)

    def open_rerun(self) -> None:
        self.run_async(
            self.rerun.open,
            success=lambda _result: self.status.set(
                "Rerun abierto." if self.rerun.available else f"Rerun no disponible: {self.rerun.error}"
            ),
        )

    def camera_configuration(self) -> list[dict[str, object]]:
        return [
            {
                "index": camera.index,
                "rotation_degrees": camera.rotation_degrees,
                "brightness_percent": camera.brightness_percent,
                "contrast_percent": camera.contrast_percent,
                "saturation_percent": camera.saturation_percent,
                "zoom_factor": camera.zoom_factor,
            }
            for camera in self.cameras
        ]

    def start_demonstration(self) -> None:
        if self.twin is not None:
            self.status.set("En el gemelo digital no se graban demostraciones: no se mezclan con las reales.")
            return
        if self.execution is not None:
            self.status.set("Terminá primero la tarea automática.")
            return
        if self.demonstration is not None:
            self.status.set("Ya hay una demostración activa.")
            return
        if len(self.cameras) != 2 or self.robot is None or not self.robot.connected:
            self.status.set("Abrí las dos cámaras y conectá el robot.")
            return
        self.demonstration = DemonstrationSession(
            self.config.demonstrations_dir,
            {
                "port": self.robot.port,
                "cameras": self.camera_configuration(),
                "limits": {str(key): list(value) for key, value in self.config.limits.items()},
                "keyframes": list(KEYFRAME_STAGES),
            },
        )
        self.set_camera_controls_enabled(False)
        self.demo_status.set(
            f"Siguiente: Inicio (0/{len(KEYFRAME_STAGES)}) · {STAGE_INSTRUCTIONS['inicio']}"
        )
        self.log(f"Nueva demostración: {self.demonstration.path}")

    def reject_keyframe(self, session: DemonstrationSession, message: str, stage: str | None = None) -> None:
        """Muestra el rechazo junto al contador, donde el operador está mirando."""
        captured = len(session.data["stages"])
        stage = stage or session.next_stage
        label = STAGE_LABELS[stage] if stage is not None else "completa"
        text = f"FOTO NO GUARDADA · {label} ({captured}/{len(KEYFRAME_STAGES)}): {message}"
        self.status.set(text)
        self.demo_status.set(text)
        self.log(text)

    def retry_last_keyframe(self) -> None:
        session = self.demonstration
        if session is None or not session.data["stages"] or session.data["status"] != "incomplete":
            self.status.set("Primero tomá una foto en una demostración activa.")
            return
        stage = session.data["stages"][-1]["stage"]
        if not messagebox.askyesno(
            "Reintentar última foto",
            f"Volver a capturar «{STAGE_LABELS[stage]}» con las dos cámaras y los ángulos actuales.\n\n"
            "Acomodá el brazo antes de confirmar. El contador no avanzará y la toma anterior quedará como respaldo.\n\n"
            "¿Reemplazar la última captura?",
        ):
            return
        if self.demonstration is not session:
            self.status.set("Cambió la demostración; volvé a intentar.")
            return
        self.take_keyframe(replace_last=True)

    def take_keyframe(self, *, replace_last: bool = False) -> None:
        session = self.demonstration
        robot = self.robot
        if session is None or robot is None:
            self.status.set("Iniciá una demostración.")
            return
        if replace_last and not session.data["stages"]:
            self.status.set("Todavía no hay una foto para reintentar.")
            return
        if session.complete and not replace_last:
            self.status.set("Ya están todas las fotos. Guardá la demostración como exitosa o interrumpida.")
            return
        stage = session.data["stages"][-1]["stage"] if replace_last else session.next_stage
        now = time.monotonic()
        problem = validate_frame_pair(
            self.snapshots,
            now,
            maximum_age=self.config.capture.maximum_camera_age_seconds,
            maximum_skew=self.config.capture.maximum_camera_skew_seconds,
        )
        snapshot = robot.snapshot()
        missing_motors = [
            motor_id for motor_id in range(1, 7) if motor_id not in snapshot.positions
        ]
        if missing_motors:
            self.reject_keyframe(
                session,
                "Faltan posiciones de "
                + ", ".join(f"M{motor_id}" for motor_id in missing_motors)
                + ". Pulsá Leer posiciones y volvé a intentar.",
                stage=stage,
            )
            return

        # El operador decide si una toma es apta. Estas comprobaciones se
        # guardan como avisos, pero nunca bloquean la foto.
        warnings: list[str] = []
        if problem:
            warnings.append(problem)
        if snapshot.fault:
            warnings.append(f"Fallo activo: {snapshot.fault}")
        if snapshot.torque_enabled is not True:
            warnings.append("Torque del brazo no confirmado como activo.")
        _stable_pose, stability_problem = robot.stable_pose(
            now,
            maximum_age=self.config.capture.maximum_telemetry_age_seconds,
            window_seconds=self.config.capture.stable_window_seconds,
            maximum_range=self.config.capture.stable_max_range_degrees,
        )
        if stability_problem:
            warnings.append(stability_problem)
        pose = {motor_id: snapshot.positions[motor_id] for motor_id in range(1, 7)}
        for motor_id, value in pose.items():
            low, high = self.config.limits[motor_id]
            if not low <= value <= high:
                warnings.append(
                    f"M{motor_id}={value:.1f}° fuera del límite {low:.0f}..{high:.0f}°.",
                )
        assert stage is not None
        gripper_problem = validate_stage_gripper(
            stage, pose[6], snapshot.gripper_state
        )
        if gripper_problem:
            warnings.append(gripper_problem)
        try:
            session.capture(
                stage,
                self.snapshots,
                pose,
                self.observations,
                torque_enabled=snapshot.torque_enabled,
                gripper_state=snapshot.gripper_state,
                warnings=warnings,
                replace_last=replace_last,
            )
        except Exception as exc:
            self.reject_keyframe(session, str(exc), stage=stage)
            return
        captured = len(session.data["stages"])
        if session.next_stage is None:
            self.demo_status.set(f"{captured}/{len(KEYFRAME_STAGES)} poses · lista para guardar")
        else:
            self.demo_status.set(
                f"Siguiente: {STAGE_LABELS[session.next_stage]} ({captured}/{len(KEYFRAME_STAGES)}) · "
                f"{STAGE_INSTRUCTIONS[session.next_stage]}"
            )
        action = "reemplazada" if replace_last else "guardada"
        saved_text = f"Pose {captured}/{len(KEYFRAME_STAGES)} {action}: {STAGE_LABELS[stage]}."
        if warnings:
            saved_text += f" Avisos no bloqueantes: {' | '.join(warnings)}"
        self.status.set(saved_text)
        self.log(saved_text)

    def finish_demonstration(self, result: str) -> None:
        session = self.demonstration
        if session is None:
            return
        try:
            session.finish(result, self.notes.get())
        except Exception as exc:
            self.status.set(str(exc))
            return
        self.log(f"Demostración {result}: {session.path.name}")
        self.demonstration = None
        self.set_camera_controls_enabled(True)
        self.demo_status.set("Sin demostración activa")
        self.update_demo_count()

    def update_demo_count(self) -> None:
        count = len(load_successful_demonstrations(self.config.demonstrations_dir))
        self.demo_count.set(f"{count} demostraciones exitosas")

    def open_offline_test(self) -> None:
        try:
            subprocess.Popen([sys.executable, str(self.config.root / "offline_app.py")], cwd=self.config.root)
            self.status.set("Prueba offline abierta en otra ventana: no conecta ni controla el brazo.")
        except OSError as exc:
            self.status.set(f"No se pudo abrir la prueba offline: {exc}")

    def open_manual_simulator(self) -> None:
        try:
            subprocess.Popen([sys.executable, str(self.config.root / "simulador_app.py")], cwd=self.config.root)
            self.status.set("Simulador manual abierto: solo mueve el modelo 3D de Rerun.")
        except OSError as exc:
            self.status.set(f"No se pudo abrir el simulador: {exc}")

    def simulation_snapshot_pose(self):
        from brazoia.simulation_path import pose_from_snapshot
        if self.robot is None or not self.robot.connected:
            raise ValueError("Conectá el ESP32 desde la aplicación principal; esta ventana no abre el puerto.")
        return pose_from_snapshot(self.robot.snapshot(), now=time.monotonic(),
                                  maximum_age=self.config.capture.maximum_telemetry_age_seconds)

    def open_trajectory_simulator(self):
        from trayecto_app import TrajectoryApp
        window = tk.Toplevel(self.root)
        window.trajectory = TrajectoryApp(window, project_root=self.config.root,
                                         real_pose_provider=self.simulation_snapshot_pose)

    def calibration_scene(self):
        """Escena actual para la calibración de cámaras; solo lee, no ordena nada."""
        if len(self.cameras) != 2:
            raise ValueError("Abrí primero las dos cámaras en la aplicación principal.")
        problem = validate_frame_pair(
            self.snapshots,
            time.monotonic(),
            maximum_age=self.config.capture.maximum_camera_age_seconds,
            maximum_skew=self.config.capture.maximum_camera_skew_seconds,
        )
        if problem:
            raise ValueError(problem)
        if len(self.observations) != 2:
            raise ValueError("Todavía no hay detección de las dos cámaras.")
        return observation_features(self.observations)

    def open_camera_calibration(self):
        if self.twin is not None:
            self.status.set("En el gemelo las cámaras virtuales se calibran solas; camaras.json es del brazo real.")
            return
        from brazoia.calibracion_camaras import settings_from_config
        from calibracion_camaras_app import CameraCalibrationApp

        window = tk.Toplevel(self.root)
        window.camera_calibration = CameraCalibrationApp(
            window,
            project_root=self.config.root,
            scene_provider=self.calibration_scene,
            pose_provider=self.simulation_snapshot_pose,
            camera_settings=settings_from_config(self.config.cameras),
        )

    def open_calibration(self):
        if self.twin is not None:
            self.status.set("La calibración por toques de mesa es del brazo real; no está disponible en el gemelo.")
            return
        from calibracion_app import CalibrationApp
        window = tk.Toplevel(self.root)
        window.calibration = CalibrationApp(window, project_root=self.config.root,
                                            real_pose_provider=self.simulation_snapshot_pose)

    def train_model(self) -> None:
        model_path = self.config.models_dir / "politica_etapas.npz"
        self.training_status.set("Entrenando...")

        def success(report):
            self.policy = StagePolicy.load(model_path)
            model_kind = (
                "Modelo PILOTO"
                if report.demonstrations < PILOT_DEMONSTRATION_LIMIT
                else "Modelo listo"
            )
            self.training_status.set(
                f"{model_kind} · {report.demonstrations} pruebas · "
                f"MAE validación cruzada {report.mean_absolute_error_degrees:.2f}° "
                f"· peor etapa {report.maximum_stage_mae_degrees:.2f}°"
            )
            self.log(str(report))

        self.run_async(
            lambda: train_policy(
                self.config.demonstrations_dir, model_path, self.config.limits
            ),
            success=success,
            failure=lambda exc: self.training_status.set(str(exc)),
        )

    def load_model_if_present(self) -> None:
        path = self.config.models_dir / "politica_etapas.npz"
        if path.exists():
            try:
                self.policy = StagePolicy.load(path)
                demonstration_count = int(self.policy.metadata.get("demonstrations", 0))
                model_kind = (
                    "Modelo PILOTO"
                    if demonstration_count < PILOT_DEMONSTRATION_LIMIT
                    else "Modelo"
                )
                self.training_status.set(
                    f"{model_kind} cargado · {demonstration_count} demostraciones"
                )
            except Exception as exc:
                self.training_status.set(f"Modelo inválido: {exc}")

    def start_automatic(self) -> None:
        if self.execution_busy:
            self.status.set("Esperá a que termine el movimiento actual.")
            return
        if self.demonstration is not None:
            self.status.set("Terminá primero la demostración activa.")
            return
        if self.execution is not None:
            self.status.set("Ya hay una tarea automática activa; terminala o marcala como detenida.")
            return
        if self.robot is None or not self.robot.connected or len(self.cameras) != 2:
            self.status.set("Abrí cámaras y conectá el robot.")
            return
        policy = self.automatic_policy()
        if policy is None:
            return
        snapshot = self.robot.snapshot()
        if 6 not in snapshot.positions:
            self.status.set("Leé las posiciones antes de iniciar; falta M6.")
            return
        gripper_problem = validate_stage_gripper(
            "inicio", snapshot.positions[6], snapshot.gripper_state
        )
        if gripper_problem:
            self.status.set("Para iniciar la tarea automática: " + gripper_problem)
            return
        self.execution = SupervisedExecution(policy, self.config.limits)
        run_log = ExecutionLog(
            self.config.root / "datos" / ("gemelo" if self.twin is not None else "") / "ejecuciones",
            {
                "port": self.robot.port,
                "gemelo": self.twin is not None,
                "modo": self.automatic_mode.get(),
                "politica": {
                    key: value
                    for key, value in (getattr(policy, "metadata", None) or {}).items()
                    if key in POLICY_SUMMARY_KEYS
                },
                "cameras": self.camera_configuration(),
                "limits": {str(key): list(value) for key, value in self.config.limits.items()},
            },
        )

        def executor_status(text: str) -> None:
            self.worker_messages.put(("log", text))
            if text.startswith(RUN_LOG_WARNINGS):
                proposal = self.execution.proposal if self.execution is not None else None
                run_log.note("aviso", text, None if proposal is None else proposal.stage)

        # Detener/EMERGENCIA cortan también una parada pedida entre dos etapas.
        self.executor = MotionExecutor(self.robot, status=executor_status, abort=self.auto_stop)
        self.execution_log = run_log
        self.logged_scene = None
        self.automatic_verification = None
        self.execution_busy = False
        self.set_camera_controls_enabled(False)
        self.auto_status.set("Tarea iniciada. Prepará la recuperación/inicio.")

    def automatic_policy(self):
        """Modelo aprendido o política geométrica, según el modo elegido."""
        if self.automatic_mode.get() == "geometrico":
            from brazoia.politica_geometrica import load_geometric_policy

            try:
                if self.geometric_model is None:
                    from brazoia.mujoco_sim import PickPlaceExpert, VirtualArm

                    arm = VirtualArm(limits=self.config.limits)
                    self.geometric_model = (arm, PickPlaceExpert(arm))
                arm, expert = self.geometric_model
                # Se relee en cada tarea: toma una calibración recién guardada.
                if self.twin is not None:
                    if not self.twin_calibration_ready.is_set():
                        raise ValueError("todavía se están calibrando las cámaras virtuales; probá en unos segundos")
                    from brazoia.gemelo import load_twin_policy

                    policy = load_twin_policy(self.config, arm=arm, expert=expert)
                else:
                    policy = load_geometric_policy(self.config, arm=arm, expert=expert)
            except (OSError, ValueError) as exc:
                self.status.set(f"Modo geométrico: {exc}")
                return None
            self.log(f"Modo geométrico con la calibración de cámaras del {policy.metadata['calibracion']}.")
            return policy
        if self.policy is None:
            self.status.set("Entrená o cargá primero un modelo.")
            return None
        return self.policy

    def prepare_next_stage(self) -> None:
        if self.execution_busy:
            self.status.set("Esperá a que termine el movimiento actual.")
            return
        if self.execution is None or self.robot is None:
            self.status.set("Iniciá primero la tarea automática.")
            return
        snapshot = self.robot.snapshot()
        if snapshot.fault:
            self.status.set(snapshot.fault)
            return
        try:
            proposal = self.execution.prepare(snapshot.positions, self.observations)
        except Exception as exc:
            self.status.set(str(exc))
            return
        if proposal.stage_index == 1 and hasattr(self.execution.policy, "describe"):
            self.log(self.execution.policy.describe(self.execution.scene_features))
        warning = ""
        if proposal.stage_index == 1:
            self._log_scene(self.execution, self.execution_log)
            problem = self._scene_problem(self.execution)
            if problem:
                warning = f"AVISO: {problem} "
                self.log("Aviso: " + problem)
        target_text = " | ".join(f"M{i}={v:.1f}°" for i, v in proposal.target.items())
        self.auto_status.set(f"{warning}PROPUESTA: {proposal.label} · {target_text}")
        self.status.set("Revisá la propuesta y Rerun. Nada se moverá hasta confirmar.")
        if self.execution_log is not None:
            self.execution_log.proposal(proposal)
        if self.rerun.available:
            proposed = dict(snapshot.positions)
            proposed.update(proposal.target)
            self.rerun.log_pose(proposed)

    def confirm_stage(self) -> None:
        if self.execution is None or self.executor is None or self.execution.proposal is None:
            self.status.set("Prepará una etapa antes de confirmar.")
            return
        if self.execution_busy:
            self.status.set("La etapa ya se está ejecutando.")
            return
        proposal = self.execution.proposal
        target_text = "\n".join(f"M{i}: {v:.1f}°" for i, v in proposal.target.items())
        gripper_text = {
            "close": "\nDespués cerrará la garra por presión.",
            "open": "\nDespués abrirá la garra.",
        }.get(proposal.gripper_action, "")
        if not messagebox.askyesno(
            "Confirmar movimiento",
            f"Etapa: {proposal.label}\n\n{target_text}{gripper_text}\n\n¿Confirmás avanzar?",
        ):
            self.status.set("Etapa no confirmada; el brazo no se movió.")
            return
        self.execution_busy = True
        # Confirmar es una orden nueva: deja atrás un Detener o una EMERGENCIA anterior.
        self.auto_stop.clear()
        self.emergency_active = False
        self.auto_status.set(f"Ejecutando: {proposal.label}")

        def success(_result):
            self.execution_busy = False
            if self.execution is None:
                return
            if self.execution_log is not None:
                self.execution_log.completed(proposal.stage, *self._robot_state())
            self.execution.mark_completed()
            if self.execution.finished:
                self.verify_final_result()
            else:
                self._run_capture(self.execution_log, proposal.stage)
                self.auto_status.set("Etapa terminada. Prepará la siguiente.")

        def failure(exc):
            self.execution_busy = False
            self.auto_status.set(f"BLOQUEADO: {exc}")
            self.log(f"Etapa {proposal.stage} bloqueada: {exc}")
            if self.execution_log is not None:
                self.execution_log.note("bloqueo", str(exc), proposal.stage)
                self._run_capture(self.execution_log, f"{proposal.stage}_bloqueo")

        self.run_async(
            lambda: self.executor.execute(proposal),
            success=success,
            failure=failure,
        )

    def stop_automatic(self) -> None:
        self.auto_stop.set()
        if self.executor is not None:
            self.executor.stop()
        self.auto_status.set("Detenido por el operador.")

    def _robot_state(self) -> tuple[dict[int, float] | None, str | None]:
        if self.robot is None:
            return None, None
        snapshot = self.robot.snapshot()
        return dict(snapshot.positions), snapshot.gripper_state

    def _run_capture(self, run_log: ExecutionLog | None, label: str) -> dict | None:
        """Fotos y ángulos de este momento en el registro; un error acá nunca frena la tarea."""
        if run_log is None:
            return None
        try:
            return run_log.capture(label, list(self.snapshots), list(self.observations), *self._robot_state())
        except Exception as exc:
            self.worker_messages.put(("log", f"No se pudo guardar la foto de {label}: {exc}"))
            return None

    def _log_scene(self, execution: SupervisedExecution, run_log: ExecutionLog | None) -> None:
        """Foto de Inicio y lo que la política sacó de ella, una vez por escena."""
        features = execution.scene_features
        if run_log is None or features is None or features is self.logged_scene:
            return
        self.logged_scene = features
        summary: dict[str, object] = {"tipo": "aprendido"}
        if hasattr(execution.policy, "summary"):
            try:
                summary = execution.policy.summary(features)
            except Exception as exc:
                summary = {"error": str(exc)}
        run_log.scene(summary)
        self._run_capture(run_log, "inicio")

    def verify_final_result(self) -> None:
        frames = [item.frame for item in self.snapshots]
        observations = list(self.observations)
        if len(observations) == 2 and len(frames) == 2 and all(frame is not None for frame in frames):
            passed, checks = verify_cube_in_container(frames, observations)
        else:
            passed, checks = False, []
        self.automatic_verification = passed
        if self.execution_log is not None:
            record = self._run_capture(self.execution_log, "verificacion_final")
            self.execution_log.verification(passed, checks, None if record is None else record["index"])
        text = "posible éxito" if passed else "no confirmado"
        cameras = ", ".join(
            f"{name} {check.state.replace('_', ' ')}" for name, check in zip(("frontal", "lateral"), checks)
        ) or "faltan imágenes"
        self.auto_status.set(
            f"Secuencia terminada · verificación visual: {text} ({cameras}). Marcá el resultado humano."
        )

    def finish_execution(self, result: str) -> None:
        if self.execution_log is None:
            self.status.set("No hay una ejecución para evaluar.")
            return
        if self.execution_busy:
            self.status.set("Primero detené el movimiento y esperá la confirmación de parada.")
            return
        if result == "success" and self.execution is not None and not self.execution.finished:
            self.status.set("No se puede marcar correcto antes de completar todas las etapas.")
            return
        self.execution_log.finish(result, self.automatic_verification, self.notes.get())
        self.log(f"Ejecución guardada: {result} · {self.execution_log.path}")
        self.execution = None
        self.executor = None
        self.execution_log = None
        self.execution_busy = False
        self.set_camera_controls_enabled(True)
        self.auto_status.set("Modo automático detenido")

    def emergency(self) -> None:
        self.auto_stop.set()
        self.emergency_active = True
        if self.executor is not None:
            self.executor.stop()
        if self.manual_executor is not None:
            self.manual_executor.stop()
        if self.robot is not None:
            try:
                self.robot.send("gripper stop")
                self.robot.send("torque off")
            except Exception as exc:
                self.log(f"No se pudo confirmar la emergencia por USB: {exc}")
        if self.demonstration is not None:
            self.finish_demonstration("interrupted")
        self.auto_status.set("EMERGENCIA: torque OFF solicitado. Sostené el brazo.")

    # ----- gemelo digital -----
    def prepare_twin_calibration(self) -> None:
        from brazoia.gemelo import ensure_twin_calibration

        try:
            ensure_twin_calibration(self.config, progress=lambda text: self.worker_messages.put(("log", text)))
            self.worker_messages.put(("log", "Gemelo: calibración de las cámaras virtuales lista."))
        finally:
            self.twin_calibration_ready.set()

    def describe_twin_scene(self) -> None:
        placement = self.twin.placement
        self.twin_status.set(
            f"Escena: cubo en ({placement.cube_xy[0] * 1000:.0f}, {placement.cube_xy[1] * 1000:.0f}) mm, "
            f"recipiente en ({placement.container_xy[0] * 1000:.0f}, {placement.container_xy[1] * 1000:.0f}) mm. "
            "Nada de esto mueve el brazo real."
        )

    def new_twin_scene(self) -> None:
        if self.twin is None:
            return
        if self.execution_busy:
            self.status.set("Esperá a que termine el movimiento o usá Detener.")
            return
        if self.execution is not None:
            self.finish_execution("interrupted")
        self.twin.new_scene()
        self.twin_scene_used = False
        if self.robot is not None:
            self.robot.clear_fault()
        # La telemetría de la escena anterior tarda un instante en renovarse.
        self.execution_busy = True
        self.root.after(int((self.config.capture.stable_window_seconds + 0.3) * 1000),
                        lambda: setattr(self, "execution_busy", False))
        self.describe_twin_scene()
        self.log("Gemelo: escena nueva; brazo en Inicio con la garra cerrada.")

    def open_twin_viewer(self) -> None:
        if self.twin is None:
            return
        if self.twin_viewer is not None and self.twin_viewer.is_running():
            self.status.set("El visor 3D ya está abierto.")
            return
        import mujoco.viewer

        self.twin_viewer = mujoco.viewer.launch_passive(self.twin.arm.model, self.twin.arm.data)
        self.twin.viewer = self.twin_viewer

    # ----- tarea autónoma: las doce etapas seguidas, sin confirmar cada una -----
    def run_autonomous_task(self) -> None:
        """Hace la tarea entera sola, con el brazo real o el gemelo.

        Usa lo mismo que el flujo supervisado (política, planificador, ejecutor y
        reglas del firmware) pero no pide confirmación: Detener o EMERGENCIA la cortan
        en cualquier momento. Si una etapa se bloquea, la tarea queda en esa etapa y
        se puede seguir paso a paso o volver a pulsar este botón.
        """
        if self.execution_busy:
            self.status.set("Ya hay un movimiento en curso.")
            return
        if self.demonstration is not None:
            self.status.set("Terminá primero la demostración activa.")
            return
        robot = self.robot
        if robot is None or not robot.connected or len(self.cameras) != 2:
            self.status.set("Abrí las dos cámaras y conectá el robot.")
            return
        if self.twin is not None and self.twin_scene_used:
            self.status.set("La tarea ya se hizo en esta escena: pulsá Nueva escena.")
            return
        if self.execution is not None and self.execution.finished:
            # La anterior terminó y nadie marcó el resultado: se guarda así.
            self.finish_execution("sin_marcar")
        self.auto_stop.clear()
        self.emergency_active = False
        self.execution_busy = True
        self.auto_status.set("Preparando la tarea autónoma...")

        def operation():
            if robot.snapshot().torque_enabled is not True:
                robot.send("arm torque on")
                deadline = time.monotonic() + 3.0
                while robot.snapshot().torque_enabled is not True:
                    if time.monotonic() > deadline:
                        raise RuntimeError("M1–M5 no confirmaron torque ON.")
                    time.sleep(0.05)
            time.sleep(0.5)  # telemetría y cuadros nuevos antes de leer la escena

        def start(_result):
            self.execution_busy = False
            if self.auto_stop.is_set():
                self.auto_status.set("Detenido por el operador.")
                return
            if self.execution is None:
                if self.twin is not None:
                    self.automatic_mode.set("geometrico")
                self.start_automatic()
                if self.execution is None:
                    self.auto_status.set("No se pudo iniciar: " + self.status.get())
                    return
            self._run_autonomous_stages()

        self.run_async(operation, success=start, failure=self._autonomous_failed)

    def _wait_stable_pose(self, robot: RobotSerial, timeout: float = 5.0) -> dict[int, float]:
        capture = self.config.capture
        deadline = time.monotonic() + timeout
        reason = ""
        while time.monotonic() < deadline:
            if self.auto_stop.is_set():
                raise RuntimeError("Detenido por el operador.")
            pose, reason = robot.stable_pose(
                time.monotonic(),
                maximum_age=capture.maximum_telemetry_age_seconds,
                window_seconds=capture.stable_window_seconds,
                maximum_range=capture.stable_max_range_degrees,
            )
            if pose is not None:
                return pose
            time.sleep(0.05)
        raise RuntimeError(f"El brazo no se quedó quieto: {reason}")

    def _scene_problem(self, execution: SupervisedExecution) -> str | None:
        """Lo que impide hacer la tarea sola con esta escena (hoy: el cubo demasiado lejos)."""
        if not hasattr(execution.policy, "summary") or execution.scene_features is None:
            return None
        from brazoia.politica_geometrica import cube_distance_problem

        try:
            return cube_distance_problem(execution.policy.summary(execution.scene_features))
        except Exception:
            return None  # el plan ya falló o falló al ubicar: lo informa prepare()

    def _inicio_problem(self, snapshots, observations, now: float | None = None) -> str | None:
        """La foto de Inicio con la que se planifica todo tiene que ser actual y el cubo
        tiene que estar afuera del recipiente (si no, es la escena de la tarea anterior)."""
        stale = validate_frame_pair(
            snapshots,
            time.monotonic() if now is None else now,
            maximum_age=self.config.capture.maximum_camera_age_seconds + 0.3,
            maximum_skew=1.0,
        )
        if stale:
            return f"las cámaras no tienen imagen actual ({stale})"
        if len(observations) != 2:
            return "falta la detección de una cámara"
        inside, _ = verify_cube_in_container([item.frame for item in snapshots], observations)
        if inside:
            return "el cubo ya está dentro del recipiente. Sacalo para empezar otra tarea."
        return None

    def _set_auto_status_from_worker(self, text: str) -> None:
        self.worker_messages.put(("ui", (self.auto_status.set, text)))

    def _run_autonomous_stages(self) -> None:
        execution, executor, run_log = self.execution, self.executor, self.execution_log
        robot = self.robot
        twin = self.twin is not None
        who = "Gemelo" if twin else "Brazo"
        self.execution_busy = True
        self.auto_status.set(f"{who}: haciendo la tarea solo, sin pedir confirmación. Detener lo corta.")

        def stopped():
            # execute() limpia el stop_event del ejecutor: la tarea usa su propia señal.
            if self.auto_stop.is_set():
                raise RuntimeError("Detenido por el operador.")

        def operation():
            # Lo que quedó preparado en el modo paso a paso se planificó desde otra pose.
            execution.cancel_proposal()
            if execution.next_stage_index > 1:
                # La etapa 1 ya se hizo paso a paso, donde el cubo lejos solo avisa.
                problem = self._scene_problem(execution)
                if problem:
                    raise _Rejected("No sigue: " + problem)
            recoveries = 0
            while not execution.finished:
                stopped()
                snapshot = robot.snapshot()
                if snapshot.fault:
                    raise RuntimeError(snapshot.fault)
                current = self._wait_stable_pose(robot)
                time.sleep(0.15)  # cuadros tomados con el brazo ya quieto
                snapshots, observations = self.latest_scene
                taken_at = time.monotonic()  # planificar la etapa 1 tarda más de 1 s
                stage = KEYFRAME_STAGES[execution.next_stage_index]
                try:
                    if execution.next_stage_index == 1:
                        # Antes de planificar: con el cubo en el recipiente la ubicación ya falla.
                        problem = self._inicio_problem(snapshots, observations, taken_at)
                        if problem:
                            raise _Rejected("No arranca: " + problem)
                    proposal = execution.prepare(current, observations)
                    stage = proposal.stage
                    if proposal.recovery:
                        recoveries += 1
                        if recoveries > 2:
                            raise _Rejected("el brazo no queda en la pose de Inicio después de dos "
                                            "recuperaciones (algún motor a más de 2°). Revisalo en «Mover a mano».")
                    elif proposal.stage_index == 1:
                        if hasattr(execution.policy, "describe"):
                            self.worker_messages.put(("log", execution.policy.describe(execution.scene_features)))
                        self._log_scene(execution, run_log)
                        problem = self._scene_problem(execution)
                        if problem:
                            raise _Rejected("No arranca: " + problem)
                    step = f"{who} · {proposal.label}: {len(proposal.groups)} tramos"
                    self.worker_messages.put(("log", step))
                    self._set_auto_status_from_worker(f"{step} · sin pedir confirmación. Detener lo corta.")
                    if run_log is not None:
                        run_log.proposal(proposal)
                    stopped()
                    executor.execute(proposal)
                except Exception as exc:
                    # Si se vuelve a intentar, se replanifica desde donde quedó el brazo.
                    execution.cancel_proposal()
                    if run_log is not None:
                        if isinstance(exc, _Rejected):
                            run_log.note("rechazo", str(exc), stage)
                        else:
                            run_log.note("bloqueo", str(exc), stage)
                            self._run_capture(run_log, f"{stage}_bloqueo")
                    raise
                if run_log is not None:
                    run_log.completed(proposal.stage, *self._robot_state())
                execution.mark_completed()
                time.sleep(0.3)  # cuadros nuevos para la etapa siguiente
                if not execution.finished:
                    self._run_capture(run_log, proposal.stage)
            time.sleep(1.0)  # que el cubo se asiente antes de mirar
            return self.twin.cube_in_container() if twin else None

        def finished(inside):
            self.execution_busy = False
            self.verify_final_result()
            if not twin:
                return  # verify_final_result pide marcar el resultado humano
            self.twin_scene_used = True
            verdict = "quedó" if inside else "NO quedó"
            self.log(f"Gemelo: según la física, el cubo {verdict} dentro del recipiente.")
            self.notes.set("Gemelo digital: resultado según la física de MuJoCo.")
            self.finish_execution("success" if inside else "failed_place")
            self.auto_status.set(f"Gemelo: tarea terminada; el cubo {verdict} en el recipiente. "
                                 "Nueva escena para repetir.")

        self.run_async(operation, success=finished, failure=self._autonomous_failed)

    def _autonomous_failed(self, exc) -> None:
        self.execution_busy = False
        if self.emergency_active:
            self.auto_status.set("EMERGENCIA: torque OFF solicitado. Sostené el brazo.")
            self.log(f"Tarea autónoma cortada por EMERGENCIA: {exc}")
            return
        if self.auto_stop.is_set():
            text = "Detenido por el operador."
        else:
            text = f"BLOQUEADO: {exc}"
        stage = ""
        if isinstance(exc, _Rejected):
            stage = " No se movió nada: corregí la escena y volvé a pulsar el botón."
        elif self.execution is not None and not self.execution.finished:
            index = self.execution.next_stage_index
            stage = f" Quedó antes de {STAGE_LABELS[KEYFRAME_STAGES[index]]}: podés seguir paso a paso, " \
                    "volver a «Hacer la tarea sola» o marcar el resultado."
        self.auto_status.set(text + stage)
        self.log(f"Tarea autónoma: {text}")

    def open_data_folder(self) -> None:
        path = self.config.root / "datos"
        path.mkdir(exist_ok=True)
        os.startfile(path)

    def tick(self) -> None:
        if self.closing:
            return
        try:
            self._drain_workers()
            self.snapshots = [camera.snapshot() for camera in self.cameras]
            observations = []
            for index, snapshot in enumerate(self.snapshots):
                if snapshot.frame is None:
                    self.camera_labels[index].configure(image="", text=snapshot.status)
                    continue
                observation = detect_scene(
                    snapshot.frame, **detection_parameters(self.config.vision, index)
                )
                observations.append(observation)
                frame = draw_observation(snapshot.frame, observation)
                self._show_frame(index, frame)
            # Se publica entera: otro hilo (la tarea del gemelo) puede leerla en cualquier momento.
            self.observations = observations
            self.latest_scene = (list(self.snapshots), observations)
            self._update_robot_state()
        except Exception as exc:
            self.log(f"ERROR: {exc}")
        self.root.after(50, self.tick)

    def _show_frame(self, index: int, frame) -> None:
        height, width = frame.shape[:2]
        scale = min(PREVIEW_SIZE[0] / width, PREVIEW_SIZE[1] / height)
        resized = cv2.resize(frame, (max(1, int(width * scale)), max(1, int(height * scale))))
        image = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)))
        self.preview_images[index] = image
        self.camera_labels[index].configure(image=image, text="")

    def _update_robot_state(self) -> None:
        robot = self.robot
        if robot is None:
            return
        snapshot = robot.snapshot()
        now = time.monotonic()
        for motor_id in range(1, 7):
            value = snapshot.positions.get(motor_id)
            age = now - snapshot.received_at.get(motor_id, 0.0)
            self.angle_vars[motor_id].set("—" if value is None or age > 0.5 else f"{value:.1f}°")
        torque = "ON" if snapshot.torque_enabled is True else "OFF" if snapshot.torque_enabled is False else "desconocido"
        self.torque_status.set(f"Torque: {torque}")
        self.gripper_status.set(f"Garra: {snapshot.gripper_state}")
        if self.rerun.available and now - self.last_rerun_update > 0.15:
            self.rerun.log_pose(snapshot.positions)
            self.last_rerun_update = now
        for _ in range(300):
            try:
                event: RobotEvent = robot.events.get_nowait()
            except queue.Empty:
                break
            if event.kind != "rx" or not event.payload.startswith("POS ID"):
                self.log(f"[{event.kind}] {event.payload}")

    def _drain_workers(self) -> None:
        for _ in range(100):
            try:
                kind, payload = self.worker_messages.get_nowait()
            except queue.Empty:
                return
            if kind == "log":
                self.log(str(payload))
                continue
            callback, value = payload
            if callback is not None:
                callback(value)
            elif kind == "failure":
                self.status.set(str(value))
                self.log(f"Error: {value}")

    def close(self) -> None:
        if self.closing:
            return
        if self.demonstration is not None or self.execution is not None:
            if not messagebox.askyesno(
                "Trabajo activo",
                "Hay una demostración o tarea activa. ¿Interrumpirla, apagar torque y cerrar?",
            ):
                return
        self.closing = True
        self.auto_stop.set()  # la tarea autónoma no empieza otra etapa
        if self.executor is not None:
            self.executor.stop()
        if self.manual_executor is not None:
            self.manual_executor.stop()
        if self.demonstration is not None:
            self.demonstration.finish("interrupted", "Ventana cerrada")
        if self.execution_log is not None:
            self.execution_log.finish(
                "interrupted", self.automatic_verification, "Ventana cerrada"
            )
        for camera in self.cameras:
            camera.close()
        if self.robot is not None:
            try:
                self.robot.send("gripper stop")
                self.robot.send("torque off")
            except Exception:
                pass
            self.robot.disconnect()
        if self.twin is not None:
            self.twin.stop()
            if self.twin_viewer is not None:
                self.twin_viewer.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    application = App(root, gemelo="--gemelo" in sys.argv)
    if "--trayecto-simulado" in sys.argv:
        root.after(200, application.open_trajectory_simulator)
    if "--calibracion" in sys.argv:
        root.after(200, application.open_calibration)
    root.mainloop()

