"""Visor 3D de MuJoCo para exponer: solo la escena, sin los paneles de los costados.

`MujocoArmView` copia la pose del brazo real en el modelo 3D (en lugar de Rerun): no
simula física ni manda órdenes, solo muestra los ángulos que informa el firmware.
`launch_scene_viewer` abre el visor de una simulación (gemelo, simulación MuJoCo) con
el mismo encuadre. En el visor, Tab y Shift+Tab muestran u ocultan los paneles.
"""
from __future__ import annotations

import threading
from pathlib import Path

import mujoco
import numpy as np

# Encuadre del video demo (video_demo.py): el brazo de costado, un poco desde arriba.
VIEW_LOOKAT = (0.0, -0.15, 0.13)
VIEW_DISTANCE = 0.85
VIEW_AZIMUTH = 90.0
VIEW_ELEVATION = -22.0


def frame_camera(viewer) -> None:
    with viewer.lock():
        viewer.cam.lookat[:] = VIEW_LOOKAT
        viewer.cam.distance = VIEW_DISTANCE
        viewer.cam.azimuth = VIEW_AZIMUTH
        viewer.cam.elevation = VIEW_ELEVATION


def launch_scene_viewer(model, data, *, panels: bool = False):
    """Visor pasivo con la escena sola (sin paneles, salvo `panels`) y el encuadre de exponer.

    Bloquea hasta que la ventana está lista (y para siempre si el hilo del visor falla):
    no llamarlo desde el hilo de la interfaz."""
    import mujoco.viewer

    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=panels, show_right_ui=panels)
    frame_camera(viewer)
    return viewer


class MujocoArmView:
    """El brazo real en el visor de MuJoCo, como hacía Rerun: copia los ángulos M1–M6.

    Todo lo del visor (abrirlo, tomar su lock, sync) pasa en un hilo propio. La interfaz
    solo deja la última pose con `log_pose`, que nunca espera: el hilo de MuJoCo retiene
    su lock mientras se arrastra la ventana (bucle modal de Windows), y si la interfaz lo
    esperara quedaría congelada, con EMERGENCIA incluido."""

    UPDATE_SECONDS = 0.05

    def __init__(self, assets_root: Path, *, limits=None):
        self.assets_root = Path(assets_root)
        self.limits = limits
        self.arm = None
        self.viewer = None
        self.error = ""
        self._pose: dict[int, float] | None = None
        self._pose_lock = threading.Lock()
        self._closing = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self.viewer is not None and self.viewer.is_running()

    @property
    def active(self) -> bool:
        """Abierto o abriéndose."""
        return self._thread is not None and self._thread.is_alive()

    def open(self) -> None:
        """Empieza a abrir el visor en su hilo (o nada, si ya está abierto). No bloquea."""
        if self.active or self.running:
            return
        self.error = ""
        self.viewer = None
        self._closing.clear()
        self._thread = threading.Thread(target=self._run, name="visor-mujoco", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            if self.arm is None:
                from .mujoco_sim import VirtualArm

                self.arm = VirtualArm(self.assets_root, limits=self.limits)
                self._hide_objects()
            viewer = launch_scene_viewer(self.arm.model, self.arm.data)
        except Exception as exc:  # sin OpenGL, sin modelo: la app sigue igual
            self.error = str(exc) or type(exc).__name__
            return
        self.viewer = viewer
        while not self._closing.is_set() and viewer.is_running():
            with self._pose_lock:
                pose, self._pose = self._pose, None
            if pose:
                with viewer.lock():
                    for motor, value in pose.items():
                        self.arm.data.qpos[self.arm.qadr[motor]] = self.arm.to_qpos(motor, value)
                    mujoco.mj_forward(self.arm.model, self.arm.data)
                viewer.sync()
            self._closing.wait(self.UPDATE_SECONDS)

    def _hide_objects(self) -> None:
        """El cubo y el recipiente del modelo no son los reales: quedan bajo la mesa."""
        arm = self.arm
        arm.data.qpos[arm.cube_qadr:arm.cube_qadr + 3] = [0.0, 0.0, arm.floor_z - 1.0]
        arm.data.mocap_pos[arm.container_mocap] = [0.0, 0.0, arm.floor_z - 1.0]
        mujoco.mj_forward(arm.model, arm.data)

    def log_pose(self, positions: dict[int, float]) -> None:
        """Deja la última pose (grados lógicos) para el hilo del visor. Nunca espera al visor."""
        values = {motor: float(value) for motor, value in positions.items()
                  if motor in range(1, 7) and value is not None and np.isfinite(value)}
        if values:
            with self._pose_lock:
                self._pose = values

    def close(self) -> None:
        self._closing.set()
        viewer = self.viewer
        if viewer is not None:
            try:
                viewer.close()
            except Exception:
                pass
        self.viewer = None
