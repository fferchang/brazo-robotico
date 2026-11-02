"""Pick & place del SO-100 en MuJoCo, solo virtual.

No importa el controlador serie, no abre puertos, no usa cámaras reales y no
modifica demostraciones ni modelos. Ver SIMULACION_MUJOCO.md.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np

from brazoia.mujoco_sim import (PickPlaceExpert, PlanningError, VirtualArm, episode_metadata,
                                run_episode, sample_placement, save_metadata)


ROOT = Path(__file__).resolve().parent


class ViewerClosed(Exception):
    pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Pick & place virtual del SO-100 en MuJoCo. No usa el ESP32.")
    parser.add_argument("--episodios", type=int, default=3, help="Cantidad de episodios (por defecto 3).")
    parser.add_argument("--semilla", type=int, default=None, help="Semilla para repetir las mismas posiciones.")
    parser.add_argument("--sin-visor", action="store_true", help="Sin ventana: corre lo más rápido posible.")
    parser.add_argument("--ritmo", type=float, default=1.0, help="Velocidad del visor respecto del tiempo real.")
    parser.add_argument("--rerun", action="store_true", help="También muestra brazo, cubo y recipiente en Rerun.")
    parser.add_argument("--fotos", action="store_true",
                        help="Guarda fotos de las cámaras virtuales y metadata en datos/simulacion/.")
    parser.add_argument("--cerrar-al-terminar", action="store_true",
                        help="Cierra el visor al terminar en lugar de esperar a que lo cierres.")
    return parser.parse_args(argv)


class RerunScene:
    """Brazo con la vista habitual de Rerun, más cubo y recipiente como cajas."""

    def __init__(self, arm: VirtualArm):
        from brazoia.rerun_view import RerunArmView

        self.arm = arm
        self.view = RerunArmView(ROOT / "assets", recording_name="brazo_ia_SIMULACION_MUJOCO",
                                 view_name="SIMULACIÓN MUJOCO · SOLO VIRTUAL", clamp_joint_limits=False)
        self.view.open()
        if not self.view.available:
            raise RuntimeError(f"No se pudo abrir Rerun: {self.view.error}")
        self.ticks = 0

    def __call__(self, arm: VirtualArm):
        self.ticks += 1
        if self.ticks % 5:
            return  # 10 cuadros por segundo simulado alcanzan para mirar.
        import rerun as rr

        self.view.log_pose(arm.logical_pose())
        dims = arm.dimensions
        cube = arm.data.xquat[arm.cube_body]
        container = arm.data.mocap_quat[arm.container_mocap]
        outer = dims.container_outer
        frame = rr.CoordinateFrame("base")
        self.view.recording.log("simulacion/cubo", rr.Boxes3D(
            centers=[arm.cube_position()], half_sizes=[[dims.cube_size / 2] * 3],
            quaternions=[rr.Quaternion(xyzw=[cube[1], cube[2], cube[3], cube[0]])],
            colors=[[220, 55, 25]], fill_mode="solid"), frame)
        center = arm.data.mocap_pos[arm.container_mocap] + [0, 0, dims.container_wall_height / 2]
        self.view.recording.log("simulacion/recipiente", rr.Boxes3D(
            centers=[center], half_sizes=[[outer[0] / 2, outer[1] / 2, dims.container_wall_height / 2]],
            quaternions=[rr.Quaternion(xyzw=[container[1], container[2], container[3], container[0]])],
            colors=[[160, 215, 40]]), frame)


class PhotoRecorder:
    """Fotos de las cámaras virtuales en cada etapa, con el mismo nombre que las reales."""

    def __init__(self, arm: VirtualArm, folder: Path):
        import mujoco

        self.arm = arm
        self.folder = folder
        self.renderer = mujoco.Renderer(arm.model, height=480, width=640)
        self.images = {}

    def __call__(self, record):
        from PIL import Image

        from brazoia.config import KEYFRAME_STAGES

        index = KEYFRAME_STAGES.index(record.stage) + 1
        self.folder.mkdir(parents=True, exist_ok=True)
        names = []
        for camera in ("frontal", "lateral"):
            name = f"{index:02d}_{record.stage}_{camera}.png"
            self.renderer.update_scene(self.arm.data, camera=camera)
            Image.fromarray(self.renderer.render()).save(self.folder / name)
            names.append(name)
        self.images[record.stage] = names


def make_viewer_observer(viewer, pace):
    start = {"wall": None, "sim": None}

    def observe(arm):
        if not viewer.is_running():
            raise ViewerClosed
        viewer.sync()
        if start["wall"] is None:
            start["wall"], start["sim"] = time.perf_counter(), arm.data.time
        ahead = (arm.data.time - start["sim"]) / pace - (time.perf_counter() - start["wall"])
        if ahead > 0:
            time.sleep(ahead)

    return observe


def main(argv=None):
    args = parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    print("SIMULACIÓN MUJOCO · SOLO VIRTUAL · no se conecta al ESP32 ni mueve el brazo real")
    arm = VirtualArm(ROOT / "assets")
    expert = PickPlaceExpert(arm)
    rng = np.random.default_rng(args.semilla)
    observers = []
    if args.rerun:
        observers.append(RerunScene(arm))
    viewer, viewer_threads = None, []
    if not args.sin_visor:
        import mujoco.viewer

        before = set(threading.enumerate())
        viewer = mujoco.viewer.launch_passive(arm.model, arm.data)
        viewer_threads = [thread for thread in threading.enumerate() if thread not in before]
        observers.append(make_viewer_observer(viewer, max(0.05, args.ritmo)))
    arm.observer = (lambda current: [observer(current) for observer in observers]) if observers else None

    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    summary, warnings, holding = Counter(), Counter(), []
    try:
        for episode in range(1, args.episodios + 1):
            placement = sample_placement(rng, arm.dimensions)
            try:
                plan = expert.plan(placement)
            except PlanningError as exc:
                summary["sin plan"] += 1
                print(f"[{episode}] Sin plan: {exc}")
                continue
            photos = PhotoRecorder(arm, ROOT / "datos" / "simulacion" / f"sim_{session}_{episode:03d}") if args.fotos else None
            if viewer is not None:
                observers[-1] = make_viewer_observer(viewer, max(0.05, args.ritmo))
            result = run_episode(arm, expert, placement, plan=plan, on_stage=photos)
            summary["exitoso" if result.success else "fallido"] += 1
            if result.holding_m6 is not None:
                holding.append(result.holding_m6)
            for record in result.records:
                warnings["con avisos" if record.rule_warnings else "sin avisos"] += 1
            cube = np.round(np.array(placement.cube_xy) * 1000).astype(int).tolist()
            container = np.round(np.array(placement.container_xy) * 1000).astype(int).tolist()
            print(f"[{episode}] {'ÉXITO' if result.success else 'FALLA'} · cubo {cube} mm · recipiente {container} mm"
                  f" · M6 al sostener {result.holding_m6 or 0:.1f}° · {result.simulated_seconds:.0f} s simulados")
            for reason in result.reasons + result.plan_warnings:
                print(f"     {reason}")
            if photos is not None:
                payload = episode_metadata(result, arm.dimensions)
                for stage in payload["stages"]:
                    stage["images"] = photos.images.get(stage["stage"], [])
                save_metadata(photos.folder / "metadata.json", payload)
                print(f"     Fotos: {photos.folder.relative_to(ROOT)}")
        if viewer is not None and not args.cerrar_al_terminar:
            print("Episodios terminados. Podés girar la cámara con el mouse y empujar el cubo "
                  "(doble clic sobre él y Ctrl + arrastrar con el botón derecho). Cerrá la ventana para salir.")
            arm.observer = None
            while viewer.is_running():
                arm.step(0.02)
                viewer.sync()
                time.sleep(0.02)
    except (ViewerClosed, KeyboardInterrupt):
        print("Simulación interrumpida.")
    finally:
        if viewer is not None:
            viewer.close()
            # El visor corre en un hilo daemon: salir sin esperarlo rompe el cierre de OpenGL.
            for thread in viewer_threads:
                thread.join(timeout=5)

    total = sum(summary.values())
    print(f"\nResumen: {summary['exitoso']}/{total} exitosos, {summary['fallido']} fallidos, "
          f"{summary['sin plan']} sin plan.")
    if holding:
        print(f"M6 al sostener el cubo: {min(holding):.1f}–{max(holding):.1f}° (en las demos reales: 36–45°).")
    keyframes = sum(warnings.values())
    if keyframes:
        print(f"Poses clave que las reglas actuales del brazo real rechazarían: "
              f"{warnings['con avisos']}/{keyframes} (límites de config.json o planificador 2D).")
    return 0 if summary["fallido"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
