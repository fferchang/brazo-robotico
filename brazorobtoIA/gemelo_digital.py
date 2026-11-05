"""El brazo virtual hace solo la tarea completa, con el mismo código que el brazo real.

No usa el ESP32 ni las cámaras reales. Por cada escena:

1. el gemelo digital ubica cubo y recipiente al azar y deja el brazo en Inicio;
2. las cámaras virtuales sacan la foto de Inicio y la política geométrica ubica
   los objetos con la calibración de esas cámaras;
3. `SupervisedExecution` prepara cada etapa y `MotionExecutor` la manda por
   `RobotSerial` al firmware virtual, como en la aplicación, pero confirmando solo;
4. la física dice si el cubo terminó en el recipiente.

Lo que pase acá vale para el modelo 3D: el brazo real puede diferir (ver
docs/GEMELO_DIGITAL.md). Ningún resultado habilita movimientos del brazo real.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from brazoia.config import detection_parameters, load_config, validate_stage_gripper
from brazoia.execution import MotionExecutor, SupervisedExecution
from brazoia.gemelo import (
    TWIN_DATA_DIR,
    VIRTUAL_PORT,
    DigitalTwin,
    VirtualCameraStream,
    ensure_twin_calibration,
    load_twin_policy,
    twin_config,
)
from brazoia.mujoco_sim import PickPlaceExpert, VirtualArm
from brazoia.robot import RobotSerial
from brazoia.vision import cube_inside_container, detect_scene


ROOT = Path(__file__).resolve().parent


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="El brazo virtual hace solo la tarea, con el código del brazo real.")
    parser.add_argument("--episodios", type=int, default=3, help="Escenas a probar (por defecto 3).")
    parser.add_argument("--semilla", type=int, default=1, help="Semilla de las ubicaciones.")
    parser.add_argument("--visor", action="store_true", help="Abre la ventana de MuJoCo para ver el brazo.")
    parser.add_argument("--ritmo", type=float, default=1.0,
                        help="Velocidad respecto del tiempo real (1 = real). Las esperas del ejecutor son de "
                             "tiempo real: con más de 1 dejan de medir lo que pasaría en el brazo.")
    parser.add_argument("--giro-max", type=float, default=5.0,
                        help="Giro máximo de cubo y recipiente en grados (alineados a mano).")
    parser.add_argument("--recalibrar", action="store_true", help="Recalcula la calibración de las cámaras virtuales.")
    args = parser.parse_args(argv)
    if args.episodios < 1:
        parser.error("--episodios tiene que ser 1 o más.")
    if args.ritmo <= 0:
        parser.error("--ritmo tiene que ser mayor que 0.")
    return args


class TwinSession:
    """Gemelo, cámaras virtuales, puerto serie virtual y política geométrica."""

    def __init__(self, config, twin: DigitalTwin, log=print):
        self.config = config
        self.twin = twin
        self.log = log
        self.cameras = [VirtualCameraStream(twin, "frontal", 0), VirtualCameraStream(twin, "lateral", 1)]
        self.robot = RobotSerial(VIRTUAL_PORT, config.baud, opener=twin.open_port)
        self.robot.connect()
        planner_arm = VirtualArm(config.root / "assets", limits=config.limits)
        self.policy = load_twin_policy(config, arm=planner_arm, expert=PickPlaceExpert(planner_arm))

    def close(self) -> None:
        for camera in self.cameras:
            camera.close()
        self.robot.disconnect()
        for camera in self.cameras:
            camera.thread.join(timeout=2.0)

    def observations(self, timeout: float = 3.0):
        """Detecciones de un par de cuadros tomados después de esta llamada."""
        since = time.monotonic()
        deadline = since + timeout
        while time.monotonic() < deadline:
            snapshots = [camera.snapshot() for camera in self.cameras]
            if all(item.frame is not None and item.timestamp > since for item in snapshots):
                return [detect_scene(item.frame, **detection_parameters(self.config.vision, index))
                        for index, item in enumerate(snapshots)]
            time.sleep(0.03)
        raise RuntimeError("Las cámaras virtuales no entregaron imagen: " +
                           "; ".join(camera.snapshot().status for camera in self.cameras))

    def wait_until(self, condition, timeout: float, message: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition(self.robot.snapshot()):
                return
            time.sleep(0.03)
        raise RuntimeError(message)

    def wait_stable(self, timeout: float = 4.0) -> dict[int, float]:
        capture = self.config.capture
        deadline = time.monotonic() + timeout
        reason = ""
        while time.monotonic() < deadline:
            pose, reason = self.robot.stable_pose(time.monotonic(), maximum_age=capture.maximum_telemetry_age_seconds,
                                                  window_seconds=capture.stable_window_seconds,
                                                  maximum_range=capture.stable_max_range_degrees)
            if pose is not None:
                return pose
            time.sleep(0.05)
        raise RuntimeError(f"El brazo no se quedó quieto: {reason}")

    def run_episode(self, number: int) -> dict:
        placement = self.twin.new_scene()
        # Hasta que llegue telemetría nueva, RobotSerial muestra la escena anterior: esperar a
        # que la ventana de estabilidad tenga solo lecturas posteriores al cambio.
        time.sleep(self.config.capture.stable_window_seconds + 0.3)
        self.robot.clear_fault()
        entry = {"episodio": number, "cubo_m": [round(v, 4) for v in placement.cube_xy],
                 "recipiente_m": [round(v, 4) for v in placement.container_xy],
                 "giro_cubo_grados": round(float(np.degrees(placement.cube_yaw)), 1),
                 "etapas": [], "exito": False, "motivos": []}
        started = time.monotonic()
        log_start = self.twin.log_count
        try:
            self.wait_until(lambda snap: len(snap.positions) == 6, 3.0, "Sin telemetría del firmware virtual.")
            self.robot.send("arm torque on")
            self.wait_until(lambda snap: snap.torque_enabled is True, 3.0, "M1–M5 no confirmaron torque ON.")
            current = self.wait_stable()
            problem = validate_stage_gripper("inicio", current[6], self.robot.snapshot().gripper_state)
            if problem:
                raise RuntimeError(problem)
            execution = SupervisedExecution(self.policy, self.config.limits)
            warnings = entry.setdefault("avisos", [])
            executor = MotionExecutor(self.robot, status=lambda text: warnings.append(text)
                                      if text.startswith("Llegada con desvío") else None)
            while not execution.finished:
                current = self.wait_stable()
                proposal = execution.prepare(current, self.observations())
                if proposal.stage_index == 1:
                    estimated, _ = self.policy.plan(execution.scene_features)
                    entry["error_cubo_mm"] = round(float(np.linalg.norm(
                        np.subtract(estimated.cube_xy, placement.cube_xy)) * 1000), 1)
                    entry["error_recipiente_mm"] = round(float(np.linalg.norm(
                        np.subtract(estimated.container_xy, placement.container_xy)) * 1000), 1)
                    self.log(f"  ubicación estimada: cubo a {entry['error_cubo_mm']:.0f} mm, "
                             f"recipiente a {entry['error_recipiente_mm']:.0f} mm de la verdadera")
                stage_started = time.monotonic()
                try:
                    executor.execute(proposal)
                except Exception as exc:
                    entry["etapas"].append({"etapa": proposal.stage, "tramos": len(proposal.groups),
                                            "segundos": round(time.monotonic() - stage_started, 1),
                                            "error": str(exc)})
                    raise
                entry["etapas"].append({"etapa": proposal.stage, "tramos": len(proposal.groups),
                                        "segundos": round(time.monotonic() - stage_started, 1),
                                        "garra": self.robot.snapshot().gripper_state})
                self.log(f"  {proposal.label:<26} {len(proposal.groups):2d} tramos · "
                         f"{time.monotonic() - stage_started:4.1f} s")
                execution.mark_completed()
            time.sleep(1.0)
            entry["verificacion_visual"] = all(cube_inside_container(item) for item in self.observations())
            entry["exito"] = self.twin.cube_in_container()
            if not entry["exito"]:
                entry["motivos"].append("El cubo no quedó dentro del recipiente.")
        except Exception as exc:
            entry["motivos"].append(str(exc))
            self.log(f"  BLOQUEADO: {exc}")
        entry["segundos"] = round(time.monotonic() - started, 1)
        entry["contactos"] = self.twin.contacts()
        for name, samples in sorted(entry["contactos"].items()):
            entry["motivos"].append(f"{name} ({samples} muestras de 10 ms)")
        if entry["contactos"]:
            entry["exito"] = False
        entry["fallo_firmware"] = self.robot.snapshot().fault
        entry["firmware"] = self.twin.log_since(log_start)[-60:]
        return entry


def main(argv=None) -> int:
    args = parse_args(argv)
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass
    config = twin_config(load_config())
    ensure_twin_calibration(config, force=args.recalibrar, progress=print)
    twin = DigitalTwin(config, seed=args.semilla, pace=args.ritmo, max_yaw_degrees=args.giro_max)
    viewer = None
    if args.visor:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(twin.arm.model, twin.arm.data)
        twin.viewer = viewer
    print("Conectando al firmware virtual...", flush=True)
    session = TwinSession(config, twin, log=lambda text: print(text, flush=True))
    results = []
    started = time.monotonic()
    try:
        for number in range(1, args.episodios + 1):
            print(f"\nEscena {number}/{args.episodios}", flush=True)
            entry = session.run_episode(number)
            results.append(entry)
            verdict = "ÉXITO" if entry["exito"] else "falla"
            print(f"  → {verdict} en {entry['segundos']:.0f} s"
                  + ("" if entry["exito"] else " · " + "; ".join(entry["motivos"])), flush=True)
            if viewer is not None and not viewer.is_running():
                break
    except KeyboardInterrupt:
        print("\nInterrumpido.")
    finally:
        session.close()
        twin.stop()
        if viewer is not None:
            viewer.close()
    successes = sum(item["exito"] for item in results)
    summary = {"fecha": datetime.now().isoformat(timespec="seconds"), "semilla": args.semilla,
               "ritmo": args.ritmo, "giro_max_grados": args.giro_max, "exitos": successes,
               "episodios": results, "segundos": round(time.monotonic() - started, 1)}
    folder = ROOT / TWIN_DATA_DIR
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"gemelo_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"\nÉxitos: {successes} de {len(results)}. Detalle: {path}")
    return 0 if results and successes == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
