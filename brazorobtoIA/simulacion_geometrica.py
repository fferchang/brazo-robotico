"""Modo geométrico en la simulación MuJoCo, con cámaras virtuales. No usa el ESP32.

1. Calibra las cámaras virtuales como en el brazo real: cubo y recipiente en
   lugares al azar, una foto de cada cámara y la posición verdadera.
2. Prueba escenas nuevas: con las fotos de Inicio estima dónde están, el experto
   calcula las doce poses y la física dice si el cubo terminó en el recipiente.

Mide la cadena completa (detección, calibración, cinemática inversa, física),
pero en un mundo ideal: el modelo 3D coincide con el brazo y las cámaras
virtuales están ubicadas a ojo. No reemplaza la calibración del brazo real.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import mujoco
import numpy as np

from brazoia.calibracion_camaras import CAMERAS, Sample, describe, fit_calibration, object_pixels
from brazoia.config import load_config
from brazoia.mujoco_sim import PickPlaceExpert, PlanningError, VirtualArm, run_episode, sample_placement
from brazoia.politica_geometrica import GeometricPolicy
from brazoia.vision import detect_scene, observation_features


ROOT = Path(__file__).resolve().parent
# Las cámaras virtuales están ubicadas a ojo y ven el recipiente más chico que la
# frontal real; los umbrales estrictos de esa cámara no aplican aquí.
SIM_DETECTION = dict(minimum_red_area_ratio=0.001, minimum_lime_area_ratio=0.004,
                     lime_hsv_lower=(30, 50, 50), lime_hsv_upper=(90, 255, 255))


class VirtualCameras:
    def __init__(self, arm: VirtualArm, height=480, width=640):
        self.arm = arm
        self.renderer = mujoco.Renderer(arm.model, height=height, width=width)

    def features(self, placement) -> np.ndarray:
        """Las 32 características de la escena de Inicio, como las calcula la aplicación."""
        self.arm.reset(placement)
        observations = []
        for camera in CAMERAS:
            self.renderer.update_scene(self.arm.data, camera=camera)
            frame = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
            observations.append(detect_scene(frame, **SIM_DETECTION))
        return observation_features(observations)


def placement_by_hand(rng, max_yaw_degrees: float):
    """Escena al azar con el cubo y el recipiente alineados a mano con la base."""
    limit = math.radians(max_yaw_degrees)
    return dataclasses.replace(sample_placement(rng), cube_yaw=float(rng.uniform(-limit, limit)),
                               container_yaw=float(rng.uniform(-limit, limit)))


def calibration_samples(cameras: VirtualCameras, rng, count: int, *, containers: bool,
                        max_yaw_degrees: float) -> list[Sample]:
    samples = []
    for index in range(count):
        placement = placement_by_hand(rng, max_yaw_degrees)
        features = cameras.features(placement)
        objects = [("cubo", placement.cube_xy)] + ([("recipiente", placement.container_xy)] if containers else [])
        for kind, xy in objects:
            pixels = object_pixels(features, kind)
            if all(camera in pixels for camera in CAMERAS):
                samples.append(Sample(kind, pixels, (float(xy[0]), float(xy[1])), f"simulación {index + 1}"))
    return samples


def evaluate(policy: GeometricPolicy, cameras: VirtualCameras, rng, episodes: int, *, ideal: bool,
             max_yaw_degrees: float) -> list[dict]:
    results = []
    for index in range(episodes):
        placement = placement_by_hand(rng, max_yaw_degrees)
        entry = {"episodio": index + 1, "cubo_m": list(placement.cube_xy), "recipiente_m": list(placement.container_xy)}
        try:
            estimated, plan = policy.plan(cameras.features(placement))
        except ValueError as exc:
            entry.update(exito=False, motivos=[str(exc)])
            results.append(entry)
            print(f"  episodio {index + 1}: rechazado · {exc}", flush=True)
            continue
        entry["error_cubo_mm"] = float(np.linalg.norm(np.subtract(estimated.cube_xy, placement.cube_xy)) * 1000)
        entry["error_recipiente_mm"] = float(
            np.linalg.norm(np.subtract(estimated.container_xy, placement.container_xy)) * 1000)
        result = run_episode(policy.arm, policy.expert, placement, plan=plan)
        entry.update(exito=result.success, motivos=result.reasons)
        if ideal:
            # Mismo experto con la ubicación verdadera: separa errores de percepción del resto.
            entry["exito_con_ubicacion_real"] = run_episode(policy.arm, policy.expert, placement).success
        results.append(entry)
        print(f"  episodio {index + 1}: {'ÉXITO' if result.success else 'falla'}"
              f" · error cubo {entry['error_cubo_mm']:.0f} mm, recipiente {entry['error_recipiente_mm']:.0f} mm"
              + ("" if result.success else f" · {'; '.join(result.reasons)}"), flush=True)
    return results


def tolerance(arm: VirtualArm, expert: PickPlaceExpert, rng, magnitudes_mm, episodes: int, *,
              max_yaw_degrees: float) -> list[dict]:
    """Éxitos con un error de ubicación conocido, sin cámaras: ¿cuánto error aguanta el agarre?

    Para cada objeto y cada magnitud, el experto planifica con el objeto corrido
    esa distancia en una dirección al azar, y la física corre con la escena real.
    """
    rows = []
    for kind, field in (("cubo", "cube_xy"), ("recipiente", "container_xy")):
        for magnitude in magnitudes_mm:
            row = {"objeto": kind, "error_mm": float(magnitude), "exitos": 0, "rechazados": 0, "episodios": episodes,
                   "motivos": []}
            for _ in range(episodes):
                placement = placement_by_hand(rng, max_yaw_degrees)
                angle = rng.uniform(0.0, 2.0 * math.pi)
                shift = magnitude / 1000.0 * np.array([math.cos(angle), math.sin(angle)])
                assumed = dataclasses.replace(placement, **{field: tuple(np.add(getattr(placement, field), shift))})
                try:
                    plan = expert.plan(assumed)
                except PlanningError:
                    row["rechazados"] += 1
                    continue
                result = run_episode(arm, expert, placement, plan=plan)
                row["exitos"] += int(result.success)
                row["motivos"] += list(result.reasons)
            rows.append(row)
            print(f"  error {magnitude:4.0f} mm en {kind:10s}: {row['exitos']:3d} de {episodes} éxitos"
                  + (f" ({row['rechazados']} sin poses alcanzables)" if row["rechazados"] else ""), flush=True)
    return rows


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Modo geométrico con cámaras virtuales en MuJoCo. No usa el ESP32.")
    parser.add_argument("--muestras", type=int, default=15, help="Escenas para calibrar (por defecto 15).")
    parser.add_argument("--episodios", type=int, default=10, help="Escenas nuevas para probar (por defecto 10).")
    parser.add_argument("--semilla", type=int, default=1)
    parser.add_argument("--con-recipiente", action="store_true",
                        help="Calibra también el recipiente; si no, usa el mapeo del cubo, como el procedimiento real básico.")
    parser.add_argument("--giro-max", type=float, default=5.0,
                        help="Giro máximo del cubo y del recipiente respecto de la base, en grados (por defecto 5).")
    parser.add_argument("--comparar-ideal", action="store_true",
                        help="Repite cada episodio con la ubicación verdadera para separar errores de percepción.")
    parser.add_argument("--tolerancia", metavar="MM,MM,...",
                        help="Sin cámaras: mide los éxitos con esos errores de ubicación del cubo y del recipiente, "
                             "en mm (por ejemplo 0,5,10,15,20). Usa --episodios escenas por valor.")
    args = parser.parse_args(argv)
    if args.tolerancia is not None:
        try:
            args.tolerancia = [float(value) for value in args.tolerancia.split(",") if value.strip()]
        except ValueError:
            parser.error("--tolerancia lleva números separados por comas, por ejemplo 0,5,10,15,20.")
        if not args.tolerancia or min(args.tolerancia) < 0:
            parser.error("--tolerancia lleva números separados por comas, mayores o iguales a 0.")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass
    config = load_config()
    rng = np.random.default_rng(args.semilla)
    started = time.monotonic()
    arm = VirtualArm(limits=config.limits)
    folder = ROOT / "datos" / "simulacion"
    if args.tolerancia is not None:
        print(f"Tolerancia del agarre: {args.episodios} escenas por valor, sin cámaras.", flush=True)
        rows = tolerance(arm, PickPlaceExpert(arm), rng, args.tolerancia, args.episodios,
                         max_yaw_degrees=args.giro_max)
        summary = {"fecha": datetime.now().isoformat(timespec="seconds"), "tolerancia": rows,
                   "segundos": round(time.monotonic() - started, 1), "giro_max_grados": args.giro_max,
                   "semilla": args.semilla}
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"tolerancia_{datetime.now():%Y%m%d_%H%M%S}.json"
        path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"\nDetalle ({summary['segundos']:.0f} s): {path}")
        return 0
    cameras = VirtualCameras(arm)
    print(f"Calibrando las cámaras virtuales con {args.muestras} escenas...", flush=True)
    samples = calibration_samples(cameras, rng, args.muestras, containers=args.con_recipiente,
                                  max_yaw_degrees=args.giro_max)
    calibration = fit_calibration(samples, {camera: {"nombre": f"virtual {camera}"} for camera in CAMERAS})
    print(describe(calibration))
    policy = GeometricPolicy(calibration, config.limits, arm=arm, expert=PickPlaceExpert(arm))
    print(f"Probando {args.episodios} escenas nuevas...", flush=True)
    results = evaluate(policy, cameras, rng, args.episodios, ideal=args.comparar_ideal,
                       max_yaw_degrees=args.giro_max)
    successes = sum(item["exito"] for item in results)
    cube_errors = [item["error_cubo_mm"] for item in results if "error_cubo_mm" in item]
    summary = {
        "fecha": datetime.now().isoformat(timespec="seconds"),
        "muestras_calibracion": len(samples),
        "calibracion": calibration.to_dict()["mapeos"],
        "episodios": results,
        "exitos": successes,
        "segundos": round(time.monotonic() - started, 1),
        "giro_max_grados": args.giro_max,
    }
    print(f"\nÉxitos: {successes} de {len(results)} ({summary['segundos']:.0f} s).")
    if cube_errors:
        print(f"Error al ubicar el cubo: mediana {np.median(cube_errors):.0f} mm, máximo {np.max(cube_errors):.0f} mm.")
    if args.comparar_ideal:
        ideal = sum(item.get("exito_con_ubicacion_real", False) for item in results)
        print(f"Con la ubicación verdadera: {ideal} de {len(results)}.")
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"geometrica_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Detalle: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
