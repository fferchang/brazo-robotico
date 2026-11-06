from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .config import KEYFRAME_STAGES, STAGE_LABELS
from .planner import GRIPPER_OPEN_TARGET, MotionGroup, build_motion_plan, gripper_sweep_problem, planning_start
from .policy import StagePolicy
from .robot import RobotSerial
from .vision import SceneObservation, observation_features


StatusCallback = Callable[[str], None]

# El firmware abre con WritePosEx(6, 150 real, 300, 25): 300 pasos/s son ~26,4°/s,
# así que de cerrada (0°) a abierta (140°) tarda unos 5,3 s. Lo midió el gemelo digital.
GRIPPER_OPEN_DEGREES = 140.0
GRIPPER_OPEN_SPEED_DEG_S = 300 * 360.0 / 4095.0
# En el brazo real la garra abierta se queda entre 135,6° y 139° (casi siempre
# ~137,2°), sin llegar nunca a 1,5° de los 140° pedidos: demostraciones y
# ejecuciones de octubre y noviembre de 2026. Se acepta quieta a menos de esto.
GRIPPER_OPEN_TOLERANCE = 6.0


ARRIVAL_TOLERANCE = 1.2         # llegada normal
LOADED_ARRIVAL_TOLERANCE = 3.0  # llegada aceptada si el motor ya no se mueve
STILL_SAMPLES = 12              # ~0,6 s de lecturas cada 50 ms
STILL_RANGE = 0.3               # variación máxima para considerarlo quieto


def gripper_open_timeout(current_m6: float | None) -> float:
    """Espera de la apertura: lo que tarda el servo en recorrer lo que falta, con margen."""
    start = 0.0 if current_m6 is None else current_m6
    distance = abs(GRIPPER_OPEN_DEGREES - start)
    return max(5.0, 1.5 + 1.25 * distance / GRIPPER_OPEN_SPEED_DEG_S)


@dataclass(frozen=True)
class MotionProposal:
    stage_index: int | None
    stage: str
    label: str
    target: dict[int, float]
    groups: tuple[MotionGroup, ...]
    gripper_action: str | None
    recovery: bool = False


class MotionExecutor:
    def __init__(
        self,
        robot: RobotSerial,
        status: StatusCallback | None = None,
        abort: threading.Event | None = None,
    ):
        self.robot = robot
        self.status = status or (lambda text: None)
        self.stop_event = threading.Event()
        # Parada externa (Detener/EMERGENCIA de la app). A diferencia de stop_event,
        # execute() no la borra: una parada pedida entre dos etapas no se pierde.
        self.abort = abort

    def stop(self) -> None:
        self.stop_event.set()

    def _stop_requested(self, timeout: float = 0.0) -> bool:
        if self.abort is not None and self.abort.is_set():
            return True
        if timeout > 0:
            return self.stop_event.wait(timeout) or (self.abort is not None and self.abort.is_set())
        return self.stop_event.is_set()

    def execute(self, proposal: MotionProposal) -> None:
        self.stop_event.clear()
        if self._stop_requested():
            raise RuntimeError("Movimiento detenido por el operador.")
        snapshot = self.robot.snapshot()
        if snapshot.fault:
            raise RuntimeError(f"Fallo activo del firmware: {snapshot.fault}")
        if snapshot.torque_enabled is not True:
            raise RuntimeError("Activá torque antes de confirmar el movimiento.")
        for index, group in enumerate(proposal.groups, start=1):
            if self._stop_requested():
                raise RuntimeError("Movimiento detenido por el operador.")
            self.status(f"Tramo {index}/{len(proposal.groups)}: {group.targets}")
            self._move_group(group)
        # La garra vuelve a tener torque con estas órdenes: una parada pedida durante
        # el último tramo no tiene que deshacerse acá.
        if proposal.gripper_action is not None and self._stop_requested():
            raise RuntimeError("Movimiento detenido por el operador.")
        if proposal.gripper_action == "close":
            self.status("Cerrando la garra por presión...")
            self.robot.send("gripper close")
            self._wait_gripper("holding", timeout=16.0)
        elif proposal.gripper_action == "open":
            self.status("Abriendo la garra...")
            timeout = gripper_open_timeout(self.robot.snapshot().positions.get(6))
            self.robot.send("gripper open")
            self._wait_gripper_open(timeout=timeout)

    def _move_group(self, group: MotionGroup) -> None:
        targets = dict(group.targets)
        if len(targets) == 1:
            motor_id, target = next(iter(targets.items()))
            self.robot.send(f"{motor_id} {target:.2f}")
        elif len(targets) == 2:
            (first_id, first_target), (second_id, second_target) = targets.items()
            self.robot.send(
                f"pair {first_id} {first_target:.2f} {second_id} {second_target:.2f}"
            )
        else:
            raise ValueError("Solo se permiten uno o dos motores por tramo.")

        snapshot = self.robot.snapshot()
        largest = max(
            abs(snapshot.positions.get(motor_id, target) - target)
            for motor_id, target in targets.items()
        )
        timeout = min(8.0, max(2.0, 0.8 + largest / 45.0))
        deadline = time.monotonic() + timeout
        stable = 0
        recent: list[dict[int, float]] = []
        while time.monotonic() < deadline:
            if self._stop_requested(0.05):
                raise RuntimeError("Movimiento detenido por el operador.")
            snapshot = self.robot.snapshot()
            if snapshot.fault:
                raise RuntimeError(snapshot.fault)
            errors = {
                motor_id: abs(snapshot.positions.get(motor_id, target + 1000.0) - target)
                for motor_id, target in targets.items()
            }
            stable = stable + 1 if max(errors.values()) <= ARRIVAL_TOLERANCE else 0
            if stable >= 3:
                time.sleep(0.08)
                return
            # Con el brazo estirado y carga, el hombro cede por la gravedad y se queda quieto
            # algo antes de la orden (en el gemelo, M2 1,1–1,4°). Se acepta si quedó quieto cerca.
            recent = (recent + [{motor_id: snapshot.positions.get(motor_id, 1e9) for motor_id in targets}])[
                -STILL_SAMPLES:
            ]
            if (
                len(recent) == STILL_SAMPLES
                and max(errors.values()) <= LOADED_ARRIVAL_TOLERANCE
                and all(
                    max(sample[motor_id] for sample in recent) - min(sample[motor_id] for sample in recent)
                    <= STILL_RANGE
                    for motor_id in targets
                )
            ):
                detail = ", ".join(f"M{i} a {errors[i]:.1f}°" for i in targets)
                self.status(f"Llegada con desvío (motor quieto): {detail}")
                time.sleep(0.08)
                return
        description = ", ".join(f"M{i}={value:.1f}°" for i, value in targets.items())
        raise RuntimeError(f"Sin confirmación de llegada: {description}")

    def _wait_gripper_open(self, timeout: float) -> None:
        """Abierta: M6 a 1,5° de 140°, o quieta a menos de GRIPPER_OPEN_TOLERANCE."""
        target = GRIPPER_OPEN_DEGREES
        deadline = time.monotonic() + timeout
        stable = 0
        recent: list[float] = []
        last = None
        while time.monotonic() < deadline:
            if self._stop_requested(0.05):
                raise RuntimeError("Movimiento detenido por el operador.")
            snapshot = self.robot.snapshot()
            if snapshot.fault:
                raise RuntimeError(snapshot.fault)
            last = snapshot.positions.get(6)
            error = abs(last - target) if last is not None else math.inf
            stable = stable + 1 if error <= 1.5 else 0
            if stable >= 3:
                return
            recent = (recent + [last if last is not None else math.inf])[-STILL_SAMPLES:]
            if (
                len(recent) == STILL_SAMPLES
                and error <= GRIPPER_OPEN_TOLERANCE
                and max(recent) - min(recent) <= STILL_RANGE
            ):
                self.status(f"Garra abierta a {last:.1f}° (quieta, a {error:.1f}° de {target:.0f}°).")
                return
        reading = "sin lectura" if last is None else f"quedó en {last:.1f}°"
        raise RuntimeError(f"M6 no confirmó {target:.1f}°: {reading}.")

    def _wait_gripper(self, expected: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._stop_requested(0.05):
                raise RuntimeError("Movimiento detenido por el operador.")
            snapshot = self.robot.snapshot()
            if snapshot.fault:
                raise RuntimeError(snapshot.fault)
            if snapshot.gripper_state == expected:
                return
        raise RuntimeError("La garra no confirmó el contacto.")


class SupervisedExecution:
    """Prepara una etapa y nunca la ejecuta sin confirmación externa."""

    def __init__(self, policy: StagePolicy, limits: dict[int, tuple[float, float]]):
        self.policy = policy
        self.limits = limits
        self.next_stage_index = 1
        self.finished = False
        self.proposal: MotionProposal | None = None
        # Escena vista desde la pose de Inicio; el modelo predice todas las etapas con ella.
        self.scene_features: np.ndarray | None = None

    def reset(self) -> None:
        self.next_stage_index = 1
        self.finished = False
        self.proposal = None
        self.scene_features = None

    def prepare(
        self,
        current: dict[int, float],
        observations: list[SceneObservation],
    ) -> MotionProposal:
        if self.finished:
            raise RuntimeError("La tarea ya terminó.")
        if self.proposal is not None:
            return self.proposal
        if any(motor_id not in current for motor_id in range(1, 6)):
            raise ValueError("Falta la pose actual de M1..M5.")

        canonical = {
            motor_id: float(self.policy.canonical_start[motor_id - 1])
            for motor_id in range(1, 6)
        }
        needs_recovery = max(
            abs(current[motor_id] - canonical[motor_id]) for motor_id in range(1, 6)
        ) > 2.0
        # El encoder puede quedar unas décimas fuera de un límite tras una orden al límite.
        start = planning_start(current, self.limits)
        if self.next_stage_index == 1 and needs_recovery:
            target = canonical
            groups = build_motion_plan(start, target, self.limits)
            self.proposal = MotionProposal(
                stage_index=None,
                stage="recuperacion_inicio",
                label="Recuperar pose inicial",
                target=target,
                groups=tuple(groups),
                gripper_action=None,
                recovery=True,
            )
            return self.proposal

        stage_index = self.next_stage_index
        stage = KEYFRAME_STAGES[stage_index]
        if stage_index == 1:
            # El brazo sigue en Inicio: es la misma escena con la que se entrenó.
            # Después el brazo tapa o lleva el cubo y la vista ya no sirve.
            self.scene_features = observation_features(observations)
        if self.scene_features is None:
            raise RuntimeError("Falta la escena de Inicio; reiniciá la tarea automática.")
        if stage == "guardado":
            target = {
                motor_id: float(self.policy.canonical_guard[motor_id - 1])
                for motor_id in range(1, 6)
            }
        else:
            target = self.policy.predict(stage_index, self.scene_features)
        # La política geométrica agrega puntos previos (descenso, empuje y retiro);
        # el modelo aprendido no tiene.
        via_points = self.policy.via(stage_index, self.scene_features) if hasattr(self.policy, "via") else []
        groups = []
        for waypoint in (*via_points, target):
            planned = build_motion_plan(start, waypoint, self.limits)
            groups.extend(planned)
            # El planificador no mueve un motor por menos de 0,5°: el tramo siguiente
            # parte de lo ordenado, no del punto. Si no, con puntos cercanos (el empuje
            # avanza de a 4 mm) un motor que cambia poco en cada uno nunca se movería.
            for group in planned:
                start.update(dict(group.targets))
        gripper_action = {
            "aproximacion_2": "open",
            "aproximacion_3": "close",
            "apertura_sobre_cubo": "open",
            "agarre": "close",
            "soltado": "open",
            "guardado": "close",
        }.get(stage)
        if gripper_action == "open":
            # Los tramos se revisan con la garra como está; abrirla al final agranda su
            # contorno y puede barrer la mesa si queda horizontal y baja.
            problem = gripper_sweep_problem(
                (target[2], target[3], target[4]), current.get(6), GRIPPER_OPEN_TARGET
            )
            if problem:
                raise ValueError(f"{STAGE_LABELS[stage]}: no se puede abrir la garra ahí: {problem}.")
        self.proposal = MotionProposal(
            stage_index=stage_index,
            stage=stage,
            label=STAGE_LABELS[stage],
            target=target,
            groups=tuple(groups),
            gripper_action=gripper_action,
        )
        return self.proposal

    def mark_completed(self) -> None:
        if self.proposal is None:
            raise RuntimeError("No hay una etapa preparada.")
        if not self.proposal.recovery:
            assert self.proposal.stage_index is not None
            self.next_stage_index = self.proposal.stage_index + 1
            if self.next_stage_index >= len(KEYFRAME_STAGES):
                self.finished = True
        self.proposal = None

    def cancel_proposal(self) -> None:
        self.proposal = None


def canonical_pose_text(values: np.ndarray) -> str:
    return " | ".join(f"M{motor_id}={values[motor_id - 1]:.1f}°" for motor_id in range(1, 6))

