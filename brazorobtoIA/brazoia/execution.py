from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .config import KEYFRAME_STAGES, STAGE_LABELS
from .planner import MotionGroup, build_motion_plan
from .policy import StagePolicy
from .robot import RobotSerial
from .vision import SceneObservation, observation_features


StatusCallback = Callable[[str], None]


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
    def __init__(self, robot: RobotSerial, status: StatusCallback | None = None):
        self.robot = robot
        self.status = status or (lambda text: None)
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def execute(self, proposal: MotionProposal) -> None:
        self.stop_event.clear()
        snapshot = self.robot.snapshot()
        if snapshot.fault:
            raise RuntimeError(f"Fallo activo del firmware: {snapshot.fault}")
        if snapshot.torque_enabled is not True:
            raise RuntimeError("Activá torque antes de confirmar el movimiento.")
        for index, group in enumerate(proposal.groups, start=1):
            if self.stop_event.is_set():
                raise RuntimeError("Movimiento detenido por el operador.")
            self.status(f"Tramo {index}/{len(proposal.groups)}: {group.targets}")
            self._move_group(group)
        if proposal.gripper_action == "close":
            self.status("Cerrando la garra por presión...")
            self.robot.send("gripper close")
            self._wait_gripper("holding", timeout=16.0)
        elif proposal.gripper_action == "open":
            self.status("Abriendo la garra...")
            self.robot.send("gripper open")
            self._wait_motor(6, 140.0, timeout=5.0)

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
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.05):
                raise RuntimeError("Movimiento detenido por el operador.")
            snapshot = self.robot.snapshot()
            if snapshot.fault:
                raise RuntimeError(snapshot.fault)
            arrived = all(
                abs(snapshot.positions.get(motor_id, target + 1000.0) - target) <= 1.2
                for motor_id, target in targets.items()
            )
            stable = stable + 1 if arrived else 0
            if stable >= 3:
                time.sleep(0.08)
                return
        description = ", ".join(f"M{i}={value:.1f}°" for i, value in targets.items())
        raise RuntimeError(f"Sin confirmación de llegada: {description}")

    def _wait_motor(self, motor_id: int, target: float, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        stable = 0
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.05):
                raise RuntimeError("Movimiento detenido por el operador.")
            snapshot = self.robot.snapshot()
            if snapshot.fault:
                raise RuntimeError(snapshot.fault)
            arrived = abs(snapshot.positions.get(motor_id, target + 1000.0) - target) <= 1.5
            stable = stable + 1 if arrived else 0
            if stable >= 3:
                return
        raise RuntimeError(f"M{motor_id} no confirmó {target:.1f}°.")

    def _wait_gripper(self, expected: str, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.05):
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
        if self.next_stage_index == 1 and needs_recovery:
            target = canonical
            groups = build_motion_plan(current, target, self.limits)
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
        start = dict(current)
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

