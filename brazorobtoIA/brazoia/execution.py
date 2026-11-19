from __future__ import annotations

import copy
import dataclasses
from itertools import product
import math
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .config import KEYFRAME_STAGES, STAGE_LABELS
from .planner import (
    GRIPPER_OPEN_TARGET,
    MotionGroup,
    build_motion_plan,
    classify_pose,
    edge_safe,
    floor_escape,
    gripper_envelope,
    gripper_sweep_problem,
    planning_start,
)
from .policy import StagePolicy
from .robot import RobotSerial
from .vision import SceneObservation, observation_features


StatusCallback = Callable[[str], None]


@contextmanager
def _time_phase(timings: dict[str, float], phase: str):
    started = time.monotonic()
    try:
        yield
    finally:
        timings[phase] += time.monotonic() - started

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
# En un punto de paso (no el último de la etapa) alcanza con pasar cerca: no se espera a
# que el brazo quede quieto. La orden siguiente sale con el brazo todavía llegando.
PASS_TOLERANCE = 1.5
# El firmware ignora (un motor) o rechaza (un par) una orden que llega a menos de 50 ms
# de la anterior (MOVE_COOLDOWN_MS); se deja un poco más.
MOVE_SPACING_SECONDS = 0.065
# Tramos «juntos»: los grupos de un mismo punto salen seguidos, sin esperar la llegada de
# cada uno, y los motores se mueven a la vez. Solo para saltos chicos, donde cualquier
# combinación intermedia queda cerca de lo revisado (ver SupervisedExecution._together_problem).
TOGETHER_MAX_DEGREES = 10.0
# Movimientos grandes (salir de Inicio, Traslado 2, volver): en vez de ir en escalera de a
# dos motores, se parten en pasos de CHAIN_STEP_DEGREES sobre la línea recta y cada paso va
# con todos los motores a la vez, si su caja pasa las mismas revisiones. La medición del 6/11
# en el brazo real dio el brazo entero por debajo de ~300 mA: la fuente solo sufría con la
# garra apretando (hasta ~1 A).
CHAIN_STEP_DEGREES = 6.0
# Encadenar sin frenar: en un punto de paso se manda el siguiente a mitad de camino si la caja
# entre los puntos y los posibles rezagos de motores pasa las revisiones, hasta este tamaño.
BLEND_MAX_DEGREES = 2 * CHAIN_STEP_DEGREES
BLEND_FRACTION = 0.5
# Con la orden `sync` (ver SupervisedExecution.sync_orders), elegidos en el gemelo (19/11,
# semillas 1 y 4, una vuelta de DEMO cada una): con pasos de 6° y el punto siguiente
# mandado apenas el motor más atrasado está a un paso del destino, la vuelta bajó de
# 42,7–44,1 s (pair) a 33,1–35,9 s y las frenadas del perfil antes de la orden siguiente,
# de 97–98% a 43%. Con 4° frenaban menos (17–19%), pero preparar Guardado tardaba ~11 s
# en vez de ~6 s y la vuelta quedaba en ~40 s. La caja del paso anticipado sigue
# acotada por BLEND_MAX_DEGREES y revisada en 2D y 3D como siempre.
SYNC_CHAIN_STEP_DEGREES = CHAIN_STEP_DEGREES
SYNC_BLEND_FRACTION = 1.0
# Garra del modelo aprendido y del modo paso a paso sin política geométrica: la de las
# demostraciones a mano (abrir del todo, cerrar sobre el cubo y volver a abrir).
LEGACY_GRIPPER_ACTIONS = {
    "aproximacion_2": "open",
    "aproximacion_3": "close",
    "apertura_sobre_cubo": "open",
    "agarre": "close",
    "soltado": "open",
    "guardado": "close",
}
M6_ANGLE_SPEED_DEG_S = 450 * 360.0 / 4095.0  # MOVE_SPEED del firmware para «6 <grados>»
MOVE_ACK_TIMEOUT_SECONDS = 0.6
# Con la orden `sync` (firmware nuevo) cada punto sale en un solo mensaje con hasta cinco
# motores, en vez de hasta tres órdenes de dos motores separadas 65 ms. Al encadenar, el
# punto siguiente se manda apenas la telemetría muestra el pasaje: se revisa cada 10 ms
# en vez de 50 ms para que la orden llegue antes de que el servo empiece a frenar.
SYNC_POLL_SECONDS = 0.01
# Preparación anticipada: mientras se ejecuta la etapa anterior, otro hilo prepara la
# siguiente desde la pose ordenada al final de la anterior. Guardado tardaba 3–7 s en
# prepararse después de soltar (revisión 3D de la cadena larga con la garra cerrándose).
ANTICIPATED_STAGES = ("guardado",)
# Se usa el plan anticipado si, al terminar, M1–M5 quedaron a esto o menos de lo
# ordenado: la misma tolerancia con que el ejecutor da por llegado un punto, o sea el
# mismo desvío que ya hay entre dos tramos de un plan normal. Más lejos, se replanifica.
ANTICIPATED_POSE_TOLERANCE = ARRIVAL_TOLERANCE
# Al cerrar sobre el cubo, M6 sube a ~1 A durante ~0,5 s y la fuente cae de 11,7 a 7,7 V
# (medición del 6/11 en el brazo real). Hasta que afloje no se levanta el cubo: con
# `power telem` prendido, hasta dos muestras seguidas de M6 bajo GRIP_SETTLE_MA; sin
# telemetría, GRIP_SETTLE_SECONDS fijos (en la medición bajó de 300 mA a los ~0,6 s).
GRIP_SETTLE_MA = 300.0
# Umbral de contacto del cierre por presión, en crudo (6,5 mA por unidad). El firmware
# arranca con 92 (~600 mA) y lo pierde al reiniciarse, así que se manda antes de cada cierre.
# Con 92 el cierre seguía apretando hasta ~1 A y la fuente caía a 6,9–8,2 V (6/11). Con 60
# (~390 mA) detecta el contacto antes; en la física agarra y sostiene igual (40/40 con
# ambos, M6 sosteniendo a ~38,5° en vez de ~38°). La garra libre consume 6–20 mA.
GRIPPER_CONTACT_RAW = 60
GRIP_SETTLE_SECONDS = 0.8
GRIP_SETTLE_TIMEOUT_SECONDS = 2.0
POWER_FRESH_SECONDS = 0.5
GRIPPER_CLOSED_DEGREES = 0.0


def gripper_open_timeout(current_m6: float | None) -> float:
    """Espera de la apertura: lo que tarda el servo en recorrer lo que falta, con margen."""
    start = 0.0 if current_m6 is None else current_m6
    distance = abs(GRIPPER_OPEN_DEGREES - start)
    return max(5.0, 1.5 + 1.25 * distance / GRIPPER_OPEN_SPEED_DEG_S)


@dataclass(frozen=True)
class MotionSegment:
    """Los grupos `groups[start:end]` que llevan a un mismo punto, de paso o final."""

    start: int
    end: int
    together: bool = False  # sus grupos salen seguidos y los motores se mueven a la vez
    final: bool = True      # el último punto de la etapa: se espera que el brazo quede quieto
    # Con `final` en False: se pasa al punto siguiente cuando al motor más atrasado le falta
    # esto o menos (grados), o PASS_TOLERANCE si es mayor.
    handoff: float = 0.0


@dataclass(frozen=True)
class MotionProposal:
    stage_index: int | None
    stage: str
    label: str
    target: dict[int, float]
    groups: tuple[MotionGroup, ...]
    # "open" (abrir del todo), "close" (cerrar por presión) o "angle" (M6 a `gripper_degrees`).
    gripper_action: str | None
    recovery: bool = False
    gripper_degrees: float | None = None
    # Con "angle": índice del grupo antes del cual se ordena M6, que se mueve mientras va el
    # brazo; None, después de todos. La espera de M6 es siempre al final.
    gripper_at_group: int | None = None
    # Vacío: cada grupo por separado, esperando que el brazo quede quieto (como antes).
    segments: tuple[MotionSegment, ...] = ()


class MotionExecutor:
    def __init__(
        self,
        robot: RobotSerial,
        status: StatusCallback | None = None,
        abort: threading.Event | None = None,
        *,
        firmware_waypoints: bool = False,
        sync_orders: bool | None = None,
    ):
        self.robot = robot
        self.status = status or (lambda text: None)
        self.stop_event = threading.Event()
        # Parada externa (Detener/EMERGENCIA de la app). A diferencia de stop_event,
        # execute() no la borra: una parada pedida entre dos etapas no se pierde.
        self.abort = abort
        self.firmware_waypoints = firmware_waypoints
        # None: usar `sync` si el firmware lo anunció (`caps`); False: nunca (como el 6/11).
        self.sync_orders = sync_orders
        self._last_move_sent = -math.inf

    @property
    def uses_sync(self) -> bool:
        return (self.sync_orders is not False and isinstance(self.robot, RobotSerial)
                and getattr(self.robot, "sync_supported", False) is True)

    def stop(self) -> None:
        self.stop_event.set()

    def _stop_requested(self, timeout: float = 0.0) -> bool:
        if self.abort is not None and self.abort.is_set():
            return True
        if timeout > 0:
            return self.stop_event.wait(timeout) or (self.abort is not None and self.abort.is_set())
        return self.stop_event.is_set()

    def execute(self, proposal: MotionProposal) -> dict[str, float]:
        started = time.monotonic()
        timings = dict.fromkeys(("orders", "arrival", "settling", "gripper", "after_grasp"), 0.0)
        self.stop_event.clear()
        if self._stop_requested():
            raise RuntimeError("Movimiento detenido por el operador.")
        snapshot = self.robot.snapshot()
        if snapshot.fault:
            raise RuntimeError(f"Fallo activo del firmware: {snapshot.fault}")
        if snapshot.torque_enabled is not True:
            raise RuntimeError("Activá torque antes de confirmar el movimiento.")
        total = len(proposal.groups)
        segments = proposal.segments or tuple(MotionSegment(index, index + 1) for index in range(total))
        # Ensayo acotado a Traslado 2: garra sosteniendo y puntos sin desvíos secuenciales.
        if (self.firmware_waypoints and isinstance(self.robot, RobotSerial)
                and proposal.stage == "traslado_2" and proposal.gripper_action is None):
            points = [proposal.groups[segment.start:segment.end] for segment in segments]
            if all((segment.together or len(groups) == 1) and self._batchable(groups)
                   for segment, groups in zip(segments, points)):
                for groups in points:
                    self._send_waypoint(groups, timings)
                timings["total"] = time.monotonic() - started
                return timings
        angle = proposal.gripper_action == "angle"
        closing = proposal.gripper_action == "close"
        use_sync = self.uses_sync
        holds_before = getattr(snapshot, "gripper_holds", None)
        gripper_sent = False
        pending_targets: dict[int, float] = {}
        for segment in segments:
            if proposal.gripper_at_group == segment.start and not gripper_sent and (angle or closing):
                # La garra empieza a moverse acá y sigue mientras va el brazo.
                with _time_phase(timings, "gripper"):
                    if angle:
                        self._send_gripper_angle(proposal.gripper_degrees)
                    else:
                        self._send_gripper_close()
                gripper_sent = True
            groups = proposal.groups[segment.start:segment.end]
            sync_targets = self._sync_targets(groups) if use_sync else None
            if sync_targets is not None and (segment.together or len(groups) == 1):
                # Un solo mensaje: los motores arrancan a la vez. Es lo mismo que los grupos
                # «juntos», cuya caja ya se revisó, sin el desfase de 65 ms entre grupos.
                if self._stop_requested():
                    raise RuntimeError("Movimiento detenido por el operador.")
                self.status(f"Tramo {segment.start + 1}–{segment.end}/{total}: sync {sync_targets}")
                with _time_phase(timings, "orders"):
                    self._send_move("sync 0 " + " ".join(f"{motor} {target:.2f}"
                                                         for motor, target in sync_targets.items()))
                pending_targets.update(sync_targets)
                self._wait_arrival(sync_targets if segment.handoff else pending_targets,
                                   settle=segment.final, handoff=segment.handoff, timings=timings,
                                   poll=SYNC_POLL_SECONDS)
                if not segment.handoff:
                    pending_targets = {}
                continue
            if segment.together and len(groups) > 1:
                # Seguidos: el firmware revisa cada orden con la pose de ese momento, que la
                # preparación ya revisó (todas las combinaciones intermedias del salto).
                targets: dict[int, float] = {}
                for offset, group in enumerate(groups):
                    if self._stop_requested():
                        raise RuntimeError("Movimiento detenido por el operador.")
                    self.status(f"Tramo {segment.start + offset + 1}/{total}: {group.targets}")
                    with _time_phase(timings, "orders"):
                        self._send_group(group)
                    targets.update(dict(group.targets))
                pending_targets.update(targets)
                # Al cortar la cadena, también tienen que llegar los motores que
                # recibieron su última orden en un tramo anterior.
                self._wait_arrival(targets if segment.handoff else pending_targets,
                                   settle=segment.final, handoff=segment.handoff, timings=timings)
                if not segment.handoff:
                    pending_targets = {}
                continue
            for offset, group in enumerate(groups):
                if self._stop_requested():
                    raise RuntimeError("Movimiento detenido por el operador.")
                self.status(f"Tramo {segment.start + offset + 1}/{total}: {group.targets}")
                with _time_phase(timings, "orders"):
                    self._send_group(group)
                last = offset == len(groups) - 1
                self._wait_arrival(dict(group.targets), settle=segment.final or not last,
                                   handoff=segment.handoff if last else 0.0, timings=timings)
        # La garra vuelve a tener torque con estas órdenes: una parada pedida durante
        # el último tramo no tiene que deshacerse acá.
        if proposal.gripper_action is not None and self._stop_requested():
            raise RuntimeError("Movimiento detenido por el operador.")
        if angle:
            with _time_phase(timings, "gripper"):
                if not gripper_sent:
                    self._send_gripper_angle(proposal.gripper_degrees)
                self.status(f"Garra a {proposal.gripper_degrees:.0f}°...")
                # Se acepta M6 quieta a menos de GRIPPER_OPEN_TOLERANCE, como con «gripper open».
                self._wait_arrival({6: float(proposal.gripper_degrees)}, settle=True,
                                   speed_deg_s=M6_ANGLE_SPEED_DEG_S, loaded_tolerance=GRIPPER_OPEN_TOLERANCE)
        elif closing:
            with _time_phase(timings, "gripper"):
                if not gripper_sent:
                    holds_before = getattr(self.robot.snapshot(), "gripper_holds", None)
                    self._send_gripper_close()
                self._wait_gripper_hold(holds_before, timeout=16.0)
                if proposal.stage == "agarre" and getattr(self.robot.snapshot(), "gripper_hold_reason", "") == "closed_limit":
                    # Cerró hasta el tope sin tocar nada: no hay cubo entre los dedos.
                    raise RuntimeError("La garra cerró hasta el tope sin agarrar el cubo (closed_limit).")
            if proposal.stage == "agarre":
                with _time_phase(timings, "after_grasp"):
                    self._wait_grip_settle()
        elif proposal.gripper_action == "open":
            with _time_phase(timings, "gripper"):
                self.status("Abriendo la garra...")
                timeout = gripper_open_timeout(self.robot.snapshot().positions.get(6))
                self.robot.send("gripper open")
                self._wait_gripper_open(timeout=timeout)
        timings["total"] = time.monotonic() - started
        return timings

    def _move_group(self, group: MotionGroup) -> None:
        self._send_group(group)
        self._wait_arrival(dict(group.targets), settle=True)

    def _send_move(self, command: str) -> None:
        """Una orden de movimiento, respetando el enfriamiento del firmware.

        Con el ESP32 (o el gemelo) espera el acuse («Moviendo ID…» / «PAIR_MOVE_OK…»): el
        firmware mide su enfriamiento cuando procesa la orden, no cuando la PC la manda, y
        un rechazo o un AJUSTE SEGURO tienen que cortar antes de la orden siguiente."""
        wait = self._last_move_sent + MOVE_SPACING_SECONDS - time.monotonic()
        if wait > 0 and self._stop_requested(wait):
            raise RuntimeError("Movimiento detenido por el operador.")
        acknowledged = isinstance(self.robot, RobotSerial)
        acks = self.robot.snapshot().move_acks if acknowledged else 0
        if self._stop_requested():
            raise RuntimeError("Movimiento detenido por el operador.")
        self.robot.send(command)
        self._last_move_sent = time.monotonic()
        if not acknowledged:
            return
        deadline = time.monotonic() + MOVE_ACK_TIMEOUT_SECONDS
        while True:
            snapshot = self.robot.snapshot()
            if snapshot.fault:
                raise RuntimeError(snapshot.fault)
            if snapshot.move_acks > acks:
                # El AJUSTE SEGURO llega antes del acuse: ya habría cortado arriba.
                self._last_move_sent = time.monotonic()
                return
            if time.monotonic() > deadline:
                raise RuntimeError(f"El firmware no confirmó la orden «{command}».")
            time.sleep(0.005)

    def _wait_grip_settle(self) -> None:
        """Después de agarrar, hasta que baje la corriente de M6 (ver GRIP_SETTLE_MA)."""
        reading = getattr(self.robot, "power_reading", None)
        started = time.monotonic()
        sample, received = reading() if callable(reading) else (None, 0.0)
        if sample is None or started - received > POWER_FRESH_SECONDS:
            if self._stop_requested(GRIP_SETTLE_SECONDS):
                raise RuntimeError("Movimiento detenido por el operador.")
            return
        self.status("Esperando a que la garra afloje antes de levantar...")
        low, last, current = 0, received, None
        deadline = started + GRIP_SETTLE_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            if self._stop_requested(0.03):
                raise RuntimeError("Movimiento detenido por el operador.")
            sample, received = reading()
            if sample is None or received <= last:
                continue
            last = received
            current = sample.currents_ma[5]
            low = low + 1 if current is not None and current < GRIP_SETTLE_MA else 0
            if low >= 2:
                return
        detail = "sin lectura" if current is None else f"{current:.0f} mA"
        self.status(f"La garra sigue consumiendo {detail} después de agarrar; se levanta igual.")

    def _send_gripper_close(self) -> None:
        self.status("Cerrando la garra por presión...")
        self.robot.send(f"gripper current {GRIPPER_CONTACT_RAW}")
        self.robot.send("gripper close")

    def _wait_gripper_hold(self, holds_before: int | None, timeout: float) -> None:
        """Hasta un GRIPPER_HOLD posterior a la orden; uno de antes no cuenta."""
        if holds_before is None:
            self._wait_gripper("holding", timeout=timeout)
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._stop_requested(0.05):
                raise RuntimeError("Movimiento detenido por el operador.")
            snapshot = self.robot.snapshot()
            if snapshot.fault:
                raise RuntimeError(snapshot.fault)
            if snapshot.gripper_holds > holds_before:
                return
        raise RuntimeError("La garra no confirmó el contacto.")

    @staticmethod
    def _sync_targets(groups: tuple[MotionGroup, ...]) -> dict[int, float] | None:
        """Destinos para una sola orden `sync`: cada motor de M1..M5 una sola vez. Si un
        motor aparece en dos grupos (un desvío del planificador), van en orden como antes."""
        motors = [motor for group in groups for motor, _ in group.targets]
        if not motors or len(motors) != len(set(motors)) or not all(1 <= motor <= 5 for motor in motors):
            return None
        return {motor: target for group in groups for motor, target in group.targets}

    @staticmethod
    def _batchable(groups: tuple[MotionGroup, ...]) -> bool:
        motors = [motor for group in groups for motor, _ in group.targets]
        return (bool(groups) and all(1 <= len(group.targets) <= 2 for group in groups)
                and len(motors) == len(set(motors)) and all(1 <= motor <= 5 for motor in motors))

    def _send_waypoint(self, groups: tuple[MotionGroup, ...], timings: dict[str, float]) -> None:
        if self._stop_requested():
            raise RuntimeError("Movimiento detenido por el operador.")
        waypoint_id = self.robot.next_waypoint_id()
        command = [f"waypoint {waypoint_id}"]
        for group in groups:
            command.append(str(len(group.targets)))
            command.extend(f"{motor} {target:.2f}" for motor, target in group.targets)
        started = time.monotonic()
        accepted = None
        heartbeat_at = started + 0.2
        complete = False
        try:
            self.robot.send(" ".join(command))
            while not self._stop_requested(0.01):
                now = time.monotonic()
                reply = self.robot.waypoint_reply(waypoint_id)
                if reply is not None:
                    state, reason = reply
                    if state in ("REJECTED", "CANCELLED"):
                        raise RuntimeError(f"Punto {waypoint_id}: {state.lower()} {reason}".rstrip())
                    if accepted is None:
                        accepted = now
                        timings["orders"] += accepted - started
                    if state == "DONE":
                        if fault := self.robot.snapshot().fault:
                            raise RuntimeError(fault)
                        timings["arrival"] += now - accepted
                        self._last_move_sent = now
                        complete = True
                        return
                if fault := self.robot.snapshot().fault:
                    raise RuntimeError(fault)
                if now - started > (30.0 if accepted is not None else 1.0):
                    raise RuntimeError(f"Sin confirmación del punto {waypoint_id}.")
                if accepted is not None and now >= heartbeat_at:
                    self.robot.send(f"waypoint keepalive {waypoint_id}")
                    heartbeat_at = now + 0.2
            raise RuntimeError("Movimiento detenido por el operador.")
        finally:
            if not complete:
                # La pérdida del enlace también hace vencer el punto en el firmware.
                try:
                    self.robot.send(f"waypoint cancel {waypoint_id}")
                except (OSError, RuntimeError):
                    pass

    def _send_group(self, group: MotionGroup) -> None:
        targets = dict(group.targets)
        if len(targets) == 1:
            motor_id, target = next(iter(targets.items()))
            self._send_move(f"{motor_id} {target:.2f}")
        elif len(targets) == 2:
            (first_id, first_target), (second_id, second_target) = targets.items()
            self._send_move(f"pair {first_id} {first_target:.2f} {second_id} {second_target:.2f}")
        else:
            raise ValueError("Solo se permiten uno o dos motores por tramo.")

    def _send_gripper_angle(self, degrees: float | None) -> None:
        if degrees is None or not math.isfinite(degrees):
            raise ValueError("Falta el ángulo de M6.")
        self.status(f"Moviendo la garra a {degrees:.0f}° por ángulo...")
        self._send_move(f"6 {degrees:.2f}")

    def _wait_arrival(self, targets: dict[int, float], *, settle: bool, speed_deg_s: float = 45.0,
                      loaded_tolerance: float = LOADED_ARRIVAL_TOLERANCE, handoff: float = 0.0,
                      timings: dict[str, float] | None = None, poll: float = 0.05) -> None:
        """Hasta que los motores lleguen. Si se pide quietud, se mide desde la primera
        lectura de la ventana aceptada hasta que termina la espera.

        `poll` menor que 0,05 s solo adelanta el pasaje a un punto de paso: la quietud y la
        llegada con desvío cuentan lecturas nuevas de telemetría, no vueltas del bucle."""
        started = time.monotonic()
        snapshot = self.robot.snapshot()
        largest = max(
            abs(snapshot.positions.get(motor_id, target) - target)
            for motor_id, target in targets.items()
        )
        timeout = min(8.0, max(2.0, 0.8 + largest / speed_deg_s))
        deadline = time.monotonic() + timeout
        # Si el supervisor de la fuente baja la velocidad, el plazo se estira igual.
        slowest = 1.0
        stable = 0
        stable_since = started
        recent: list[tuple[float, dict[int, float]]] = []
        last_reading = None
        while time.monotonic() < deadline:
            if self._stop_requested(poll):
                raise RuntimeError("Movimiento detenido por el operador.")
            snapshot = self.robot.snapshot()
            if snapshot.fault:
                raise RuntimeError(snapshot.fault)
            scale = getattr(snapshot, "speed_scale", 1.0)
            if 0 < scale < slowest:
                slowest = scale
                deadline = started + timeout / slowest
            errors = {
                motor_id: abs(snapshot.positions.get(motor_id, target + 1000.0) - target)
                for motor_id, target in targets.items()
            }
            sampled_at = time.monotonic()
            if not settle and max(errors.values()) <= max(PASS_TOLERANCE, handoff):
                if timings is not None:
                    timings["arrival"] += sampled_at - started
                return
            if poll < 0.05:
                reading = tuple(snapshot.received_at.get(motor_id) for motor_id in targets)
                if reading == last_reading:
                    continue  # sin telemetría nueva: no cuenta para la quietud
                last_reading = reading
            stable = stable + 1 if max(errors.values()) <= ARRIVAL_TOLERANCE else 0
            if stable == 1:
                stable_since = sampled_at
            if stable >= 3:
                if settle:
                    time.sleep(0.08)
                if timings is not None:
                    timings["arrival"] += stable_since - started
                    timings["settling"] += time.monotonic() - stable_since
                return
            # Con el brazo estirado y carga, el hombro cede por la gravedad y se queda quieto
            # algo antes de la orden (en el gemelo, M2 1,1–1,4°). Se acepta si quedó quieto cerca.
            positions = {motor_id: snapshot.positions.get(motor_id, 1e9) for motor_id in targets}
            recent = (recent + [(sampled_at, positions)])[-STILL_SAMPLES:]
            if (
                len(recent) == STILL_SAMPLES
                and max(errors.values()) <= loaded_tolerance
                and all(
                    max(sample[motor_id] for _, sample in recent) - min(sample[motor_id] for _, sample in recent)
                    <= STILL_RANGE
                    for motor_id in targets
                )
            ):
                detail = ", ".join(f"M{i} a {errors[i]:.1f}°" for i in targets)
                self.status(f"Llegada con desvío (motor quieto): {detail}")
                time.sleep(0.08)
                if timings is not None:
                    quiet_since = recent[0][0]
                    timings["arrival"] += quiet_since - started
                    timings["settling"] += time.monotonic() - quiet_since
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

    def __init__(
        self, policy: StagePolicy, limits: dict[int, tuple[float, float]], *,
        direct_demo: bool = False, carried_cube_check: Callable[[], Callable[[], bool] | None] | None = None,
        sync_orders: bool = False,
    ):
        self.policy = policy
        # Con `sync` cada paso es un mensaje: los pasos de un solo grupo también van
        # «juntos» (su caja ya se revisó en la cadena) y pueden encadenarse.
        self.sync_orders = sync_orders
        self.chain_step = SYNC_CHAIN_STEP_DEGREES if sync_orders else CHAIN_STEP_DEGREES
        self.blend_fraction = SYNC_BLEND_FRACTION if sync_orders else BLEND_FRACTION
        # Solo en la copia que prepara por adelantado (ver start_ahead).
        self._m6_band: tuple[float, float] | None = None
        self.direct_demo = direct_demo
        self.carried_cube_check = carried_cube_check
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
        waypoints = (*via_points, target)
        if hasattr(self.policy, "gripper_step"):
            # La política geométrica elige cómo y cuándo mover la garra (mujoco_sim.EXPERT_GRIPPER_STEPS).
            gripper_action, gripper_degrees, timing = self.policy.gripper_step(stage_index, self.scene_features)
        else:
            gripper_action, gripper_degrees, timing = LEGACY_GRIPPER_ACTIONS.get(stage), None, "after"
        m6 = current.get(6)
        if gripper_action == "angle" and (m6 is None or not math.isfinite(m6)):
            raise ValueError("Falta la lectura de M6 para mover la garra por ángulo.")
        if (stage == "traslado_2" and not via_points and gripper_action is None
                and self.direct_demo and self.carried_cube_check is not None):
            proposal = self._direct_transfer(start, m6)
            if proposal is not None:
                self.proposal = proposal
                return proposal
        moving_from = len(waypoints)
        if gripper_action in ("angle", "close"):
            moving_from = {"before": 0, "final": len(waypoints) - 1}.get(timing, len(waypoints))
        if gripper_action == "close" and moving_from < len(waypoints) and (m6 is None or not math.isfinite(m6)):
            moving_from = len(waypoints)
        try:
            groups, segments, gripper_segment = self._plan_waypoints(start, waypoints, moving_from, m6,
                                                                     gripper_degrees)
        except ValueError:
            if moving_from == len(waypoints):
                raise
            # Con la garra moviéndose no hay ruta para el peor caso: se mueve al final, quieta.
            moving_from = len(waypoints)
            groups, segments, gripper_segment = self._plan_waypoints(start, waypoints, moving_from, m6,
                                                                     gripper_degrees)
        opening_to = None
        if gripper_action == "open":
            opening_to = GRIPPER_OPEN_TARGET
        elif gripper_action == "angle" and moving_from == len(waypoints) and gripper_degrees > m6:
            opening_to = gripper_degrees
        if opening_to is not None:
            # Los tramos se revisan con la garra como está; abrirla al final agranda su
            # contorno y puede barrer la mesa si queda horizontal y baja. Si se abre mientras
            # va el brazo, los tramos ya se planificaron con la apertura más desfavorable.
            problem = gripper_sweep_problem((target[2], target[3], target[4]), self._m6_low(m6), opening_to)
            if problem:
                raise ValueError(f"{STAGE_LABELS[stage]}: no se puede abrir la garra ahí: {problem}.")
        at_group = None
        if gripper_action in ("angle", "close") and gripper_segment is not None:
            at_group = segments[gripper_segment].start
        self.proposal = MotionProposal(
            stage_index=stage_index,
            stage=stage,
            label=STAGE_LABELS[stage],
            target=target,
            groups=tuple(groups),
            gripper_action=gripper_action,
            gripper_degrees=gripper_degrees,
            gripper_at_group=at_group,
            segments=tuple(segments),
        )
        return self.proposal

    def _direct_transfer(self, start, m6) -> MotionProposal | None:
        if m6 is None or not math.isfinite(m6) or self.policy.container_obstacle:
            return None
        index = KEYFRAME_STAGES.index("traslado_3")
        scene = self.scene_features
        if self.policy.via(index, scene) or self.policy.gripper_step(index, scene)[0] is not None:
            return None
        check = self.carried_cube_check()
        if check is None:
            return None
        target = self.policy.predict(index, scene)
        chain = self._straight_chain(start, target, m6, (m6,), 1, direct=True, carried_cube_clears=check)
        if chain is None:
            return None
        groups, _ = chain[0]
        return MotionProposal(
            index, "traslado_3", STAGE_LABELS["traslado_3"], target, tuple(groups), None,
            segments=(MotionSegment(0, len(groups), len(groups) > 1, True),),
        )

    def _plan_waypoints(self, start, waypoints, moving_from, m6, gripper_degrees):
        """Grupos y segmentos para recorrer `waypoints` desde `start` (que se actualiza).

        Desde el punto `moving_from` la garra se mueve mientras va el brazo: esos tramos se
        planifican sin M6 conocido, con la apertura más desfavorable para la mesa."""
        start = dict(start)
        initial = dict(start)
        groups: list[MotionGroup] = []
        segments: list[MotionSegment] = []
        gripper_segment = None  # primer segmento con la garra en movimiento
        # Cerrar por presión termina en GRIPPER_CLOSED (o antes, si toca algo).
        m6_goal = GRIPPER_CLOSED_DEGREES if gripper_degrees is None else gripper_degrees
        for index, waypoint in enumerate(waypoints):
            moving = index >= moving_from
            if index == moving_from and m6_goal > self._m6_low(m6):
                # Abre mientras va el brazo: la apertura empieza en esta pose, que tiene que
                # admitir todo el barrido sin acercar la garra a la mesa (con el M6 real, no
                # con el escape de la garra ya abierta) y quedar sobre el margen con cualquier
                # apertura. Si no, la garra abre recién al llegar.
                planar = (start[2], start[3], start[4])
                problem = gripper_sweep_problem(planar, self._m6_low(m6), m6_goal)
                if problem or floor_escape(planar, None) is not None:
                    raise ValueError(f"no se puede abrir la garra en camino: {problem or 'brazo bajo'}")
            if index == moving_from:
                gripper_segment = len(segments)
            unknown_m6 = moving or self._rules_m6(m6) is None
            planning = {key: value for key, value in start.items() if not (unknown_m6 and key == 6)}
            m6_values = self._static_m6_values(m6)
            if moving:
                m6_values = self._sweep_m6_values(m6, m6_goal)
            m6_rules = None if moving else self._rules_m6(m6)
            last_waypoint = index == len(waypoints) - 1
            chain = None
            if self._chainable(start, waypoint):
                stage = KEYFRAME_STAGES[self.next_stage_index]
                direct_stage = stage in ("aproximacion_1", "guardado") or (
                    stage == "traslado_2" and len(waypoints) == 1 and not moving
                )
                if (self.direct_demo and last_waypoint and direct_stage
                        and getattr(self.policy, "container_obstacle", True) is False
                        and all(angle is not None and math.isfinite(angle) for angle in m6_values)):
                    chain = self._straight_chain(planning, waypoint, m6_rules, m6_values, 1, direct=True)
                    if chain is not None and segments:
                        # El regreso largo empieza después de confirmar la retirada vertical.
                        segments[-1] = dataclasses.replace(segments[-1], final=True)
                if chain is None:
                    chain = self._chain(planning, waypoint, m6_rules, m6_values)
            if chain is not None:
                # Paso a paso sobre la línea recta, con todos los motores juntos en cada uno.
                for offset, (planned, after) in enumerate(chain):
                    first = len(groups)
                    groups.extend(planned)
                    segments.append(MotionSegment(first, len(groups), len(planned) > 1 or self.sync_orders,
                                                  last_waypoint and offset == len(chain) - 1))
                    start.update(after)
                continue
            before = dict(start)
            planned = build_motion_plan(planning, waypoint, self.limits)
            first = len(groups)
            groups.extend(planned)
            # El planificador no mueve un motor por menos de 0,5°: el tramo siguiente
            # parte de lo ordenado, no del punto. Si no, con puntos cercanos (el empuje
            # avanza de a 4 mm) un motor que cambia poco en cada uno nunca se movería.
            for group in planned:
                start.update(dict(group.targets))
            together = len(planned) > 1 and self._together_problem(before, start, m6_rules, m6_values) is None
            segments.append(MotionSegment(first, len(groups), together, last_waypoint))
        return groups, self._blend(groups, segments, initial, gripper_segment, m6, m6_goal), gripper_segment

    # ----- M6 dentro de una banda (preparación anticipada) -----
    # Sin banda, M6 es la lectura actual. Con banda (lo, hi), el plan tiene que valer para
    # cualquier M6 entre lo y hi: el 3D se revisa con muestras de la banda a ≤8° (como el
    # barrido de la garra) y el 2D con el contorno de M6, que es el mismo en toda la banda
    # o, si no, el más desfavorable (M6 desconocido).
    def _m6_low(self, m6):
        return m6 if self._m6_band is None else self._m6_band[0]

    def _rules_m6(self, m6):
        band = self._m6_band
        if band is None:
            return m6
        return m6 if gripper_envelope(band[0]) == gripper_envelope(band[1]) else None

    def _static_m6_values(self, m6):
        band = self._m6_band
        if band is None:
            return (m6,)
        return self._gripper_sweep_samples(band[0], band[1])

    def _sweep_m6_values(self, m6, goal):
        band = self._m6_band
        if band is None:
            return self._gripper_sweep_samples(m6, goal)
        far = band[1] if goal <= band[0] else band[0]  # el extremo de la banda más lejos del destino
        return tuple(sorted(set(self._gripper_sweep_samples(far, goal)) | set(self._static_m6_values(m6))))

    @staticmethod
    def _gripper_sweep_samples(start, goal):
        count = max(1, math.ceil(abs(goal - start) / 8.0))
        return tuple(start + (goal - start) * index / count for index in range(count + 1))

    def _chainable(self, start, waypoint) -> bool:
        """Solo con el modelo 3D (política geométrica) y si el salto no es chico."""
        return hasattr(self.policy, "motion_problems") and max(
            abs(waypoint[motor] - start[motor]) for motor in range(1, 6)) > TOGETHER_MAX_DEGREES

    def _chain(self, planning, waypoint, m6, m6_values):
        """[(grupos, pose ordenada M1–M5), ...] para ir de a CHAIN_STEP_DEGREES por la recta de
        `planning` a `waypoint`; None si algún paso no puede ir con los motores juntos (entonces
        el salto va como antes)."""
        origin = {motor: planning[motor] for motor in range(1, 6)}
        largest = max(abs(waypoint[motor] - origin[motor]) for motor in range(1, 6))
        count = max(2, math.ceil(largest / self.chain_step))
        return self._straight_chain(planning, waypoint, m6, m6_values, count)

    def _straight_chain(
        self, planning, waypoint, m6, m6_values, count, *, direct=False, carried_cube_clears=None,
    ):
        origin = {motor: planning[motor] for motor in range(1, 6)}
        current = dict(planning)
        chain = []
        for step in range(1, count + 1):
            target = {motor: origin[motor] + (waypoint[motor] - origin[motor]) * step / count
                      for motor in range(1, 6)}
            try:
                planned = build_motion_plan(current, target, self.limits)
            except ValueError:
                return None
            after = dict(current)
            moved = [motor for group in planned for motor, _ in group.targets]
            for group in planned:
                after.update(dict(group.targets))
            if len(moved) != len(set(moved)) or any(abs(after[motor] - target[motor]) > 0.5 + 1e-9
                                                    for motor in range(1, 6)):
                return None  # el planificador 2D tuvo que desviarse de la recta
            limit = TOGETHER_MAX_DEGREES
            if direct:
                limit = max(abs(after[motor] - current[motor]) for motor in origin) + 1e-8
            if not planned or self._together_problem(current, after, m6, m6_values, limit=limit) is not None:
                return None
            if direct:
                axes = [sorted({current[m], after[m], (current[m] + after[m]) / 2}) for m in origin]
                for *angles, gripper in product(*axes, m6_values):
                    pose = {**dict(enumerate(angles, 1)), 6: gripper}
                    if self.policy.arm.pose_problems(pose) or (
                        carried_cube_clears is not None and not carried_cube_clears()
                    ):
                        return None
            chain.append((planned, {motor: after[motor] for motor in range(1, 6)}))
            current = after
        return chain

    def _blend(self, groups, segments, initial, gripper_segment, m6, m6_goal):
        """Marca los puntos de paso donde se puede mandar el siguiente a mitad de camino: los
        dos tramos van con los motores juntos y la caja incluye los tres puntos y el posible
        rezago de las órdenes anteriores."""
        poses = []
        current = {motor: initial[motor] for motor in range(1, 6)}
        for segment in segments:
            for group in groups[segment.start:segment.end]:
                current.update(dict(group.targets))
            poses.append(dict(current))
        result = list(segments)
        before = {motor: initial[motor] for motor in range(1, 6)}
        entry_bounds = (before, before)
        for index, segment in enumerate(segments[:-1]):
            following = segments[index + 1]
            if segment.final or not segment.together or not following.together:
                before = poses[index]
                entry_bounds = (before, before)
                continue
            moving = gripper_segment is not None and index + 1 >= gripper_segment
            m6_rules = None if moving else self._rules_m6(m6)
            m6_values = self._sweep_m6_values(m6, m6_goal) if moving else self._static_m6_values(m6)
            target = poses[index]
            step = max(abs(target[motor] - before[motor]) for motor in range(1, 6))
            corners = (*entry_bounds, before, target, poses[index + 1])
            if self._together_problem(before, poses[index + 1], m6_rules, m6_values, poses=corners,
                                      limit=BLEND_MAX_DEGREES) is None:
                handoff = self.blend_fraction * step
                result[index] = dataclasses.replace(segment, handoff=handoff)
                # Al encadenar de nuevo, un motor que no recibió orden en este tramo
                # todavía puede estar llegando al destino del tramo anterior.
                low, high = entry_bounds
                tolerance = max(PASS_TOLERANCE, handoff)
                moved = {motor for group in groups[segment.start:segment.end] for motor, _ in group.targets}
                entry_bounds = (
                    {motor: max(min(low[motor], before[motor], target[motor]), target[motor] - tolerance)
                     if motor in moved else low[motor] for motor in target},
                    {motor: min(max(high[motor], before[motor], target[motor]), target[motor] + tolerance)
                     if motor in moved else high[motor] for motor in target},
                )
            else:
                entry_bounds = (target, target)
            before = target
        return result

    def _together_problem(self, before, after, m6, m6_values, *, poses=None,
                          limit: float = TOGETHER_MAX_DEGREES) -> str | None:
        """None si los motores pueden ir a la vez de `before` a `after`.

        Con los grupos seguidos, cada motor llega a su ritmo: el brazo pasa por cualquier
        combinación intermedia. Se revisan las esquinas y aristas de la caja que forman
        (reglas 2D del firmware, en el peor caso de M6 si la garra se mueve) y, si la
        política tiene el modelo 3D, sus choques. Solo para saltos de hasta
        TOGETHER_MAX_DEGREES (`limit`). `poses`: varios puntos; la caja va del mínimo al
        máximo de cada motor entre ellos (al encadenar, el brazo corta camino entre tres)."""
        poses = poses or (before, after)
        low = {motor: min(pose[motor] for pose in poses) for motor in range(1, 6)}
        high = {motor: max(pose[motor] for pose in poses) for motor in range(1, 6)}
        changed = [motor for motor in range(1, 6) if high[motor] - low[motor] > 1e-9]
        if not changed:
            return None
        if max(high[motor] - low[motor] for motor in changed) > limit:
            return "salto grande"
        planar = [motor for motor in (2, 3, 4) if motor in changed]
        start_planar = tuple(before[motor] for motor in (2, 3, 4))
        escape_from = floor_escape(start_planar, m6)

        def corner(mask):
            return tuple(high[motor] if motor in planar and mask & (1 << planar.index(motor)) else
                         (low[motor] if motor in planar else before[motor]) for motor in (2, 3, 4))

        masks = range(1 << len(planar))
        for mask in masks:
            if not classify_pose(corner(mask), m6, escape_from)[0]:
                return "esquina fuera de las reglas"
            for bit in range(len(planar)):
                if not mask & (1 << bit) and not edge_safe(corner(mask), corner(mask | (1 << bit)), m6=m6,
                                                            escape_from=escape_from):
                    return "arista fuera de las reglas"
        if hasattr(self.policy, "motion_problems"):
            problems = self.policy.motion_problems(low, high, m6_values)
            if problems:
                return "; ".join(problems)
        return None

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

    # ----- preparación anticipada -----
    def start_ahead(self, observations: list[SceneObservation], stage: str = "guardado") -> "AheadPlan | None":
        """Empieza a preparar `stage` en otro hilo, desde la pose final prevista de la etapa
        anterior: su destino según la política y M6 en la banda de su acción de garra.

        Se llama apenas se conoce la escena (después de preparar la etapa 1). El hilo usa
        una copia de la política con su propio modelo 3D: no comparte datos con la
        planificación de las otras etapas, que sigue en paralelo."""
        if stage not in ANTICIPATED_STAGES or self.scene_features is None or self.finished:
            return None
        index = KEYFRAME_STAGES.index(stage)
        previous = index - 1
        if self.next_stage_index > index or not hasattr(self.policy, "gripper_step"):
            return None
        action, degrees, _ = self.policy.gripper_step(previous, self.scene_features)
        if action != "angle" or degrees is None:
            return None  # abrir del todo o cerrar por presión: no se sabe dónde queda M6
        # El ejecutor da la garra por llegada a GRIPPER_OPEN_TOLERANCE del ángulo pedido.
        center = float(degrees)
        band = (max(self.limits[6][0], center - GRIPPER_OPEN_TOLERANCE),
                min(self.limits[6][1], center + GRIPPER_OPEN_TOLERANCE))
        assumed = {motor: float(value) for motor, value in self.policy.predict(previous, self.scene_features).items()
                   if motor in range(1, 6)}
        plan = AheadPlan(index, dict(assumed), band)
        state = copy.copy(self)
        state.proposal = None
        state.next_stage_index = index
        state.finished = False
        state._m6_band = band

        def work() -> None:
            started = time.monotonic()
            try:
                state.policy = _independent_policy(self.policy)
                plan.proposal = state.prepare({**assumed, 6: center}, observations)
            except Exception as exc:  # se replanifica normalmente y ahí se informa
                plan.error = exc
            finally:
                plan.seconds = time.monotonic() - started

        plan.thread = threading.Thread(target=work, name="preparar-anticipado", daemon=True)
        plan.thread.start()
        return plan

    def adopt_ahead(self, plan: "AheadPlan", current: dict[int, float],
                    timeout: float = 30.0) -> tuple[MotionProposal | None, str]:
        """El plan anticipado si sigue valiendo para la pose medida; si no, (None, motivo)
        y hay que llamar a prepare() como siempre."""
        started = time.monotonic()
        plan.thread.join(timeout)
        waited = time.monotonic() - started
        if plan.thread.is_alive():
            return None, "la preparación anticipada no terminó a tiempo"
        if self.finished or self.proposal is not None or self.next_stage_index != plan.stage_index:
            return None, "cambió la etapa"
        if plan.error is not None or plan.proposal is None:
            return None, f"la preparación anticipada falló: {plan.error}"
        if any(motor not in current for motor in range(1, 7)):
            return None, "falta la lectura de algún motor"
        error = max(abs(current[motor] - plan.assumed[motor]) for motor in range(1, 6))
        if error > ANTICIPATED_POSE_TOLERANCE:
            return None, f"el brazo quedó a {error:.1f}° de lo ordenado"
        low, high = plan.m6_band
        if not low <= current[6] <= high:
            return None, f"M6 quedó en {current[6]:.1f}°, fuera de {low:.1f}–{high:.1f}°"
        self.proposal = plan.proposal
        return plan.proposal, f"preparada por adelantado ({plan.seconds:.1f} s; se esperó {waited:.1f} s)"



def _independent_policy(policy):
    """Copia de la política con su propio modelo 3D (las revisiones escriben en el MjData
    del modelo). El plan del experto ya calculado se comparte: no cambia."""
    arm = getattr(policy, "arm", None)
    if arm is None or not hasattr(arm, "pose_problems"):
        return policy
    clone = copy.copy(policy)
    clone.arm = type(arm)(arm.assets_root, limits=arm.limits, dimensions=arm.dimensions,
                          floor_margin_mm=arm.floor_margin_mm)
    return clone


@dataclass
class AheadPlan:
    stage_index: int
    assumed: dict[int, float]          # M1–M5 ordenados al final de la etapa anterior
    m6_band: tuple[float, float]       # M6 con que vale el plan
    thread: threading.Thread | None = None
    proposal: MotionProposal | None = None
    error: Exception | None = None
    seconds: float = 0.0


def canonical_pose_text(values: np.ndarray) -> str:
    return " | ".join(f"M{motor_id}={values[motor_id - 1]:.1f}°" for motor_id in range(1, 6))

