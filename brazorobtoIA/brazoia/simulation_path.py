"""Trayectorias exclusivamente virtuales; no genera órdenes para el robot."""
from __future__ import annotations

from dataclasses import dataclass
from itertools import permutations
import math
import time

from .config import INITIAL_POSE


def validate_pose(pose, limits):
    result = {}
    for i in range(1, 7):
        try:
            value = float(pose[i])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Falta un ángulo válido de M{i}.") from exc
        low, high = limits[i]
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"M{i}={value:g}° fuera de {low:g}–{high:g}°. No se corrigió la pose.")
        result[i] = value
    return result


def pose_from_snapshot(snapshot, *, now, maximum_age):
    """Lee una copia ya recibida; nunca solicita telemetría ni cambia torque."""
    for i in range(1, 7):
        age = now - snapshot.received_at.get(i, -math.inf)
        if i not in snapshot.positions or not 0 <= age <= maximum_age:
            raise ValueError(f"Falta telemetría reciente de M{i}. Conectá el ESP32 en la aplicación principal.")
        if not math.isfinite(float(snapshot.positions[i])):
            raise ValueError(f"Telemetría inválida de M{i}.")
    return {i: float(snapshot.positions[i]) for i in range(1, 7)}


class SimulationCancelled(ValueError):
    pass


@dataclass(frozen=True)
class SimulationTrajectory:
    waypoints: tuple[dict[int, float], ...]
    minimum_bound_mm: float
    margin_mm: float
    kind: str
    evaluations: int

    def duration(self, degrees_per_second=20.0):
        if not math.isfinite(degrees_per_second) or degrees_per_second <= 0:
            raise ValueError("Velocidad virtual inválida")
        return sum(max(abs(b[i] - a[i]) for i in a) / degrees_per_second
                   for a, b in zip(self.waypoints, self.waypoints[1:]))

    def pose_at(self, seconds, degrees_per_second=20.0):
        self.duration(degrees_per_second)  # Valida la velocidad también para poses idénticas.
        remaining = max(0.0, seconds)
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            duration = max(abs(b[i] - a[i]) for i in a) / degrees_per_second
            if duration and remaining < duration:
                fraction = remaining / duration
                return {i: a[i] + (b[i] - a[i]) * fraction for i in a}
            remaining -= duration
        return dict(self.waypoints[-1])


class FloorPathPlanner:
    """Verifica interpolación continua usando una cota conservadora de altura.

    Busca recta articular y, si falla, rutas pasando por una pose elevada,
    probando los seis órdenes de M2/M3/M4. No es un planificador completo:
    no encontrar ruta no demuestra que no exista. Solo verifica garra/piso.
    """
    def __init__(self, geometry, limits, *, margin_mm=5.0, cancel=None, timeout=15.0):
        if not math.isfinite(margin_mm) or margin_mm < 5:
            raise ValueError("El margen de esta prueba debe ser de al menos 5 mm.")
        self.geometry, self.limits, self.margin = geometry, limits, margin_mm
        self.cancel, self.timeout = cancel, timeout
        self.cache = {}
        self.deadline = math.inf

    def check_cancel(self):
        if self.cancel is not None and self.cancel.is_set():
            raise SimulationCancelled("Cálculo cancelado.")
        if time.monotonic() > self.deadline:
            raise ValueError("Se agotó el tiempo de búsqueda. No se certificó un recorrido; probá otra pose.")

    def height(self, pose):
        self.check_cancel()
        key = tuple(pose[i] for i in range(1, 7))
        if key not in self.cache:
            height = float(self.geometry.clearance(pose)["height_mm"])
            if not math.isfinite(height):
                raise ValueError("El modelo devolvió una altura inválida.")
            self.cache[key] = height
        return self.cache[key]

    def edge_bound(self, start, end, depth=0):
        low = min(self.height(start), self.height(end))
        if low <= self.margin:
            return None
        # Todo punto del intervalo está a <= media longitud de uno de los extremos.
        bound = low - self.geometry.motion_bound_mm(start, end) / 2
        if bound > self.margin:
            return bound
        if depth >= 18:
            return None  # No se pudo certificar: nunca asumir que pasa.
        middle = {i: (start[i] + end[i]) / 2 for i in start}
        left = self.edge_bound(start, middle, depth + 1)
        if left is None:
            return None
        right = self.edge_bound(middle, end, depth + 1)
        return None if right is None else min(left, right)

    def check_path(self, path):
        bounds = []
        for a, b in zip(path, path[1:]):
            bound = self.edge_bound(a, b)
            if bound is None:
                return None
            bounds.append(bound)
        return min(bounds) if bounds else self.height(path[0])

    def lift(self, pose):
        high = validate_pose({**pose, **{i: INITIAL_POSE[i] for i in (2, 3, 4)}}, self.limits)
        if self.check_path([pose, high]) is not None:
            return [pose, high]
        for order in permutations((2, 3, 4)):
            path = [pose]
            for motor in order:
                path.append({**path[-1], motor: high[motor]})
            if self.check_path(path) is not None:
                return path
        return None

    def plan(self, start, goal):
        self.cache.clear()
        self.deadline = time.monotonic() + self.timeout
        start, goal = validate_pose(start, self.limits), validate_pose(goal, self.limits)
        for label, pose in (("Inicio", start), ("Final", goal)):
            height = self.height(pose)
            if height <= self.margin:
                raise ValueError(f"{label}: la garra queda a {height:.1f} mm del piso virtual; "
                                 f"se requieren más de {self.margin:g} mm. Revisá la pose o la calibración.")
        path = [start, goal]
        bound = self.check_path(path)
        kind = "Directo"
        if bound is None:
            first, last = self.lift(start), self.lift(goal)
            if first is None or last is None:
                raise ValueError("No se encontró un recorrido con margen en las rutas probadas. No se reproducirá.")
            path = first + list(reversed(last))
            bound = self.check_path(path)
            kind = "Con elevación intermedia"
            if bound is None:
                raise ValueError("La conexión entre poses elevadas no mantiene el margen. No se reproducirá.")
        unique = [path[0]]
        for pose in path[1:]:
            if pose != unique[-1]:
                unique.append(pose)
        return SimulationTrajectory(tuple(dict(p) for p in unique), bound, self.margin, kind, len(self.cache))
