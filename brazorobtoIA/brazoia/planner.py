from __future__ import annotations

import heapq
import math
from dataclasses import dataclass


PLANAR_LIMITS = {2: (90.0, 270.0), 3: (90.0, 276.0), 4: (75.0, 278.0)}
# Cadena plana del modelo 3D calibrado (URDF + calibracion.json con los toques de
# mesa del 30/10/2026), en mm y grados. tests/test_planner.py verifica que coincida
# con GripperGeometry: si cambia la calibración, recalcularla acá y en el firmware.
H_BASE, W_BASE = 121.9, 35.0
L1, OFFSET_M3, L2, L3 = 112.814, 27.0, 135.0, 60.1
M2_ZERO, M3_ZERO, M4_ZERO = -86.1243, -97.7163, -175.0883
# Rectángulo de la garra para autocolisiones, hasta la punta de los dedos.
GRIPPER_LEN, GRIPPER_WIDTH = 106.4, 65.0
# Único límite de piso: el contorno de la garra a más de 5 mm de la mesa.
FLOOR_TOUCH = 5.0
# Contorno de la garra alrededor del eje de M5, válido para cualquier giro de M5,
# según la apertura de M6: (M6 máximo, ((t inicial, t final, radio), ...)) en mm,
# con t medido a lo largo de los dedos desde el eje de M5. Sale de las mallas con
# 1 mm de margen. Al abrirse, el dedo móvil sobresale hasta 104 mm del eje.
GRIPPER_ENVELOPES = (
    (12.0, ((-1.0, 60.0, 40.0), (60.0, 80.0, 31.0), (80.0, 95.0, 31.0), (95.0, 108.0, 25.0))),
    (47.0, ((-1.0, 20.0, 39.0), (20.0, 40.0, 44.0), (40.0, 60.0, 55.0), (60.0, 80.0, 65.0),
            (80.0, 95.0, 68.0), (95.0, 108.0, 65.0))),
    (128.0, ((-15.0, 0.0, 101.0), (0.0, 108.0, 104.0))),
    (math.inf, ((-26.0, 0.0, 98.0), (0.0, 20.0, 81.0), (20.0, 40.0, 49.0), (40.0, 60.0, 39.0),
                (60.0, 80.0, 30.0), (80.0, 95.0, 23.0), (95.0, 108.0, 18.0))),
)
# Sin lectura de M6 vale el contorno de cualquier apertura.
UNKNOWN_GRIPPER_ENVELOPE = ((-26.0, 0.0, 101.0), (0.0, 108.0, 104.0))
# Si la pose de partida ya está bajo el margen de piso (por ejemplo, la garra se
# cerró sobre el cubo cerca de la mesa), se aceptan poses que no queden más cerca
# de la mesa que la de partida, para poder salir.
ESCAPE_TOLERANCE = 0.5


def _add(a, b):
    return a[0] + b[0], a[1] + b[1]


def _sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def _scale(value, amount):
    return value[0] * amount, value[1] * amount


def _length(value):
    return math.hypot(value[0], value[1])


def _vec(size, angle):
    return size * math.cos(angle), size * math.sin(angle)


def _point_segment_distance(point, start, end):
    segment, relative = _sub(end, start), _sub(point, start)
    denominator = segment[0] ** 2 + segment[1] ** 2
    if denominator <= 1e-9:
        return _length(_sub(point, start))
    amount = max(
        0.0,
        min(
            1.0,
            (relative[0] * segment[0] + relative[1] * segment[1]) / denominator,
        ),
    )
    return _length(_sub(point, _add(start, _scale(segment, amount))))


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a, b, point):
    return (
        min(a[0], b[0]) <= point[0] <= max(a[0], b[0])
        and min(a[1], b[1]) <= point[1] <= max(a[1], b[1])
    )


def _segments_intersect(a, b, c, d):
    values = _orient(a, b, c), _orient(a, b, d), _orient(c, d, a), _orient(c, d, b)
    if abs(values[0]) < 1e-6 and _on_segment(a, b, c):
        return True
    if abs(values[1]) < 1e-6 and _on_segment(a, b, d):
        return True
    if abs(values[2]) < 1e-6 and _on_segment(c, d, a):
        return True
    if abs(values[3]) < 1e-6 and _on_segment(c, d, b):
        return True
    return (values[0] > 0) != (values[1] > 0) and (values[2] > 0) != (
        values[3] > 0
    )


def compute_geometry(pose):
    m2, m3, m4 = pose
    first_angle = math.radians(m2 + M2_ZERO)
    second_angle = math.radians(m3 + M3_ZERO)
    third_angle = math.radians(m4 + M4_ZERO)
    p0 = (0.0, H_BASE)
    p1 = _add(p0, _vec(L1, first_angle))
    p1_offset = _add(p1, _vec(OFFSET_M3, first_angle + math.pi / 2.0))
    p2 = _add(p1_offset, _vec(L2, first_angle + second_angle))
    p3 = _add(p2, _vec(L3, first_angle + second_angle + third_angle))
    points = [(-p[0], p[1]) for p in (p0, p1, p1_offset, p2, p3)]
    p0, p1, p1_offset, p2, p3 = points
    direction = _sub(p3, p2)
    size = _length(direction) or 1.0
    unit = direction[0] / size, direction[1] / size
    normal = -unit[1], unit[0]
    half = GRIPPER_WIDTH / 2.0
    corners = [_add(p3, _scale(normal, -half)), _add(p3, _scale(normal, half))]
    corners += [
        _add(corners[1], _scale(unit, GRIPPER_LEN)),
        _add(corners[0], _scale(unit, GRIPPER_LEN)),
    ]
    return points, corners


def gripper_envelope(m6=None):
    if m6 is None or not math.isfinite(m6):
        return UNKNOWN_GRIPPER_ENVELOPE
    return next(segments for top, segments in GRIPPER_ENVELOPES if m6 <= top)


def floor_clearance(pose, m6=None):
    """Altura mínima (mm) del contorno de la garra sobre la mesa, con cualquier giro de M5."""
    points, _ = compute_geometry(pose)
    p2, p3 = points[3], points[4]
    direction = _sub(p3, p2)
    size = _length(direction) or 1.0
    unit = direction[0] / size, direction[1] / size
    normal = -unit[1], unit[0]
    return min(
        p3[1] + along * unit[1] + side * radius * normal[1]
        for start, end, radius in gripper_envelope(m6)
        for along in (start, end)
        for side in (-1.0, 1.0)
    )


def floor_escape(start, m6=None):
    """Altura de piso de `start` si ya está bajo el margen; si no, None."""
    clearance = floor_clearance(start, m6)
    return clearance if clearance <= FLOOR_TOUCH else None


def _collides_base(corners):
    x0, x1, y0, y1 = -W_BASE / 2.0, W_BASE / 2.0, 0.0, H_BASE
    if any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in corners):
        return True
    rectangle = [
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ]
    return any(
        _segments_intersect(corners[index], corners[(index + 1) % 4], a, b)
        for index in range(4)
        for a, b in rectangle
    )


def _collides_link(corners, start, end, thickness=25.0):
    radius = thickness / 2.0
    if any(_point_segment_distance(corner, start, end) <= radius for corner in corners):
        return True
    for index in range(4):
        edge_start, edge_end = corners[index], corners[(index + 1) % 4]
        if _segments_intersect(edge_start, edge_end, start, end):
            return True
        if _point_segment_distance(edge_start, start, end) <= radius:
            return True
        if _point_segment_distance(edge_end, start, end) <= radius:
            return True
    return False


def dynamic_m4_max(m2, m3):
    value = 278.0
    if m2 <= 100 and m3 <= 110:
        value = min(value, 130.0)
    if m2 <= 120 and m3 <= 100:
        value = min(value, 120.0)
    if m2 >= 240 and m3 >= 140:
        value = min(value, 200.0)
    if m2 >= 260:
        value = min(value, 185.0)
    if m3 >= 250:
        value = min(value, 230.0)
    if m2 >= 185 and m3 >= 245:
        value = min(value, 250.0)
    if m2 >= 200 and m3 >= 255:
        value = min(value, 245.0)
    return value


def classify_pose(pose, m6=None, escape_from=None):
    """`m6`: apertura de la garra durante el movimiento; sin dato, la más desfavorable.
    `escape_from`: altura de piso de la pose de partida, si ya estaba bajo el margen."""
    m2, m3, m4 = pose
    if not all(
        PLANAR_LIMITS[motor_id][0] <= value <= PLANAR_LIMITS[motor_id][1]
        for motor_id, value in zip((2, 3, 4), pose)
    ):
        return False, "LIMITE"
    if m4 > dynamic_m4_max(m2, m3):
        return False, "REGLA_M4"
    points, corners = compute_geometry(pose)
    clearance = floor_clearance(pose, m6)
    if clearance <= FLOOR_TOUCH and (escape_from is None or clearance < escape_from - ESCAPE_TOLERANCE):
        return False, "PISO"
    if _collides_base(corners):
        return False, "COLISION_BASE"
    if _collides_link(corners, points[0], points[1]):
        return False, "COLISION_L1"
    if _collides_link(corners, points[2], points[3]):
        return False, "COLISION_L2"
    return True, "OK"


def edge_safe(start, end, resolution=1.0, m6=None, escape_from=None):
    distance = max(abs(a - b) for a, b in zip(start, end))
    steps = max(1, math.ceil(distance / resolution))
    for index in range(1, steps + 1):
        amount = index / steps
        pose = tuple(a + (b - a) * amount for a, b in zip(start, end))
        if not classify_pose(pose, m6, escape_from)[0]:
            return False
    return True


def plan_path(start, goal, step=5.0, m6=None, escape_from=None):
    start = tuple(float(value) for value in start)
    goal = tuple(float(value) for value in goal)
    start_ok, start_reason = classify_pose(start, m6, escape_from)
    goal_ok, goal_reason = classify_pose(goal, m6, escape_from)
    if not start_ok:
        return None, f"Pose inicial insegura: {start_reason}"
    if not goal_ok:
        return None, f"Pose objetivo insegura: {goal_reason}"
    if edge_safe(start, goal, m6=m6, escape_from=escape_from):
        return [goal], "OK"

    rounded_goal = tuple(round(value / step) * step for value in goal)
    if not classify_pose(rounded_goal, m6, escape_from)[0]:
        rounded_goal = goal
    rounded_start = tuple(round(value / step) * step for value in start)
    grid_start = (
        rounded_start
        if classify_pose(rounded_start, m6, escape_from)[0]
        and edge_safe(start, rounded_start, m6=m6, escape_from=escape_from)
        else start
    )

    def heuristic(node):
        return sum(abs(a - b) for a, b in zip(node, rounded_goal))

    frontier = [(heuristic(grid_start), 0.0, grid_start)]
    previous = {grid_start: None}
    cost = {grid_start: 0.0}
    found = None
    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current_cost != cost[current]:
            continue
        if max(abs(a - b) for a, b in zip(current, rounded_goal)) <= step and edge_safe(
            current, goal, m6=m6, escape_from=escape_from
        ):
            found = current
            break
        for axis in range(3):
            for direction in (-1, 1):
                candidate = list(current)
                candidate[axis] = round((candidate[axis] + direction * step) / step) * step
                candidate = tuple(candidate)
                motor_id = axis + 2
                if not PLANAR_LIMITS[motor_id][0] <= candidate[axis] <= PLANAR_LIMITS[motor_id][1]:
                    continue
                if candidate in previous or not classify_pose(candidate, m6, escape_from)[0]:
                    continue
                if not edge_safe(current, candidate, m6=m6, escape_from=escape_from):
                    continue
                next_cost = current_cost + step
                previous[candidate] = current
                cost[candidate] = next_cost
                heapq.heappush(
                    frontier,
                    (next_cost + heuristic(candidate), next_cost, candidate),
                )
    if found is None:
        return None, "No se encontró una ruta segura."
    path = []
    current = found
    while current is not None:
        path.append(current)
        current = previous[current]
    path.reverse()
    if path[0] != start:
        path.insert(0, start)
    if path[-1] != goal:
        path.append(goal)
    compressed = [path[0]]
    for pose in path[1:]:
        if len(compressed) >= 2:
            anchor, previous_pose = compressed[-2], compressed[-1]
            axis_a = [i for i in range(3) if abs(previous_pose[i] - anchor[i]) > 1e-6]
            axis_b = [i for i in range(3) if abs(pose[i] - previous_pose[i]) > 1e-6]
            if axis_a == axis_b and edge_safe(anchor, pose, m6=m6, escape_from=escape_from):
                compressed[-1] = pose
                continue
        compressed.append(pose)
    return compressed[1:], "OK"


@dataclass(frozen=True)
class MotionGroup:
    targets: tuple[tuple[int, float], ...]


def build_motion_plan(
    current: dict[int, float],
    target: dict[int, float],
    limits: dict[int, tuple[float, float]],
    *,
    tolerance: float = 0.5,
) -> list[MotionGroup]:
    for motor_id, value in target.items():
        low, high = limits[motor_id]
        if not low <= value <= high:
            raise ValueError(f"M{motor_id} fuera de límite: {value:.1f}°")
    if any(motor_id not in current for motor_id in range(1, 6)):
        raise ValueError("Falta la pose actual de M1..M5.")

    start_planar = tuple(current[motor_id] for motor_id in (2, 3, 4))
    goal_planar = tuple(target.get(motor_id, current[motor_id]) for motor_id in (2, 3, 4))
    m6 = current.get(6)  # La garra no cambia mientras se mueve el brazo.
    escape_from = floor_escape(start_planar, m6)
    waypoints, reason = plan_path(start_planar, goal_planar, m6=m6, escape_from=escape_from)
    if waypoints is None:
        raise ValueError(reason)

    groups: list[MotionGroup] = []
    auxiliary = [
        (motor_id, target[motor_id])
        for motor_id in (1, 5)
        if motor_id in target and abs(target[motor_id] - current[motor_id]) >= tolerance
    ]
    if auxiliary:
        groups.append(MotionGroup(tuple(auxiliary)))

    previous_planar = start_planar
    for waypoint in waypoints:
        changed = tuple(
            (motor_id, waypoint[index])
            for index, motor_id in enumerate((2, 3, 4))
            if abs(waypoint[index] - previous_planar[index]) >= tolerance
        )
        if len(changed) <= 2:
            if changed:
                groups.append(MotionGroup(changed))
        else:
            # Una arista geométricamente segura puede modificar los tres ejes,
            # pero el firmware admite como máximo dos. Buscar un orden seguro.
            pending = _minimal_planar_groups(previous_planar, waypoint, tolerance, m6, escape_from)
            if pending is None:
                raise ValueError("No existe un orden seguro de movimientos de hasta dos motores.")
            groups.extend(MotionGroup(item) for item in pending)
        previous_planar = waypoint
    return groups


def _minimal_planar_groups(start, goal, tolerance, m6=None, escape_from=None):
    changed_axes = [
        index for index in range(3) if abs(goal[index] - start[index]) >= tolerance
    ]
    final_mask = (1 << len(changed_axes)) - 1

    def pose(mask):
        values = list(start)
        for bit, axis in enumerate(changed_axes):
            if mask & (1 << bit):
                values[axis] = goal[axis]
        return tuple(values)

    queue = [0]
    previous = {0: None}
    action: dict[int, tuple[tuple[int, float], ...]] = {}
    while queue:
        mask = queue.pop(0)
        if mask == final_mask:
            break
        remaining = [bit for bit in range(len(changed_axes)) if not mask & (1 << bit)]
        groups = []
        for first_offset, first in enumerate(remaining):
            for second in remaining[first_offset + 1 :]:
                groups.append((first, second))
        groups.extend((bit,) for bit in remaining)
        for bits in groups:
            next_mask = mask
            for bit in bits:
                next_mask |= 1 << bit
            if next_mask in previous:
                continue
            current_pose, next_pose = pose(mask), pose(next_mask)
            if not classify_pose(next_pose, m6, escape_from)[0] or not edge_safe(
                current_pose, next_pose, m6=m6, escape_from=escape_from
            ):
                continue
            previous[next_mask] = mask
            action[next_mask] = tuple(
                (changed_axes[bit] + 2, goal[changed_axes[bit]]) for bit in bits
            )
            queue.append(next_mask)
    if final_mask not in previous:
        return None
    reversed_actions = []
    mask = final_mask
    while mask:
        reversed_actions.append(action[mask])
        parent = previous[mask]
        if parent is None:
            break
        mask = parent
    reversed_actions.reverse()
    return reversed_actions

