"""Calibración modelo-brazo a partir de telemetría real; solo cálculos.

Toques de mesa: cada uno es la telemetría M1–M6 con la garra apoyada en la
mesa. Si el modelo fuera exacto, el punto más bajo de la garra quedaría a 0 mm
en todos. El ajuste estima la altura real de la mesa respecto del modelo y
correcciones de cero de M2–M4 que expliquen los toques.

Garra: compara el M6 al que los dedos reales se tocan con el del modelo.

No escribe calibracion.json ni ningún otro archivo: propone valores.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


TOUCH_MOTORS = (2, 3, 4)
MINIMUM_NOISE_MM = 0.5     # Ruido mínimo supuesto: resolución del encoder y del apoyo.
MAXIMUM_SIGMA_DEGREES = 1.0
FINGERTIP_BAND = (-0.104, -0.094)  # Zona de las puntas en el marco de la garra (m).


def _shifted(pose, offsets):
    return {i: float(pose[i]) + offsets.get(i, 0.0) for i in range(1, 7)}


def touch_heights(geometry, poses, *, offsets=None, table_mm=0.0):
    """Altura (mm) del punto más bajo de la garra sobre la mesa real, por pose."""
    offsets = offsets or {}
    return np.array([geometry.clearance(_shifted(pose, offsets))["height_mm"] - table_mm for pose in poses])


@dataclass(frozen=True)
class TouchFit:
    table_mm: float                  # Mesa real respecto de la del modelo; + = más alta.
    offsets: dict[int, float]        # Grados a sumar a la lectura; solo motores estimados.
    table_sigma_mm: float
    offset_sigmas: dict[int, float]
    rms_before_mm: float             # Modelo actual, sin corregir nada.
    rms_table_only_mm: float         # Corrigiendo solo la altura de la mesa.
    rms_after_mm: float              # Con la mesa y los ceros estimados.
    residuals_mm: list[float]
    identifiable: bool               # True si se estimaron todos los motores pedidos.
    missing: tuple[int, ...] = ()    # Motores que los toques no alcanzaron a estimar.
    unneeded: tuple[int, ...] = ()   # Estimables, pero corregirlos no baja el error.


def _rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def _least_squares(geometry, poses, motors):
    """Gauss-Newton de [mesa, ceros de `motors`]; devuelve parámetros, sigmas y residuos."""
    def residuals(params):
        return touch_heights(geometry, poses, offsets=dict(zip(motors, params[1:])), table_mm=params[0])

    params = np.array([float(touch_heights(geometry, poses).mean())] + [0.0] * len(motors))
    jacobian = np.empty((len(poses), len(params)))
    for _ in range(15):
        current = residuals(params)
        jacobian[:, 0] = -1.0
        for column in range(1, len(params)):
            step = np.zeros_like(params)
            step[column] = 0.05
            jacobian[:, column] = (residuals(params + step) - residuals(params - step)) / 0.1
        delta, *_ = np.linalg.lstsq(jacobian, -current, rcond=None)
        params = params + delta
        if np.max(np.abs(delta)) < 1e-4:
            break
    final = residuals(params)
    variance = max(float(final @ final) / max(1, len(poses) - len(params)), MINIMUM_NOISE_MM ** 2)
    sigmas = np.sqrt(np.clip(np.diag(variance * np.linalg.pinv(jacobian.T @ jacobian)), 0.0, None))
    return params, sigmas, final


def fit_touches(geometry, poses, *, motors=TOUCH_MOTORS):
    """Ajusta la mesa y los ceros que los toques permiten estimar.

    Primero intenta todos los `motors` juntos. Si alguno queda con más de 1° de
    incertidumbre, prueba de a un motor y se queda con el que más baja el error
    (al menos un 10 %). Si ninguno sirve, corrige solo la mesa.
    """
    poses = [dict(pose) for pose in poses]
    if len(poses) < len(motors) + 3:
        raise ValueError(f"Hacen falta al menos {len(motors) + 3} toques; hay {len(poses)}.")
    before = touch_heights(geometry, poses)
    table_only = _least_squares(geometry, poses, ())
    chosen, estimable = ((), *table_only), tuple(motors)
    full = _least_squares(geometry, poses, tuple(motors))
    if np.all(full[1][1:] <= MAXIMUM_SIGMA_DEGREES):
        chosen = (tuple(motors), *full)
    else:
        singles = [((motor,), *_least_squares(geometry, poses, (motor,))) for motor in motors]
        singles = [single for single in singles if single[2][1] <= MAXIMUM_SIGMA_DEGREES]
        estimable = tuple(single[0][0] for single in singles)
        useful = [single for single in singles if _rms(single[3]) < 0.9 * _rms(table_only[2])]
        if useful:
            chosen = min(useful, key=lambda single: _rms(single[3]))
    fitted, params, sigmas, residuals = chosen
    return TouchFit(
        table_mm=float(params[0]),
        offsets={motor: float(value) for motor, value in zip(fitted, params[1:])},
        table_sigma_mm=float(sigmas[0]),
        offset_sigmas={motor: float(value) for motor, value in zip(fitted, sigmas[1:])},
        rms_before_mm=_rms(before),
        rms_table_only_mm=_rms(table_only[2]),
        rms_after_mm=_rms(residuals),
        residuals_mm=[float(value) for value in residuals],
        identifiable=fitted == tuple(motors),
        missing=tuple(motor for motor in motors if motor not in estimable),
        unneeded=tuple(motor for motor in estimable if motor not in fitted),
    )


def proposed_changes(geometry, fit: TouchFit):
    """Valores sugeridos para calibracion.json y la altura del eje M2; no se escriben."""
    visual = {motor: float(geometry.calibration[motor].get("visual_offset_degrees", 0.0))
              + delta * float(geometry.calibration[motor]["direction"])
              for motor, delta in fit.offsets.items()}
    return {"shoulder_height_mm": geometry.shoulder_height_mm - fit.table_mm, "visual_offset_degrees": visual}


def _jaw_vertices(geometry, filename):
    return next(vertices for _, name, vertices in geometry.meshes if name.endswith(filename))


def fingertip_gap_mm(geometry, m6):
    """Separación entre las caras internas de las puntas para un M6 lógico, o None."""
    fixed, moving = _jaw_vertices(geometry, "Fixed_Jaw.stl"), _jaw_vertices(geometry, "Moving_Jaw.stl")
    pose = {1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0, 6: float(m6)}
    transforms, _ = geometry.transforms(pose)
    relative = np.linalg.inv(transforms["gripper"]) @ transforms["jaw"]
    moving = moving @ relative[:3, :3].T + relative[:3, 3]
    low, high = FINGERTIP_BAND
    fixed_band = fixed[(fixed[:, 1] > low) & (fixed[:, 1] < high)]
    moving_band = moving[(moving[:, 1] > low) & (moving[:, 1] < high) & (np.abs(moving[:, 2]) < 0.01)]
    if not len(moving_band):
        return None
    return float((fixed_band[:, 0].min() - moving_band[:, 0].max()) * 1000)


def model_m6_for_gap(geometry, gap_mm=0.0, *, low=-40.0, high=40.0):
    """M6 lógico al que el modelo deja `gap_mm` entre las puntas (bisección)."""
    gap_high = fingertip_gap_mm(geometry, high)
    if gap_high is None or gap_high <= gap_mm:
        raise ValueError(f"El modelo no abre {gap_mm:g} mm en el rango buscado.")
    gap_low = fingertip_gap_mm(geometry, low)
    if gap_low is not None and gap_low > gap_mm:
        raise ValueError("El modelo no cierra tanto en el rango buscado.")
    for _ in range(40):
        middle = (low + high) / 2
        gap = fingertip_gap_mm(geometry, middle)
        if gap is not None and gap > gap_mm:
            high = middle
        else:
            low = middle
    return (low + high) / 2


def gripper_offset(geometry, m6_reading, gap_mm=0.0):
    """Grados a sumar a la lectura de M6 para que el modelo reproduzca la garra real."""
    return model_m6_for_gap(geometry, gap_mm) - float(m6_reading)


def describe_fit(geometry, fit: TouchFit):
    """Resumen en castellano para mostrar en la ventana o la consola."""
    changes = proposed_changes(geometry, fit)
    lines = [
        f"Error del modelo actual: {fit.rms_before_mm:.1f} mm RMS "
        f"(en un toque perfecto la garra queda a 0 mm).",
        f"Mesa real {fit.table_mm:+.1f} ± {fit.table_sigma_mm:.1f} mm respecto del modelo "
        f"→ eje M2 a {changes['shoulder_height_mm']:.1f} mm de la mesa (hoy {geometry.shoulder_height_mm:g} mm).",
        f"Corrigiendo solo la mesa: {fit.rms_table_only_mm:.1f} mm RMS.",
    ]
    if fit.offsets:
        lines.append(f"Corrigiendo también {'los ceros' if len(fit.offsets) > 1 else 'el cero'}: " + ", ".join(
            f"M{motor} {delta:+.1f}° ± {fit.offset_sigmas[motor]:.1f}°" for motor, delta in fit.offsets.items())
            + f" → {fit.rms_after_mm:.1f} mm RMS.")
    def listed(motors):
        names = [f"M{motor}" for motor in motors]
        return names[0] if len(names) == 1 else ", ".join(names[:-1]) + " y " + names[-1]

    if fit.unneeded:
        lines.append(f"{listed(fit.unneeded)}: corregir el cero no baja el error; no hace falta cambiarlo.")
    if fit.missing:
        lines.append(f"Con estos toques no se {'pueden' if len(fit.missing) > 1 else 'puede'} estimar "
                     f"{listed(fit.missing)}: sumá toques con la garra inclinada 30–45° y a distintas "
                     "distancias de la base.")
    worst = max(range(len(fit.residuals_mm)), key=lambda index: abs(fit.residuals_mm[index]))
    lines.append(f"Peor toque después del ajuste: #{worst + 1} ({fit.residuals_mm[worst]:+.1f} mm).")
    return "\n".join(lines)


def degrees_text(pose):
    return "  ".join(f"M{i} {pose[i]:.1f}" for i in range(1, 7) if i in pose and math.isfinite(pose[i]))
