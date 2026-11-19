"""Límite dinámico de motores en movimiento a la vez, según corriente y tensión.

Solo para ensayos en el gemelo (ver docs/RELEVO_MOVIMIENTO_Y_CORRIENTE.md, «Límite
dinámico en el gemelo»). Nada de esto valida la fuente real: los umbrales son
parámetros a medir en el brazo, no garantías.

La idea:
- Se parte de `base` motores (hoy 2, lo que ya anda). Ese cupo vale siempre, salvo con
  la tensión por debajo de `pause_v`, donde no sale ninguna orden nueva.
- Para pasar de `base` hace falta una lectura fresca, tensión y corriente con margen y
  la garra sin pedir corriente (prioridad del agarre). Se suma de a un motor, y cada
  suma necesita `calm_s` sin incidentes desde el último incidente y la última suma.
- Antes de despachar se reserva `start_reserve_ma` por cada motor que arranca: el pico
  de arranque no llega a verse en el muestreo de 100 ms, así que se presupuesta antes.
- Un pico, una caída de tensión o la garra apretando vuelven el límite a `base`. Una
  lectura vencida también lo deja en `base` mientras dure.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class PowerReading:
    """Una muestra de `power telem`: tiempo (s), corriente total y de M6 (mA), tensión mínima (V)."""

    t: float
    total_ma: float
    v_min: float
    m6_ma: float = 0.0


@dataclass(frozen=True)
class LimitSettings:
    base: int = 2
    ceiling: int = 5               # techo solo para el gemelo
    budget_ma: float = 2000.0      # los 2 A que propuso el usuario: parámetro experimental
    start_reserve_ma: float = 200.0  # lo que se supone que suma un motor al arrancar
    ok_v: float = 10.5             # por debajo no se suman motores
    pause_v: float = 9.0           # por debajo no sale ninguna orden nueva
    # Caída de la fuente por ampere: el 6/11, ~1 A de la garra la llevó de 11,7 a 7,7 V.
    # Con esto, 2 A serían ~3,7 V: el presupuesto también mira la tensión proyectada.
    v_per_a: float = 4.0
    stale_s: float = 0.25          # 2,5 muestras de 100 ms
    calm_s: float = 0.5
    m6_priority_ma: float = 300.0  # como GRIP_SETTLE_MA: la garra todavía está apretando

    def __post_init__(self) -> None:
        if not 1 <= self.base <= self.ceiling:
            raise ValueError("base y techo inválidos")


@dataclass
class DynamicMotorLimit:
    settings: LimitSettings = field(default_factory=LimitSettings)

    def __post_init__(self) -> None:
        self.limit = self.settings.base
        self.reading: PowerReading | None = None
        self.last_incident = -math.inf
        self.last_raise = -math.inf
        self.events: list[dict] = []

    # ----- lecturas -----
    def incident(self, reading: PowerReading) -> str | None:
        s = self.settings
        if reading.v_min < s.ok_v:
            return "tension_baja"
        if reading.total_ma > s.budget_ma:
            return "pico_de_corriente"
        if reading.m6_ma > s.m6_priority_ma:
            return "garra_apretando"
        return None

    def feed(self, reading: PowerReading) -> None:
        if self.reading is not None and reading.t < self.reading.t:
            return  # muestra vieja fuera de orden: se descarta
        self.reading = reading
        reason = self.incident(reading)
        if reason is not None:
            self.last_incident = reading.t
            if self.limit != self.settings.base:
                self.events.append({"t_s": round(reading.t, 3), "evento": "baja", "motivo": reason,
                                    "de": self.limit, "a": self.settings.base})
            self.limit = self.settings.base

    def fresh(self, now: float) -> bool:
        return self.reading is not None and now - self.reading.t <= self.settings.stale_s

    # ----- decisión -----
    def update(self, now: float, *, want_more: bool = False) -> int:
        """Límite vigente en `now`. Con `want_more` (hay un grupo esperando cupo) suma un
        motor si se cumplen las condiciones: no crece sin necesidad."""
        s = self.settings
        if not self.fresh(now):
            if self.limit != s.base:
                self.events.append({"t_s": round(now, 3), "evento": "baja", "motivo": "lectura_vencida",
                                    "de": self.limit, "a": s.base})
            self.limit = s.base
            self.last_incident = max(self.last_incident, now)
            return self.limit
        reading = self.reading
        calm = now - self.last_incident >= s.calm_s and now - self.last_raise >= s.calm_s
        headroom = reading.total_ma + s.start_reserve_ma <= s.budget_ma
        if (want_more and self.limit < s.ceiling and calm and headroom and self.incident(reading) is None):
            self.events.append({"t_s": round(now, 3), "evento": "sube", "de": self.limit, "a": self.limit + 1,
                                "total_ma": round(reading.total_ma), "v_min": round(reading.v_min, 2)})
            self.limit += 1
            self.last_raise = now
        return self.limit

    def check(self, now: float, active: set[int], adding: set[int]) -> str | None:
        """None si se puede despachar `adding` con `active` moviéndose; si no, el motivo."""
        s = self.settings
        if self.reading is not None and self.fresh(now) and self.reading.v_min < s.pause_v:
            return "pausa_por_tension"
        moving = active | adding
        limit = self.update(now, want_more=len(moving) > self.limit)
        if len(moving) > limit:
            return "cupo"
        if len(moving) > s.base:
            # Más allá de la base: presupuesto previo al despacho con la última lectura.
            extra_ma = len(adding - active) * s.start_reserve_ma
            if self.reading.total_ma + extra_ma > s.budget_ma:
                return "presupuesto"
            if self.reading.v_min - extra_ma / 1000.0 * s.v_per_a < s.pause_v:
                return "tension_proyectada"
        return None
