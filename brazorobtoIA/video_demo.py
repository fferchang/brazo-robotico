"""Video de demostración: el brazo virtual hace la tarea varias veces seguidas.

Usa la misma escena física y el mismo experto por IK que simulacion_mujoco.py. En
cada ronda el cubo y el recipiente cambian de lugar al azar; el video muestra la
etapa en curso, un contador de rondas y el resultado de cada una. Graba un MP4 en
datos/videos/ sin abrir ventanas.

Solo virtual: no importa el controlador serie, no abre puertos ni usa cámaras
reales. Ver docs/VIDEO_DEMO.md.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from brazoia.config import KEYFRAME_STAGES


ROOT = Path(__file__).resolve().parent
FPS = 25
SIM_TICK = 0.02        # VirtualArm llama a su observador cada 20 ms simulados.
FADE_SECONDS = 0.5     # Fundido a negro entre rondas.

# Lo que lee el público en cada etapa, en lugar de los nombres internos.
AUDIENCE_LABELS = {
    "inicio": "Posición inicial",
    "aproximacion_1": "Acercándose al cubo",
    "aproximacion_2": "Acercándose al cubo",
    "aproximacion_3": "Acercándose al cubo",
    "apertura_sobre_cubo": "Abriendo la garra sobre el cubo",
    "descenso_abierto": "Bajando y acomodando el cubo",
    "agarre": "Agarrando el cubo",
    "traslado_1": "Levantando el cubo",
    "traslado_2": "Llevando el cubo al recipiente",
    "traslado_3": "Llevando el cubo al recipiente",
    "soltado": "Soltando el cubo en el recipiente",
    "guardado": "Volviendo a la posición inicial",
}

# Cámara libre que gira despacio frente al brazo (azimut 90° = de frente, desde −y) y
# acompaña la altura de la garra: se aleja y sube cuando el brazo se estira.
CAMERA_LOOKAT_Y = -0.15
CAMERA_FOLLOW_SECONDS = 1.0  # Constante de tiempo del suavizado, en segundos de video.
CAMERA_AZIMUTH = 90.0
CAMERA_SWING = 38.0          # ± grados de azimut.
CAMERA_SWING_PERIOD = 36.0   # Segundos de video por vaivén completo.
CAMERA_ELEVATION = -22.0
CAMERA_NOD = 5.0             # ± grados de elevación.
CAMERA_NOD_PERIOD = 53.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Graba un video del brazo virtual haciendo la tarea varias veces. No usa el ESP32.")
    parser.add_argument("--rondas", type=int, default=6, help="Cantidad de rondas (por defecto 6, ~2 min).")
    parser.add_argument("--semilla", type=int, default=None,
                        help="Repite las mismas posiciones del cubo y del recipiente.")
    parser.add_argument("--ritmo", type=float, default=1.5,
                        help="Velocidad del video respecto del tiempo real (por defecto 1,5).")
    parser.add_argument("--fullhd", action="store_true", help="1920 × 1080 en lugar de 1280 × 720.")
    parser.add_argument("--liviano", action="store_true",
                        help="MPEG-4: ~8 veces más liviano que H.264, algo más borroso; no lo abren los navegadores.")
    parser.add_argument("--salida", default=None, help="Archivo .mp4 (por defecto, en datos/videos/).")
    parser.add_argument("--abrir", action="store_true", help="Abre el video al terminar.")
    args = parser.parse_args(argv)
    if args.rondas < 1:
        parser.error("--rondas tiene que ser 1 o más")
    if not 0.25 <= args.ritmo <= 8:
        parser.error("--ritmo tiene que estar entre 0,25 y 8")
    return args


def stage_label(completed: int) -> str:
    """Texto de la etapa en curso cuando ya terminaron `completed` etapas."""
    return AUDIENCE_LABELS[KEYFRAME_STAGES[min(completed, len(KEYFRAME_STAGES) - 1)]]


def camera_angles(video_seconds: float) -> tuple[float, float]:
    """(azimut, elevación) en grados para el instante del video."""
    azimuth = CAMERA_AZIMUTH + CAMERA_SWING * math.sin(2 * math.pi * video_seconds / CAMERA_SWING_PERIOD)
    elevation = CAMERA_ELEVATION + CAMERA_NOD * math.sin(2 * math.pi * video_seconds / CAMERA_NOD_PERIOD)
    return azimuth, elevation


def camera_target(gripper) -> tuple[tuple[float, float, float], float]:
    """(punto al que mira, distancia) para la garra en `gripper` (metros, marco base)."""
    x, _, z = (float(value) for value in gripper)
    lookat = (0.3 * max(-0.2, min(0.2, x)), CAMERA_LOOKAT_Y, 0.10 + 0.26 * max(0.0, z - 0.10))
    return lookat, 0.76 + 0.50 * max(0.0, z - 0.15)


def decimal(value: float) -> str:
    return f"{value:g}".replace(".", ",")


# ----- textos en pantalla -----

@dataclass(frozen=True)
class Hud:
    round: int = 0
    rounds: int = 0
    finished: int = 0
    successes: int = 0
    completed: int = 0                   # Etapas terminadas en la ronda (0–12).
    banner: tuple[str, str, bool] | None = None  # (título, detalle, éxito)
    card: str | None = None              # "titulo", "resumen" o None.


def _font(size: int, bold: bool = False):
    names = ("segoeuib.ttf", "arialbd.ttf", "DejaVuSans-Bold.ttf") if bold else \
        ("segoeui.ttf", "arial.ttf", "DejaVuSans.ttf")
    folders = [Path(os.environ.get("WINDIR", r"C:\Windows")) / "Fonts", Path("/usr/share/fonts/truetype/dejavu")]
    for name in names:
        for folder in folders:
            if (folder / name).is_file():
                return ImageFont.truetype(str(folder / name), size)
    return ImageFont.load_default(size=size)


class HudPainter:
    """Dibuja los textos como una capa RGBA, guardada por estado: cambian pocas veces."""

    INK = (255, 255, 255, 255)
    SOFT = (225, 230, 236, 255)
    PANEL = (18, 22, 28, 175)
    ACCENT = (240, 120, 40, 255)
    OK = (40, 150, 75, 235)
    BAD = (185, 50, 40, 235)

    def __init__(self, width: int, height: int, pace: float):
        self.size = (width, height)
        self.pace = pace
        s = height / 720
        self.s = s
        self.fonts = {
            "small": _font(round(20 * s)), "body": _font(round(24 * s)),
            "label": _font(round(32 * s), bold=True), "head": _font(round(27 * s), bold=True),
            "banner": _font(round(38 * s), bold=True), "huge": _font(round(62 * s), bold=True),
        }
        self.cache: dict[Hud, Image.Image] = {}

    def __call__(self, hud: Hud) -> Image.Image:
        if hud not in self.cache:
            if len(self.cache) > 64:
                self.cache.clear()
            self.cache[hud] = self._paint(hud)
        return self.cache[hud]

    def _text_size(self, draw, text, font):
        # Alto desde la línea del ascendente (donde se apoya el texto con anchor "la").
        _, _, right, bottom = draw.textbbox((0, 0), text, font=font)
        return right, bottom

    def _panel(self, draw, box):
        draw.rounded_rectangle(box, radius=round(12 * self.s), fill=self.PANEL)

    def _paint(self, hud: Hud) -> Image.Image:
        width, height = self.size
        s, f = self.s, self.fonts
        layer = Image.new("RGBA", self.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(layer)
        margin, pad = round(24 * s), round(16 * s)
        speed = f"Video ×{decimal(self.pace)}" if self.pace != 1 else "Velocidad real"

        if hud.card is not None:
            draw.rectangle((0, 0, width, height), fill=(10, 12, 16, 165))
            if hud.card == "titulo":
                lines = [("Brazo robótico SO-100", f["huge"], self.INK),
                         ("Agarra el cubo rojo y lo deja en el recipiente verde.", f["body"], self.SOFT),
                         ("Simulación física en MuJoCo: el cubo y el recipiente cambian de lugar en cada ronda.",
                          f["body"], self.SOFT)]
            else:
                lines = [(f"{hud.successes} de {hud.finished} rondas", f["huge"], self.INK),
                         ("con el cubo dentro del recipiente", f["label"], self.SOFT),
                         (f"Simulación física en MuJoCo · {speed.lower()}", f["body"], self.SOFT)]
            sizes = [self._text_size(draw, text, font) for text, font, _ in lines]
            gap = round(18 * s)
            y = (height - sum(h for _, h in sizes) - gap * (len(lines) - 1)) / 2
            for (text, font, color), (w, h) in zip(lines, sizes):
                draw.text(((width - w) / 2, y), text, font=font, fill=color, anchor="la")
                y += h + gap
            return layer

        # Arriba a la izquierda, sobre el fondo: etapa en curso y avance de las doce etapas.
        # El ancho sale de la frase más larga para que el panel no cambie de tamaño.
        label = stage_label(hud.completed)
        stages = len(KEYFRAME_STAGES)
        span = max(self._text_size(draw, text, f["label"])[0] for text in AUDIENCE_LABELS.values())
        _, lh = self._text_size(draw, label, f["label"])
        bar_h, bar_gap, gap = round(10 * s), round(6 * s), round(12 * s)
        self._panel(draw, (margin, margin, margin + 2 * pad + span, margin + 2 * pad + lh + gap + bar_h))
        draw.text((margin + pad, margin + pad), label, font=f["label"], fill=self.INK)
        bar_top = margin + pad + lh + gap
        segment = (span - bar_gap * (stages - 1)) / stages
        for index in range(stages):
            x0 = margin + pad + index * (segment + bar_gap)
            if index < hud.completed:
                color = self.INK
            elif index == hud.completed:
                color = self.ACCENT
            else:
                color = (255, 255, 255, 70)
            draw.rounded_rectangle((x0, bar_top, x0 + segment, bar_top + bar_h), radius=bar_h // 2, fill=color)

        # Arriba a la derecha: ronda y resultados.
        counter = f"Ronda {hud.round} de {hud.rounds}"
        tally = f"Cubo en el recipiente: {hud.successes} de {hud.finished}" if hud.finished else "Primera ronda"
        cw, ch = self._text_size(draw, counter, f["head"])
        tw, th = self._text_size(draw, tally, f["small"])
        box_w = 2 * pad + max(cw, tw)
        self._panel(draw, (width - margin - box_w, margin, width - margin, margin + 2 * pad + ch + th + 8 * s))
        draw.text((width - margin - pad - cw, margin + pad), counter, font=f["head"], fill=self.INK)
        draw.text((width - margin - pad - tw, margin + pad + ch + 8 * s), tally, font=f["small"], fill=self.SOFT)

        # Abajo a la izquierda, chico para no tapar la mesa: qué es.
        caption = f"Brazo robótico SO-100 · simulación física en MuJoCo · {speed.lower()}"
        pw, ph = self._text_size(draw, caption, f["small"])
        small_pad = round(10 * s)
        self._panel(draw, (margin, height - margin - ph - 2 * small_pad, margin + pw + 2 * pad, height - margin))
        draw.text((margin + pad, height - margin - ph - small_pad), caption, font=f["small"], fill=self.SOFT)

        if hud.banner is not None:
            heading, detail, success = hud.banner
            hw, hh = self._text_size(draw, heading, f["banner"])
            dw, dh = self._text_size(draw, detail, f["body"]) if detail else (0, 0)
            box_w = max(hw, dw) + 4 * pad
            box_h = hh + (dh + round(10 * s) if detail else 0) + 3 * pad
            x0, y0 = (width - box_w) / 2, height * 0.34 - box_h / 2
            draw.rounded_rectangle((x0, y0, x0 + box_w, y0 + box_h), radius=round(18 * s),
                                   fill=self.OK if success else self.BAD)
            draw.text(((width - hw) / 2, y0 + 1.4 * pad), heading, font=f["banner"], fill=self.INK)
            if detail:
                draw.text(((width - dw) / 2, y0 + 1.4 * pad + hh + round(10 * s)), detail, font=f["body"],
                          fill=self.INK)
        return layer


# ----- video -----

class Mp4Writer:
    """MP4 con H.264 de Windows (Media Foundation), que se ve en cualquier reproductor pero
    pesa ~2,8 MB por segundo a 720p (OpenCV no deja bajarle la tasa), o MPEG-4 de OpenCV
    (`light`): ~8 veces más liviano, algo más borroso, y no lo abren los navegadores."""

    def __init__(self, path: Path, fps: int, size: tuple[int, int], *, light: bool = False):
        import cv2

        self.cv2 = cv2
        attempts = [("H.264", cv2.CAP_MSMF, "H264")] if os.name == "nt" and not light else []
        attempts.append(("MPEG-4", cv2.CAP_FFMPEG, "mp4v"))
        for codec, api, fourcc in attempts:
            writer = cv2.VideoWriter(str(path), api, cv2.VideoWriter_fourcc(*fourcc), fps, size)
            if writer.isOpened():
                self.writer, self.codec = writer, codec
                return
            writer.release()
        raise RuntimeError(f"OpenCV no pudo crear {path}")

    def write(self, rgb: np.ndarray):
        self.writer.write(self.cv2.cvtColor(rgb, self.cv2.COLOR_RGB2BGR))

    def close(self):
        self.writer.release()


class DemoRecorder:
    """Saca un cuadro cada `pace / fps` segundos simulados con la cámara en vaivén."""

    def __init__(self, arm, writer, width: int, height: int, *, pace: float, fps: int = FPS):
        import mujoco

        self.mujoco = mujoco
        self.arm = arm
        self.writer = writer
        self.fps = fps
        self.frame_period = pace / fps
        self.painter = HudPainter(width, height, pace)
        arm.model.vis.global_.offwidth = max(arm.model.vis.global_.offwidth, width)
        arm.model.vis.global_.offheight = max(arm.model.vis.global_.offheight, height)
        # Solo imagen: la mesa más grande no cambia la física (el plano choca igual en
        # toda su extensión) y la luz de la cámara aclara los bordes.
        table = arm.model.geom("mesa").id
        arm.model.geom_size[table][:2] = [1.1, 0.9]
        arm.model.vis.headlight.ambient[:] = [0.28] * 3
        arm.model.vis.headlight.diffuse[:] = [0.42] * 3
        self.renderer = mujoco.Renderer(arm.model, height=height, width=width)
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.view = None  # (lookat, distancia) suavizados.
        self.hud = Hud()
        self.frames = 0
        self.pending = 0.0
        self.last_time = None
        self.fade_in_left = 0

    @property
    def fade_frames(self):
        return max(1, round(FADE_SECONDS * self.fps))

    def update(self, **changes):
        self.hud = replace(self.hud, **changes)

    def fade_in(self):
        self.fade_in_left = self.fade_frames

    # Observador de VirtualArm: corre durante el episodio.
    def observe(self, arm):
        now = arm.data.time
        elapsed = now - self.last_time if self.last_time is not None else SIM_TICK
        if not 0 < elapsed <= 5 * SIM_TICK:  # El reinicio de cada episodio vuelve el reloj a cero.
            elapsed = SIM_TICK
        self.last_time = now
        self.pending += elapsed
        while self.pending >= self.frame_period - 1e-9:
            self.pending -= self.frame_period
            self.write_frame()

    # on_stage de run_episode: terminó una etapa.
    def stage_done(self, record):
        self.update(completed=min(self.hud.completed + 1, len(KEYFRAME_STAGES)))

    def hold(self, seconds: float, *, fade_out: bool = False):
        """Sigue la física con la escena quieta (sin observador) durante `seconds` de video."""
        frames = max(1, round(seconds * self.fps))
        for index in range(frames):
            self.arm.step(self.frame_period)
            self.write_frame(min(1.0, (frames - index - 1) / self.fade_frames) if fade_out else 1.0)
        self.last_time = None

    def write_frame(self, fade: float = 1.0):
        self.camera.azimuth, self.camera.elevation = camera_angles(self.frames / self.fps)
        lookat, distance = camera_target(self.arm.data.site_xpos[self.arm.site_id])
        if self.view is None:
            self.view = (np.array(lookat), distance)
        else:
            blend = 1.0 - math.exp(-1.0 / (self.fps * CAMERA_FOLLOW_SECONDS))
            self.view = (self.view[0] + (np.array(lookat) - self.view[0]) * blend,
                         self.view[1] + (distance - self.view[1]) * blend)
        self.camera.lookat[:] = self.view[0]
        self.camera.distance = self.view[1]
        self.renderer.update_scene(self.arm.data, camera=self.camera)
        image = Image.fromarray(self.renderer.render()).convert("RGBA")
        image.alpha_composite(self.painter(self.hud))
        frame = np.asarray(image.convert("RGB"))
        if self.fade_in_left:
            fade = min(fade, 1.0 - self.fade_in_left / self.fade_frames)
            self.fade_in_left -= 1
        if fade < 1.0:
            frame = (frame.astype(np.float32) * max(0.0, fade)).astype(np.uint8)
        self.writer.write(frame)
        self.frames += 1

    def close(self):
        self.renderer.close()


def short_reason(result) -> str:
    reason = result.reasons[0] if result.reasons else ""
    return reason if len(reason) <= 70 else reason[:67] + "…"


def main(argv=None):
    args = parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    from brazoia.mujoco_sim import PickPlaceExpert, PlanningError, VirtualArm, run_episode, sample_placement

    print("VIDEO DEMO · SOLO VIRTUAL · no se conecta al ESP32 ni mueve el brazo real")
    ticks_per_frame = args.ritmo / (FPS * SIM_TICK)
    if ticks_per_frame < 1 or abs(ticks_per_frame - round(ticks_per_frame)) > 1e-6:
        print(f"Aviso: con --ritmo {decimal(args.ritmo)} algunos cuadros quedan desparejos; "
              "conviene un múltiplo de 0,5.")
    seed = args.semilla if args.semilla is not None else int(np.random.default_rng().integers(1, 100_000))
    rng = np.random.default_rng(seed)
    width, height = (1920, 1080) if args.fullhd else (1280, 720)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = Path(args.salida) if args.salida else ROOT / "datos" / "videos" / f"demo_brazo_{stamp}.mp4"
    output.parent.mkdir(parents=True, exist_ok=True)

    arm = VirtualArm(ROOT / "assets")
    expert = PickPlaceExpert(arm)
    writer = Mp4Writer(output, FPS, (width, height), light=args.liviano)
    recorder = DemoRecorder(arm, writer, width, height, pace=args.ritmo)
    print(f"Semilla {seed} (para repetir: --semilla {seed}) · {args.rondas} rondas · {width} × {height} · "
          f"video ×{decimal(args.ritmo)} · {writer.codec}")
    started = time.perf_counter()
    successes = 0
    try:
        for number in range(1, args.rondas + 1):
            for _ in range(50):
                placement = sample_placement(rng, arm.dimensions)
                try:
                    plan = expert.plan(placement)
                    break
                except PlanningError as exc:
                    print(f"     (ubicación descartada, sin plan: {exc})")
            else:
                raise RuntimeError("No se encontró una ubicación con plan en 50 intentos")
            recorder.update(round=number, rounds=args.rondas, completed=0, banner=None, card=None)
            if number == 1:
                arm.reset(placement)
                recorder.update(card="titulo")
                recorder.fade_in()
                recorder.hold(4.0)
                recorder.update(card=None)
            else:
                recorder.fade_in()
            arm.observer = recorder.observe
            try:
                result = run_episode(arm, expert, placement, plan=plan, on_stage=recorder.stage_done)
            finally:
                arm.observer = None
            successes += result.success
            banner = ("El cubo quedó en el recipiente", "", True) if result.success else \
                ("Esta vez no lo logró", short_reason(result), False)
            recorder.update(finished=number, successes=successes, banner=banner)
            last = number == args.rondas
            recorder.hold(2.5, fade_out=not last)
            cube = np.round(np.array(placement.cube_xy) * 1000).astype(int).tolist()
            container = np.round(np.array(placement.container_xy) * 1000).astype(int).tolist()
            print(f"[{number}/{args.rondas}] {'ÉXITO' if result.success else 'FALLA'} · cubo {cube} mm · "
                  f"recipiente {container} mm · {recorder.frames / FPS:.0f} s de video · "
                  f"{time.perf_counter() - started:.0f} s de cálculo")
            for reason in result.reasons:
                print(f"     {reason}")
        recorder.update(banner=None, card="resumen")
        recorder.hold(4.5, fade_out=True)
    except KeyboardInterrupt:
        print("Grabación interrumpida: el video queda hasta donde llegó.")
    finally:
        recorder.close()
        writer.close()

    seconds = recorder.frames / FPS
    size_mb = output.stat().st_size / 1e6 if output.is_file() else 0.0
    print(f"\nVideo: {output} ({int(seconds // 60)}:{int(seconds % 60):02d} min, {size_mb:.0f} MB, "
          f"{successes} de {recorder.hud.finished} rondas con el cubo en el recipiente).")
    if args.abrir and os.name == "nt":
        os.startfile(output)  # noqa: S606 - abre el video con el reproductor de Windows.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
