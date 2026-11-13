"""Video de demostración: textos, cámara, formato y una ronda grabada en chico."""
from __future__ import annotations

import io
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path

import numpy as np

from brazoia.config import KEYFRAME_STAGES

try:
    import mujoco
except ImportError:  # La simulación es opcional para el resto de la aplicación.
    mujoco = None

import video_demo
from video_demo import AUDIENCE_LABELS, Hud, HudPainter, camera_angles, camera_target, stage_label


ROOT = Path(__file__).resolve().parents[1]


class VideoDemoTextTests(unittest.TestCase):
    def test_every_stage_has_an_audience_label(self):
        self.assertEqual(set(AUDIENCE_LABELS), set(KEYFRAME_STAGES))
        self.assertEqual(stage_label(0), "Posición inicial")
        self.assertEqual(stage_label(len(KEYFRAME_STAGES)), AUDIENCE_LABELS["guardado"])

    def test_defaults_and_validation(self):
        args = video_demo.parse_args([])
        self.assertEqual((args.rondas, args.ritmo, args.fullhd, args.liviano), (6, 1.5, False, False))
        for wrong in (["--rondas", "0"], ["--ritmo", "20"]):
            with self.subTest(wrong=wrong), self.assertRaises(SystemExit), redirect_stderr(io.StringIO()):
                video_demo.parse_args(wrong)

    def test_painter_draws_panels_banner_and_cards(self):
        painter = HudPainter(320, 180, 1.5)
        hud = Hud(round=1, rounds=3)
        layer = np.asarray(painter(hud))
        self.assertIs(painter(hud), painter(hud))  # Guardada: no se redibuja cada cuadro.
        self.assertGreater(layer[10:40, 10:60, 3].max(), 0)    # Etapa, arriba a la izquierda.
        self.assertGreater(layer[10:40, -60:-10, 3].max(), 0)  # Ronda, arriba a la derecha.
        self.assertEqual(layer[70:90, 150:170, 3].max(), 0)    # El centro queda libre.
        banner = np.asarray(painter(Hud(round=1, rounds=3, finished=1, successes=1,
                                        banner=("El cubo quedó en el recipiente", "", True))))
        self.assertGreater(banner[58:64, 150:170, 3].min(), 0)  # Cartel a un tercio de la altura.
        card = np.asarray(painter(Hud(card="titulo")))
        self.assertGreater(card[:, :, 3].min(), 0)  # Oscurece todo el cuadro.


class VideoDemoCameraTests(unittest.TestCase):
    def test_camera_swings_in_front_of_the_arm(self):
        self.assertEqual(camera_angles(0.0), (90.0, -22.0))
        for seconds in np.linspace(0, 300, 301):
            azimuth, elevation = camera_angles(seconds)
            self.assertTrue(50 <= azimuth <= 130 and -30 <= elevation <= -15)

    def test_camera_backs_off_when_the_arm_stretches(self):
        low_lookat, low_distance = camera_target((0.05, -0.2, 0.08))
        high_lookat, high_distance = camera_target((0.0, -0.1, 0.52))
        self.assertGreater(high_distance, low_distance)
        self.assertGreater(high_lookat[2], low_lookat[2])
        self.assertLess(high_lookat[2], 0.25)  # Sin subir tanto que se pierdan cubo y recipiente.


class Mp4WriterTests(unittest.TestCase):
    def test_writes_a_readable_mp4(self):
        import cv2

        for light in (True, False):
            with self.subTest(light=light), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "prueba.mp4"
                writer = video_demo.Mp4Writer(path, 25, (320, 180), light=light)
                for value in range(10):
                    writer.write(np.full((180, 320, 3), value * 20, np.uint8))
                writer.close()
                capture = cv2.VideoCapture(str(path))
                ok, frame = capture.read()
                capture.release()
                self.assertTrue(ok)
                self.assertEqual(frame.shape, (180, 320, 3))


class FrameCounter:
    codec = "prueba"

    def __init__(self):
        self.frames = []

    def write(self, rgb):
        self.frames.append(float(rgb.mean()))

    def close(self):
        pass


@unittest.skipIf(mujoco is None, "MuJoCo no está instalado")
class VideoDemoRecordingTests(unittest.TestCase):
    def test_records_a_round_at_the_requested_pace(self):
        from brazoia.mujoco_sim import PickPlaceExpert, VirtualArm, run_episode, sample_placement

        arm = VirtualArm(ROOT / "assets")
        expert = PickPlaceExpert(arm)
        writer = FrameCounter()
        recorder = video_demo.DemoRecorder(arm, writer, 160, 90, pace=1.5)
        placement = sample_placement(np.random.default_rng(3), arm.dimensions)
        recorder.update(round=1, rounds=1)
        recorder.fade_in()
        arm.observer = recorder.observe
        try:
            result = run_episode(arm, expert, placement, on_stage=recorder.stage_done)
        finally:
            arm.observer = None
        self.assertTrue(result.success, result.reasons)
        self.assertEqual(recorder.hud.completed, len(KEYFRAME_STAGES))
        # Un cuadro cada 1,5 / 25 s simulados (el reinicio suma 0,3 s antes de la etapa 1).
        expected = (result.simulated_seconds + 0.3) / recorder.frame_period
        self.assertAlmostEqual(len(writer.frames), expected, delta=3)
        self.assertLess(writer.frames[0], writer.frames[recorder.fade_frames + 1])  # Fundido de entrada.
        frames = len(writer.frames)
        recorder.hold(1.0, fade_out=True)
        self.assertEqual(len(writer.frames) - frames, 25)
        self.assertEqual(writer.frames[-1], 0.0)  # Termina en negro.
        recorder.close()

    def test_video_does_not_import_serial_controller(self):
        code = ("import sys, video_demo; "
                "print(int('serial' in sys.modules or 'brazoia.robot' in sys.modules))")
        output = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, check=True)
        self.assertEqual(output.stdout.strip(), "0")


if __name__ == "__main__":
    unittest.main()
