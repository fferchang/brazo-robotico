"""Visor 3D de MuJoCo que copia la pose del brazo real (reemplaza a Rerun en la barra)."""
from __future__ import annotations

import threading
import time
import unittest
from contextlib import contextmanager
from unittest import mock

import numpy as np

from brazoia.config import INITIAL_POSE, load_config
from brazoia.mujoco_view import MujocoArmView, launch_scene_viewer

CONFIG = load_config()


class FakeViewer:
    def __init__(self):
        self.cam = mock.Mock(lookat=np.zeros(3))
        self.synced = 0
        self.open = True
        self.lock_threads = set()

    @contextmanager
    def lock(self):
        self.lock_threads.add(threading.get_ident())
        yield

    def sync(self):
        self.synced += 1

    def is_running(self):
        return self.open

    def close(self):
        self.open = False


def wait_for(condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.01)
    return False


class MujocoArmViewTests(unittest.TestCase):
    def test_scene_viewer_hides_the_side_panels_and_frames_the_arm(self) -> None:
        viewer = FakeViewer()
        with mock.patch("mujoco.viewer.launch_passive", return_value=viewer) as launch:
            self.assertIs(launch_scene_viewer("modelo", "datos"), viewer)
        self.assertEqual(launch.call_args.kwargs, {"show_left_ui": False, "show_right_ui": False})
        self.assertGreater(viewer.cam.distance, 0.3)

    def test_copies_the_real_pose_from_its_own_thread(self) -> None:
        view = MujocoArmView(CONFIG.root / "assets", limits=CONFIG.limits)
        viewer = FakeViewer()
        with mock.patch("brazoia.mujoco_view.launch_scene_viewer", return_value=viewer):
            view.open()  # no bloquea: arma el modelo y abre la ventana en su hilo
            self.assertTrue(view.active)
            self.assertTrue(wait_for(lambda: view.running))
        arm = view.arm
        self.assertLess(arm.data.qpos[arm.cube_qadr + 2], arm.floor_z - 0.5)
        pose = {**INITIAL_POSE, 6: 40.0}
        view.log_pose(pose)
        self.assertTrue(wait_for(lambda: viewer.synced >= 1))
        for motor, value in pose.items():
            self.assertAlmostEqual(arm.data.qpos[arm.qadr[motor]], arm.to_qpos(motor, value))
        # La interfaz nunca toma el lock del visor (arrastrar la ventana lo retiene).
        self.assertNotIn(threading.get_ident(), viewer.lock_threads)
        view.log_pose({1: None, 7: 10.0})  # lecturas que faltan o motores que no existen
        time.sleep(0.2)
        self.assertEqual(viewer.synced, 1)
        view.close()
        self.assertFalse(view.running)
        self.assertTrue(wait_for(lambda: not view.active))

    def test_open_failure_is_reported_without_blocking(self) -> None:
        view = MujocoArmView(CONFIG.root / "assets", limits=CONFIG.limits)
        with mock.patch("brazoia.mujoco_view.launch_scene_viewer", side_effect=RuntimeError("sin OpenGL")):
            view.open()
            self.assertTrue(wait_for(lambda: not view.active))
        self.assertFalse(view.running)
        self.assertIn("OpenGL", view.error)


if __name__ == "__main__":
    unittest.main()
