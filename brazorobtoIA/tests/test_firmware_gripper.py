"""Compila el firmware en el PC y corre las pruebas de la garra (tests/firmware_host).

Se omite si no hay compilador de C++. No reemplaza compilar para el ESP32.
"""
from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "firmware_host"))
import build  # noqa: E402


@unittest.skipUnless(shutil.which("g++") or shutil.which("clang++"), "sin compilador de C++")
class FirmwareGripperHostTests(unittest.TestCase):
    def test_gripper_current_supervision(self):
        with tempfile.TemporaryDirectory() as folder:
            try:
                program = build.build(Path(folder))
            except subprocess.CalledProcessError as error:
                self.fail("El firmware no compila en el PC:\n" + error.stderr)
            result = subprocess.run([str(program)], capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("OK 0", result.stdout)


if __name__ == "__main__":
    unittest.main()
