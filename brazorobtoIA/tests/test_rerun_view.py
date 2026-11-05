import os
import shutil
import sys
import unittest
from pathlib import Path
from unittest import mock

from brazoia.rerun_view import ensure_viewer_on_path


class EnsureViewerOnPathTest(unittest.TestCase):
    def test_adds_interpreter_folder_when_viewer_missing_from_path(self):
        if not shutil.which("rerun", path=str(Path(sys.executable).resolve().parent)):
            self.skipTest("No hay visor rerun junto a este Python.")
        clean = os.pathsep.join(
            entry for entry in os.environ.get("PATH", "").split(os.pathsep)
            if not entry or not shutil.which("rerun", path=entry)
        )
        with mock.patch.dict(os.environ, {"PATH": clean}):
            self.assertIsNone(shutil.which("rerun"))
            ensure_viewer_on_path()
            self.assertIsNotNone(shutil.which("rerun"))

    def test_leaves_path_alone_when_viewer_already_found(self):
        with mock.patch("brazoia.rerun_view.shutil.which", return_value="rerun"), \
                mock.patch.dict(os.environ, {"PATH": "sin_cambios"}):
            ensure_viewer_on_path()
            self.assertEqual(os.environ["PATH"], "sin_cambios")


if __name__ == "__main__":
    unittest.main()
