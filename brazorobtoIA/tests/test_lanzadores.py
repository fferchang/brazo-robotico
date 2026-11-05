"""Los lanzadores .bat viven en lanzadores/ y arrancan los scripts de la raíz."""
from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LANZADORES = ROOT / "lanzadores"
PYTHON_VENV = '".venv\\Scripts\\python.exe"'


class LanzadoresTest(unittest.TestCase):
    def test_no_quedan_bat_en_la_raiz(self):
        self.assertEqual([], sorted(p.name for p in ROOT.glob("*.bat")))

    def test_cada_bat_vuelve_a_la_raiz_y_llama_un_script_que_existe(self):
        bats = sorted(LANZADORES.glob("*.bat"))
        self.assertTrue(bats, "no hay lanzadores en lanzadores/")
        for bat in bats:
            with self.subTest(bat=bat.name):
                texto = bat.read_text(encoding="utf-8")
                self.assertIn('cd /d "%~dp0.."', texto)
                if bat.name == "INSTALAR.bat":
                    self.assertTrue((ROOT / "requirements.txt").is_file())
                    continue
                scripts = re.findall(re.escape(PYTHON_VENV) + r"\s+(\S+\.py)", texto)
                self.assertTrue(scripts, "no llama a ningún script .py")
                for script in scripts:
                    self.assertTrue((ROOT / script).is_file(), script)


if __name__ == "__main__":
    unittest.main()
