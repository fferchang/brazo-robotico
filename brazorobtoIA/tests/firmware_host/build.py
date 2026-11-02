"""Compila el firmware del ESP32 en el PC para probar su lógica sin hardware.

Como el IDE de Arduino, genera los prototipos de las funciones del .ino y los
inserta antes de la primera función. Uso manual:
    python tests/firmware_host/build.py      (compila y corre las pruebas)
"""
from __future__ import annotations

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIRMWARE = HERE.parents[1] / "firmware" / "firmware_secuencias.ino"
SIGNATURE_START = re.compile(r"^(?!static\b|struct\b|enum\b|typedef\b|class\b)[A-Za-z_][\w\s\*&<>:]*?\b(\w+)\s*\(")


def _strip_defaults(signature: str) -> str:
    head, _, params = signature.partition("(")
    params = params.rsplit(")", 1)[0]
    cleaned = [re.sub(r"\s*=\s*[^,]+$", "", item.strip()) for item in params.split(",")] if params.strip() else []
    return f"{head}({', '.join(cleaned)});"


def preprocess(source: str) -> str:
    lines = source.splitlines()
    prototypes: list[str] = []
    first_function: int | None = None
    index = 0
    while index < len(lines):
        line = lines[index]
        match = SIGNATURE_START.match(line)
        if match and not line.rstrip().endswith(";"):
            end = index
            while end < len(lines) and not lines[end].rstrip().endswith("{"):
                if lines[end].rstrip().endswith(";"):
                    break
                end += 1
            if end < len(lines) and lines[end].rstrip().endswith("{"):
                signature = " ".join(item.strip() for item in lines[index:end + 1])[:-1].strip()
                if match.group(1) not in ("if", "for", "while", "switch"):
                    prototypes.append(_strip_defaults(signature))
                    if first_function is None:
                        first_function = index
                index = end
        index += 1
    if first_function is None:
        raise ValueError("No se encontraron funciones en el firmware.")
    lines[first_function:first_function] = ["// Prototipos generados para la compilación en el PC.", *prototypes, ""]
    return "\n".join(lines) + "\n"


def build(output_dir: Path) -> Path:
    compiler = shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        raise FileNotFoundError("No hay compilador de C++ (g++ o clang++).")
    (output_dir / "firmware_preprocesado.ino").write_text(
        preprocess(FIRMWARE.read_text(encoding="utf-8")), encoding="utf-8"
    )
    binary = output_dir / ("gripper_harness.exe" if sys.platform == "win32" else "gripper_harness")
    subprocess.run(
        [compiler, "-std=c++17", "-Wall", "-Wno-unused-function", "-Wno-unused-variable",
         "-I", str(output_dir), "-I", str(HERE), "-o", str(binary), str(HERE / "gripper_harness.cpp")],
        check=True, capture_output=True, text=True,
    )
    return binary


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as folder:
        try:
            program = build(Path(folder))
        except subprocess.CalledProcessError as error:
            print(error.stderr)
            raise SystemExit(1)
        raise SystemExit(subprocess.run([str(program)]).returncode)
