"""Prueba de hardware de solo lectura de las dos cámaras y el ESP32.

No mueve el brazo ni cambia el torque: al ESP32 solo le envía `help`,
`posall` y `scan`. Ver la sección "Prueba de hardware" del README.
"""
from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

from brazoia.config import load_config
from brazoia.diagnostico import Options, run_diagnostic, save_report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Prueba de solo lectura de las cámaras y el ESP32. No mueve el brazo "
            "ni cambia el torque."
        )
    )
    parser.add_argument("--puerto", help="Puerto serie del ESP32 (por defecto, el de config.json).")
    parser.add_argument(
        "--segundos", type=float, default=5.0, help="Duración de la medición (por defecto 5)."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--solo-camaras", action="store_true", help="No conectar el ESP32.")
    mode.add_argument("--solo-esp32", action="store_true", help="No abrir las cámaras.")
    parser.add_argument("--salida", type=Path, help="Carpeta para los resultados.")
    args = parser.parse_args(argv)
    if args.segundos <= 0:
        parser.error("--segundos tiene que ser mayor que 0.")
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass

    config = load_config()
    output = args.salida or (
        config.root / "datos" / "prueba_hardware" / datetime.now().strftime("prueba_%Y%m%d_%H%M%S")
    )
    print("Prueba de hardware de solo lectura: no mueve el brazo ni cambia el torque.")
    print("Al ESP32 solo se le envían help, posall y scan.")
    print()
    report = run_diagnostic(
        config,
        Options(
            output_dir=output,
            cameras=not args.solo_esp32,
            esp32=not args.solo_camaras,
            port=args.puerto,
            duration=args.segundos,
        ),
        progress=print,
    )
    save_report(report, output)
    print()
    print(report.to_text())
    print()
    print(f"Resultados: {output}")
    print("Para revisarlos, compartí el contenido de reporte.txt.")
    return 1 if report.failed else 0


if __name__ == "__main__":
    sys.exit(main())
