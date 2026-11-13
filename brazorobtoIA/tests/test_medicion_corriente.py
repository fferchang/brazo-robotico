"""Medición de la fuente: `power telem` del firmware, del gemelo y el CSV de la app."""
from __future__ import annotations

import csv
import tempfile
import time
import unittest
from pathlib import Path

from brazoia.robot import POWER_LOG_COLUMNS, RobotSerial, parse_power

FIRMWARE = Path(__file__).resolve().parent.parent / "firmware" / "firmware_secuencias.ino"


class ParsePowerTests(unittest.TestCase):
    def test_reads_currents_total_and_lowest_voltage(self) -> None:
        sample = parse_power("POWER_TELEM ms=1234 I=10,20,-1,40,50,60 total_ma=1170 v_min_dv=118 v_id=3")
        self.assertEqual(sample.firmware_ms, 1234)
        self.assertEqual(sample.currents_ma, (65.0, 130.0, None, 260.0, 325.0, 390.0))
        self.assertEqual(sample.total_ma, 1170.0)
        self.assertEqual((sample.v_min, sample.v_id), (11.8, 3))
        self.assertIsNone(parse_power("POWER_TELEM ms=1 I=1,2,3 total_ma=1 v_min_dv=1 v_id=1"))
        self.assertIsNone(parse_power("POS ID 1: 2048 (~180.0 deg)"))

    def test_firmware_prints_what_the_parser_expects(self) -> None:
        source = FIRMWARE.read_text(encoding="utf-8")
        for fragment in ('F("POWER_TELEM ms=")', 'F(" I=")', 'F(" total_ma=")', 'F(" v_min_dv=")', 'F(" v_id=")',
                         '"power telem on"', "processPowerTelemetry();"):
            self.assertIn(fragment, source)


class PowerLogTests(unittest.TestCase):
    def test_samples_go_to_the_csv_with_angles_and_stage(self) -> None:
        robot = RobotSerial("PRUEBA", 115200)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "corriente" / "medicion.csv"
            robot.start_power_log(path)
            robot._consume_line("M1=2048(180.0deg) | M2=2048(180.0deg) | M3=1024(90.0deg) | M4=2048(180.0deg) | "
                                "M5=2992(263.0deg) | M6=114(10.0deg)", time.monotonic())
            robot.power_label = "descenso_abierto"
            robot._consume_line("POWER_TELEM ms=500 I=100,200,150,80,20,10 total_ma=3640 v_min_dv=115 v_id=2",
                                time.monotonic())
            robot.stop_power_log()
            # Después de parar ya no graba.
            robot._consume_line("POWER_TELEM ms=600 I=1,1,1,1,1,1 total_ma=39 v_min_dv=120 v_id=1", time.monotonic())
            with path.open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        self.assertEqual(list(rows[0]), POWER_LOG_COLUMNS)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["I2_mA"], "1300")
        self.assertEqual(rows[0]["total_mA"], "3640")
        self.assertEqual(rows[0]["v_min_V"], "11.5")
        self.assertEqual(rows[0]["M3_deg"], "90.0")
        self.assertEqual(rows[0]["etapa"], "descenso_abierto")
        self.assertEqual(robot.power_last.total_ma, 39.0)


class TwinPowerTelemetryTests(unittest.TestCase):
    def test_twin_answers_like_the_firmware(self) -> None:
        from brazoia.gemelo import DigitalTwin

        twin = DigitalTwin(seed=3, pace=0.0)
        try:
            twin.handle_line("power telem on")
            for _ in range(30):
                twin.tick()
            lines = twin.log_since(0)
        finally:
            twin.stop()
        self.assertIn("POWER_TELEMETRY=on", lines)
        samples = [parse_power(line) for line in lines if line.startswith("POWER_TELEM ")]
        self.assertGreaterEqual(len(samples), 2)
        self.assertTrue(all(sample is not None for sample in samples))


if __name__ == "__main__":
    unittest.main()
