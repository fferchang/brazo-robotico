import unittest

from brazoia.limite_dinamico import DynamicMotorLimit, LimitSettings, PowerReading


def calm_reading(t, total=300.0, v=11.4, m6=20.0):
    return PowerReading(t=t, total_ma=total, v_min=v, m6_ma=m6)


class DynamicMotorLimitTests(unittest.TestCase):
    def test_base_vale_sin_lecturas_y_no_crece(self):
        limit = DynamicMotorLimit()
        self.assertIsNone(limit.check(0.0, set(), {1, 5}))
        self.assertEqual(limit.check(0.0, {1, 5}, {2}), "cupo")
        self.assertEqual(limit.limit, 2)

    def test_suma_de_a_uno_con_calma_y_hasta_el_techo(self):
        limit = DynamicMotorLimit()
        limit.feed(calm_reading(0.0))
        self.assertIsNone(limit.check(0.0, {1, 5}, {2}))
        self.assertEqual(limit.limit, 3)
        # Recién subió: hace falta otro intervalo de calma para el cuarto.
        limit.feed(calm_reading(0.2))
        self.assertEqual(limit.check(0.2, {1, 2, 5}, {3}), "cupo")
        for t in (0.5, 1.0, 1.5, 2.0):
            limit.feed(calm_reading(t))
            limit.check(t, {1, 2, 3, 4, 5}, {6})
        self.assertEqual(limit.limit, 5)

    def test_no_crece_sin_necesidad(self):
        limit = DynamicMotorLimit()
        for t in (0.0, 0.6, 1.2):
            limit.feed(calm_reading(t))
            self.assertIsNone(limit.check(t, set(), {1}))
        self.assertEqual(limit.limit, 2)

    def test_pico_vuelve_a_la_base_y_bloquea_la_suma(self):
        limit = DynamicMotorLimit()
        limit.feed(calm_reading(0.0))
        limit.check(0.0, {1, 5}, {2})
        limit.feed(calm_reading(0.1, total=2300.0))
        self.assertEqual(limit.limit, 2)
        limit.feed(calm_reading(0.2))
        self.assertEqual(limit.check(0.2, {1, 5}, {2}), "cupo")
        limit.feed(calm_reading(0.7))
        self.assertIsNone(limit.check(0.7, {1, 5}, {2}))

    def test_caida_de_tension_pausa_todo(self):
        limit = DynamicMotorLimit()
        limit.feed(calm_reading(0.0, v=8.5))
        self.assertEqual(limit.check(0.0, set(), {1}), "pausa_por_tension")
        limit.feed(calm_reading(0.1, v=10.0))
        self.assertIsNone(limit.check(0.1, set(), {1}))
        self.assertEqual(limit.check(0.1, {1, 5}, {2}), "cupo")

    def test_lectura_vencida_deja_la_base(self):
        limit = DynamicMotorLimit()
        limit.feed(calm_reading(0.0))
        limit.check(0.0, {1, 5}, {2})
        self.assertEqual(limit.check(0.4, {1, 5}, {2}), "cupo")
        self.assertEqual(limit.limit, 2)
        self.assertIsNone(limit.check(0.4, {1}, {5}))

    def test_garra_apretando_tiene_prioridad(self):
        limit = DynamicMotorLimit()
        limit.feed(calm_reading(0.0, m6=450.0))
        self.assertEqual(limit.check(0.0, {1, 5}, {2}), "cupo")

    def test_presupuesto_previo_al_despacho(self):
        limit = DynamicMotorLimit(LimitSettings(start_reserve_ma=400.0, budget_ma=2000.0))
        limit.feed(calm_reading(0.0, total=1500.0))
        # 1500 + 400 cabe para subir el cupo, pero dos arranques nuevos no.
        self.assertEqual(limit.check(0.0, {1}, {2, 3}), "presupuesto")

    def test_descarta_muestras_fuera_de_orden(self):
        limit = DynamicMotorLimit()
        limit.feed(calm_reading(1.0))
        limit.feed(calm_reading(0.5, total=5000.0))
        self.assertEqual(limit.reading.t, 1.0)


if __name__ == "__main__":
    unittest.main()
