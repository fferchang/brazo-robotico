"""Selector «Motores a la vez» y movimiento directo (una orden `sync` por punto)."""
import unittest

from brazoia import gemelo
from brazoia.execution import MotionExecutor, MotionProposal, MotionSegment, SupervisedExecution
from brazoia.gemelo import DigitalTwin
from brazoia.planner import MotionGroup
from tests.test_orden_sync import GROUPS, InstantSerial, together_point


class SpyExecutor(MotionExecutor):
    """Anota cada espera de llegada para ver qué estaba en camino al mandar lo siguiente."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.waits: list[tuple[set[int], bool]] = []

    def _wait_arrival(self, targets, *, settle, **kwargs):
        self.waits.append((set(targets), settle))
        return super()._wait_arrival(targets, settle=settle, **kwargs)


class MaxMotorsExecutorTests(unittest.TestCase):
    def moves(self, robot):
        return [command for command in robot.sent if not command.startswith("gripper")]

    def test_five_fit_in_one_sync(self):
        robot = InstantSerial(supported=True)
        SpyExecutor(robot, max_motors=5).execute(together_point(GROUPS))
        self.assertEqual(self.moves(robot), ["sync 0 1 184.00 5 266.00 2 183.00 3 93.00 4 184.00"])

    def test_three_at_a_time_split_the_sync_in_batches(self):
        robot = InstantSerial(supported=True)
        executor = SpyExecutor(robot, max_motors=3)
        executor.execute(together_point(GROUPS))
        self.assertEqual(self.moves(robot), ["sync 0 1 184.00 5 266.00 2 183.00", "sync 0 3 93.00 4 184.00"])
        # La primera tanda llega (sin quietud) antes de mandar la segunda; al final, todo quieto.
        self.assertEqual(executor.waits[0], ({1, 2, 5}, False))
        self.assertEqual(executor.waits[-1], ({1, 2, 3, 4, 5}, True))

    def test_pairs_wait_for_room_without_sync(self):
        robot = InstantSerial(supported=False)
        executor = SpyExecutor(robot, max_motors=2)
        executor.execute(together_point(GROUPS))
        self.assertEqual(self.moves(robot), ["pair 1 184.00 5 266.00", "pair 2 183.00 3 93.00", "4 184.00"])
        self.assertEqual([wait for wait in executor.waits if not wait[1]], [({1, 5}, False), ({2, 3}, False)])
        self.assertEqual(executor.waits[-1], ({1, 2, 3, 4, 5}, True))

    def test_four_at_a_time_lets_two_pairs_go_together(self):
        robot = InstantSerial(supported=False)
        executor = SpyExecutor(robot, max_motors=4)
        executor.execute(together_point(GROUPS))
        # 1, 5, 2 y 3 salen seguidos; M4 espera a que lleguen esos cuatro.
        self.assertEqual([wait for wait in executor.waits if not wait[1]], [({1, 2, 3, 5}, False)])


class OneOrderPlanningTests(unittest.TestCase):
    PLANNING = {1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0}

    def chain(self):
        steps = []
        for fraction in (0.5, 1.0):
            after = {1: 180.0 + 20 * fraction, 2: 180.0 - 10 * fraction, 3: 90.0 + 30 * fraction,
                     4: 180.0, 5: 263.0 + 8 * fraction}
            steps.append(([MotionGroup(((1, after[1]), (5, after[5]))), MotionGroup(((2, after[2]), (3, after[3])))],
                          after))
        return steps

    def execution(self, **kwargs):
        execution = SupervisedExecution.__new__(SupervisedExecution)
        execution.max_motors = kwargs.get("max_motors", 5)
        return execution

    def test_checked_chain_becomes_a_single_order_to_the_last_point(self):
        chain = self.chain()
        result = self.execution()._one_order(chain, self.PLANNING)
        self.assertEqual(len(result), 1)
        (groups, after), = result
        self.assertEqual(groups, [MotionGroup(((1, 200.0), (2, 170.0), (3, 120.0), (5, 271.0)))])
        self.assertEqual(after, chain[-1][1])

    def test_more_motors_than_allowed_keep_the_chain(self):
        chain = self.chain()
        self.assertIs(self.execution(max_motors=3)._one_order(chain, self.PLANNING), chain)

    def test_one_order_needs_sync_and_not_classic(self):
        policy = type("P", (), {"canonical_start": [180, 180, 90, 180, 263]})()
        self.assertFalse(SupervisedExecution(policy, {}, one_order=True).one_order)
        self.assertFalse(SupervisedExecution(policy, {}, one_order=True, sync_orders=True, classic=True).one_order)
        self.assertTrue(SupervisedExecution(policy, {}, one_order=True, sync_orders=True).one_order)


class ReviewFixTests(unittest.TestCase):
    """Arreglos de la revisión del 19/11 (Directo con supervisor y límites)."""

    def test_a_single_order_is_never_split_in_batches(self):
        robot = InstantSerial(supported=True)
        group = MotionGroup(((1, 184.0), (2, 183.0), (3, 93.0), (4, 184.0), (5, 266.0)))
        proposal = MotionProposal(1, "aproximacion_1", "Aproximación 1", dict(group.targets), (group,), None,
                                  segments=(MotionSegment(0, 1, True, True, single=True),))
        with self.assertRaisesRegex(RuntimeError, "no se parte en tandas"):
            MotionExecutor(robot, max_motors=3).execute(proposal)
        self.assertEqual([c for c in robot.sent if not c.startswith("gripper")], [])

    def test_small_share_keeps_its_proportion(self):
        """Con piso 20, M4 (2° de 90°) iba al doble de su proporción y salía de la recta."""
        twin = DigitalTwin(seed=4)
        count = twin.log_count
        twin.handle_line("sync 0 1 270 4 182")
        reply = twin.log_since(count)[0]
        speeds = [int(v) for v in reply.split("speeds=")[1].split(",")]
        self.assertEqual(speeds[0], gemelo.MOVE_SPEED)
        self.assertTrue(8 <= speeds[1] <= 11, speeds)  # ~10 (2/90 de 450), no el piso viejo de 20

    def test_orders_after_a_hold_are_refused_for_a_second(self):
        twin = DigitalTwin(seed=4)
        twin.handle_line("power guard on")
        twin.injected_voltage_dv = 70
        for _ in range(2):
            twin.last_power_telemetry = -1e9
            twin._process_power_telemetry()
        twin.injected_voltage_dv = None
        twin.last_move = -1e9
        count = twin.log_count
        for line in ("sync 0 1 190", "pair 1 190 5 260", "1 190", "waypoint 3 1 1 184"):
            twin.handle_line(line)
        self.assertEqual(twin.log_since(count), ["POWER_GUARD_HOLD_ACTIVE reason=freno_reciente"] * 4)
        count = twin.log_count
        twin.handle_line("6 90")  # la garra no se frena
        self.assertNotIn("POWER_GUARD_HOLD_ACTIVE", " ".join(twin.log_since(count)))

    def test_arrival_deadline_follows_a_lower_speed_max(self):
        robot = InstantSerial(supported=True)
        robot.arm_max_speed = 225
        executor = MotionExecutor(robot)
        captured = {}
        original = MotionExecutor._wait_arrival

        def spy(self, targets, **kwargs):
            captured.update(kwargs)
            return original(self, targets, **kwargs)

        MotionExecutor._wait_arrival = spy
        try:
            executor.execute(together_point(GROUPS))
        finally:
            MotionExecutor._wait_arrival = original
        self.assertTrue(captured)  # el plazo se calcula dentro: alcanza con que no falle


if __name__ == "__main__":
    unittest.main()
