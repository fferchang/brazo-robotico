"""Protocolo de puntos de firmware en el puerto simulado."""
import threading
import unittest

import mujoco

from brazoia.execution import MotionExecutor, MotionProposal, MotionSegment
from brazoia.planner import MotionGroup

from brazoia.gemelo import DigitalTwin
from brazoia.robot import RobotSerial


class WaypointTests(unittest.TestCase):
    def setUp(self):
        self.twin = DigitalTwin(seed=4)

    def advance(self, ticks, *, keepalive=False):
        for tick in range(ticks):
            if keepalive and tick % 20 == 0:
                self.twin.handle_line("waypoint keepalive 1")
            self.twin.tick()

    def start(self):
        self.twin.handle_line("waypoint 1 2 1 184 5 267 2 2 182 3 93")
        self.assertIn("WAYPOINT_ACCEPTED id=1", self.twin.log_since(0))

    def test_point_runs_groups_in_order_and_reports_completion(self):
        self.start()
        self.advance(12)
        self.assertEqual(self.twin.profiles[1].target, 184.0)
        self.assertEqual(self.twin.profiles[2].target, 180.0)
        self.twin.handle_line("2 190")
        self.assertIn("WAYPOINT_BUSY id=1", self.twin.log_since(0))
        self.advance(200, keepalive=True)
        self.assertIn("WAYPOINT_DONE id=1", self.twin.log_since(0))
        self.assertEqual(self.twin.profiles[2].target, 182.0)

    def test_cancel_discards_pending_groups_after_first_dispatch(self):
        self.start()
        self.advance(12)
        self.twin.handle_line("waypoint cancel 1")
        self.advance(200, keepalive=True)
        self.assertIn("WAYPOINT_CANCELLED id=1", self.twin.log_since(0))
        self.assertEqual(self.twin.profiles[1].target, 184.0)
        self.assertEqual(self.twin.profiles[2].target, 180.0)

    def test_cube_loss_cancels_pending_groups_and_holds_release(self):
        self.start()
        stop = threading.Event()
        self.twin.start_cube_monitor(stop)
        self.twin.set_cube_monitor_stage("traslado_3")
        self.advance(1)
        arm = self.twin.arm
        arm.data.qpos[arm.cube_qadr] += 0.2
        mujoco.mj_forward(arm.model, arm.data)
        self.advance(2)
        self.assertFalse(self.twin.set_cube_monitor_stage("soltado"))
        self.assertFalse(stop.is_set())
        self.advance(12)
        self.assertTrue(stop.is_set())
        self.assertIn("WAYPOINT_CANCELLED id=1", self.twin.log_since(0))
        self.assertEqual(self.twin.profiles[2].target, 180.0)
        self.assertEqual(self.twin.finish_cube_monitor().stage, "traslado_3")

    def test_heartbeat_expiration_prevents_late_motion(self):
        self.twin.handle_line("waypoint 1 2 1 230 5 290 2 2 182 3 93")
        self.advance(200)
        self.assertIn("WAYPOINT_REJECTED id=1 reason=heartbeat", self.twin.log_since(0))
        self.assertEqual(self.twin.profiles[2].target, 180.0)

    def test_gripper_closing_and_malformed_points_are_rejected(self):
        self.twin.handle_line("waypoint 1 1 6 90")
        self.twin.handle_line("waypoint 2 2 1 184 1 185")
        self.twin.gripper_closing = True
        self.twin.handle_line("waypoint 3 1 1 184")
        self.assertEqual(self.twin.log_since(0)[-3:], [
            "WAYPOINT_REJECTED id=1 reason=motores",
            "WAYPOINT_REJECTED id=2 reason=motores",
            "WAYPOINT_REJECTED id=3 reason=gripper_activa",
        ])
        self.assertIsNone(self.twin.waypoint)

    def test_cancelled_identifier_cannot_be_replayed(self):
        self.start()
        self.twin.handle_line("waypoint cancel 1")
        self.twin.handle_line("waypoint 1 1 1 184")
        self.assertIn("WAYPOINT_REJECTED id=1 reason=id_repetido", self.twin.log_since(0))
        self.assertIsNone(self.twin.waypoint)

    def test_gripper_watch_interrupts_before_next_group(self):
        self.start()
        self.advance(12)
        self.twin.gripper_reliefs += 1
        self.advance(1)
        self.assertIn("WAYPOINT_REJECTED id=1 reason=gripper_activa", self.twin.log_since(0))
        self.assertEqual(self.twin.profiles[2].target, 180.0)

    def test_host_ignores_reply_for_a_previous_point(self):
        robot = RobotSerial("SIMULADO", 115200)
        robot._waypoint_id = 0
        self.assertEqual(robot.next_waypoint_id(), 1)
        self.assertEqual(robot.next_waypoint_id(), 2)
        robot._consume_line("WAYPOINT_ACCEPTED id=2", 1.0)
        robot._consume_line("WAYPOINT_DONE id=1", 1.1)
        self.assertEqual(robot.waypoint_reply(2), ("ACCEPTED", ""))
        robot._consume_line("WAYPOINT_REJECTED id=2 reason=heartbeat", 1.2)
        self.assertEqual(robot.waypoint_reply(2), ("REJECTED", "heartbeat"))

    def test_executor_stop_sends_cancel_to_firmware(self):
        submitted = threading.Event()
        sent = []

        class SimulatedSerial(RobotSerial):
            def send(self, command):
                sent.append(command)
                if command.startswith("waypoint 1 "):
                    self._consume_line("WAYPOINT_ACCEPTED id=1", 1.0)
                    submitted.set()

        robot = SimulatedSerial("SIMULADO", 115200)
        robot._waypoint_id = 0
        robot.torque_enabled = True
        executor = MotionExecutor(robot, firmware_waypoints=True)
        point = MotionProposal(
            8, "traslado_2", "Traslado 2", {1: 184.0, 5: 267.0},
            (MotionGroup(((1, 184.0), (5, 267.0))),), None,
            segments=(MotionSegment(0, 1),),
        )
        errors = []

        def execute():
            try:
                executor.execute(point)
            except RuntimeError as exc:
                errors.append(str(exc))

        worker = threading.Thread(target=execute)
        worker.start()
        try:
            self.assertTrue(submitted.wait(1))
        finally:
            executor.stop()
            worker.join(timeout=1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, ["Movimiento detenido por el operador."])
        self.assertEqual(sent[-1], "waypoint cancel 1")


if __name__ == "__main__":
    unittest.main()
