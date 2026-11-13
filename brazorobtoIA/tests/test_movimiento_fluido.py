"""Etapa 1 de la vuelta más rápida: garra una sola vez y menos paradas."""
from __future__ import annotations

import threading
import time
import types
import unittest
from unittest.mock import patch

from brazoia.calibracion_camaras import fit_calibration, settings_from_config
from brazoia.config import INITIAL_POSE, KEYFRAME_STAGES, load_config
from brazoia.execution import (
    MOVE_SPACING_SECONDS,
    MotionExecutor,
    MotionProposal,
    MotionSegment,
    SupervisedExecution,
)
from brazoia.gripper_geometry import model_fingerprint
from brazoia.mujoco_sim import (
    GRIPPER_GRASP_OPEN,
    PickPlaceExpert,
    Placement,
    VirtualArm,
)
from brazoia.planner import MotionGroup
from brazoia.politica_geometrica import GeometricPolicy
from brazoia.robot import RobotSerial
from tests.test_calibracion_camaras import samples_for, scene_features, workspace_points
from tests.test_politica_geometrica import observations_for

CONFIG = load_config()
PLACEMENT = Placement((0.08, -0.25), 0.0, (-0.12, -0.22), 0.0)


class ExpertGripperTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.arm = VirtualArm(limits=CONFIG.limits)
        cls.expert = PickPlaceExpert(cls.arm)
        cls.plan = cls.expert.plan(PLACEMENT)

    def keyframe(self, stage):
        return self.plan.keyframes[KEYFRAME_STAGES.index(stage)]

    def test_opens_once_partially_while_moving_and_closes_by_angle_on_the_way_home(self) -> None:
        steps = {k.stage: (k.gripper_action, k.gripper_degrees, k.gripper_timing)
                 for k in self.plan.keyframes if k.gripper_action}
        self.assertEqual(steps, {
            "aproximacion_2": ("angle", GRIPPER_GRASP_OPEN, "before"),
            "agarre": ("close", None, "after"),
            "soltado": ("angle", GRIPPER_GRASP_OPEN, "after"),
            "guardado": ("close", None, "final"),
        })
        self.assertEqual(self.plan.warnings, [])

    def test_path_check_covers_the_gripper_moving_with_the_arm(self) -> None:
        # Un tramo que choca solo con la garra a mitad de camino tiene que aparecer en los avisos.
        checked = []
        original = self.arm.segment_problems

        def recording(start, end, samples=12):
            checked.append((start[6], end[6]))
            return original(start, end, samples)

        self.arm.segment_problems = recording
        try:
            self.expert._check_path(self.plan.keyframes)
        finally:
            self.arm.segment_problems = original
        # Aproximación 2 se revisa con M6 quieta en cada valor del barrido de 0° a 90°.
        values = {start for start, end in checked if start == end}
        self.assertIn(0.0, values)
        self.assertIn(GRIPPER_GRASP_OPEN, values)
        self.assertTrue(any(0.0 < value < GRIPPER_GRASP_OPEN for value in values))


class PreparedSegmentsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        arm = VirtualArm(limits=CONFIG.limits)
        calibration = fit_calibration(samples_for(workspace_points(14)) + samples_for(workspace_points(14), "recipiente"),
                                      settings_from_config(CONFIG.cameras), model=model_fingerprint(arm.assets_root))
        cls.policy = GeometricPolicy(calibration, CONFIG.limits, arm=arm, expert=PickPlaceExpert(arm))
        execution = SupervisedExecution(cls.policy, CONFIG.limits)
        current = {**INITIAL_POSE, 6: 0.0}
        observations = observations_for(PLACEMENT)
        cls.proposals = {}
        while not execution.finished:
            proposal = execution.prepare(current, observations)
            cls.scene = execution.scene_features
            cls.proposals[proposal.stage] = proposal
            execution.mark_completed()
            current = {**current, **proposal.target}
            if proposal.gripper_action == "angle":
                current[6] = proposal.gripper_degrees
            elif proposal.gripper_action == "close":
                current[6] = 40.0 if proposal.stage == "agarre" else 1.5

    def test_gripper_opens_from_the_first_group_of_approach_2(self) -> None:
        proposal = self.proposals["aproximacion_2"]
        self.assertEqual(proposal.gripper_action, "angle")
        self.assertEqual(proposal.gripper_degrees, GRIPPER_GRASP_OPEN)
        self.assertEqual(proposal.gripper_at_group, 0)
        # Ya no se cierra ni se reabre sobre el cubo.
        self.assertIsNone(self.proposals["aproximacion_3"].gripper_action)
        self.assertIsNone(self.proposals["apertura_sobre_cubo"].gripper_action)

    def test_guardado_closes_by_pressure_after_the_vertical_rise(self) -> None:
        proposal = self.proposals["guardado"]
        self.assertEqual(proposal.gripper_action, "close")
        # Empieza al salir del último punto previo (el retiro), no sobre el cubo soltado.
        via = self.policy.via(KEYFRAME_STAGES.index("guardado"), self.scene)
        current = dict(self.proposals["soltado"].target)
        for group in proposal.groups[:proposal.gripper_at_group]:
            current.update(dict(group.targets))
        self.assertTrue(all(abs(current[m] - via[-1][m]) <= 0.6 for m in range(1, 6)), current)
        self.assertGreater(proposal.gripper_at_group, 0)

    def test_big_moves_go_in_chained_steps_with_all_motors(self) -> None:
        for stage in ("aproximacion_1", "guardado"):
            proposal = self.proposals[stage]
            chained = [segment for segment in proposal.segments if segment.together]
            self.assertGreaterEqual(len(chained), 5, stage)
            # Casi todos encadenados sin frenar: se pasa al siguiente a mitad de camino.
            blended = [segment for segment in proposal.segments[:-1] if segment.handoff > 0]
            self.assertGreaterEqual(len(blended), len(proposal.segments) // 2, stage)
            self.assertTrue(proposal.segments[-1].final)
            self.assertEqual(proposal.segments[-1].handoff, 0.0)
            for segment in chained:
                moved = [m for group in proposal.groups[segment.start:segment.end] for m, _ in group.targets]
                self.assertEqual(len(moved), len(set(moved)), stage)

    def test_direct_demo_reuses_regular_chain_when_collision_check_rejects_jump(self) -> None:
        execution = SupervisedExecution(self.policy, CONFIG.limits, direct_demo=True)
        current = {**INITIAL_POSE, 6: 0.0}
        target = self.proposals["aproximacion_1"].target
        check = execution._together_problem

        def reject_wide(*args, **kwargs):
            if kwargs.get("limit", 0) > 12.0:
                return "choque en el salto directo"
            return check(*args, **kwargs)

        with patch.object(self.policy, "container_obstacle", False):
            with patch.object(execution, "_together_problem", side_effect=reject_wide) as review:
                fallback = execution._plan_waypoints(current, (target,), 1, 0.0, None)
            self.assertTrue(any(call.kwargs.get("limit", 0) > 12.0 for call in review.call_args_list))
            execution.direct_demo = False
            self.assertEqual(fallback, execution._plan_waypoints(current, (target,), 1, 0.0, None))

    def test_segments_cover_all_groups_and_only_the_last_one_settles(self) -> None:
        for stage, proposal in self.proposals.items():
            if not proposal.groups:
                continue
            self.assertEqual(proposal.segments[0].start, 0, stage)
            self.assertEqual(proposal.segments[-1].end, len(proposal.groups), stage)
            for previous, following in zip(proposal.segments, proposal.segments[1:]):
                self.assertEqual(previous.end, following.start, stage)
            self.assertEqual([segment.final for segment in proposal.segments],
                             [False] * (len(proposal.segments) - 1) + [True], stage)

    def test_the_level_push_goes_with_motors_together(self) -> None:
        descent = self.proposals["descenso_abierto"]
        together = [segment for segment in descent.segments if segment.together]
        # Casi todos los pasos del descenso y el empuje son chicos y seguros a la vez.
        self.assertGreaterEqual(len(together), len(descent.segments) // 2)

    def test_a_box_crossing_the_table_is_not_sent_together(self) -> None:
        execution = SupervisedExecution(self.policy, CONFIG.limits)
        grasp = self.proposals["agarre"].target
        lower = {**grasp, 3: grasp[3] + 9.0, 4: grasp[4] - 9.0}
        problem = execution._together_problem(grasp, lower, 90.0, (90.0,))
        self.assertIsNotNone(problem)


class BlendReviewTests(unittest.TestCase):
    def test_blend_includes_lagging_motors_and_intermediate_gripper_angles(self) -> None:
        execution = SupervisedExecution(object(), CONFIG.limits)
        initial = {1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0}
        groups = (
            MotionGroup(((1, 186.0),)), MotionGroup(((5, 264.0),)),
            MotionGroup(((2, 186.0),)), MotionGroup(((4, 181.0),)),
            MotionGroup(((2, 192.0),)), MotionGroup(((4, 182.0),)),
        )
        segments = [MotionSegment(index, index + 2, together=True, final=index == 4)
                    for index in range(0, len(groups), 2)]

        def check(before, after, m6, m6_values, *, poses, limit):
            if after[2] == 192.0:
                self.assertEqual(min(pose[1] for pose in poses), 183.0)
                self.assertIsNone(m6)
                self.assertTrue(any(0.0 < angle < 90.0 for angle in m6_values))
                return "choque con M1 rezagado y M6 a medio abrir"
            return None

        execution._together_problem = check
        result = execution._blend(groups, segments, initial, 2, 0.0, 90.0)
        self.assertEqual(result[0].handoff, 3.0)
        self.assertEqual(result[1].handoff, 0.0)


class FakeRobot:
    """Responde como el firmware: cada orden lleva el motor a su destino al instante."""

    def __init__(self, positions):
        self.positions = dict(positions)
        self.sent: list[tuple[float, str]] = []
        self.lock = threading.Lock()

    def send(self, command):
        with self.lock:
            self.sent.append((time.monotonic(), command))
            parts = command.split()
            if parts[0] == "pair":
                self.positions[int(parts[1])] = float(parts[2])
                self.positions[int(parts[3])] = float(parts[4])
            elif parts[0].isdigit():
                self.positions[int(parts[0])] = float(parts[1])

    def snapshot(self):
        with self.lock:
            return types.SimpleNamespace(positions=dict(self.positions), fault="", torque_enabled=True,
                                         gripper_state="idle")


class ExecutorTests(unittest.TestCase):
    def test_abort_during_spacing_prevents_the_next_command(self) -> None:
        robot = FakeRobot({})
        abort = threading.Event()
        executor = MotionExecutor(robot, abort=abort)
        executor._last_move_sent = time.monotonic()

        def abort_during_wait(timeout):
            abort.set()
            return False

        with patch.object(executor.stop_event, "wait", side_effect=abort_during_wait):
            with self.assertRaisesRegex(RuntimeError, "Movimiento detenido"):
                executor._send_move("1 180")
        self.assertEqual(robot.sent, [])

    def test_together_groups_go_out_spaced_and_the_gripper_moves_with_the_arm(self) -> None:
        robot = FakeRobot({1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0, 6: 0.0})
        groups = (MotionGroup(((1, 182.0), (5, 261.0))), MotionGroup(((2, 178.0), (3, 92.0))),
                  MotionGroup(((4, 183.0),)))
        proposal = MotionProposal(2, "aproximacion_2", "Aproximación 2/3", {}, groups, "angle",
                                  gripper_degrees=90.0, gripper_at_group=0,
                                  segments=(MotionSegment(0, 3, together=True, final=True),))
        timings = MotionExecutor(robot).execute(proposal)
        self.assertGreater(timings["orders"], 0.0)
        self.assertGreater(timings["arrival"], 0.0)
        self.assertGreater(timings["settling"], 0.0)
        self.assertGreater(timings["gripper"], 0.0)
        self.assertEqual(timings["after_grasp"], 0.0)
        self.assertGreaterEqual(timings["total"], sum(value for key, value in timings.items() if key != "total"))
        commands = [command for _, command in robot.sent]
        self.assertEqual(commands, ["6 90.00", "pair 1 182.00 5 261.00", "pair 2 178.00 3 92.00", "4 183.00"])
        stamps = [stamp for stamp, _ in robot.sent]
        self.assertTrue(all(b - a >= MOVE_SPACING_SECONDS - 0.005 for a, b in zip(stamps, stamps[1:])))
        # Seguidos: entre órdenes no hay espera de llegada (que dura al menos 3 lecturas de 50 ms).
        self.assertLess(stamps[-1] - stamps[0], 3 * MOVE_SPACING_SECONDS + 0.1)

    def test_end_of_blend_waits_for_motors_omitted_from_the_last_segment(self) -> None:
        robot = FakeRobot({1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0, 6: 0.0})
        groups = (MotionGroup(((1, 186.0),)), MotionGroup(((5, 269.0),)),
                  MotionGroup(((2, 186.0),)), MotionGroup(((4, 186.0),)))
        proposal = MotionProposal(1, "aproximacion_1", "Aproximación 1/3", {}, groups, None,
                                  segments=(MotionSegment(0, 2, together=True, handoff=3.0),
                                            MotionSegment(2, 4, together=True, final=True)))
        executor = MotionExecutor(robot)
        with patch.object(executor, "_wait_arrival") as wait:
            executor.execute(proposal)
        self.assertEqual(wait.call_args.args[0], {1: 186.0, 2: 186.0, 4: 186.0, 5: 269.0})
        self.assertTrue(wait.call_args.kwargs["settle"])

    def test_legacy_proposals_still_wait_for_each_group(self) -> None:
        robot = FakeRobot({1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0, 6: 0.0})
        groups = (MotionGroup(((2, 178.0), (3, 92.0))), MotionGroup(((4, 183.0),)))
        proposal = MotionProposal(1, "aproximacion_1", "Aproximación 1/3", {}, groups, None)
        MotionExecutor(robot).execute(proposal)
        stamps = [stamp for stamp, _ in robot.sent]
        self.assertGreaterEqual(stamps[1] - stamps[0], 0.15)


class FirmwareLikeSerial(RobotSerial):
    """RobotSerial sin puerto: cada orden recibe la respuesta que daría el firmware."""

    def __init__(self, positions, replies=None, hold_reason="contact", hold_delay=0.15):
        super().__init__("PRUEBA", 115200)
        self.sent: list[str] = []
        self.replies = dict(replies or {})
        self.hold_reason = hold_reason
        self.hold_delay = hold_delay
        self.torque_enabled = True
        self.gripper_state = "holding"  # el de un agarre anterior
        self.gripper_holds = 1
        for motor, value in positions.items():
            self._consume_line(f"POS ID {motor}: 0 (~{value:.1f} deg)", time.monotonic())
        self._feed_positions(positions)

    def _feed_positions(self, positions):
        self.current = dict(positions)
        for motor, value in self.current.items():
            with self.state_lock:
                self.positions[motor] = value
                self.received_at[motor] = time.monotonic()

    def send(self, command):
        self.sent.append(command)
        stamp = time.monotonic()
        if command in self.replies:
            self._consume_line(self.replies[command], stamp)
            return
        parts = command.split()
        if parts[0] == "pair":
            self.current[int(parts[1])] = float(parts[2])
            self.current[int(parts[3])] = float(parts[4])
            self._consume_line(f"PAIR_MOVE_OK ID1={parts[1]} target1={parts[2]} ID2={parts[3]} target2={parts[4]}",
                               stamp)
        elif parts[0].isdigit():
            self.current[int(parts[0])] = float(parts[1])
            self._consume_line(f"Moviendo ID {parts[0]} a {parts[1]} deg -> pos 0", stamp)
        elif command == "gripper close":
            self._consume_line("GRIPPER_CLOSE_START from=90.0 contact_raw=92", stamp)

            def hold():
                time.sleep(self.hold_delay)
                self._consume_line(f"GRIPPER_HOLD reason={self.hold_reason} current_raw=120", time.monotonic())

            threading.Thread(target=hold, daemon=True).start()
        self._feed_positions(self.current)


class ReviewFixTests(unittest.TestCase):
    START = {1: 180.0, 2: 180.0, 3: 90.0, 4: 180.0, 5: 263.0, 6: 90.0}

    def test_a_safety_adjustment_stops_the_together_jump_before_the_next_order(self) -> None:
        robot = FirmwareLikeSerial(self.START, {"pair 2 178.00 3 92.00": "AJUSTE SEGURO ID 3: 92.00 -> 90.50"})
        groups = (MotionGroup(((1, 182.0), (5, 261.0))), MotionGroup(((2, 178.0), (3, 92.0))),
                  MotionGroup(((4, 183.0),)))
        proposal = MotionProposal(5, "descenso_abierto", "Bajar", {}, groups, None,
                                  segments=(MotionSegment(0, 3, together=True, final=True),))
        with self.assertRaisesRegex(RuntimeError, "AJUSTE SEGURO"):
            MotionExecutor(robot).execute(proposal)
        self.assertNotIn("4 183.00", robot.sent)

    def test_agarre_waits_for_a_new_hold_not_the_previous_one(self) -> None:
        robot = FirmwareLikeSerial(self.START, hold_delay=0.4)
        proposal = MotionProposal(6, "agarre", "Agarre", {}, (), "close")
        started = time.monotonic()
        MotionExecutor(robot).execute(proposal)
        self.assertGreaterEqual(time.monotonic() - started, 0.35)

    def test_after_grasping_it_waits_before_lifting(self) -> None:
        from brazoia.execution import GRIP_SETTLE_SECONDS

        robot = FirmwareLikeSerial(self.START, hold_delay=0.1)
        started = time.monotonic()
        timings = MotionExecutor(robot).execute(MotionProposal(6, "agarre", "Agarre", {}, (), "close"))
        self.assertGreaterEqual(time.monotonic() - started, 0.1 + GRIP_SETTLE_SECONDS - 0.05)
        self.assertGreater(timings["gripper"], 0.0)
        self.assertGreaterEqual(timings["after_grasp"], GRIP_SETTLE_SECONDS - 0.05)
        self.assertEqual(timings["arrival"], 0.0)
        self.assertEqual(timings["settling"], 0.0)

    def test_with_power_telemetry_it_waits_until_the_gripper_current_drops(self) -> None:
        robot = FirmwareLikeSerial(self.START, hold_delay=0.05)
        stop = threading.Event()

        def telemetry():
            count = 0
            while not stop.is_set():
                m6 = 150 if count < 6 else 20  # crudo: ~975 mA al apretar, ~130 mA después
                robot._consume_line(f"POWER_TELEM ms={count * 100} I=1,10,5,2,1,{m6} total_ma=0 v_min_dv=117 v_id=5",
                                    time.monotonic())
                count += 1
                stop.wait(0.1)

        thread = threading.Thread(target=telemetry, daemon=True)
        thread.start()
        try:
            time.sleep(0.15)
            started = time.monotonic()
            MotionExecutor(robot).execute(MotionProposal(6, "agarre", "Agarre", {}, (), "close"))
            elapsed = time.monotonic() - started
        finally:
            stop.set()
            thread.join(timeout=1.0)
        # Espera las muestras altas (~0,45 s más) y corta con dos bajas, antes del límite de 2 s.
        self.assertGreater(elapsed, 0.4)
        self.assertLess(elapsed, 1.5)

    def test_every_pressure_close_sets_the_lower_contact_threshold_first(self) -> None:
        from brazoia.execution import GRIPPER_CONTACT_RAW

        robot = FirmwareLikeSerial(self.START, hold_delay=0.05)
        MotionExecutor(robot).execute(MotionProposal(6, "agarre", "Agarre", {}, (), "close"))
        index = robot.sent.index("gripper close")
        self.assertEqual(robot.sent[index - 1], f"gripper current {GRIPPER_CONTACT_RAW}")
        self.assertLess(GRIPPER_CONTACT_RAW, 92)

    def test_agarre_that_closes_on_nothing_stops_the_task(self) -> None:
        robot = FirmwareLikeSerial(self.START, hold_reason="closed_limit")
        proposal = MotionProposal(6, "agarre", "Agarre", {}, (), "close")
        with self.assertRaisesRegex(RuntimeError, "sin agarrar"):
            MotionExecutor(robot).execute(proposal)

    def test_opening_on_the_way_from_a_low_pose_waits_until_arrival(self) -> None:
        class Policy:
            canonical_start = [180.0, 180.0, 90.0, 180.0, 263.0]
            canonical_guard = canonical_start

            def predict(self, stage_index, features):
                return {1: 150.0, 2: 150.0, 3: 200.0, 4: 250.0, 5: 230.0}

            def gripper_step(self, stage_index, features):
                return "angle", 90.0, "before"

        execution = SupervisedExecution(Policy(), CONFIG.limits)
        execution.next_stage_index = 2
        execution.scene_features = [0.0]
        low = {1: 180.0, 2: 100.0, 3: 260.0, 4: 225.0, 5: 263.0, 6: 2.0}
        proposal = execution.prepare(low, [])
        self.assertEqual(proposal.gripper_action, "angle")
        self.assertIsNone(proposal.gripper_at_group)

    def test_manual_m6_opening_is_checked_against_the_table(self) -> None:
        from app import manual_motor_proposal

        now = time.monotonic()
        low = {1: 180.0, 2: 100.0, 3: 260.0, 4: 225.0, 5: 263.0, 6: 2.0}
        snapshot = types.SimpleNamespace(positions=low, received_at={motor: now for motor in low}, fault="",
                                         torque_enabled=True, gripper_state="idle")
        with self.assertRaisesRegex(ValueError, "mesa"):
            manual_motor_proposal(snapshot, 6, 140.0, CONFIG.limits, 1.0, now)
        high = {**low, 2: 180.0, 3: 90.0, 4: 180.0}
        snapshot.positions = high
        self.assertEqual(manual_motor_proposal(snapshot, 6, 140.0, CONFIG.limits, 1.0, now).target, {6: 140.0})


if __name__ == "__main__":
    unittest.main()
