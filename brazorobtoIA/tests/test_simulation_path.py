import math
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace

import numpy as np

from brazoia.config import INITIAL_POSE, load_config
from brazoia.gripper_geometry import GripperGeometry
from brazoia.simulation_path import FloorPathPlanner, SimulationCancelled, pose_from_snapshot


POSE = {**INITIAL_POSE, 6: 0.0}
LIMITS = load_config().limits


class BarrierGeometry:
    """Piso sintético: el paso directo choca y elevar M4 permite rodearlo."""
    def clearance(self, pose):
        return {"height_mm": 10 + max(abs(pose[2] - 180) - 40, pose[4] - 150)}

    def motion_bound_mm(self, a, b):
        return abs(a[2] - b[2]) + abs(a[4] - b[4])


class SimulationPathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = GripperGeometry(Path(__file__).resolve().parents[1] / "assets")

    def test_real_mesh_direct_path_preserves_endpoints_and_margin(self):
        goal = {**POSE, 1: 150, 2: 190, 5: 210, 6: 80}
        result = FloorPathPlanner(self.geometry, LIMITS).plan(POSE, goal)
        self.assertEqual(result.kind, "Directo")
        self.assertEqual(result.pose_at(0), POSE)
        self.assertEqual(result.pose_at(result.duration()), goal)
        self.assertGreater(result.minimum_bound_mm, 5)
        for seconds in np.linspace(0, result.duration(), 101):
            self.assertGreater(self.geometry.clearance(result.pose_at(seconds))["height_mm"], 5)

    def test_safe_endpoints_do_not_allow_unsafe_middle_and_detour_is_found(self):
        geometry = BarrierGeometry()
        start, goal = {**POSE, 2: 100, 4: 100}, {**POSE, 2: 260, 4: 100}
        planner = FloorPathPlanner(geometry, LIMITS)
        self.assertGreater(planner.height(start), 5)
        self.assertGreater(planner.height(goal), 5)
        self.assertIsNone(planner.edge_bound(start, goal))
        result = planner.plan(start, goal)
        self.assertEqual(result.kind, "Con elevación intermedia")
        self.assertGreater(len(result.waypoints), 2)
        self.assertEqual(result.waypoints[0], start)
        self.assertEqual(result.waypoints[-1], goal)
        for seconds in np.linspace(0, result.duration(), 500):
            self.assertGreater(geometry.clearance(result.pose_at(seconds))["height_mm"], 5)

    def test_floor_endpoint_out_of_limits_and_nan_are_rejected(self):
        planner = FloorPathPlanner(BarrierGeometry(), LIMITS)
        with self.assertRaisesRegex(ValueError, "Inicio.*piso virtual"):
            planner.plan({**POSE, 4: 100}, POSE)
        for pose in ({**POSE, 2: 400}, {**POSE, 6: math.nan}):
            with self.assertRaises(ValueError):
                planner.plan(pose, POSE)

    def test_cancel_timeout_and_identical_pose(self):
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(SimulationCancelled):
            FloorPathPlanner(self.geometry, LIMITS, cancel=cancel).plan(POSE, POSE)
        with self.assertRaisesRegex(ValueError, "tiempo"):
            FloorPathPlanner(self.geometry, LIMITS, timeout=-1).plan(POSE, POSE)
        result = FloorPathPlanner(self.geometry, LIMITS).plan(POSE, POSE)
        self.assertEqual(result.duration(), 0)
        self.assertEqual(result.pose_at(0), POSE)

    def test_motion_bound_covers_real_vertices_for_all_six_axes(self):
        rng = np.random.default_rng(12)
        for _ in range(25):
            a = {i: float(rng.uniform(*LIMITS[i])) for i in range(1, 7)}
            b = {i: a[i] + float(rng.uniform(-1, 1)) for i in range(1, 7)}
            bound = self.geometry.motion_bound_mm(a, b)
            ta, _ = self.geometry.transforms(a)
            tb, _ = self.geometry.transforms(b)
            for link, _, vertices in self.geometry.meshes:
                wa = vertices @ ta[link][:3, :3].T + ta[link][:3, 3]
                wb = vertices @ tb[link][:3, :3].T + tb[link][:3, 3]
                self.assertLessEqual(float(np.linalg.norm(wb-wa, axis=1).max())*1000, bound+1e-8)

    def test_snapshot_reader_rejects_stale_missing_nonfinite_without_mutation(self):
        snapshot = SimpleNamespace(positions=dict(POSE), received_at={i: 99.8 for i in POSE})
        result = pose_from_snapshot(snapshot, now=100, maximum_age=.5)
        result[1] = 123
        self.assertEqual(snapshot.positions[1], 180)
        snapshot.received_at[6] = 98
        with self.assertRaisesRegex(ValueError, "M6"):
            pose_from_snapshot(snapshot, now=100, maximum_age=.5)
        snapshot.received_at[6] = 99.8
        snapshot.positions[6] = math.nan
        with self.assertRaises(ValueError):
            pose_from_snapshot(snapshot, now=100, maximum_age=.5)


if __name__ == "__main__":
    unittest.main()
