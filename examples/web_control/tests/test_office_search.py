import unittest
import math
import numpy as np
from collections import deque
from unittest.mock import Mock

from office_search import FloorMap, OfficeSearch
from navigation import Decision
from test_navigation import decision


class OfficeSearchTests(unittest.TestCase):
    def checked_route(self):
        search, scene = self.head_wait()
        search.settling = 0
        search.phase = "route"
        search.motion = None
        search.checked_forward = dict(amount=30, pose=(0, 0, 0), bearing=0, speed=0.3)
        scene.motion_pose.return_value = (0.01, 0, 0)
        scene.proximity.return_value = {"visible": False}
        return search, scene

    def test_forward_starts_after_reset_and_deducts_head_drift(self):
        search, scene = self.checked_route()
        search.tick(scene, "紫色球")
        self.assertEqual(search.motion.action, "forward")
        self.assertAlmostEqual(search.motion.amount, 29)
        self.assertIsNone(search.checked_forward)

    def test_head_wait_does_not_start_forward(self):
        search, scene = self.checked_route()
        search.settling = 10
        search.waiting_for_head = False
        search.tick(scene, "紫色球")
        self.assertIsNone(search.motion)
        scene.motion_step.assert_not_called()

    def test_large_pose_change_discards_ground_evidence(self):
        for pose in [(0.04, 0, 0), (0, 0, math.radians(6))]:
            search, scene = self.checked_route()
            scene.motion_pose.return_value = pose
            search.tick(scene, "紫色球")
            self.assertIsNone(search.motion)
            self.assertIsNone(search.checked_forward)

    def test_visible_target_after_head_reset_returns_to_model(self):
        search, scene = self.checked_route()
        scene.proximity.return_value = {"visible": True}
        self.assertEqual(search.tick(scene, "紫色球"), "target_visible")
        self.assertIsNone(search.motion)

    def head_wait(self):
        search = OfficeSearch.__new__(OfficeSearch)
        search.head_mode = "unknown"
        search.head_samples = deque(maxlen=10)
        search.steps = search.head_wait_steps = 0
        scene = Mock()
        scene.head_feedback.return_value = {"camera_pitch_deg": -20}
        scene.head_command_reached.return_value = True
        scene.body_stable.return_value = True
        search.prepare_head(scene, "head_down")
        return search, scene

    def test_stable_head_wait_ends_after_minimum_time(self):
        search, scene = self.head_wait()
        for _ in range(34):
            search.tick(scene, "寻找紫色球")
        self.assertGreater(search.settling, 0)
        search.tick(scene, "寻找紫色球")
        self.assertEqual(search.settling, 0)
        self.assertFalse(search.waiting_for_head)

    def test_unstable_body_uses_full_head_wait(self):
        search, scene = self.head_wait()
        scene.body_stable.return_value = False
        for _ in range(149):
            search.tick(scene, "寻找紫色球")
        self.assertEqual(search.settling, 1)
        search.tick(scene, "寻找紫色球")
        self.assertEqual(search.settling, 0)

    def test_repeated_head_target_does_not_restart_wait(self):
        search, scene = self.head_wait()
        search.settling = 0
        search.waiting_for_head = False
        search.prepare_head(scene, "head_down")
        self.assertEqual(search.settling, 0)
        scene.navigation_action.assert_called_once_with("head_down")

    def test_high_level_contract(self):
        self.assertEqual(Decision.parse(decision("search_103", 0)).action, "search_103")
        with self.assertRaises(ValueError):
            Decision.parse(decision("search_103", 1))
        with self.assertRaises(ValueError):
            Decision.parse(decision("search_104", 0))

    def test_review_preserves_scan_progress_and_observation_point(self):
        search, scene = self.head_wait()
        search.phase = "scan"
        search.region, search.index = "103", 1
        search.scan_angle = 60
        search.segments = []
        search.motion = Mock()
        search.motion.angle = 17
        search.motion.result.return_value = {"actual_angle_deg": 17}
        search.interrupt(scene)
        self.assertEqual(search.scan_angle, 77)
        self.assertEqual(search.phase, "checkpoint")
        search.start("search_103", scene)
        self.assertEqual(search.phase, "scan")
        self.assertEqual(search.scan_angle, 77)
        self.assertEqual(search.index, 1)

    def test_path_goes_around_inflated_furniture(self):
        plan = FloorMap([(np.array([0, 0]), np.array([0.3, 0.5]), np.eye(2))])
        start = (-1, 0)
        path = plan.path(start, (1, 0))
        self.assertGreater(len(path), 1)
        for a, b in zip([start, *path], path):
            self.assertTrue(plan.clear_line(a, b))
        self.assertGreater(
            sum(math.dist(a, b) for a, b in zip([start, *path], path)), 2
        )

    def test_closed_room_does_not_allow_wall_crossing(self):
        plan = FloorMap([(np.array([0, 0]), np.array([0.1, 2.5]), np.eye(2))])
        with self.assertRaises(ValueError):
            plan.path((-1, 0), (1, 0))

    def test_obstacle_memory_changes_free_cells(self):
        plan = FloorMap([])
        self.assertTrue(plan.free((0, 0)))
        plan.blocked.add(plan.cell((0, 0)))
        self.assertFalse(plan.free((0, 0)))


if __name__ == "__main__":
    unittest.main()
