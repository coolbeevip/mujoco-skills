import unittest
from unittest.mock import Mock, patch
from concurrent.futures import Future

import numpy as np

from navigation import Navigation
from office_search import OfficeSearch, FloorMap, RouteError
from test_navigation import decision
from test_navigation_skills import SkillScene
from object_memory import ObjectMemory


class TargetApproachTests(unittest.TestCase):
    def search(self):
        scene = Mock()
        scene.motion_pose.return_value = (-1, 0, 0)
        scene.proximity.return_value = {"visible": False}
        scene.head_feedback.return_value = {"camera_pitch_deg": 0}
        with patch.object(FloorMap, "from_scene", return_value=FloorMap([])):
            search = OfficeSearch(scene)
        return search, scene

    def test_failed_direction_is_excluded_but_target_is_retained(self):
        search, scene = self.search()
        search.start_approach(scene, (0, 0))
        old = search.goal_xy.copy()
        search.reject_approach()
        search.start_approach(scene, (0, 0))
        self.assertGreaterEqual(np.linalg.norm(search.goal_xy - old), 0.15)
        np.testing.assert_array_equal(search.approach_target, (0, 0))

    def test_periodic_review_resumes_existing_detour_when_target_hidden(self):
        search, scene = self.search()
        search.start_approach(scene, (0, 0))
        old_path = search.path
        search.interrupt(scene)
        search.start_approach(scene, (0.01, 0))
        self.assertIs(search.path, old_path)
        self.assertEqual(search.phase, "route")
        scene.proximity.return_value = {"visible": True, "near": False}
        self.assertFalse(search.target_seen(scene, "紫色"))

    def test_four_centimeter_remainder_finishes_waypoint_without_unsafe_step(self):
        search, scene = self.search()
        search.start_approach(scene, (0, 0))
        search.settling = 0
        search.head_mode = "head_reset"
        search.path = [np.array([-0.96, 0])]
        search.tick(scene, "紫色")
        self.assertEqual(search.phase, "scan")
        self.assertIsNone(search.motion)

    def test_route_failure_preserves_target_even_without_other_regions(self):
        search, scene = self.search()
        search.start_approach(scene, (0, 0))
        search.visited = {
            k: list(range(len(v)))
            for k, v in __import__("office_search").POINTS.items()
        }
        nav = Navigation("走到紫色小球前", lambda *a: None)
        nav.search = search
        nav.target_memory = {"position": [0, 0], "observed_at_s": 1}
        nav.recover_route(scene, RouteError("insufficient_clearance", "桌腿挡路"))
        self.assertEqual(nav.phase, "settling")
        self.assertEqual(nav.target_memory["position"], [0, 0])
        self.assertEqual(len(search.rejected_approach_goals), 1)

    def test_new_task_uses_scene_memory_after_long_occlusion(self):
        scene = SkillScene()
        memory = ObjectMemory()
        self.addCleanup(memory.close)
        memory.begin_scene("office")
        memory.observe([dict(object_id="ball:purple", position=[2, -2])], 1, [0, 0, 0])
        memory.observe([], 200, [0, 0, 0])
        scene.object_memory = memory
        original = scene.navigation_context
        scene.navigation_context = lambda: {
            **original(),
            "office_search": {
                "observer_pose": {"x_m": 0, "y_m": 0},
                "current_region": "103",
            },
        }
        scene.proximity = lambda task: {"visible": False}
        future = Future()
        future.set_result({**decision("right", 30), "target_visible": False})
        submit = Mock(return_value=future)
        nav = Navigation("走到紫色小球前", submit)
        nav.phase = "observing"
        search = Mock()
        search.result.return_value = {"status": "running"}
        with patch("office_search.OfficeSearch", return_value=search):
            nav.tick(scene)
            nav.tick(scene)
        self.assertEqual(nav.history[-1]["action"], "approach_target")
        self.assertEqual(
            submit.call_args.kwargs["context"]["target_memory"]["position"], [2, -2]
        )
        search.start_approach.assert_called_once_with(scene, [2, -2])
