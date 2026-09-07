import unittest
from unittest.mock import Mock, patch
from concurrent.futures import Future

from navigation import Navigation
from test_navigation import decision
from test_navigation_skills import SkillScene


class OfficeNavigationTests(unittest.TestCase):
    def navigate(self, distance):
        scene = SkillScene()
        original = scene.navigation_context
        scene.navigation_context = lambda: {
            **original(),
            "obstacles": {"front_clearance_cm": distance},
            "office_search": {
                "observer_pose": {"x_m": 0, "y_m": 0, "heading_deg": 0},
                "current_region": "开放办公区",
            },
        }
        future = Future()
        future.set_result(decision("forward", 30))
        nav = Navigation("寻找紫色小球", lambda *a, **k: future)
        nav.phase = "observing"
        nav.tick(scene)
        nav.tick(scene)
        return nav

    def test_near_obstacle_blocks_forward(self):
        nav = self.navigate(20)
        self.assertEqual(nav.history[-1]["action"], "wait")
        self.assertEqual(nav.history[-1]["model_action"], "forward")
        self.assertEqual(nav.phase, "waiting")

    def test_depth_shortens_segment(self):
        nav = self.navigate(35)
        self.assertEqual(nav.history[-1]["amount"], 17)
        self.assertLessEqual(nav.motion.amount, 17)

    def test_observation_position_is_retained(self):
        nav = self.navigate(None)
        self.assertEqual(nav.history[-1]["region"], "开放办公区")
        self.assertEqual(nav.history[-1]["observed_from"]["heading_deg"], 0)

    def test_obstacle_appearing_mid_motion_stops_before_next_step(self):
        from test_navigation import Scene
        from motion import Motion

        scene = Scene()
        scene.obstacle_clearance = lambda: {"front_clearance_cm": 15}
        nav = Navigation("走到紫色小球前", lambda *a: None)
        nav.phase = "moving"
        nav.motion = Motion("forward", 30, scene.motion_pose())
        nav.history = [decision("forward", 30)]
        nav.tick(scene)
        self.assertEqual(scene.steps, 0)
        self.assertEqual(nav.phase, "settling")
        self.assertEqual(nav.history[-1]["execution"]["status"], "obstacle_stop")

    def test_cancel_high_level_search_prevents_more_route_steps(self):
        from test_navigation import Scene

        class Search:
            def result(self):
                return {"status": "running", "search_phase": "route"}

        scene = Scene()
        nav = Navigation("找到紫色小球", lambda *a: None)
        nav.phase, nav.search = "searching", Search()
        nav.history = [decision("search_103", 0)]
        nav.cancel(scene)
        nav.tick(scene)
        self.assertEqual(scene.steps, 0)
        self.assertEqual(nav.history[-1]["execution"]["status"], "cancelled")

    def test_repeated_turns_switch_to_structured_search(self):
        scene = SkillScene()
        original = scene.navigation_context
        scene.navigation_context = lambda: {
            **original(),
            "office_search": {
                "observer_pose": {"x_m": 0, "y_m": 0, "heading_deg": 0},
                "current_region": "开放办公区",
            },
        }
        future = Future()
        future.set_result({**decision("left", 30), "target_visible": False})
        nav = Navigation("寻找紫色小球", lambda *a, **k: future)
        nav.history = [
            {**decision(a, 30), "observed_from": {"x_m": 0, "y_m": 0}}
            for a in ("left", "right", "left", "right")
        ]
        search = Mock()
        search.result.return_value = {"status": "running"}
        nav.phase = "observing"
        with patch("office_search.OfficeSearch", return_value=search):
            nav.tick(scene)
            nav.tick(scene)
        self.assertEqual(nav.phase, "searching")
        self.assertEqual(nav.history[-1]["action"], "search_open")
        search.start.assert_called_once_with("search_open", scene)


if __name__ == "__main__":
    unittest.main()
