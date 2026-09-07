"""规划失败可以重新决策，但不能通过放宽安全边界恢复。"""

import unittest
from concurrent.futures import Future
from unittest.mock import Mock

from test_navigation import Navigation
from office_search import RouteError, FloorMap
from test_motion_observation import SampleScene


class RouteRecoveryTests(unittest.TestCase):
    def setup_route(self):
        scene = SampleScene()
        submit = Mock(return_value=Future())
        nav = Navigation("寻找紫色球", submit)
        nav.history = [dict(observation=1, action="search_103", amount=0)]
        nav.observations = 1
        nav.search = Mock()
        nav.search.region = "103"
        nav.search.visited = {k: [] for k in ["open", "101", "102", "103"]}
        nav.search.map.free.return_value = True
        nav.search.map.path.return_value = []
        return nav, scene, submit

    def test_recoverable_failure_sends_new_observation_and_alternatives(self):
        nav, scene, submit = self.setup_route()
        nav.recover_route(scene, RouteError("region_unreachable", "该分区不可达"))
        self.assertEqual(nav.phase, "settling")
        self.assertEqual(nav.history[-1]["execution"]["status"], "route_blocked")
        alternatives = nav.search.last_failure["reachable_other_actions"]
        self.assertEqual(set(alternatives), {"search_open", "search_101", "search_102"})
        nav.search.context.return_value = {
            "last_route_failure": nav.search.last_failure
        }
        for _ in range(51):
            nav.tick(scene)
        self.assertEqual(nav.phase, "thinking")
        self.assertEqual(submit.call_count, 1)
        self.assertEqual(
            submit.call_args.kwargs["context"]["search_plan"]["last_route_failure"][
                "reachable_other_actions"
            ],
            alternatives,
        )

    def test_unsafe_start_stops_without_relaxing_map(self):
        nav, scene, submit = self.setup_route()
        nav.search.map.free.return_value = False
        nav.recover_route(scene, RouteError("unsafe_start", "当前位置不安全"))
        self.assertEqual(nav.phase, "failed")
        self.assertIn("当前位置", nav.reason)
        nav.search.map.path.assert_not_called()
        submit.assert_not_called()

    def test_no_alternatives_and_repeated_failure_have_finite_stop(self):
        nav, scene, _ = self.setup_route()
        nav.search.map.path.side_effect = RouteError("no_route", "不通")
        nav.recover_route(scene, RouteError("region_unreachable", "不通"))
        self.assertEqual(nav.phase, "failed")
        self.assertIn("没有其他可达", nav.reason)
        nav, scene, _ = self.setup_route()
        for _ in range(3):
            nav.recover_route(scene, RouteError("region_unreachable", "不通"))
            self.assertEqual(nav.phase, "settling")
        nav.recover_route(scene, RouteError("region_unreachable", "不通"))
        self.assertEqual(nav.phase, "failed")
        self.assertIn("连续三次", nav.reason)

    def test_start_and_goal_errors_are_distinct(self):
        plan = FloorMap([])
        for start, goal, code in [
            ((3, 0), (0, 0), "unsafe_start"),
            ((0, 0), (3, 0), "unsafe_goal"),
        ]:
            with self.assertRaises(RouteError) as error:
                plan.path(start, goal)
            self.assertEqual(error.exception.code, code)
