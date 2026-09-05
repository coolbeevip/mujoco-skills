import unittest
from concurrent.futures import Future
from test_navigation_skills import SkillScene
from test_navigation import decision
from navigation import Navigation


class ProximityGateTests(unittest.TestCase):
    def execute(
        self, action, distance, bearing=0, visible=True, history=None, seated=False
    ):
        scene = SkillScene()
        scene.proximity = lambda task: dict(
            source="head_rgbd",
            color="blue",
            visible=visible,
            surface_distance_cm=distance,
            bearing_deg=bearing,
            near=visible and 18 <= distance <= 28 and abs(bearing) <= 12,
        )
        if seated:
            scene.active, scene.stage = "sitstand", "seated"
        future = Future()
        future.set_result(decision(action, 0))
        nav = Navigation("走到蓝色圆柱面前鞠躬后坐下", lambda *args, **kwargs: future)
        if history:
            nav.history = history
        nav.phase = "observing"
        nav.tick(scene)
        nav.tick(scene)
        return nav, scene

    def test_far_bow_sit_done_are_replaced_by_approach(self):
        for action in ("ground_pick", "sit", "done"):
            nav, _ = self.execute(action, 80)
            self.assertEqual(nav.history[-1]["action"], "forward")
            self.assertEqual(nav.history[-1]["model_action"], action)
            self.assertEqual(nav.phase, "moving")

    def test_boundary_above_arrival_does_not_wait_forever(self):
        nav, _ = self.execute("done", 28.2)
        self.assertEqual(nav.history[-1]["action"], "forward")
        self.assertEqual(nav.motion.amount, 5)

    def test_missing_or_off_axis_depth_cannot_unlock_arrival(self):
        for visible, bearing in ((False, 0), (True, 30)):
            nav, _ = self.execute("done", 25, bearing, visible)
            self.assertEqual(nav.history[-1]["action"], "left")

    def test_near_sit_first_runs_required_bow(self):
        nav, _ = self.execute("sit", 25)
        self.assertEqual(nav.history[-1]["action"], "ground_pick")

    def test_seated_body_shift_preserves_verified_arrival(self):
        history = [
            dict(action="ground_pick", execution=dict(status="reached")),
            dict(
                action="sit",
                proximity=dict(near=True),
                execution=dict(status="reached"),
            ),
        ]
        nav, scene = self.execute("done", 32, history=history, seated=True)
        self.assertEqual(nav.history[-1]["action"], "done")
        self.assertEqual(nav.phase, "confirming")
        self.assertEqual(scene.stage, "seated")

    def test_unverified_seated_position_cannot_claim_arrival(self):
        nav, _ = self.execute("done", 80, seated=True)
        self.assertEqual(nav.history[-1]["action"], "stand")


if __name__ == "__main__":
    unittest.main()
