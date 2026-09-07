import sys
import unittest
from concurrent.futures import Future
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation import Decision, Navigation


def decision(action="forward", amount=10):
    return dict(
        action=action,
        amount=amount,
        target_visible=True,
        confidence=0.9,
        evidence="图像中央可见红色方块",
    )


class Scene:
    def __init__(self):
        self.steps = 0
        self.actions = []
        self.stops = 0
        self.x = 0.0
        self.yaw = 0.0

    def motion_pose(self):
        return self.x, 0, self.yaw

    def motion_step(self, command):
        self.actions.append(command)
        self.x += command[0] * 0.02
        self.yaw += command[2] * 0.02
        self.step()

    def stop(self):
        self.stops += 1

    def step(self):
        self.steps += 1

    def observe(self):
        return f"head frame at {self.steps}".encode()

    def action(self, name):
        self.actions.append(name)
        return "command updated"


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.scene = Scene()
        self.calls = []
        self.future = Future()

        def submit(*args):
            self.calls.append(args)
            return self.future

        self.nav = Navigation("走到红色方块前", submit)

    def thinking(self):
        for _ in range(51):
            self.nav.tick(self.scene)
        self.assertEqual(self.nav.phase, "thinking")

    def test_only_head_image_task_and_history_reach_provider(self):
        self.thinking()
        snapshot = self.nav.status()["current_snapshot"]
        image_id = snapshot["url"].rsplit("/", 1)[-1]
        self.assertEqual(self.nav.observation_images[image_id], self.calls[0][1])
        self.assertEqual(self.calls, [("走到红色方块前", b"head frame at 50", ())])
        before = self.scene.steps
        for _ in range(100):
            self.nav.tick(self.scene)
        self.assertEqual(self.scene.steps, before)

    def test_history_photos_are_for_ui_only_and_task_ids_are_unique(self):
        self.thinking()
        self.future.set_result(decision("wait", 0.2))
        self.nav.tick(self.scene)
        row = self.nav.status()["history"][0]
        self.assertEqual(row["snapshot"], self.nav.status()["current_snapshot"])
        self.assertNotIn("snapshot", self.nav.history[0])
        other = Navigation("另一个任务", lambda *args: Future())
        self.assertNotEqual(other.image_session, self.nav.image_session)

    def test_move_is_bounded_and_followed_by_fresh_observation(self):
        self.thinking()
        self.future.set_result(decision())
        self.nav.tick(self.scene)
        self.future = Future()
        for _ in range(200):
            self.nav.tick(self.scene)
            if len(self.calls) == 2:
                break
        self.assertEqual(len(self.calls), 2)
        self.assertEqual(self.nav.history[0]["execution"]["status"], "reached")
        self.assertAlmostEqual(
            self.nav.history[0]["execution"]["actual_distance_cm"], 10, delta=2.5
        )
        self.assertNotEqual(self.calls[0][1], self.calls[1][1])
        self.assertEqual(self.nav.phase, "thinking")

    def test_long_forward_requires_new_decision_at_checkpoint(self):
        self.thinking()
        self.future.set_result(decision("forward", 50))
        self.nav.tick(self.scene)
        self.assertEqual(self.nav.motion.amount, 30)
        self.future = Future()
        for _ in range(300):
            self.nav.tick(self.scene)
            if len(self.calls) == 2:
                break
        result = self.nav.history[0]["execution"]
        self.assertEqual(result["status"], "checkpoint")
        self.assertAlmostEqual(result["actual_distance_cm"], 30, delta=2.5)
        self.assertAlmostEqual(result["remaining_plan_cm"], 20, delta=2.5)
        before = self.scene.steps
        for _ in range(100):
            self.nav.tick(self.scene)
        self.assertEqual(self.scene.steps, before)
        # 新图发现目标偏离时可以转向，不能自动执行旧计划的剩余距离。
        self.future.set_result(decision("left", 20))
        self.nav.tick(self.scene)
        self.assertEqual(self.nav.motion.action, "left")

    def test_uncertain_target_shortens_forward_segment(self):
        self.thinking()
        self.future.set_result({**decision("forward", 50), "target_visible": False})
        self.nav.tick(self.scene)
        self.assertEqual(self.nav.motion.amount, 10)

    def test_forward_plan_limit(self):
        self.assertEqual(Decision.parse(decision("forward", 50)).amount, 50)
        with self.assertRaises(ValueError):
            Decision.parse(decision("forward", 51))

    def test_done_requires_two_new_visual_observations(self):
        self.thinking()
        self.future.set_result(decision("done", 0))
        self.nav.tick(self.scene)
        self.assertTrue(self.nav.active)
        self.assertEqual(self.nav.phase, "confirming")
        self.future = Future()
        for _ in range(26):
            self.nav.tick(self.scene)
        self.assertEqual(self.nav.observations, 2)
        self.future.set_result(decision("done", 0))
        self.nav.tick(self.scene)
        self.assertEqual(self.nav.phase, "completed")
        self.assertFalse(any(any(command) for command in self.scene.actions))

    def test_cancel_discards_late_response(self):
        self.future.set_running_or_notify_cancel()
        self.thinking()
        self.nav.cancel(self.scene)
        self.future.set_result(decision())
        self.nav.tick(self.scene)
        self.assertEqual(self.nav.phase, "cancelled")
        self.assertFalse(any(any(command) for command in self.scene.actions))

    def test_timeout_and_provider_error_fail_closed(self):
        self.thinking()
        self.future.set_exception(TimeoutError("provider timeout"))
        self.nav.tick(self.scene)
        self.assertEqual(self.nav.phase, "failed")
        self.assertFalse(any(any(command) for command in self.scene.actions))
        nav = Navigation(
            "find ball", lambda *args: Future(), timeout_s=1, clock=lambda: 0
        )
        nav.clock = lambda: 2
        nav.tick(self.scene)
        self.assertEqual(nav.phase, "failed")

    def test_no_false_success_at_decision_limit(self):
        self.nav.max_decisions = 1
        self.thinking()
        self.future.set_result(decision("done", 0))
        for _ in range(27):
            self.nav.tick(self.scene)
        self.assertEqual(self.nav.phase, "failed")

    def test_response_validation(self):
        for changes in (
            {"action": "exec"},
            {"amount": 100},
            {"amount": float("nan")},
            {"confidence": True},
            {"target_visible": "yes"},
            {"evidence": ""},
            {"action": "done", "amount": 0, "target_visible": False},
            {"action": "done", "amount": 0, "confidence": 0.1},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                Decision.parse({**decision(), **changes})
        with self.assertRaises(ValueError):
            Decision.parse({**decision(), "code": "arbitrary"})


if __name__ == "__main__":
    unittest.main()
