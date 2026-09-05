"""原子动作调度测试：状态窗口结束不等于物理任务成功。"""

import unittest
from unittest.mock import Mock
from concurrent.futures import Future

from test_navigation import Scene, decision
from navigation import Navigation, Decision, SKILLS, HEAD_ACTIONS
from scenes import MicroduckScene


class SkillScene(Scene):
    def __init__(self):
        super().__init__()
        self.active = None
        self.stage = "idle"
        self.count = 0

    def navigation_context(self):
        return dict(available_actions=sorted(SKILLS), posture=self.stage)

    def navigation_action(self, action):
        self.active = action
        self.stage = "executing"
        self.count = 0

    def step(self):
        super().step()
        if self.active:
            self.count += 1
            if self.count >= 4:
                if self.active == "sit":
                    self.active, self.stage = "sitstand", "seated"
                elif self.active != "sitstand":
                    self.active, self.stage = None, "idle"

    def state(self):
        return dict(active=self.active, stage=self.stage, recovery_control_steps=0)

    def navigation_hold(self):
        self.step()


class SkillTests(unittest.TestCase):
    def test_head_adjustment_waits_for_fresh_image_and_reports_camera_pitch(self):
        for action in HEAD_ACTIONS:
            scene = SkillScene()
            scene.navigation_action = Mock()
            scene.head_feedback = lambda: dict(
                camera_pitch_deg=-25.0, target_offset_deg=20.1
            )
            calls = []

            def submit(*args, **kwargs):
                calls.append(args)
                future = Future()
                if len(calls) == 1:
                    future.set_result(decision(action, 0))
                return future

            nav = Navigation("调整头部", submit)
            for _ in range(202):
                nav.tick(scene)
            self.assertEqual(nav.phase, "observing")
            self.assertEqual(len(calls), 1)
            nav.tick(scene)
            self.assertEqual(len(calls), 2)
            self.assertEqual(nav.history[0]["execution"]["camera_pitch_deg"], -25)
            scene.navigation_action.assert_called_once_with(action)

    def test_explicit_postures_do_not_toggle_unintentionally(self):
        scene = MicroduckScene.__new__(MicroduckScene)
        scene.behaviors = Mock(active="sitstand", stage="seated", recovery=0)
        scene.action = Mock(return_value="standing up")
        self.assertEqual(scene.navigation_action("sit"), "already seated")
        scene.action.assert_not_called()
        with self.assertRaisesRegex(ValueError, "先站起"):
            scene.navigation_action("kick_left")
        scene.navigation_action("stand")
        scene.action.assert_called_once_with("sitstand")

    def test_unsupported_scene_action_is_rejected_before_execution(self):
        scene = SkillScene()
        scene.navigation_context = lambda: dict(available_actions=["crouch"])
        future = Future()
        future.set_result(decision("sit", 0))
        nav = Navigation("坐下", lambda *args, **kwargs: future)
        for _ in range(52):
            nav.tick(scene)
        self.assertEqual(nav.phase, "failed")
        self.assertIn("不支持", nav.reason)
        self.assertIsNone(scene.active)

    def test_all_actions_validate_with_zero_amount_only(self):
        for action in SKILLS:
            self.assertEqual(Decision.parse(decision(action, 0)).action, action)
            with self.assertRaises(ValueError):
                Decision.parse(decision(action, 1))

    def test_each_skill_waits_then_reports_execution_and_new_image(self):
        for action in SKILLS - {"stop"} - HEAD_ACTIONS:
            with self.subTest(action=action):
                scene, calls = SkillScene(), []

                def submit(*args, **kwargs):
                    calls.append((args, kwargs))
                    future = Future()
                    if len(calls) == 1:
                        future.set_result(decision(action, 0))
                    return future

                nav = Navigation("执行动作", submit)
                for _ in range(60):
                    nav.tick(scene)
                self.assertEqual(nav.phase, "thinking")
                self.assertEqual(len(calls), 2)
                self.assertEqual(nav.history[0]["execution"]["status"], "reached")
                self.assertFalse(nav.history[0]["execution"]["action_success_verified"])
                self.assertIn("context", calls[0][1])
                self.assertNotEqual(calls[0][0][1], calls[1][0][1])
                if action == "sit":
                    self.assertEqual(scene.stage, "seated")

    def test_cancel_during_skill_marks_record_cancelled(self):
        scene = SkillScene()
        future = Future()
        future.set_result(decision("roulade", 0))
        nav = Navigation("翻滚", lambda *args, **kwargs: future)
        for _ in range(52):
            nav.tick(scene)
        self.assertEqual(nav.phase, "skill")
        nav.cancel(scene)
        self.assertEqual(nav.history[-1]["execution"]["status"], "cancelled")
        before = scene.steps
        nav.tick(scene)
        self.assertEqual(scene.steps, before)


if __name__ == "__main__":
    unittest.main()
