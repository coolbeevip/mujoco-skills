"""短暂出现、停止后消失的候选必须留下原帧；不调用真实模型。"""

import base64
import unittest
from concurrent.futures import Future
from unittest.mock import Mock

from test_navigation import Scene, Navigation, decision
from motion import Motion
from vlm import Config, Provider


class SampleScene(Scene):
    def observation_sample(self, task):
        return self.observe(), 55 <= self.steps < 65

    def state(self):
        return {"time_s": self.steps * 0.02}


class MotionObservationTests(unittest.TestCase):
    def test_transient_target_during_turn_preserves_both_images(self):
        scene, calls = SampleScene(), []

        def submit(task, picture, history, **context):
            calls.append((picture, context))
            future = Future()
            if len(calls) == 1:
                future.set_result(decision("left", 60))
            return future

        nav = Navigation("寻找紫色球", submit)
        for _ in range(400):
            nav.tick(scene)
            if len(calls) == 2:
                break
        self.assertEqual(len(calls), 2)
        trigger = calls[1][1]["context"]["trigger_image"]
        self.assertNotEqual(calls[1][0], trigger)
        self.assertLess(nav.history[0]["execution"]["actual_angle_deg"], 60)
        self.assertEqual(nav.history[0]["execution"]["trigger"], "target_candidate")
        snapshot = nav.status()["current_snapshot"]
        image_id = snapshot["trigger"]["url"].rsplit("/", 1)[-1]
        self.assertEqual(nav.observation_images[image_id], trigger)
        self.assertFalse(scene.observation_sample(nav.task)[1])
        nav.cancel(scene)

    def test_existing_candidate_does_not_interrupt_and_buffer_is_bounded(self):
        scene = SampleScene()
        scene.observation_sample = lambda task: (b"image", True)
        nav = Navigation("紫色球", Mock())
        nav.phase = "moving"
        nav.candidate_visible = True
        nav.motion = Motion("left", 60, scene.motion_pose())
        for _ in range(1000):
            self.assertFalse(nav.monitor_motion(scene))
        self.assertEqual(len(nav.motion_frames), 15)

    def test_periodic_search_review_without_visible_candidate(self):
        scene = SampleScene()
        nav = Navigation("紫色球", Mock())
        nav.phase = "searching"
        nav.search = Mock()
        nav.search.result.return_value = {"search_phase": "checkpoint"}
        nav.history = [{}]
        for _ in range(749):
            self.assertFalse(nav.monitor_motion(scene))
        self.assertTrue(nav.monitor_motion(scene))
        nav.search.interrupt.assert_called_once_with(scene)
        self.assertEqual(nav.trigger_frame["event"], "periodic_review")
        self.assertEqual(nav.phase, "settling")

    def test_edge_flicker_does_not_repeatedly_interrupt(self):
        scene = SampleScene()
        nav = Navigation("紫色球", Mock())
        nav.phase = "moving"
        nav.candidate_visible = True
        scene.observation_sample = lambda task: (b"edge", nav.monitor_steps % 20 == 0)
        for _ in range(100):
            self.assertFalse(nav.monitor_motion(scene))
        self.assertTrue(nav.candidate_visible)
        scene.observation_sample = lambda task: (b"empty", False)
        for _ in range(50):
            self.assertFalse(nav.monitor_motion(scene))
        self.assertFalse(nav.candidate_visible)

    def test_both_protocols_send_current_then_trigger_without_binary_in_text(self):
        context = {
            "trigger_image": b"trigger",
            "motion_observation": {"event": "target_candidate"},
        }
        for protocol in ["openai", "ollama"]:
            provider = Provider(Config(protocol, "test-model", "http://127.0.0.1:9999"))
            _, body = provider.payload("紫色球", b"current", [], context)
            user = body["messages"][-1]
            if protocol == "ollama":
                images = user["images"]
            else:
                images = [
                    p["image_url"]["url"].split(",", 1)[1]
                    for p in user["content"]
                    if p["type"] == "image_url"
                ]
            self.assertEqual(
                [base64.b64decode(p) for p in images], [b"current", b"trigger"]
            )
        self.assertEqual(context["trigger_image"], b"trigger")
