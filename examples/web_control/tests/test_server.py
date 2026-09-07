"""无图形依赖的接口与调度测试：假场景只替换物理，真实 HTTP 保持不变。"""

import json
import sys
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scenes import catalog, render_size
from server import Engine, ThreadingHTTPServer, handler


class FakeScene:
    def __init__(self, id, cache):
        self.id, self.steps, self.moving, self.closed = id, 0, False, False

    def state(self):
        return {
            "time_s": self.steps * 0.02,
            "policy": "walking" if self.moving else "standing",
        }

    def step(self):
        self.steps += 1

    def action(self, id):
        if id not in {
            a["id"] for s in catalog() if s["id"] == self.id for a in s["actions"]
        }:
            raise ValueError("unsupported")
        self.moving = id != "stop"
        return "command updated"

    def stop(self):
        self.moving = False

    def frame(self, view="external"):
        return b"fake frame"

    def observe(self):
        return b"head frame"

    def depth_preview(self):
        return b"depth frame"

    def motion_pose(self):
        return getattr(self, "x", 0), 0, getattr(self, "yaw", 0)

    def motion_step(self, command):
        self.x = getattr(self, "x", 0) + command[0] * 0.02
        self.yaw = getattr(self, "yaw", 0) + command[2] * 0.02
        self.moving = any(command)
        self.step()

    def view(self, *args):
        self.camera = args

    def resize(self, width):
        self.render_width = width

    def close(self):
        self.closed = True


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.engine = Engine(Path("unused"), FakeScene)

    def test_two_views_render_without_advancing_physics(self):
        self.send("load", scene="navigation")
        self.assertEqual(self.engine.frame, b"fake frame")
        self.assertEqual(self.engine.head_frame, b"head frame")
        self.assertEqual(self.engine.depth_frame, b"depth frame")
        self.assertTrue(self.engine.read()["depth_frame_available"])
        self.assertTrue(self.engine.read()["head_frame_available"])
        previous_steps = self.engine.scene.steps
        self.engine.render()
        self.assertEqual(self.engine.scene.steps, previous_steps)

    def test_failed_head_render_does_not_publish_partial_frame_pair(self):
        self.send("load", scene="walk")
        previous = self.engine.frame, self.engine.head_frame, self.engine.frame_id
        self.engine.scene.frame = lambda view: b"new external frame"

        def fail():
            raise ValueError("camera failed")

        self.engine.scene.observe = fail
        with self.assertRaises(ValueError):
            self.engine.render()
        self.assertEqual(
            (self.engine.frame, self.engine.head_frame, self.engine.frame_id), previous
        )

    def test_depth_failure_keeps_all_previous_frames(self):
        self.send("load", scene="walk")
        previous = (
            self.engine.frame,
            self.engine.head_frame,
            self.engine.depth_frame,
            self.engine.frame_id,
        )
        self.engine.scene.frame = lambda view: b"new external frame"

        def fail():
            raise ValueError("depth failed")

        self.engine.scene.depth_preview = fail
        with self.assertRaises(ValueError):
            self.engine.render()
        self.assertEqual(
            (
                self.engine.frame,
                self.engine.head_frame,
                self.engine.depth_frame,
                self.engine.frame_id,
            ),
            previous,
        )

    def send(self, op, **extra):
        future = self.engine.submit(
            dict(op=op, generation=self.engine.generation, **extra)
        )
        self.engine.tick()
        return future.result()

    def test_load_starts_paused_and_steps_only_after_resume(self):
        self.send("load", scene="walk")
        self.assertEqual(self.engine.scene.steps, 0)
        self.send("resume")
        self.assertEqual(self.engine.scene.steps, 1)
        self.send("action", action="forward")
        self.assertTrue(self.engine.scene.moving)
        self.send("pause")
        steps = self.engine.scene.steps
        self.engine.tick()
        self.assertEqual(self.engine.scene.steps, steps)
        self.assertFalse(self.engine.scene.moving)

    def test_navigation_is_explicit_and_cancelled_on_reset(self):
        from concurrent.futures import Future

        self.send("load", scene="navigation")
        with self.assertRaises(ValueError):
            self.send("task_start", task="走到红色方块前")
        self.engine.navigator_submit = lambda *args: Future()
        self.send("task_start", task="走到红色方块前")
        nav = self.engine.navigation
        self.assertTrue(nav.active)
        with self.assertRaises(ValueError):
            self.send("action", action="forward")
        self.send("reset")
        self.assertEqual(nav.phase, "cancelled")
        self.assertIsNone(self.engine.navigation)
        self.assertTrue(self.engine.paused)

    def test_invalid_state_fails_active_navigation(self):
        from concurrent.futures import Future

        self.send("load", scene="navigation")
        self.engine.navigator_submit = lambda *args: Future()
        self.send("task_start", task="走到红色方块前")
        self.engine.scene.state = lambda: {"time_s": float("nan")}
        self.engine.publish()
        self.assertTrue(self.engine.paused)
        self.assertEqual(self.engine.navigation.phase, "failed")
        self.assertFalse(self.engine.scene.moving)
        self.assertIn("状态无效", self.engine.navigation.reason)

    def test_navigation_cancelled_on_heartbeat_loss(self):
        from concurrent.futures import Future

        self.send("load", scene="navigation")
        self.engine.navigator_submit = lambda *args: Future()
        self.send("task_start", task="走到红色方块前")
        self.engine.tick(now=self.engine.last_seen + 4)
        self.assertEqual(self.engine.navigation.phase, "cancelled")
        self.assertTrue(self.engine.paused)

    def test_head_view_does_not_advance_physics(self):
        self.send("load", scene="walk")
        self.send("view", view="head")
        self.assertEqual(self.engine.view, "head")
        self.assertEqual(self.engine.scene.steps, 0)
        with self.assertRaises(ValueError):
            self.send("view", view="unknown")

    def test_resolution_validation_and_paused_refresh(self):
        self.send("load", scene="walk")
        scene = self.engine.scene
        frame_id = self.engine.frame_id
        self.send("resolution", width=1920)
        self.assertIs(scene, self.engine.scene)
        self.assertEqual(scene.render_width, 1920)
        self.assertEqual(scene.steps, 0)
        self.assertTrue(self.engine.paused)
        self.assertGreater(self.engine.frame_id, frame_id)
        for width in (True, 1920.0, 0, 3840, "1280", None):
            with self.assertRaises(ValueError):
                self.send("resolution", width=width)
        self.assertEqual(scene.render_width, 1920)
        self.assertEqual(render_size(1280), (1280, 720))

    def test_resolution_failure_preserves_pause_state(self):
        self.send("load", scene="walk")

        def fail(width):
            raise ValueError("allocation failed")

        self.engine.scene.resize = fail
        with self.assertRaises(ValueError):
            self.send("resolution", width=1920)
        self.assertTrue(self.engine.paused)
        self.assertEqual(self.engine.scene.steps, 0)

    def test_heartbeat_loss_pauses_without_auto_resume(self):
        self.send("load", scene="walk")
        self.send("resume")
        self.send("action", action="forward")
        self.engine.tick(now=self.engine.last_seen + 4)
        self.assertTrue(self.engine.paused)
        self.assertFalse(self.engine.scene.moving)
        self.engine.heartbeat()
        self.engine.tick()
        self.assertTrue(self.engine.paused)

    def test_stale_generation_cannot_control_new_scene(self):
        self.send("load", scene="walk")
        generation, old = self.engine.generation, self.engine.scene
        self.send("load", scene="roller")
        self.assertTrue(old.closed)
        future = self.engine.submit(dict(op="resume", generation=generation))
        self.engine.tick()
        with self.assertRaises(ValueError):
            future.result()
        self.assertTrue(self.engine.paused)

    def test_validation_and_reset(self):
        self.send("load", scene="roller")
        with self.assertRaises(ValueError):
            self.send("action", action="forward")
        self.send("resume")
        with self.assertRaises(ValueError):
            self.send("action", action="kick_left")
        for distance in [float("nan"), 0, 4, "1", True]:
            with self.assertRaises(ValueError):
                self.send("camera", azimuth=0, elevation=-20, distance=distance)
        self.send("camera", azimuth=30, elevation=-20, distance=1)
        self.assertEqual(self.engine.scene.camera, (30, -20, 1))
        self.send("camera", azimuth=30, elevation=-20, distance=1, pan=[1, -2, 0.5])
        self.assertEqual(self.engine.scene.camera, (30, -20, 1, [1, -2, 0.5]))
        for pan in [None, [], [1, 2], [0, 0, True], [0, 0, float("nan")], [21, 0, 0]]:
            with self.assertRaises(ValueError):
                self.send("camera", azimuth=30, elevation=-20, distance=1, pan=pan)
        self.send("reset")
        self.assertEqual(self.engine.scene.steps, 0)
        self.assertTrue(self.engine.paused)

    def test_physics_failure_latches_until_reset(self):
        self.send("load", scene="walk")

        def fail():
            raise ValueError("physics invalid")

        self.engine.scene.step = fail
        self.send("resume")
        self.assertTrue(self.engine.paused)
        self.assertIn("physics invalid", self.engine.error)
        with self.assertRaises(ValueError):
            self.send("resume")
        self.send("reset")
        self.assertEqual(self.engine.error, "")

    def test_failed_load_stops_previous_scene(self):
        self.send("load", scene="walk")
        previous = self.engine.scene

        def fail(*args):
            raise ValueError("asset missing")

        self.engine.factory = fail
        with self.assertRaises(ValueError):
            self.send("load", scene="ball")
        self.assertTrue(previous.closed)
        self.assertTrue(self.engine.paused)
        self.assertIsNone(self.engine.scene)
        self.assertFalse(self.engine.frame)
        self.assertFalse(self.engine.head_frame)
        self.assertIn("asset missing", self.engine.error)

    def test_cancelled_queued_request_does_not_execute(self):
        future = self.engine.submit(dict(op="load", scene="walk", generation=0))
        future.cancel()
        self.engine.tick()
        self.assertIsNone(self.engine.scene)

    def test_nonfinite_state_still_publishes_error(self):
        self.send("load", scene="walk")
        self.engine.scene.state = lambda: {"time_s": float("nan")}
        self.engine.publish()
        state = self.engine.read()
        self.assertTrue(state["paused"])
        self.assertIn("状态无效", state["error"])
        json.dumps(state, allow_nan=False)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.engine = Engine(Path("unused"), FakeScene)
        self.http = ThreadingHTTPServer(
            ("127.0.0.1", 0), handler(self.engine, "test-token")
        )
        self.thread = threading.Thread(target=self.http.serve_forever)
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.http.server_port}"

    def tearDown(self):
        self.http.shutdown()
        self.http.server_close()
        self.thread.join()

    def get(self, path, **kwargs):
        with urlopen(Request(self.base + path, **kwargs), timeout=5) as response:
            return response.read()

    def test_object_memory_api_is_cached_and_reset_marks_old_world_historical(self):
        self.engine.apply(dict(op="load", scene="office", generation=0))
        self.engine.object_memory.observe(
            [dict(object_id="ball:purple", name="紫色小球", position=[2, -2])],
            1,
            [0, 0, 0],
        )
        self.engine.render()
        record = json.loads(self.get("/api/object-memory"))["objects"][0]
        self.assertEqual(record["position"], [2, -2])
        self.assertFalse(record["historical"])
        self.engine.apply(dict(op="reset", generation=self.engine.generation))
        record = json.loads(self.get("/api/object-memory"))["objects"][0]
        self.assertTrue(record["historical"])
        self.assertFalse(record["position_verified_now"])
        self.assertEqual(self.engine.object_memory.current(), [])

    def test_observation_image_is_immutable_and_released_with_task(self):
        with self.engine.lock:
            self.engine.observation_images = {"task-one-1": b"exact submitted png"}
            self.engine.head_frame = b"new live frame"
        self.assertEqual(
            self.get("/api/observation-image/task-one-1"), b"exact submitted png"
        )
        self.engine.publish()
        with self.assertRaises(HTTPError) as error:
            self.get("/api/observation-image/task-one-1")
        self.assertEqual(error.exception.code, 404)

    def test_catalog_and_static_allowlist(self):
        data = json.loads(self.get("/api/catalog"))
        self.assertEqual(
            [s["id"] for s in data["scenes"]],
            ["walk", "ball", "roller", "navigation", "office"],
        )
        self.assertEqual(
            [len(s["actions"]) for s in data["scenes"]], [12, 12, 5, 12, 12]
        )
        self.assertIn(b"app.js", self.get("/"))
        for path in [
            "/server.py",
            "/../microduck/.cache",
            "/api/frame",
            "/api/head-frame",
            "/api/depth-frame",
        ]:
            with self.assertRaises(HTTPError) as caught:
                self.get(path)
            self.assertEqual(caught.exception.code, 404)

    def test_head_frame_route_returns_cached_image_and_generation(self):
        self.engine.apply(dict(op="load", scene="navigation", generation=0))
        for path, expected in [
            ("/api/frame", b"fake frame"),
            ("/api/head-frame", b"head frame"),
            ("/api/depth-frame", b"depth frame"),
        ]:
            with urlopen(self.base + path, timeout=5) as response:
                self.assertEqual(response.read(), expected)
                self.assertEqual(response.headers["Content-Type"], "image/png")
                self.assertEqual(
                    response.headers["X-Scene-Generation"], str(self.engine.generation)
                )
                self.assertEqual(
                    response.headers["X-Frame-Id"], str(self.engine.frame_id)
                )
        self.assertEqual(self.engine.scene.steps, 0)

    def test_http_command_is_executed_by_engine_queue(self):
        headers = {"Content-Type": "application/json", "X-Control-Token": "test-token"}
        with ThreadPoolExecutor(max_workers=1) as pool:
            response = pool.submit(
                self.get,
                "/api/command",
                headers=headers,
                data=json.dumps(dict(op="load", scene="ball", generation=0)).encode(),
            )
            queued = self.engine.commands.get(timeout=3)
            self.assertIsNone(self.engine.scene)
            self.engine.commands.put_nowait(queued)
            self.engine.tick()
            result = json.loads(response.result(timeout=3))
        self.assertEqual(result["state"]["scene"], "ball")
        self.assertTrue(result["state"]["paused"])

    def test_post_token_origin_host_and_json_validation(self):
        headers = {"Content-Type": "application/json", "X-Control-Token": "test-token"}
        for extra, code in [
            ({"X-Control-Token": "wrong"}, 403),
            ({"Origin": "https://example.com"}, 403),
            ({"Host": "attacker.example"}, 403),
            ({"Content-Type": "text/plain"}, 415),
        ]:
            with self.assertRaises(HTTPError) as caught:
                self.get("/api/heartbeat", data=b"{}", headers={**headers, **extra})
            self.assertEqual(caught.exception.code, code)
        self.assertTrue(
            json.loads(self.get("/api/heartbeat", data=b"{}", headers=headers))["ok"]
        )
        for body in [b"[]", b"{", b" " * 4097]:
            with self.assertRaises(HTTPError) as caught:
                self.get("/api/command", data=body, headers=headers)
            self.assertEqual(caught.exception.code, 400)


if __name__ == "__main__":
    unittest.main()
