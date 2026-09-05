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

    def frame(self):
        return b"fake frame"

    def view(self, *args):
        self.camera = args

    def resize(self, width):
        self.render_width = width

    def close(self):
        self.closed = True


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.engine = Engine(Path("unused"), FakeScene)

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

    def test_catalog_and_static_allowlist(self):
        data = json.loads(self.get("/api/catalog"))
        self.assertEqual([s["id"] for s in data["scenes"]], ["walk", "ball", "roller"])
        self.assertEqual([len(s["actions"]) for s in data["scenes"]], [9, 9, 5])
        self.assertIn(b"app.js", self.get("/"))
        for path in ["/server.py", "/../microduck/.cache", "/api/frame"]:
            with self.assertRaises(HTTPError) as caught:
                self.get(path)
            self.assertEqual(caught.exception.code, 404)

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
