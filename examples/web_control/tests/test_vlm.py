"""使用本机 HTTP 服务核对模型协议，不调用真实模型、不产生云端费用。"""

import base64
import json
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vlm import Config, Provider, load_env, TransientFailure
from server import Engine
from test_server import FakeScene


DECISION = dict(
    action="forward",
    amount=10,
    target_visible=True,
    confidence=0.9,
    evidence="红色方块位于画面中央",
)


class ConfigTests(unittest.TestCase):
    def test_env_file_loading_preserves_environment(self):
        path = Mock()
        path.read_text.return_value = "# local\nexport VLM_MODEL='file-model'\nVLM_API_KEY=local-key\nHOME=ignored\n"
        env = {"VLM_MODEL": "process-model"}
        load_env(path, env)
        self.assertEqual(
            env, {"VLM_MODEL": "process-model", "VLM_API_KEY": "local-key"}
        )

    def test_invalid_env_file_is_atomic_and_does_not_expose_secret(self):
        path = Mock()
        path.read_text.return_value = 'VLM_MODEL=model\nVLM_API_KEY="secret\n'
        env = {}
        with self.assertRaises(ValueError) as context:
            load_env(path, env)
        self.assertEqual(env, {})
        self.assertNotIn("secret", str(context.exception))

    def test_missing_env_file_is_optional(self):
        path = Mock()
        path.exists.return_value = False
        load_env(path, {})
        path.read_text.assert_not_called()

    def test_key_alone_does_not_enable_inference(self):
        self.assertIsNone(Config.from_env({"OPENAI_API_KEY": "secret"}))

    def test_openai_environment_configuration(self):
        config = Config.from_env(
            dict(
                OPENAI_API_KEY="secret",
                OPENAI_BASE_URL="https://example.com/v1/",
                OPENAI_MODEL="vision-model",
            )
        )
        self.assertEqual(config.provider, "openai-compatible")
        self.assertEqual(config.base_url, "https://example.com/v1")
        self.assertEqual(config.model, "vision-model")
        self.assertEqual(config.api_key, "secret")

    def test_default_openai_endpoint(self):
        config = Config.from_env(
            dict(OPENAI_API_KEY="secret", OPENAI_MODEL="vision-model")
        )
        self.assertEqual(config.base_url, "https://api.openai.com/v1")

    def test_ollama_does_not_inherit_openai_configuration(self):
        config = Config.from_env(
            dict(
                VLM_PROVIDER="ollama",
                VLM_MODEL="local-model",
                OPENAI_MODEL="remote-model",
                OPENAI_API_KEY="secret",
                OPENAI_BASE_URL="https://example.com/v1",
            )
        )
        self.assertEqual(config.base_url, "http://127.0.0.1:11434")
        self.assertEqual(config.api_key, "")

    def test_official_key_is_not_sent_to_other_provider(self):
        env = dict(
            VLM_PROVIDER="openai-compatible",
            VLM_MODEL="chosen-model",
            OPENAI_API_KEY="secret",
            VLM_BASE_URL="https://example.com/v1",
        )
        with self.assertRaises(ValueError):
            Config.from_env(env)
        env["VLM_API_KEY"] = "dedicated-key"
        config = Config.from_env(env)
        self.assertNotIn("dedicated-key", repr(config))
        self.assertNotIn("dedicated-key", json.dumps(config.public()))

    def test_remote_http_and_bad_key_rejected(self):
        env = dict(
            VLM_PROVIDER="openai-compatible",
            VLM_MODEL="chosen-model",
            VLM_BASE_URL="http://example.com/v1",
            VLM_API_KEY="secret",
        )
        with self.assertRaises(ValueError):
            Config.from_env(env)
        env["VLM_BASE_URL"] = "https://example.com/v1"
        env["VLM_API_KEY"] = "secret\ninvalid"
        with self.assertRaisesRegex(ValueError, "控制字符"):
            Config.from_env(env)


class ProtocolTests(unittest.TestCase):
    def test_transient_failure_then_success_reuses_same_observation(self):
        with patch.object(
            self.provider,
            "_infer",
            side_effect=[TransientFailure("网络超时"), DECISION],
        ) as infer:
            self.assertEqual(
                self.provider.infer("寻找蓝球", b"same-image", []), DECISION
            )
        self.assertEqual(infer.call_count, 2)
        self.assertEqual(
            infer.call_args_list[0].args[:4], infer.call_args_list[1].args[:4]
        )

    def test_http_503_retries_three_times_but_401_does_not(self):
        self.status = 503
        with self.assertRaisesRegex(ValueError, "3 次后仍失败"):
            self.provider.infer("任务", b"png", [])
        self.assertEqual(len(self.received), 3)
        self.received.clear()
        self.status = 401
        with self.assertRaisesRegex(ValueError, "HTTP 401"):
            self.provider.infer("任务", b"png", [])
        self.assertEqual(len(self.received), 1)

    def test_cancel_running_request_prevents_retry(self):
        entered, release = threading.Event(), threading.Event()

        def failing(*args):
            entered.set()
            release.wait(2)
            raise TransientFailure("网络超时")

        with patch.object(self.provider, "_infer", side_effect=failing) as infer:
            future = self.provider.submit("任务", b"png", [])
            self.assertTrue(entered.wait(2))
            future.cancel()
            release.set()
            with self.assertRaisesRegex(ValueError, "已取消"):
                future.result(timeout=2)
            self.assertEqual(infer.call_count, 1)

    def setUp(self):
        owner = self
        self.status = 200
        self.response = {
            "choices": [
                {"finish_reason": "stop", "message": {"content": json.dumps(DECISION)}}
            ]
        }
        self.received = []

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                owner.received.append(
                    (
                        self.path,
                        json.loads(
                            self.rfile.read(int(self.headers["Content-Length"]))
                        ),
                    )
                )
                self.send_response(owner.status)
                if owner.status == 302:
                    self.send_header("Location", "/leaked")
                self.end_headers()
                response = (
                    owner.response() if callable(owner.response) else owner.response
                )
                self.wfile.write(json.dumps(response).encode())

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.server.server_port}"
        self.provider = Provider(Config("openai-compatible", "test-model", self.base))

    def tearDown(self):
        self.provider.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def test_chat_image_and_decision(self):
        result = self.provider.submit("走到红色方块前", b"test-png", ()).result(
            timeout=3
        )
        self.assertEqual(result, DECISION)
        path, body = self.received[0]
        self.assertEqual(path, "/chat/completions")
        content = body["messages"][1]["content"]
        self.assertEqual(
            json.loads(content[0]["text"]),
            {"task": "走到红色方块前", "executed_history": []},
        )
        self.assertEqual(
            content[1]["image_url"]["url"],
            "data:image/png;base64," + base64.b64encode(b"test-png").decode(),
        )

    def test_ollama_image_and_schema(self):
        self.provider.close()
        self.provider = Provider(Config("ollama", "test-model", self.base))
        self.response = {"done": True, "message": {"content": json.dumps(DECISION)}}
        self.assertEqual(self.provider.infer("任务", b"png", []), DECISION)
        path, body = self.received[0]
        self.assertEqual(path, "/api/chat")
        self.assertFalse(body["stream"])
        self.assertEqual(
            body["messages"][1]["images"], [base64.b64encode(b"png").decode()]
        )
        self.assertFalse(body["format"]["additionalProperties"])

    def test_http_error_body_not_exposed(self):
        self.status, self.response = 401, {"error": "secret-key-and-request"}
        with self.assertRaises(ValueError) as context:
            self.provider.infer("任务", b"png", [])
        self.assertIn("HTTP 401", str(context.exception))
        self.assertNotIn("secret-key", str(context.exception))

    def test_redirect_not_followed(self):
        self.status = 302
        with self.assertRaisesRegex(ValueError, "重定向"):
            self.provider.infer("任务", b"png", [])
        self.assertEqual(len(self.received), 1)

    def test_truncation_and_invalid_envelope_rejected(self):
        self.response["choices"][0]["finish_reason"] = "length"
        with self.assertRaisesRegex(ValueError, "完整"):
            self.provider.infer("任务", b"png", [])
        self.assertEqual(len(self.received), 2)
        self.response = []
        with self.assertRaisesRegex(ValueError, "协议"):
            self.provider.infer("任务", b"png", [])

    def test_truncation_retries_once_with_larger_budget(self):
        self.response = lambda: {
            "choices": [
                {
                    "finish_reason": "length" if len(self.received) == 1 else "stop",
                    "message": {"content": json.dumps(DECISION)},
                }
            ]
        }
        self.assertEqual(self.provider.infer("任务", b"png", []), DECISION)
        self.assertEqual(
            [body["max_completion_tokens"] for _, body in self.received], [2048, 4096]
        )

    def test_refusal_is_not_retried(self):
        self.response["choices"][0]["message"]["refusal"] = "not allowed"
        with self.assertRaisesRegex(ValueError, "明确拒绝"):
            self.provider.infer("任务", b"png", [])
        self.assertEqual(len(self.received), 1)

    def test_exchange_logs_include_text_but_not_key_or_image(self):
        self.provider.config = Config(
            "openai-compatible", "test-model", self.base, "private-api-key"
        )
        with self.assertLogs("web_control.vlm", level="INFO") as captured:
            self.provider.infer(
                "寻找蓝色物体 private-api-key", b"private-image-bytes", []
            )
        output = "\n".join(captured.output)
        self.assertIn("寻找蓝色物体", output)
        self.assertIn("请求", output)
        self.assertIn("响应", output)
        self.assertIn("finish_reason", output)
        self.assertNotIn("private-api-key", output)
        self.assertNotIn(base64.b64encode(b"private-image-bytes").decode(), output)

    def test_engine_executes_http_decision_then_observes_again(self):
        engine = Engine(
            Path("unused"),
            FakeScene,
            self.provider.submit,
            self.provider.config.public(),
        )
        try:
            engine.apply(dict(op="load", scene="navigation", generation=0))
            engine.apply(
                dict(
                    op="task_start", task="走到红色方块前", generation=engine.generation
                )
            )
            # 假场景和固定模型响应只证明接口连通，不证明真实视觉导航成功。
            deadline = time.monotonic() + 3
            while len(engine.navigation.history) < 2 and time.monotonic() < deadline:
                engine.heartbeat()
                engine.tick()
                time.sleep(0.001)
            self.assertEqual(len(engine.navigation.history), 2)
            self.assertGreater(engine.scene.steps, 100)
            self.assertEqual(engine.navigation.phase, "moving")
            self.assertEqual(len(self.received), 2)
            second = self.received[1][1]["messages"][1]["content"][0]["text"]
            self.assertEqual(
                json.loads(second)["executed_history"][0]["action"], "forward"
            )
            self.assertTrue(engine.read()["model_config"]["configured"])
            engine.apply(dict(op="task_cancel", generation=engine.generation))
            self.assertEqual(engine.navigation.phase, "cancelled")
            self.assertFalse(engine.scene.moving)
        finally:
            engine.close()


if __name__ == "__main__":
    unittest.main()
