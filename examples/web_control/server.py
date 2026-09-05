"""本机网页服务；HTTP 线程只收发数据，主线程独占策略、物理与 OpenGL。"""

import argparse
import json
import math
import queue
import secrets
import threading
import time
from concurrent.futures import Future, TimeoutError
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from scenes import MICRODUCK, MicroduckScene, catalog, render_size

ROOT = Path(__file__).resolve().parent


class Engine:
    def __init__(self, cache, factory=MicroduckScene):
        self.cache, self.factory = cache, factory
        self.commands = queue.Queue(maxsize=32)
        self.lock = threading.Lock()
        self.scene = None
        self.scene_id = None
        self.generation = 0
        self.paused = True
        self.error = ""
        self.message = "请选择并加载场景"
        self.last_seen = time.monotonic()
        self.frame = b""
        self.frame_id = 0
        self.snapshot = {}
        self.publish()

    def submit(self, request):
        future = Future()
        self.commands.put_nowait((request, future))
        return future

    def heartbeat(self):
        with self.lock:
            self.last_seen = time.monotonic()

    def publish(self):
        try:
            state = self.scene.state() if self.scene else {}
            json.dumps(state, allow_nan=False)
        except Exception as error:
            # 物理数值已损坏时仍要能返回故障状态，不能让 NaN 破坏整个状态接口。
            self.paused = True
            self.error = f"状态无效，请重置：{error}"
            state = {}
        with self.lock:
            self.snapshot = dict(
                scene=self.scene_id,
                generation=self.generation,
                paused=self.paused,
                error=self.error,
                message=self.message,
                frame_id=self.frame_id,
                **state,
            )

    def read(self):
        with self.lock:
            return dict(self.snapshot)

    def render(self):
        frame = self.scene.frame()
        with self.lock:
            self.frame = frame
            self.frame_id += 1

    def apply(self, request):
        op = request.get("op")
        if op not in {
            "load",
            "reset",
            "pause",
            "resume",
            "action",
            "camera",
            "resolution",
        }:
            raise ValueError("未知操作")
        # 每次重建场景都更换代号。旧页面尚未返回的请求不能控制新机器人。
        if request.get("generation") != self.generation:
            raise ValueError("场景已变化，请等待页面刷新后重试")
        if op in {"load", "reset"}:
            id = request.get("scene") if op == "load" else self.scene_id
            if id not in {item["id"] for item in catalog()}:
                raise ValueError("未知场景")
            self.paused = True
            # 先停旧场景；即使新资产损坏，旧机器人也不会继续运动。
            if self.scene:
                self.scene.close()
            self.scene, self.scene_id = None, None
            with self.lock:
                self.frame = b""
            self.generation += 1
            try:
                self.scene = self.factory(id, self.cache)
                self.scene_id = id
                self.render()
            except Exception as error:
                self.error = f"加载失败：{error}"
                raise ValueError(self.error) from error
            self.error = ""
            self.message = "场景已加载并暂停，点击继续仿真开始"
            return self.message
        if not self.scene:
            raise ValueError("请先加载场景")
        if op == "resolution":
            width, height = render_size(request.get("width"))
            self.scene.resize(width)
            try:
                self.render()
            except Exception as error:
                self.paused = True
                self.scene.stop()
                self.error = f"渲染失败，请重置：{error}"
                raise ValueError(self.error) from error
            return f"实际渲染分辨率已更新为 {width} × {height}"
        if op == "pause":
            self.paused = True
            self.scene.stop()
            self.message = "已暂停物理时间并清除移动目标；进行中的动作保留进度"
        elif op == "resume":
            if self.error:
                raise ValueError("仿真异常，请重置场景")
            self.paused = False
            self.heartbeat()
            self.message = "仿真运行中"
        elif op == "action":
            if self.error or self.paused:
                raise ValueError("请先恢复仿真；异常状态需要重置")
            self.message = self.scene.action(request.get("action"))
        elif op == "camera":
            values = [request.get(k) for k in ("azimuth", "elevation", "distance")]
            if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
                raise ValueError("相机参数必须是有限数值")
            azimuth, elevation, distance = values
            if not (
                -360 <= azimuth <= 360
                and -85 <= elevation <= -5
                and 0.25 <= distance <= 3
            ):
                raise ValueError("相机参数超出范围")
            self.scene.view(*values)
            self.render()
            return "视角已更新"
        return self.message

    def tick(self, now=None):
        # 每个主循环最多处理 8 个请求，避免请求洪流挤占全部物理时间。
        for _ in range(8):
            try:
                request, future = self.commands.get_nowait()
            except queue.Empty:
                break
            if not future.set_running_or_notify_cancel():
                continue
            try:
                result = self.apply(request)
                self.publish()
                future.set_result(dict(message=result, state=self.read()))
            except Exception as error:
                self.publish()
                future.set_exception(error)
        now = time.monotonic() if now is None else now
        with self.lock:
            stale = now - self.last_seen > 3
        if self.scene and not self.paused:
            if stale:
                self.paused = True
                self.scene.stop()
                self.message = "网页心跳中断，已自动暂停；请手动继续"
            else:
                try:
                    self.scene.step()
                except Exception as error:
                    self.paused = True
                    self.scene.stop()
                    self.error = f"仿真已停止：{error}"
        self.publish()

    def close(self):
        if self.scene:
            self.scene.close()


def handler(engine, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, status, body, kind="application/json"):
            if kind == "application/json":
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; img-src 'self' blob: data:; frame-ancestors 'none'",
            )
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def local(self):
            port = self.server.server_port
            return self.headers.get("Host") in {
                f"127.0.0.1:{port}",
                f"localhost:{port}",
            }

        def do_GET(self):
            if not self.local():
                return self.reply(403, {"error": "只允许本机访问"})
            path = self.path.split("?", 1)[0]
            if path == "/api/catalog":
                return self.reply(200, {"scenes": catalog(), "token": token})
            if path == "/api/state":
                return self.reply(200, engine.read())
            if path == "/api/frame":
                with engine.lock:
                    frame = engine.frame
                return (
                    self.reply(200, frame, "image/png")
                    if frame
                    else self.reply(404, {"error": "尚无画面"})
                )
            # 显式白名单：不能通过静态服务读取策略、缓存或任意磁盘文件。
            static = {
                "/": ("index.html", "text/html; charset=utf-8"),
                "/styles.css": ("styles.css", "text/css"),
                "/app.js": ("app.js", "text/javascript"),
                "/display.js": ("display.js", "text/javascript"),
            }
            if path not in static:
                return self.reply(404, {"error": "未找到资源"})
            file, kind = static[path]
            self.reply(200, (ROOT / file).read_bytes(), kind)

        def do_POST(self):
            # 随机令牌 + JSON 请求 + Host/Origin 检查，避免其他网站驱动本机仿真。
            origin = self.headers.get("Origin")
            if (
                not self.local()
                or self.headers.get("X-Control-Token") != token
                or (origin and origin != f"http://{self.headers.get('Host')}")
            ):
                return self.reply(403, {"error": "请求来源或控制令牌无效"})
            if self.headers.get("Content-Type") != "application/json":
                return self.reply(415, {"error": "需要 application/json"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("请求大小无效")
                self.connection.settimeout(5)
                request = json.loads(self.rfile.read(length))
                if not isinstance(request, dict):
                    raise ValueError("请求必须是对象")
                if self.path == "/api/heartbeat":
                    engine.heartbeat()
                    return self.reply(200, {"ok": True})
                if self.path != "/api/command":
                    return self.reply(404, {"error": "未知接口"})
                future = engine.submit(request)
                try:
                    result = future.result(timeout=30)
                except TimeoutError:
                    future.cancel()
                    return self.reply(
                        503,
                        {"error": "请求超时，请查看当前状态；已开始的加载可能仍在完成"},
                    )
                return self.reply(200, result)
            except queue.Full:
                return self.reply(503, {"error": "请求繁忙，请稍后重试"})
            except (ValueError, TypeError) as error:
                return self.reply(400, {"error": str(error)})
            except Exception as error:
                return self.reply(500, {"error": str(error)})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--cache", type=Path, default=MICRODUCK / ".cache")
    args = parser.parse_args()
    engine = Engine(args.cache)
    server = ThreadingHTTPServer(
        ("127.0.0.1", args.port), handler(engine, secrets.token_urlsafe(32))
    )
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"仿真工作台 http://127.0.0.1:{server.server_port}/", flush=True)
    next_frame = 0.0
    try:
        while True:
            start = time.monotonic()
            engine.tick(start)
            if (
                engine.scene
                and not engine.paused
                and not engine.error
                and start >= next_frame
            ):
                try:
                    engine.render()
                except Exception as error:
                    engine.paused = True
                    engine.scene.stop()
                    engine.error = f"渲染失败，请重置：{error}"
                engine.publish()
                next_frame = time.monotonic() + 0.05
            # 不补跑积压时间：机器较慢时仿真随之变慢，动作仍按固定物理步推进。
            time.sleep(max(0, 0.02 - (time.monotonic() - start)))
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        engine.close()


if __name__ == "__main__":
    main()
