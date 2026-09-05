"""场景适配器：声明网页按钮，并把动作交给已有 MicroDuck 控制器。"""

import struct
import sys
import zlib
from pathlib import Path

MICRODUCK = Path(__file__).resolve().parents[1] / "microduck"
RENDER_WIDTHS = (640, 960, 1280, 1600, 1920)


def render_size(width):
    if type(width) is not int or width not in RENDER_WIDTHS:
        raise ValueError("渲染宽度必须是 640、960、1280、1600 或 1920")
    return width, width * 9 // 16


def action(id, label, key, policy, group="skills"):
    return dict(id=id, label=label, key=key, policy=policy, group=group)


def catalog():
    scenes = []
    for id, title in [("walk", "步行"), ("ball", "带球"), ("roller", "带轮")]:
        roller = id == "roller"
        actions = [
            action(
                "forward", "前进", "w", "roller" if roller else "walking", "movement"
            ),
            action("left", "左转", "a", "roller" if roller else "walking", "movement"),
            action("right", "右转", "d", "roller" if roller else "walking", "movement"),
            action(
                "stop", "停止运动", "s", "roller" if roller else "standing", "movement"
            ),
        ]
        actions += (
            [action("crouch", "蹲伏", "c", "crouch")]
            if roller
            else [
                action("sitstand", "坐下 / 站起", "y", "sitstand"),
                action("ground_pick", "俯身", "g", "ground_pick"),
                action("kick_left", "左脚踢球", "k", "kick_left"),
                action("kick_right", "右脚踢球", "l", "kick_right"),
                action("roulade", "单次翻滚", "r", "roulade"),
            ]
        )
        scenes.append(
            dict(
                id=id,
                title=f"MicroDuck · {title}",
                actions=actions,
                description={
                    "walk": "步行模型 · 不含球，踢球按钮仅执行踢腿",
                    "ball": "带球模型 · 踢球前自动将球放到对应脚前",
                    "roller": "带轮模型 · 转向输入为相对航向误差",
                }[id],
            )
        )
    return scenes


def png(rgb):
    """RGB → PNG；只用标准库编码，浏览器直接显示，不增加图像依赖。"""
    height, width, _ = rgb.shape

    def chunk(kind, data):
        return (
            struct.pack(">I", len(data))
            + kind
            + data
            + struct.pack(">I", zlib.crc32(kind + data))
        )

    rows = b"".join(b"\0" + row.tobytes() for row in rgb)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows, 1))
        + chunk(b"IEND", b"")
    )


class MicroduckScene:
    def __init__(self, scene_id, cache):
        self.spec = next(item for item in catalog() if item["id"] == scene_id)
        # 旧样例是可直接运行的脚本，使用同级模块导入；只在适配器内接入它。
        if str(MICRODUCK) not in sys.path:
            sys.path.insert(0, str(MICRODUCK))
        import mujoco
        from behaviors import Behaviors
        from runtime import Runtime

        self.runtime = Runtime(
            cache,
            mode="roller" if scene_id == "roller" else "walk",
            ball=scene_id == "ball",
        )
        self.behaviors = Behaviors(self.runtime)
        self.renderer = None
        self.resize(1280)
        self.camera = mujoco.MjvCamera()
        self.camera.distance = 0.85
        self.camera.azimuth = 135
        self.camera.elevation = -20

    def action(self, id):
        entry = next((item for item in self.spec["actions"] if item["id"] == id), None)
        if entry is None:
            raise ValueError("当前场景不支持此动作")
        # 前进按钮 → w → Behaviors 保存 (0.3, 0, 0) 目标。
        # 这里只改变目标；step() 才会读取状态、推理关节目标并推进物理。
        return self.behaviors.handle(entry["key"])

    def stop(self):
        self.behaviors.command = (0, 0, 0)

    def step(self):
        self.behaviors.step()

    def state(self):
        r = self.runtime
        return dict(
            time_s=float(r.data.time),
            policy=r.active_policy,
            position_m=r.data.xpos[r.root_id].tolist(),
            render_width=self.renderer.width,
            render_height=self.renderer.height,
            **self.behaviors.status(),
        )

    def resize(self, width):
        import mujoco

        width, height = render_size(width)
        if self.renderer and self.renderer.width == width:
            return
        # CSS 大小决定图片显示多大，离屏缓冲区决定真正有多少细节。
        # 只重建渲染器：保留相机、策略记忆、关节状态与仿真时间。
        model = self.runtime.model
        old_size = (model.vis.global_.offwidth, model.vis.global_.offheight)
        model.vis.global_.offwidth, model.vis.global_.offheight = width, height
        try:
            renderer = mujoco.Renderer(model, height=height, width=width)
        except Exception:
            model.vis.global_.offwidth, model.vis.global_.offheight = old_size
            raise
        previous, self.renderer = self.renderer, renderer
        if previous:
            previous.close()

    def view(self, azimuth, elevation, distance):
        self.camera.azimuth = azimuth
        self.camera.elevation = elevation
        self.camera.distance = distance

    def frame(self):
        self.camera.lookat[:] = self.runtime.data.xpos[self.runtime.root_id]
        self.renderer.update_scene(self.runtime.data, camera=self.camera)
        return png(self.renderer.render())

    def close(self):
        if self.renderer:
            self.renderer.close()
