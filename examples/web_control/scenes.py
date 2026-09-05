"""场景适配器：声明网页按钮，并把动作交给已有 MicroDuck 控制器。"""

import struct
import math
import sys
import zlib
from pathlib import Path

MICRODUCK = Path(__file__).resolve().parents[1] / "microduck"
RENDER_WIDTHS = (640, 960, 1280, 1600, 1920)
HEAD_ACTIONS = {"head_down": 0.35, "head_up": -0.35, "head_reset": 0.0}


def navigation_scene(spec):
    """加入真实可见且可碰撞的目标物；坐标仅用于搭建场景，不提供给模型决策。"""
    import mujoco

    # 初始朝向为 +X，+Y 是机器人左侧。三个目标距起点约 0.9 m，
    # 分别位于前方、左后方和右后方；模型必须通过转身观察寻找侧后方目标。
    for name, kind, size, position, color in (
        (
            "red_box",
            mujoco.mjtGeom.mjGEOM_BOX,
            [0.09, 0.09, 0.12],
            [0.9, 0, 0.12],
            [0.9, 0.06, 0.04, 1],
        ),
        (
            "blue_cylinder",
            mujoco.mjtGeom.mjGEOM_CYLINDER,
            [0.08, 0.14, 0],
            [-0.45, 0.78, 0.14],
            [0.04, 0.18, 0.9, 1],
        ),
        (
            "yellow_ball",
            mujoco.mjtGeom.mjGEOM_SPHERE,
            [0.08, 0, 0],
            [-0.45, -0.78, 0.08],
            [0.95, 0.75, 0.03, 1],
        ),
    ):
        spec.worldbody.add_geom(
            name=name, type=kind, size=size, pos=position, rgba=color
        )


def render_size(width):
    if type(width) is not int or width not in RENDER_WIDTHS:
        raise ValueError("渲染宽度必须是 640、960、1280、1600 或 1920")
    return width, width * 9 // 16


def action(id, label, key, policy, group="skills"):
    return dict(id=id, label=label, key=key, policy=policy, group=group)


def catalog():
    scenes = []
    for id, title in [
        ("walk", "步行"),
        ("ball", "带球"),
        ("roller", "带轮"),
        ("navigation", "视觉导航"),
    ]:
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
                action("head_down", "低头", "", "head_pose"),
                action("head_up", "抬头", "", "head_pose"),
                action("head_reset", "头部回正", "", "head_pose"),
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
                    "navigation": "视觉导航 · 红色方块、蓝色圆柱、黄色球体",
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
            scene_setup=navigation_scene if scene_id == "navigation" else None,
        )
        # 固定版本模型的相机负 Z 轴朝向头壳内部，且图像上方朝身体左侧。
        # 头部局部 -Z 是脸部前方，局部 +X 是上方；绕局部 Z 转 -90° 后，
        # 相机的 +X 向右、+Y 向上、-Z 向前。只修正虚拟外参，不改资产缓存。
        self.runtime.model.camera("head_camera").quat[:] = [2**-0.5, 0, 0, -(2**-0.5)]
        mujoco.mj_forward(self.runtime.model, self.runtime.data)
        self.behaviors = Behaviors(self.runtime)
        self.renderer = None
        self.observer_renderer = None
        self.resize(1280)
        self.camera = mujoco.MjvCamera()
        self.camera.distance = 0.85
        self.camera.azimuth = 135
        self.camera.elevation = -20

    def action(self, id):
        entry = next((item for item in self.spec["actions"] if item["id"] == id), None)
        if entry is None:
            raise ValueError("当前场景不支持此动作")
        if id in HEAD_ACTIONS:
            b = self.behaviors
            seated = b.active == "sitstand" and b.stage == "seated"
            if (b.active and not seated) or b.recovery or any(b.command):
                raise ValueError("请先停止移动并等待当前动作恢复，再调整头部")
            self.runtime.set_head_pitch(HEAD_ACTIONS[id])
            return "头部目标已设置，策略将平滑调整；实际视角以摄像头画面为准"
        # 前进按钮 → w → Behaviors 保存 (0.3, 0, 0) 目标。
        # 这里只改变目标；step() 才会读取状态、推理关节目标并推进物理。
        return self.behaviors.handle(entry["key"])

    def stop(self):
        self.behaviors.command = (0, 0, 0)

    def step(self):
        self.behaviors.step()

    def navigation_context(self):
        actions = [item["id"] for item in self.spec["actions"]]
        if "sitstand" in actions:
            actions.remove("sitstand")
            actions += ["sit", "stand"]
        return dict(
            available_actions=actions,
            posture=self.behaviors.stage,
            active=self.behaviors.active,
            action_success_verified=False,
            scene_description=self.spec["description"],
            head=self.head_feedback(),
        )

    def head_feedback(self):
        r = self.runtime
        camera = r.model.camera("head_camera").id
        # MuJoCo 相机沿局部 -Z 观察；只读实际相机方向，不修改相机或关节位置。
        direction_z = -r.data.cam_xmat[camera].reshape(3, 3)[2, 2]
        return dict(
            target_offset_deg=round(math.degrees(float(r.head_target[1])), 1),
            camera_pitch_deg=round(
                math.degrees(math.asin(max(-1, min(1, direction_z)))), 1
            ),
        )

    def navigation_action(self, name):
        # 模型使用明确的坐下/站起，不使用会随当前状态反转含义的切换按钮。
        if name == "stop":
            self.stop()
            return "movement cleared"
        if name in HEAD_ACTIONS:
            return self.action(name)
        seated = (
            self.behaviors.active == "sitstand" and self.behaviors.stage == "seated"
        )
        if name == "sit" and seated:
            return "already seated"
        if (
            name == "stand"
            and not self.behaviors.active
            and not self.behaviors.recovery
        ):
            return "already standing"
        if name == "stand" and not seated:
            raise ValueError("站起需要先完成坐下动作")
        if seated and name != "stand":
            raise ValueError("当前坐姿，请先站起再执行其他动作")
        result = self.action("sitstand" if name in {"sit", "stand"} else name)
        if result.startswith("ignored:"):
            raise ValueError(result)
        return result

    def navigation_hold(self):
        # 坐姿观察必须继续坐姿策略，不能切换到步行网络而自行站起。
        if self.behaviors.active or self.behaviors.recovery:
            self.step()
        else:
            self.motion_step((0, 0, 0))

    def motion_pose(self):
        sample = self.behaviors.sample
        return sample.x, sample.y, sample.yaw

    def motion_step(self, command):
        if self.behaviors.active or self.behaviors.recovery:
            raise ValueError("当前动作尚未结束，不能执行导航运动")
        # 导航过程中保持同一个运动策略，包括停步观察阶段，避免短动作后
        # 立即切换站立网络引起姿态回摆。每个物理子步仍通过原有跌倒监测。
        policy = "roller" if self.runtime.mode == "roller" else "walking"
        if policy == "roller":
            command = (command[0], 0, max(-0.5, min(0.5, command[2])))
        self.behaviors.command = command
        self.runtime.step_policy(policy, command, observer=self.behaviors.observe)

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

    def frame(self, view="external"):
        if view not in ("external", "head"):
            raise ValueError("未知观察视角")
        self.camera.lookat[:] = self.runtime.data.xpos[self.runtime.root_id]
        self.renderer.update_scene(
            self.runtime.data, camera="head_camera" if view == "head" else self.camera
        )
        return png(self.renderer.render())

    def observe(self):
        """读取头部 RGB 图像，不读取目标坐标或外部观察画面。"""
        import mujoco

        if self.observer_renderer is None:
            self.observer_renderer = mujoco.Renderer(
                self.runtime.model, height=360, width=640
            )
        self.observer_renderer.update_scene(self.runtime.data, camera="head_camera")
        return png(self.observer_renderer.render())

    def proximity(self, task):
        from proximity import target_color, measure

        color = target_color(task)
        if self.spec["id"] != "navigation" or color is None:
            return None
        self.observe()
        renderer = self.observer_renderer
        rgb = renderer.render()
        renderer.enable_depth_rendering()
        try:
            depth = renderer.render()
        finally:
            renderer.disable_depth_rendering()
        r = self.runtime
        camera = r.model.camera("head_camera").id
        return measure(
            rgb,
            depth,
            r.model.cam_fovy[camera],
            r.data.cam_xpos[camera],
            r.data.cam_xmat[camera],
            self.motion_pose(),
            color,
        )

    def close(self):
        if self.renderer:
            self.renderer.close()
        if self.observer_renderer:
            self.observer_renderer.close()
