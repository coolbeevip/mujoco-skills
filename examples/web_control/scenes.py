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
        ("office", "办公区寻物"),
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
                    "office": "办公区 · 会议室 101/102/103、工位与五色可滚动小球",
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
        from office import office_scene, SPAWN

        self.runtime = Runtime(
            cache,
            mode="roller" if scene_id == "roller" else "walk",
            ball=scene_id == "ball",
            scene_setup=office_scene
            if scene_id == "office"
            else navigation_scene
            if scene_id == "navigation"
            else None,
            spawn_xy=SPAWN if scene_id == "office" else (0, 0),
        )
        # 固定版本模型的相机负 Z 轴朝向头壳内部，且图像上方朝身体左侧。
        # 头部局部 -Z 是脸部前方，局部 +X 是上方；绕局部 Z 转 -90° 后，
        # 相机的 +X 向右、+Y 向上、-Z 向前。只修正虚拟外参，不改资产缓存。
        self.runtime.model.camera("head_camera").quat[:] = [2**-0.5, 0, 0, -(2**-0.5)]
        mujoco.mj_forward(self.runtime.model, self.runtime.data)
        self.behaviors = Behaviors(self.runtime)
        self.renderer = None
        self.observer_renderer = None
        self.model_label = ""
        # 只选墙面和门楣，不把工位隔板、家具、门牌一起变透明。
        wall_names = {
            "west_wall",
            "east_wall",
            "north_wall",
            "south_wall",
            "meeting_divider",
            "101_left",
            "101_right",
            "102_left",
            "102_right",
            "103_south",
            "103_north",
            "103_lintel",
            "lintel_-1.5",
            "lintel_0.65",
        }
        self.preview_wall_ids = (
            {
                i
                for i in range(self.runtime.model.ngeom)
                if self.runtime.model.geom(i).name
                in {"office_" + name for name in wall_names}
            }
            if scene_id == "office"
            else set()
        )
        self.resize(1280)
        self.camera = mujoco.MjvCamera()
        self.camera_pan = [0.0, 0.0, 0.0]
        self.camera.distance = 0.85
        self.camera.azimuth = 135
        self.camera.elevation = -20
        if scene_id == "office":
            self.camera.distance = 9.0
            self.camera.elevation = -65

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
        self.remember_objects()
        actions = [item["id"] for item in self.spec["actions"]]
        if "sitstand" in actions:
            actions.remove("sitstand")
            actions += ["sit", "stand"]
        context = dict(
            available_actions=actions,
            posture=self.behaviors.stage,
            active=self.behaviors.active,
            action_success_verified=False,
            scene_description=self.spec["description"],
            head=self.head_feedback(),
        )
        if self.spec["id"] == "office":
            context["available_search_actions"] = [
                "search_open",
                "search_101",
                "search_102",
                "search_103",
            ]
            x, y, yaw = self.motion_pose()
            room = (
                "101"
                if y > 0.7 and x < 0
                else "102"
                if y > 0.7
                else "103"
                if x > 1.4
                else "开放办公区"
            )
            context["office_search"] = dict(
                current_region=room,
                observer_pose=dict(
                    x_m=round(x, 2),
                    y_m=round(y, 2),
                    heading_deg=round(math.degrees(yaw)),
                ),
                floorplan="办公室范围 x=-3..3、y=-2.5..2.5 米；101 门中心(-1.5,0.7)，102 门中心(0.65,0.7)，103 门中心(1.4,-0.8)。门宽0.8米，工位在中间；门口有实体号码。坐标仅为已知建筑平面图，不包含目标位置。",
                guidance="小球可能在房间或家具后面。原地转一圈不能搜索整间办公室；结合历史中的已观察位置分区探索，先对准门洞再进入。只把看见的区域算作观察过，不把所在房间算作已经找遍。",
            )
            context["obstacles"] = self.obstacle_clearance()
            if getattr(self, "object_memory", None) is not None:
                context["object_memory"] = self.object_memory.current()
        return context

    def remember_objects(self):
        """同时记住视野内所有颜色小球，不仅仅记录当前任务要找的颜色。"""
        memory = getattr(self, "object_memory", None)
        if memory is None or self.spec["id"] != "office":
            return
        now = float(self.runtime.data.time)
        if now - getattr(self, "memory_sample_time", -1) < 0.2:
            return
        from proximity import color_mask, measure

        rgb = self.head_rgb()
        renderer = self.observer_renderer
        renderer.enable_depth_rendering()
        try:
            depth = renderer.render()
        finally:
            renderer.disable_depth_rendering()
        r = self.runtime
        camera = r.model.camera("head_camera").id
        detections = []
        for color, label in {
            "red": "红色",
            "blue": "蓝色",
            "yellow": "黄色",
            "green": "绿色",
            "purple": "紫色",
        }.items():
            if color_mask(rgb, color).sum() < 30:
                continue
            p = measure(
                rgb,
                depth,
                r.model.cam_fovy[camera],
                r.data.cam_xpos[camera],
                r.data.cam_xmat[camera],
                self.motion_pose(),
                color,
                ball=True,
            )
            verified = p.get("visible", False)
            detections.append(
                dict(
                    object_id=f"ball:{color}",
                    name=label + ("小球" if verified else "物体候选"),
                    color=color,
                    kind="ball" if verified else "color_candidate",
                    position=p.get("estimated_position_m"),
                    source="head_rgbd" if verified else "head_rgb_color",
                )
            )
        memory.observe(detections, now, self.motion_pose())
        self.memory_sample_time = now

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

    def body_stable(self):
        """用真实基座速度辅助判定停步，不能只靠某一帧位置变化小。"""
        import numpy as np

        r = self.runtime
        adr = int(r.model.joint("trunk_base_freejoint").dofadr[0])
        velocity = r.data.qvel[adr : adr + 6]
        up = r.data.xmat[r.root_id].reshape(3, 3)[2, 2]
        return bool(
            np.isfinite(velocity).all()
            and np.linalg.norm(velocity[:3]) < 0.025
            and np.linalg.norm(velocity[3:]) < 0.25
            and up > math.cos(0.2)
        )

    def head_command_reached(self):
        import numpy as np

        return bool(
            np.max(np.abs(self.runtime.head_command - self.runtime.head_target)) < 0.001
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
            external_camera=dict(
                azimuth=self.camera.azimuth,
                elevation=self.camera.elevation,
                distance=self.camera.distance,
                pan=list(self.camera_pan),
            ),
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

    def view(self, azimuth, elevation, distance, pan=None):
        self.camera.azimuth = azimuth
        self.camera.elevation = elevation
        self.camera.distance = distance
        if pan is not None:
            self.camera_pan = list(pan)

    def set_model_label(self, name):
        # MuJoCo 的几何标签缓冲区有限；标签只进入外部画面，不进入模型的视觉输入。
        self.model_label = (
            str(name or "").encode("utf-8")[:90].decode("utf-8", errors="ignore")
        )

    def frame(self, view="external"):
        if view not in ("external", "head"):
            raise ValueError("未知观察视角")
        self.camera.lookat[:] = (
            (0, 0, 0.3)
            if self.spec["id"] == "office"
            else self.runtime.data.xpos[self.runtime.root_id]
        )
        # 每帧的默认观察中心仍可跟随机器人；用户拖动产生的偏移单独保留。
        self.camera.lookat[:] += self.camera_pan
        self.renderer.update_scene(
            self.runtime.data, camera="head_camera" if view == "head" else self.camera
        )
        if view == "external" and self.preview_wall_ids:
            import mujoco

            # 只改本帧外部渲染副本。绝不改 model.geom_rgba，因此头部 RGB、
            # 深度传感器、模型输入与物理碰撞仍使用不透明的真实墙体。
            for geom in self.renderer.scene.geoms[: self.renderer.scene.ngeom]:
                if (
                    geom.objtype == mujoco.mjtObj.mjOBJ_GEOM
                    and geom.objid in self.preview_wall_ids
                ):
                    geom.rgba[3] = 0.18
                    geom.transparent = 1
        if view == "external" and self.model_label:
            import mujoco
            import numpy as np

            scene = self.renderer.scene
            if scene.ngeom < scene.maxgeom:
                camera = self.runtime.model.camera("head_camera").id
                head_body = self.runtime.model.cam_bodyid[camera]
                position = self.runtime.data.xpos[head_body].copy()
                position[2] += 0.075
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(
                    geom,
                    mujoco.mjtGeom.mjGEOM_LABEL,
                    np.zeros(3),
                    position,
                    np.eye(3).ravel(),
                    np.array([1, 1, 1, 1], dtype=np.float32),
                )
                geom.label = self.model_label
                scene.ngeom += 1
        return png(self.renderer.render())

    def observation_sample(self, task):
        """同一帧生成原图和颜色候选，运动中不读取目标位置或模型物体 ID。"""
        from proximity import color_mask, target_color

        rgb = self.head_rgb()
        color = target_color(task)
        candidate = bool(color and color_mask(rgb, color).sum() >= 30)
        return png(rgb), candidate

    def observe(self):
        """读取头部 RGB 图像，不读取目标坐标或外部观察画面。"""
        return png(self.head_rgb())

    def head_rgb(self):
        import mujoco

        if self.observer_renderer is None:
            self.observer_renderer = mujoco.Renderer(
                self.runtime.model, height=360, width=640
            )
        self.observer_renderer.update_scene(self.runtime.data, camera="head_camera")
        return self.observer_renderer.render()

    def depth_preview(self):
        """与彩色相机同视角的深度预览；切换渲染模式不推进物理时间。"""
        from depth_preview import colorize

        renderer = self.observer_renderer
        if renderer is None:
            self.head_rgb()
            renderer = self.observer_renderer
        renderer.update_scene(self.runtime.data, camera="head_camera")
        renderer.enable_depth_rendering()
        try:
            return png(colorize(renderer.render()))
        finally:
            # 即使编码失败，也必须恢复 RGB，避免污染下一次模型观察。
            renderer.disable_depth_rendering()

    def proximity(self, task):
        from proximity import target_color, measure

        color = target_color(task)
        if self.spec["id"] not in {"navigation", "office"} or color is None:
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
            ball=self.spec["id"] == "office",
        )

    def obstacle_clearance(self):
        if self.spec["id"] != "office":
            return None
        from proximity import clearance

        self.observe()
        renderer = self.observer_renderer
        renderer.enable_depth_rendering()
        try:
            depth = renderer.render()
        finally:
            renderer.disable_depth_rendering()
        r = self.runtime
        camera = r.model.camera("head_camera").id
        return clearance(
            depth,
            r.model.cam_fovy[camera],
            r.data.cam_xpos[camera],
            r.data.cam_xmat[camera],
            self.motion_pose(),
        )

    def close(self):
        if self.renderer:
            self.renderer.close()
        if self.observer_renderer:
            self.observer_renderer.close()
