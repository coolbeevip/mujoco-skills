"""固定办公室布局。几何坐标只负责搭建场景，不作为寻球答案提供给模型。"""

import math
from pathlib import Path

SPAWN = (-1.2, -1.8)
BALL_RADIUS = 0.055
BALLS = {
    "red": ((0.05, -2.05), (0.9, 0.04, 0.03, 1)),
    "blue": ((-2.55, -2.05), (0.03, 0.12, 0.95, 1)),
    "yellow": ((-2.6, 0.05), (0.95, 0.8, 0.02, 1)),
    "green": ((-2.25, 2.05), (0.04, 0.75, 0.12, 1)),
    # 103 进门后右转、靠里侧的空地；沿隔墙内侧接近，避开会议桌腿。
    # 西侧隔墙仍将它挡在初始视野之外，需要进入房间后再观察。
    "purple": ((1.70, -1.65), (0.65, 0.04, 0.8, 1)),
}


def office_scene(spec):
    import mujoco

    wall = (0.83, 0.84, 0.81, 1)
    wood = (0.55, 0.38, 0.23, 1)
    metal = (0.15, 0.18, 0.19, 1)
    fabric = (0.27, 0.31, 0.32, 1)
    white = (0.93, 0.94, 0.91, 1)
    floor = next(g for g in spec.geoms if g.name == "floor")
    floor.material = ""
    floor.rgba = (0.42, 0.44, 0.43, 1)

    def box(name, pos, size, color, parent=None, solid=True):
        return (parent or spec.worldbody).add_geom(
            name="office_" + name,
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=pos,
            size=size,
            rgba=color,
            contype=1 if solid else 0,
            conaffinity=1 if solid else 0,
            friction=[0.8, 0.01, 0.001],
        )

    def panel(name, x, y, sx, sy):
        box(name, (x, y, 1.2), (sx, sy, 1.2), wall)
        box(name + "_skirting", (x, y, 0.055), (sx + 0.006, sy + 0.006, 0.055), metal)

    # 低对比地毯拼接缝提供尺度感，不增加地面碰撞面。
    for ix in range(12):
        for iy in range(10):
            shade = 0.40 + 0.018 * ((ix + iy) % 2)
            box(
                f"carpet_{ix}_{iy}",
                (-2.75 + ix * 0.5, -2.25 + iy * 0.5, 0.001),
                (0.249, 0.249, 0.001),
                (shade, shade + 0.025, shade + 0.02, 1),
                solid=False,
            )

    # 6×5 m 外墙；不建天花板，外部观察相机能俯看布局，头部视角仍被墙体遮挡。
    panel("west_wall", -3, 0, 0.04, 2.5)
    panel("east_wall", 3, 0, 0.04, 2.5)
    panel("north_wall", 0, 2.5, 3, 0.04)
    panel("south_wall", 0, -2.5, 3, 0.04)
    panel("meeting_divider", 0, 1.6, 0.04, 0.9)
    # 北侧 101/102 门洞各 0.8 m，门保持开启，不设置隐形碰撞面。
    for name, lo, hi in (
        ("101_left", -3, -1.9),
        ("101_right", -1.1, 0),
        ("102_left", 0, 0.25),
        ("102_right", 1.05, 3),
    ):
        panel(name, (lo + hi) / 2, 0.7, (hi - lo) / 2, 0.04)
    for x in (-1.5, 0.65):
        box("lintel_" + str(x), (x, 0.7, 2.25), (0.4, 0.04, 0.15), wall)
        for dx in (-0.41, 0.41):
            box(
                "jamb_" + str(x) + str(dx),
                (x + dx, 0.7, 1.05),
                (0.025, 0.055, 1.05),
                wood,
            )
    # 103 位于东南角，入口朝向开放办公区。
    panel("103_south", 1.4, -1.85, 0.04, 0.65)
    panel("103_north", 1.4, 0.15, 0.04, 0.55)
    box("103_lintel", (1.4, -0.8, 2.25), (0.04, 0.4, 0.15), wall)

    def sign(number, x, y, rotation=0):
        # 两块实体门牌：常规人眼高度以及机器人可读取的低位牌。不是屏幕浮层。
        segments = {"0": "abcedf", "1": "bc", "2": "abged", "3": "abgcd"}
        bars = {
            "a": (0, 0.08, 0.026, 0.005),
            "g": (0, 0, 0.026, 0.005),
            "d": (0, -0.08, 0.026, 0.005),
            "f": (-0.027, 0.04, 0.005, 0.035),
            "b": (0.027, 0.04, 0.005, 0.035),
            "e": (-0.027, -0.04, 0.005, 0.035),
            "c": (0.027, -0.04, 0.005, 0.035),
        }
        for height in (0.28, 1.55):
            body = spec.worldbody.add_body(
                name=f"office_sign_{number}_{height}",
                pos=(x, y, height),
                quat=(math.cos(rotation / 2), 0, 0, math.sin(rotation / 2)),
            )
            box(
                f"plaque_{number}_{height}",
                (0, 0, 0),
                (0.14, 0.012, 0.115),
                metal,
                body,
                False,
            )
            for index, digit in enumerate(number):
                for segment in segments[digit]:
                    dx, dz, sx, sz = bars[segment]
                    box(
                        f"digit_{number}_{height}_{index}_{segment}",
                        ((index - 1) * 0.085 + dx, -0.014, dz),
                        (sx, 0.002, sz),
                        white,
                        body,
                        False,
                    )

    sign("101", -2.12, 0.64)
    sign("102", 1.21, 0.64)
    sign("103", 1.34, -0.18, -math.pi / 2)

    def desk(name, x, y, width=1.15, depth=0.6):
        box(name + "_top", (x, y, 0.745), (width / 2, depth / 2, 0.025), wood)
        for dx in (-width / 2 + 0.055, width / 2 - 0.055):
            for dy in (-depth / 2 + 0.055, depth / 2 - 0.055):
                box(
                    name + f"_leg_{dx}_{dy}",
                    (x + dx, y + dy, 0.36),
                    (0.025, 0.025, 0.36),
                    metal,
                )

    def chair(name, x, y, facing=0):
        body = spec.worldbody.add_body(
            name="office_" + name,
            pos=(x, y, 0),
            quat=(math.cos(facing / 2), 0, 0, math.sin(facing / 2)),
        )
        box(name + "_seat", (0, 0, 0.45), (0.22, 0.21, 0.04), fabric, body)
        box(name + "_back", (0, 0.19, 0.69), (0.22, 0.035, 0.22), fabric, body)
        box(name + "_stem", (0, 0, 0.22), (0.035, 0.035, 0.22), metal, body)
        box(name + "_base_x", (0, 0, 0.055), (0.25, 0.035, 0.025), metal, body)
        box(name + "_base_y", (0, 0, 0.055), (0.035, 0.25, 0.025), metal, body)

    for group, x in enumerate((-1.4, 0.15)):
        for row, y in enumerate((-0.75, -0.15)):
            name = f"desk_{group}_{row}"
            desk(name, x, y)
            box(name + "_monitor", (x, y, 0.98), (0.20, 0.025, 0.14), metal)
            box(
                name + "_screen",
                (x, y - 0.027, 0.98),
                (0.18, 0.002, 0.115),
                (0.12, 0.16, 0.18, 1),
                solid=False,
            )
            box(name + "_monitor_base", (x, y, 0.79), (0.1, 0.08, 0.015), metal)
            box(
                name + "_keyboard",
                (x, y - 0.18, 0.78),
                (0.15, 0.045, 0.008),
                metal,
                solid=False,
            )
        box(f"partition_{group}", (x, -0.45, 0.96), (0.57, 0.025, 0.17), fabric)
        chair(f"task_chair_{group}", x, -1.25)
    desk("meeting_101", -1.5, 1.55, 1.15, 0.65)
    chair("meeting_101_a", -1.5, 2.1)
    chair("meeting_101_b", -2.5, 1.4, -math.pi / 2)
    desk("meeting_102", 1.65, 1.65, 1.05, 0.65)
    chair("meeting_102_a", 2.35, 1.65, math.pi / 2)
    chair("meeting_102_b", 0.95, 1.65, -math.pi / 2)
    desk("meeting_103", 2.3, -1.1, 0.75, 0.55)
    chair("meeting_103_a", 2.3, -0.5)
    box("file_cabinet", (-2.73, -1.0, 0.65), (0.22, 0.38, 0.65), (0.62, 0.64, 0.62, 1))
    for z in (0.3, 0.65, 1.0):
        box(
            f"cabinet_handle_{z}",
            (-2.495, -1.0, z),
            (0.012, 0.08, 0.009),
            metal,
            solid=False,
        )
    box("printer_stand", (-2.68, -0.25, 0.36), (0.25, 0.25, 0.36), wood)
    box("printer", (-2.68, -0.25, 0.85), (0.22, 0.21, 0.13), white)
    box("printer_slot", (-2.448, -0.25, 0.86), (0.003, 0.14, 0.035), metal, solid=False)
    box("sofa_seat", (0.65, -2.23, 0.35), (0.48, 0.22, 0.10), fabric)
    box("sofa_back", (0.65, -2.43, 0.62), (0.48, 0.06, 0.27), fabric)
    # 休息区实体挂画：60 cm 方框，画心朝向室内。只增加装饰，不占用地面
    # 通道；墙仍负责碰撞与遮挡。纹理随仓库保存，不依赖个人 Downloads 目录。
    spec.add_texture(
        name="office_logo_texture",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        file=str(Path(__file__).resolve().parent / "assets" / "chi-welcome-logo.png"),
    )
    textures = [""] * int(mujoco.mjtTextureRole.mjNTEXROLE)
    textures[int(mujoco.mjtTextureRole.mjTEXROLE_RGB)] = "office_logo_texture"
    spec.add_material(
        name="office_logo_material",
        textures=textures,
        texrepeat=[1, 1],
        texuniform=False,
        specular=0,
        shininess=0,
    )
    box(
        "logo_frame",
        (0.65, -2.443, 1.5),
        (0.3, 0.012, 0.3),
        (0.06, 0.06, 0.06, 1),
        solid=False,
    )
    spec.worldbody.add_geom(
        name="office_logo_picture",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        pos=(0.65, -2.429, 1.5),
        size=(0.284, 0.284, 0.001),
        quat=(0, 0, 2**-0.5, 2**-0.5),
        material="office_logo_material",
        contype=0,
        conaffinity=0,
    )
    for x in (-2.2, 0.8, 2.2):
        box(
            "window_" + str(x),
            (x, 2.449, 1.55),
            (0.5, 0.008, 0.45),
            (0.68, 0.79, 0.83, 1),
            solid=False,
        )
        box(
            "window_frame_" + str(x),
            (x, 2.435, 1.55),
            (0.012, 0.012, 0.45),
            white,
            solid=False,
        )
    for x in (-1.5, 1.5):
        spec.worldbody.add_light(
            pos=(x, 0, 2.8), dir=(0, 0, -1), diffuse=(0.45, 0.43, 0.40), castshadow=True
        )

    # 独立自由关节的小球会真实滚动，不是与地面焊接的静态目标。
    for color, (xy, rgba) in BALLS.items():
        body = spec.worldbody.add_body(
            name="office_ball_" + color, pos=(*xy, BALL_RADIUS + 0.002)
        )
        body.add_freejoint(name="office_ball_joint_" + color)
        body.add_geom(
            name="office_ball_geom_" + color,
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=(BALL_RADIUS, 0, 0),
            rgba=rgba,
            mass=0.06,
            friction=(0.8, 0.02, 0.002),
        )
