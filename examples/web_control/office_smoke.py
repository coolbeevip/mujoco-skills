"""办公室资产、初始稳定性及真实头部盲区检查；不调用模型服务。"""

from pathlib import Path
import json
import math
import numpy as np
import mujoco

from office import BALLS, BALL_RADIUS, SPAWN
from scenes import MicroduckScene, MICRODUCK
from motion import Motion


def main():
    scene = MicroduckScene("office", MICRODUCK / ".cache")
    try:
        for _ in range(100):
            scene.step()
        assert not scene.runtime.failed
        assert np.linalg.norm(np.asarray(scene.motion_pose()[:2]) - SPAWN) < 0.05
        r = scene.runtime
        picture_geom = r.model.geom("office_logo_picture")
        assert not picture_geom.contype[0] and not picture_geom.conaffinity[0]
        assert picture_geom.matid[0] == r.model.material("office_logo_material").id
        assert np.allclose(r.model.geom("office_logo_frame").size, (0.3, 0.012, 0.3))
        texture_id = r.model.texture("office_logo_texture").id
        assert r.model.tex_width[texture_id] == r.model.tex_height[texture_id] == 1024
        for color in BALLS:
            geom = r.model.geom("office_ball_geom_" + color)
            assert abs(geom.size[0] - BALL_RADIUS) < 1e-9
            joint = r.model.joint("office_ball_joint_" + color)
            assert joint.type[0] == mujoco.mjtJoint.mjJNT_FREE
            assert 0.05 < r.data.geom_xpos[geom.id][2] < 0.06
        for number in ("101", "102", "103"):
            assert r.model.body(f"office_sign_{number}_0.28").id >= 0
            assert r.model.body(f"office_sign_{number}_1.55").id >= 0
        head = scene.observe()
        original_rgba = r.model.geom_rgba.copy()
        original_collision = r.model.geom_contype.copy()
        original_qpos = r.data.qpos.copy()
        scene.frame("external")
        assert any(
            g.objtype == mujoco.mjtObj.mjOBJ_GEOM
            and g.objid == picture_geom.id
            and g.rgba[3] == 1
            for g in scene.renderer.scene.geoms[: scene.renderer.scene.ngeom]
        ), "外部墙壁透明时挂画仍应保持不透明"
        walls = [
            g
            for g in scene.renderer.scene.geoms[: scene.renderer.scene.ngeom]
            if g.objtype == mujoco.mjtObj.mjOBJ_GEOM
            and g.objid in scene.preview_wall_ids
        ]
        assert len(walls) == 14
        assert all(abs(float(g.rgba[3]) - 0.18) < 1e-6 and g.transparent for g in walls)
        assert np.array_equal(original_rgba, r.model.geom_rgba)
        assert np.array_equal(original_collision, r.model.geom_contype)
        assert np.array_equal(original_qpos, r.data.qpos)
        assert scene.observe() == head, "外部透明墙不能改变发送给模型的头部图像"
        scene.frame("head")
        assert all(
            g.rgba[3] == 1
            for g in scene.renderer.scene.geoms[: scene.renderer.scene.ngeom]
            if g.objtype == mujoco.mjtObj.mjOBJ_GEOM
            and g.objid in scene.preview_wall_ids
        ), "主预览切换头部视角后必须恢复真实墙体遮挡"
        renderer = scene.observer_renderer
        renderer.enable_segmentation_rendering()
        try:
            segments = renderer.render()
        finally:
            renderer.disable_segmentation_rendering()
        visible = {
            color: int(
                np.count_nonzero(
                    segments[:, :, 0] == r.model.geom("office_ball_geom_" + color).id
                )
            )
            for color in BALLS
        }
        assert visible["red"] > 30, visible
        assert visible["blue"] == visible["green"] == visible["purple"] == 0, visible
        measured = scene.proximity("靠近红色小球")
        assert measured["visible"], measured
        assert 115 < measured["surface_distance_cm"] < 135, measured
        assert not scene.proximity("靠近紫色小球")["visible"]
        assert scene.obstacle_clearance()["source"] == "head_depth"
        from office_search import FloorMap, POINTS

        floor_map = FloorMap.from_scene(scene)
        # 紫球从 103 门内右转、沿隔墙内侧接近；真值仅用于场景验收，
        # 不把这个停靠点交给导航控制器或模型。
        purple = np.asarray(BALLS["purple"][0])
        entrance = np.asarray((1.72, -0.8))
        direction = (purple - entrance) / np.linalg.norm(purple - entrance)
        stop_point = purple - 0.31 * direction
        assert floor_map.clear_line(entrance, stop_point), (
            "103 到紫球的接近通道被家具阻挡"
        )
        # 在原有机器人膨胀边界之外，再给步态左右各留 4 cm 摆动余量。
        lateral = np.array([-direction[1], direction[0]]) * 0.04
        for offset in (-lateral, lateral):
            assert floor_map.clear_line(entrance + offset, stop_point + offset), (
                "103 紫球接近通道没有足够的步态侧移余量"
            )
        hit = np.zeros(1, dtype=np.int32)
        ray_start = np.array([*entrance, BALL_RADIUS])
        distance = mujoco.mj_ray(
            r.model, r.data, ray_start, np.array([*direction, 0]), None, 1, -1, hit
        )
        assert distance > 0 and hit[0] == r.model.geom("office_ball_geom_purple").id, (
            "103 门内到紫球的视线被桌腿等物体挡住"
        )
        for region, points in POINTS.items():
            for point in points:
                assert floor_map.free(point), (region, point)
                try:
                    path = floor_map.path(SPAWN, point)
                except ValueError as error:
                    raise AssertionError((region, point, str(error))) from error
                assert all(
                    floor_map.clear_line(a, b) for a, b in zip([SPAWN, *path], path)
                )
        # 验证门洞没有被墙体封死。沿地面上方 15 cm 穿门射线，不能在门线上命中墙。
        for origin, direction in [
            ((-1.5, 0.4, 0.15), (0, 1, 0)),
            ((0.65, 0.4, 0.15), (0, 1, 0)),
            ((1.1, -0.8, 0.15), (1, 0, 0)),
        ]:
            hit = np.zeros(1, dtype=np.int32)
            distance = mujoco.mj_ray(
                r.model,
                r.data,
                np.asarray(origin),
                np.asarray(direction, dtype=float),
                None,
                1,
                -1,
                hit,
            )
            assert distance > 0.65, (origin, distance)
        out = Path(__file__).parent / "artifacts"
        out.mkdir(exist_ok=True)
        (out / "office-external.png").write_bytes(scene.frame())
        (out / "office-head.png").write_bytes(head)
        # 门牌存在不等于机器人能读到：从门前的独立观测点检查三个号码的每根笔画。
        # 这些专用摆位只验证渲染，不计入穿门或自主导航结果。
        root = r.model.joint("trunk_base_freejoint").qposadr[0]
        saved = r.data.qpos.copy()
        # 门内正对紫球时不仅无遮挡，也要能从实际 RGB-D 验证球体。
        # 专用相机摆位不作为自主走近成功的证据。
        yaw = math.atan2(purple[1] - entrance[1], purple[0] - entrance[0])
        r.data.qpos[root : root + 2] = entrance
        r.data.qpos[root + 3 : root + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(r.model, r.data)
        assert scene.proximity("紫色小球")["visible"], (
            "门内正对紫球时无法通过 RGB-D 识别"
        )
        (out / "office-purple-from-entry.png").write_bytes(scene.observe())
        r.data.qpos[:] = saved
        mujoco.mj_forward(r.model, r.data)
        for number, x, y, yaw in (
            ("101", -2.12, -0.8, math.pi / 2),
            ("102", 1.21, -0.8, math.pi / 2),
            ("103", 0.8, -1.2, math.atan2(1.02, 0.54)),
        ):
            r.data.qpos[root : root + 2] = [x, y]
            r.data.qpos[root + 3 : root + 7] = [
                math.cos(yaw / 2),
                0,
                0,
                math.sin(yaw / 2),
            ]
            mujoco.mj_forward(r.model, r.data)
            (out / f"office-sign-{number}.png").write_bytes(scene.observe())
            renderer.enable_segmentation_rendering()
            try:
                sign_segments = renderer.render()[:, :, 0]
            finally:
                renderer.disable_segmentation_rendering()
            for gid in range(r.model.ngeom):
                name = r.model.geom(gid).name
                if name.startswith(f"office_digit_{number}_0.28_"):
                    assert np.count_nonzero(sign_segments == gid) >= 3, name
        r.data.qpos[:] = saved
        mujoco.mj_forward(r.model, r.data)
        # 独立物理检查使用专门起点，不把这次摆位冒充为自主寻物。
        # 两段 30 cm 均由已有策略施加关节控制，运动中不修改根节点位置。
        scene.close()
        scene.renderer = scene.observer_renderer = None
        r.spawn_xy = np.array([1.08, -0.8])
        r.reset()
        r.model.camera("head_camera").quat[:] = [2**-0.5, 0, 0, -(2**-0.5)]
        mujoco.mj_forward(r.model, r.data)
        scene.resize(1280)
        from behaviors import Behaviors

        scene.behaviors = Behaviors(r)
        for _ in range(100):
            scene.step()
        for _ in range(2):
            motion = Motion("forward", 30, scene.motion_pose())
            while motion.status == "running":
                motion.tick(scene)
            assert motion.status == "reached", motion.result()
            assert not r.failed
        assert scene.motion_pose()[0] > 1.58, scene.motion_pose()
        (out / "office-door-103.png").write_bytes(scene.observe())
        # 对球施加短暂外力，仅验证其为可运动物体，不声称踢球策略已踢中。
        ball_id = r.model.body("office_ball_purple").id
        before = r.data.xpos[ball_id].copy()
        r.data.xfrc_applied[ball_id, 0] = -0.1
        for _ in range(20):
            scene.step()
        r.data.xfrc_applied[ball_id] = 0
        assert np.linalg.norm(r.data.xpos[ball_id][:2] - before[:2]) > 0.01
        print(
            json.dumps(
                dict(
                    status="passed",
                    initial_visible_pixels=visible,
                    doorway_walk_verified=True,
                    all_room_signs_in_head_view=True,
                    all_observation_points_connected=True,
                    ball_motion_verified=True,
                    search_task_verified=False,
                )
            )
        )
    finally:
        scene.close()


if __name__ == "__main__":
    main()
