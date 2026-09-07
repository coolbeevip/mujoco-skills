"""固定彩色目标的 RGB-D 测距；不使用物体 ID、几何中心或目标世界坐标。"""

import math
import numpy as np


def target_color(task):
    matches = [
        name
        for name, tokens in {
            "blue": ("蓝色", "蓝柱"),
            "red": ("红色",),
            "yellow": ("黄色",),
            "green": ("绿色",),
            "purple": ("紫色",),
        }.items()
        if any(token in task for token in tokens)
    ]
    return matches[0] if len(matches) == 1 else None


def color_mask(rgb, color):
    """颜色候选只用于请求停步复查，不能证明是球或已经到达。"""
    r, g, b = rgb.astype(float).transpose(2, 0, 1)
    masks = dict(
        blue=(b > 50) & (b > 2 * r) & (b > 1.8 * g),
        red=(r > 50) & (r > 2 * g) & (r > 2 * b),
        yellow=(r > 70) & (g > 60) & (r > 2 * b) & (g > 2 * b),
        green=(g > 50) & (g > 1.8 * r) & (g > 1.6 * b),
        purple=(r > 50) & (b > 50) & (r > 1.8 * g) & (b > 1.8 * g),
    )
    return masks[color]


def measure(
    rgb, depth, fovy, camera_position, camera_rotation, robot_pose, color, *, ball=False
):
    mask = color_mask(rgb, color) & np.isfinite(depth) & (depth > 0.03) & (depth < 3)
    v, u = np.where(mask)
    if len(u) < 30:
        return dict(source="head_rgbd", visible=False, color=color)
    # 将有颜色证据的深度像素反投影，再用自身位姿换算水平距离和方向。
    h, w = depth.shape
    focal = h / (2 * math.tan(math.radians(fovy) / 2))
    z = depth[v, u]
    rays = np.stack(
        ((u + 0.5 - w / 2) * z / focal, (h / 2 - v - 0.5) * z / focal, -z), axis=1
    )
    points = rays @ np.asarray(camera_rotation).reshape(3, 3).T + camera_position
    if ball:
        # 办公室里的彩色窗户、椅背不能被当成球。用深度点云验证物理尺寸和
        # 球面曲率；这里不读取球的仿真 ID 或预设位置。遮挡过重时宁可继续观察。
        points = points[(points[:, 2] > 0.005) & (points[:, 2] < 0.14)]
        if len(points) < 30:
            return dict(source="head_rgbd", visible=False, color=color)
        local = points - points.mean(axis=0)
        center, _, rank, _ = np.linalg.lstsq(
            2 * local, np.sum(local * local, axis=1), rcond=None
        )
        radii = np.linalg.norm(local - center, axis=1)
        radius = float(np.median(radii))
        if rank < 3 or not 0.035 <= radius <= 0.075 or np.std(radii) > 0.008:
            return dict(source="head_rgbd", visible=False, color=color)
    delta = np.median(points[:, :2], axis=0) - np.asarray(robot_pose[:2])
    distance = float(np.linalg.norm(delta)) * 100
    bearing = math.degrees(
        math.remainder(math.atan2(delta[1], delta[0]) - robot_pose[2], math.tau)
    )
    # 距离摘要与 near 使用同一精度；不能显示 28.0 cm 却按隐藏小数拒绝到达，
    # 否则上层在“已到边界”和“仍需前进”之间得到矛盾证据。
    distance, bearing = round(distance, 1), round(bearing, 1)
    return dict(
        **(
            {"estimated_position_m": (center + points.mean(axis=0))[:2].tolist()}
            if ball
            else {}
        ),
        source="head_rgbd",
        visible=True,
        color=color,
        surface_distance_cm=round(distance, 1),
        bearing_deg=round(bearing, 1),
        near=18 <= distance <= 28 and abs(bearing) <= 12,
    )


def clearance(depth, fovy, camera_position, camera_rotation, robot_pose):
    """头部深度图内，机器人前方 26 cm 宽通道中的最近低位障碍。

    地板不是障碍；桌腿、椅脚、墙和球都是。结果只覆盖当前相机视野，
    不是完整地图，也不保证转身侧面的盲区安全。
    """
    h, w = depth.shape
    v, u = np.mgrid[0:h:3, 0:w:3]
    z = depth[v, u]
    focal = h / (2 * math.tan(math.radians(fovy) / 2))
    rays = np.stack(
        ((u + 0.5 - w / 2) * z / focal, (h / 2 - v - 0.5) * z / focal, -z), axis=-1
    )
    points = (
        rays.reshape(-1, 3) @ np.asarray(camera_rotation).reshape(3, 3).T
        + camera_position
    )
    dx, dy = points[:, 0] - robot_pose[0], points[:, 1] - robot_pose[1]
    c, s = math.cos(robot_pose[2]), math.sin(robot_pose[2])
    forward, lateral = dx * c + dy * s, -dx * s + dy * c
    valid = (
        np.isfinite(points).all(axis=1)
        & (points[:, 2] > 0.025)
        & (points[:, 2] < 0.24)
        & (forward > 0.08)
        & (forward < 2)
        & (np.abs(lateral) < 0.13)
    )
    distances = forward[valid]
    # 只有图中确实看到一段连续地面才提供较长探索步长的依据。
    # 每个纵向小段都要有左右两侧地面采样，不能拿远处一块地板推断整条通道。
    floor = np.isfinite(points).all(axis=1) & (np.abs(points[:, 2]) < 0.012)
    covered = 0
    for lo in np.arange(0.2, 1.01, 0.1):
        strip = floor & (forward >= lo) & (forward < lo + 0.1)
        if not all(
            np.any(strip & (lateral >= a) & (lateral < a + 0.13)) for a in (-0.13, 0)
        ):
            break
        covered = round((lo + 0.1) * 100)
    return dict(
        source="head_depth",
        observed_floor_cm=covered,
        corridor_width_cm=26,
        front_clearance_cm=round(float(np.min(distances)) * 100, 1)
        if len(distances) >= 3
        else None,
        scope="仅当前视野内低位障碍，不覆盖侧后方盲区",
    )
