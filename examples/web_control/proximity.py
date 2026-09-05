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
        }.items()
        if any(token in task for token in tokens)
    ]
    return matches[0] if len(matches) == 1 else None


def measure(rgb, depth, fovy, camera_position, camera_rotation, robot_pose, color):
    r, g, b = rgb.astype(float).transpose(2, 0, 1)
    masks = dict(
        blue=(b > 50) & (b > 2 * r) & (b > 1.8 * g),
        red=(r > 50) & (r > 2 * g) & (r > 2 * b),
        yellow=(r > 70) & (g > 60) & (r > 2 * b) & (g > 2 * b),
    )
    mask = masks[color] & np.isfinite(depth) & (depth > 0.03) & (depth < 3)
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
    delta = np.median(points[:, :2], axis=0) - np.asarray(robot_pose[:2])
    distance = float(np.linalg.norm(delta)) * 100
    bearing = math.degrees(
        math.remainder(math.atan2(delta[1], delta[0]) - robot_pose[2], math.tau)
    )
    return dict(
        source="head_rgbd",
        visible=True,
        color=color,
        surface_distance_cm=round(distance, 1),
        bearing_deg=round(bearing, 1),
        near=18 <= distance <= 28 and abs(bearing) <= 12,
    )
