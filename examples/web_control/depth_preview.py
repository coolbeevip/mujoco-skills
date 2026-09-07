"""把以米为单位的相机深度转换成固定色标，仅用于控制台显示。"""

import numpy as np


def colorize(depth):
    # 固定范围才能跨帧比较：同一种颜色始终代表同一深度，不能逐帧拉伸。
    # 色标与 styles.css 一致：红橙（近）→ 黄 → 青 → 蓝（远）。
    values = np.asarray(depth)
    valid = np.isfinite(values) & (values > 0)
    normalized = (np.clip(np.where(valid, values, 0.1), 0.1, 5) - 0.1) / 4.9
    colors = np.array([[240, 80, 40], [250, 200, 80], [64, 180, 180], [40, 80, 180]])
    rgb = np.stack(
        [np.interp(normalized, [0, 1 / 3, 2 / 3, 1], colors[:, c]) for c in range(3)],
        axis=-1,
    ).astype(np.uint8)
    rgb[~valid] = 0
    return rgb
