"""验证真实头部相机与导航目标；不调用大模型，不代表导航任务已完成。"""

import argparse
import json
import struct
import zlib
from pathlib import Path

import numpy as np

from scenes import MICRODUCK, MicroduckScene


def rgb_pixels(picture):
    """解码本样例生成的无滤波 RGB PNG，仅用于离屏渲染检查。"""
    width, height = struct.unpack(">II", picture[16:24])
    compressed, offset = bytearray(), 8
    while offset < len(picture):
        length = struct.unpack(">I", picture[offset : offset + 4])[0]
        if picture[offset + 4 : offset + 8] == b"IDAT":
            compressed.extend(picture[offset + 8 : offset + 8 + length])
        offset += length + 12
    rows = np.frombuffer(zlib.decompress(compressed), dtype=np.uint8).reshape(
        height, width * 3 + 1
    )
    assert np.all(rows[:, 0] == 0)
    return rows[:, 1:].astype(np.int16)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, help="可选：保存头部与外部画面用于目视检查"
    )
    args = parser.parse_args()
    scene = MicroduckScene("navigation", MICRODUCK / ".cache")
    try:
        for _ in range(100):
            scene.step()
        state = scene.runtime.snapshot()
        head = scene.observe()
        scene.set_model_label("ric/qwen3.7-plus")
        external = scene.frame()
        assert (
            scene.renderer.scene.geoms[scene.renderer.scene.ngeom - 1].label
            == "ric/qwen3.7-plus"
        )
        # 改动外部观察相机，不应改变头部相机图像或机器人状态。
        scene.view(45, -35, 1.8)
        assert scene.frame() != external
        # OpenGL 重复渲染可能有少量 1 色阶量化差异，不要求压缩字节完全相同。
        difference = np.abs(rgb_pixels(scene.observe()) - rgb_pixels(head))
        assert difference.max() <= 1 and difference.mean() < 0.001
        assert struct.unpack(">II", head[16:24]) == (640, 360)
        assert scene.runtime.snapshot() == state
        assert scene.runtime.model.camera("head_camera").id >= 0
        camera_id = scene.runtime.model.camera("head_camera").id
        rotation = scene.runtime.data.cam_xmat[camera_id].reshape(3, 3)
        assert -rotation[0, 2] > 0.8, "头部相机应朝向机器人前方"
        assert rotation[2, 1] > 0.8, "头部相机图像上方应朝上"
        for name in ("red_box", "blue_cylinder", "yellow_ball"):
            assert scene.runtime.model.geom(name).id >= 0
        red = scene.runtime.model.geom("red_box").pos
        blue = scene.runtime.model.geom("blue_cylinder").pos
        yellow = scene.runtime.model.geom("yellow_ball").pos
        assert red[0] > 0 and abs(red[1]) < 0.01
        assert blue[0] < 0 and blue[1] > 0
        assert yellow[0] < 0 and yellow[1] < 0
        for target in (red, blue, yellow):
            assert 0.85 < (target[0] ** 2 + target[1] ** 2) ** 0.5 < 0.95
        if args.output:
            args.output.mkdir(parents=True, exist_ok=True)
            (args.output / "head.png").write_bytes(head)
            (args.output / "external.png").write_bytes(external)
        print(
            json.dumps(
                dict(
                    status="passed",
                    camera="head_camera",
                    image_size=[640, 360],
                    simulation_time_s=state["time_s"],
                    task_completed=False,
                )
            )
        )
    finally:
        scene.close()


if __name__ == "__main__":
    main()
