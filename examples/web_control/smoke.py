"""真实资产与离屏渲染冒烟检查；不宣称机器人已经完成踢球等任务。"""

import json
import struct

from scenes import MICRODUCK
from server import Engine


def main():
    engine = Engine(MICRODUCK / ".cache")
    results = []

    def command(op, **extra):
        return engine.apply(dict(op=op, generation=engine.generation, **extra))

    def steps(count):
        for _ in range(count):
            engine.heartbeat()
            engine.tick()
            if engine.error:
                raise AssertionError(engine.error)

    try:
        for id in ("walk", "ball", "roller"):
            command("load", scene=id)
            assert engine.frame.startswith(b"\x89PNG\r\n\x1a\n")
            initial_frame = engine.frame
            command("resume")
            steps(250)
            start = engine.scene.state()["position_m"][0]
            assert command("action", action="forward") == "command updated"
            steps(100)
            end = engine.scene.state()["position_m"][0]
            # 这里只确认存在可测的前向位移，不把期望 0.3 m/s 当成跟踪精度承诺。
            # 轮式策略实际位移可能显著较小，报告保留原始测量供检查。
            assert end - start > 0.01, (id, start, end)
            command("action", action="stop")
            steps(150)
            if id != "walk":
                action = "kick_left" if id == "ball" else "crouch"
                assert command("action", action=action).startswith("started:"), id
                steps(300)
                assert engine.scene.behaviors.stage == "idle"
            engine.render()
            assert engine.frame != initial_frame
            command("pause")
            before = engine.scene.state()["time_s"]
            steps(10)
            assert engine.scene.state()["time_s"] == before
            command("camera", azimuth=45, elevation=-30, distance=1)
            # 暂停时切换分辨率也立即刷新；不能重置位姿、相机或策略进度。
            before_state = engine.scene.runtime.snapshot()
            for width in (1920, 640, 1280):
                command("resolution", width=width)
                assert struct.unpack(">II", engine.frame[16:24]) == (
                    width,
                    width * 9 // 16,
                )
                assert engine.scene.runtime.snapshot() == before_state
                assert engine.scene.camera.azimuth == 45
            results.append(
                dict(
                    scene=id,
                    time_s=before,
                    forward_m=end - start,
                    png_bytes=len(engine.frame),
                    status="passed",
                )
            )
        print(json.dumps(results, indent=2))
    finally:
        engine.close()


if __name__ == "__main__":
    main()
