"""故意提前报告完成，验证真实仿真中距离门槛和近处鞠躬、坐下动作链。"""

import json
import argparse
import time
from pathlib import Path
from concurrent.futures import Future

from navigation import Navigation
from scenes import MicroduckScene, MICRODUCK


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--live", action="store_true", help="调用已配置的真实模型，可能产生费用"
    )
    args = parser.parse_args()
    provider = None
    scene = MicroduckScene("navigation", MICRODUCK / ".cache")

    def submit(*args, **kwargs):
        future = Future()
        future.set_result(
            dict(
                action="done",
                amount=0,
                target_visible=True,
                confidence=0.9,
                evidence="固定测试响应，故意提前报告完成",
            )
        )
        return future

    if args.live:
        from vlm import Config, Provider, load_env

        load_env(Path(__file__).resolve().parents[2] / ".env")
        config = Config.from_env()
        if config is None:
            raise ValueError("请先配置模型")
        provider = Provider(config)
    nav = Navigation(
        "找到蓝色圆柱体，走到它面前鞠躬后坐下来。你可以低头观察地面判断距离",
        provider.submit if provider else submit,
    )
    try:
        count = 0
        while nav.active:
            nav.tick(scene)
            if len(nav.history) > count:
                count = len(nav.history)
                print(json.dumps(nav.history[-1], ensure_ascii=False), flush=True)
            if nav.phase == "thinking":
                time.sleep(0.02)
        stages = [(h["action"], h.get("proximity")) for h in nav.history]
        print(
            json.dumps(
                dict(
                    phase=nav.phase,
                    reason=nav.reason,
                    stages=stages,
                    final=scene.navigation_context(),
                ),
                ensure_ascii=False,
            ),
            flush=True,
        )
        assert nav.phase == "completed"
        assert scene.behaviors.stage == "seated"
        actions = [h["action"] for h in nav.history]
        assert actions.index("ground_pick") < actions.index("sit")
        for h in nav.history:
            if h["action"] in {"ground_pick", "sit"}:
                assert h["proximity"]["near"]
        # 仅测试使用物体坐标作为独立真值，控制器和模型不读取这些坐标。
        import numpy as np

        r = scene.runtime
        truth = (
            np.linalg.norm(
                r.data.geom_xpos[r.model.geom("blue_cylinder").id][:2]
                - r.data.xpos[r.root_id][:2]
            )
            - 0.08
        )
        assert 0.15 < truth < 0.38, truth
        print(
            json.dumps(
                dict(
                    status="passed",
                    true_surface_distance_cm=round(float(truth) * 100, 1),
                    real_model_verified=args.live,
                )
            )
        )
    finally:
        if provider:
            provider.close()
        scene.close()


if __name__ == "__main__":
    main()
