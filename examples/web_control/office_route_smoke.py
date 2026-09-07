"""固定高层响应验证真实步行、穿门、盲区发现与走近；不调用远程模型。"""

import json
from concurrent.futures import Future
from pathlib import Path

from navigation import Navigation
from scenes import MicroduckScene, MICRODUCK


def main():
    scene = MicroduckScene("office", MICRODUCK / ".cache")
    task = "找到紫色小球，走到它面前"
    calls = []

    def submit(task, image, history, context):
        p = context["proximity"]
        # 固定选择一个房间只测试控制链，不能视为真实模型具有分区选择能力。
        action = "done" if p.get("visible") else "search_103"
        future = Future()
        future.set_result(
            dict(
                action=action,
                amount=0,
                target_visible=p.get("visible", False),
                confidence=0.9,
                evidence="本地固定响应验证高层搜索与到达门槛",
            )
        )
        calls.append(action)
        return future

    nav = Navigation(task, submit, max_decisions=100, timeout_s=600)
    out = Path(__file__).parent / "artifacts"
    out.mkdir(exist_ok=True)
    try:
        assert not scene.proximity(task)["visible"]
        prior = None
        while nav.active:
            nav.tick(scene)
            state = (nav.phase, len(nav.history))
            if state != prior:
                print(
                    json.dumps(
                        dict(
                            phase=nav.phase,
                            decisions=len(nav.history),
                            pose=scene.motion_pose(),
                        ),
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                prior = state
        report = dict(
            phase=nav.phase,
            reason=nav.reason,
            calls=calls,
            history=nav.history,
            final_proximity=scene.proximity(task),
            real_model_verified=False,
        )
        (out / "office-route.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2)
        )
        (out / "office-route-final.png").write_bytes(scene.observe())
        print(
            json.dumps(
                {k: v for k, v in report.items() if k != "history"}, ensure_ascii=False
            ),
            flush=True,
        )
        assert nav.phase == "completed", nav.reason
        assert report["final_proximity"]["near"]
        assert scene.motion_pose()[0] > 1.4
        # 真值仅供测试核对，不进入地图、规划器或模型上下文。
        import numpy as np

        r = scene.runtime
        truth = (
            float(
                np.linalg.norm(
                    r.data.xpos[r.model.body("office_ball_purple").id][:2]
                    - scene.motion_pose()[:2]
                )
            )
            - 0.055
        )
        assert 0.15 < truth < 0.32, truth
        print(
            json.dumps(
                dict(status="passed", true_surface_distance_cm=round(truth * 100, 1))
            ),
            flush=True,
        )
    finally:
        if nav.active:
            nav.cancel(scene, "本地测试结束")
        scene.close()


if __name__ == "__main__":
    main()
