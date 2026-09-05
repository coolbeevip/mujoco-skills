"""真实头部策略检查：验证输入、实际摄像头方向与坐姿兼容，不调用远程模型。"""

import json

from scenes import MICRODUCK, MicroduckScene


def main():
    scene = MicroduckScene("navigation", MICRODUCK / ".cache")
    try:
        for _ in range(150):
            scene.navigation_hold()
        for seated in (False, True):
            if seated:
                scene.navigation_action("sit")
                for _ in range(150):
                    scene.navigation_hold()
                assert scene.behaviors.stage == "seated"
            values = {}
            for name in ("head_reset", "head_down", "head_up", "head_reset"):
                scene.navigation_action(name)
                for _ in range(150):
                    scene.navigation_hold()
                values[name] = scene.head_feedback()["camera_pitch_deg"]
                assert scene.observe().startswith(b"\x89PNG")
                assert not scene.runtime.failed
                if seated:
                    assert scene.behaviors.stage == "seated"
                print(
                    json.dumps(
                        dict(seated=seated, action=name, **scene.head_feedback())
                    ),
                    flush=True,
                )
            assert values["head_down"] < values["head_reset"] - 5, values
            assert values["head_up"] > values["head_reset"] + 5, values
    finally:
        scene.close()
    print(json.dumps(dict(status="passed", model_task_verified=False)))


if __name__ == "__main__":
    main()
