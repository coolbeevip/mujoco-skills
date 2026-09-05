"""真实运动反馈检查：不调用大模型，不修改机器人位姿，不宣称导航任务完成。"""

import json

from motion import Motion
from scenes import MICRODUCK, MicroduckScene


def execute(scene, action, amount):
    motion = Motion(action, amount, scene.motion_pose())
    for _ in range(600):
        motion.tick(scene)
        if motion.status != "running":
            break
    report = dict(action=action, target=amount, **motion.result())
    print(json.dumps(report), flush=True)
    assert motion.status == "reached", report
    target = -amount if action == "right" else amount
    assert abs(motion.progress - target) <= (2.5 if action == "forward" else 5)


def main():
    for action, amount in [
        ("left", 10),
        ("right", 10),
        ("left", 30),
        ("right", 30),
        ("left", 60),
        ("right", 60),
        ("forward", 5),
        ("forward", 25),
        ("forward", 30),
    ]:
        scene = MicroduckScene("navigation", MICRODUCK / ".cache")
        try:
            for _ in range(50):
                scene.motion_step((0, 0, 0))
            execute(scene, action, amount)
        finally:
            scene.close()
    # 连续搜索转身，确保下一次决策不会把前一次完成的转向抵消。
    scene = MicroduckScene("navigation", MICRODUCK / ".cache")
    try:
        for _ in range(50):
            scene.motion_step((0, 0, 0))
        for _ in range(3):
            execute(scene, "left", 45)
    finally:
        scene.close()
    print(json.dumps(dict(status="passed", navigation_task_verified=False)))


if __name__ == "__main__":
    main()
