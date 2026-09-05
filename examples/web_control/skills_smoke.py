"""用真实策略验证视觉任务的原子动作调度；不调用模型，不宣称动作效果成功。"""

import json
from concurrent.futures import Future

from navigation import Navigation
from scenes import MICRODUCK, MicroduckScene


def execute(scene, actions):
    pending_actions = iter(actions)

    def submit(task, picture, history, context):
        assert picture.startswith(b"\x89PNG")
        action = next(pending_actions, "done")
        assert action == "done" or action in context["available_actions"]
        future = Future()
        future.set_result(
            dict(
                action=action,
                amount=0,
                target_visible=True,
                confidence=0.9,
                evidence="固定测试响应，不是模型视觉判断",
            )
        )
        return future

    nav = Navigation("检查原子动作调度", submit)
    for _ in range(4000):
        nav.tick(scene)
        if not nav.active:
            break
    report = dict(
        actions=actions,
        phase=nav.phase,
        reason=nav.reason,
        state=scene.navigation_context(),
        history=nav.history,
    )
    print(json.dumps(report, ensure_ascii=False), flush=True)
    assert nav.phase == "completed", report
    for item in nav.history:
        if item["action"] != "done":
            assert item["execution"]["status"] == "reached", item


def main():
    for scene_id, actions in [
        ("navigation", ["sit"]),
        ("navigation", ["sit", "stand"]),
        ("navigation", ["ground_pick"]),
        ("navigation", ["kick_left"]),
        ("navigation", ["kick_right"]),
        ("navigation", ["roulade"]),
        ("roller", ["crouch"]),
    ]:
        scene = MicroduckScene(scene_id, MICRODUCK / ".cache")
        try:
            execute(scene, actions)
        finally:
            scene.close()
    print(json.dumps(dict(status="passed", model_task_verified=False)))


if __name__ == "__main__":
    main()
