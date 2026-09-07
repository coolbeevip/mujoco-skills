"""真实模型办公区寻物检查；显式 --live 才会发送图像并产生模型调用费用。"""

import argparse
import json
import time
from pathlib import Path

from navigation import Navigation
from scenes import MicroduckScene, MICRODUCK
from vlm import Config, Provider, load_env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true")
    args = parser.parse_args()
    if not args.live:
        parser.error("需要显式 --live；仅检查场景请运行 office_smoke.py")
    load_env(Path(__file__).resolve().parents[2] / ".env")
    config = Config.from_env()
    if config is None:
        raise ValueError("请先配置视觉模型")
    provider = Provider(config)
    scene = MicroduckScene("office", MICRODUCK / ".cache")
    task = "找到紫色小球，走到它面前。球可能在家具后或会议室中，需要分区探索；不要只在起点转圈。"
    nav = Navigation(task, provider.submit, max_decisions=100, timeout_s=600)
    out = Path(__file__).parent / "artifacts"
    out.mkdir(exist_ok=True)
    try:
        assert not scene.proximity(task)["visible"]
        count = 0
        while nav.active:
            nav.tick(scene)
            if len(nav.history) > count:
                count = len(nav.history)
                print(json.dumps(nav.history[-1], ensure_ascii=False), flush=True)
                (out / "office-search-latest.png").write_bytes(scene.observe())
            if nav.phase == "thinking":
                time.sleep(0.02)
        report = dict(
            phase=nav.phase,
            reason=nav.reason,
            history=nav.history,
            final_proximity=scene.proximity(task),
        )
        (out / "office-search.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2)
        )
        print(
            json.dumps(
                {k: v for k, v in report.items() if k != "history"}, ensure_ascii=False
            )
        )
        assert nav.phase == "completed", nav.reason
        assert report["final_proximity"].get("near")
    finally:
        if nav.active:
            nav.cancel(scene, "测试结束")
        provider.close()
        scene.close()


if __name__ == "__main__":
    main()
