"""本地比较步行速度与停步误差；不调用模型、不改默认速度。"""

import json
import math
from pathlib import Path

from motion import Motion
from scenes import MICRODUCK, MicroduckScene


def main():
    results = []
    for head in ("head_reset", "head_down"):
        for speed in (0.3, 0.35, 0.4):
            scene = MicroduckScene("navigation", MICRODUCK / ".cache")
            row = dict(head=head, speed_command_m_s=speed, target_cm=30)
            try:
                for _ in range(100):
                    scene.step()
                scene.navigation_action(head)
                for _ in range(150):
                    scene.navigation_hold()
                motion = Motion("forward", 30, scene.motion_pose(), forward_speed=speed)
                while motion.status == "running":
                    motion.tick(scene)
                row.update(motion.result())
                stopped = scene.motion_pose()
                # 再保持一秒，验证提前结束复核后没有明显继续滑行或回摆。
                for _ in range(50):
                    scene.navigation_hold()
                motion.measure(scene.motion_pose())
                row.update(
                    final_distance_cm=round(motion.distance, 2),
                    post_stop_drift_cm=round(
                        math.dist(stopped[:2], scene.motion_pose()[:2]) * 100, 2
                    ),
                    max_tilt_deg=round(scene.behaviors.max_tilt_deg, 2),
                )
                row["passed"] = (
                    motion.status == "reached"
                    and abs(motion.distance - 30) <= 2.5
                    and row["post_stop_drift_cm"] < 1
                    and not scene.runtime.failed
                )
            except Exception as error:
                row.update(passed=False, error=str(error))
            finally:
                scene.close()
            results.append(row)
            print(json.dumps(row), flush=True)
    out = Path(__file__).parent / "artifacts"
    out.mkdir(exist_ok=True)
    (out / "speed-comparison.json").write_text(json.dumps(results, indent=2))
    # 此检查仅为固定初始状态的平地对比，不证明办公区提速后的避障或成功率。


if __name__ == "__main__":
    main()
