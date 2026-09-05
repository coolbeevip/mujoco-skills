"""无窗口运行官方策略，报告实际执行及恢复状态，不代表动作效果验收。"""

import argparse
import json
import platform
import sys

from assets import DEFAULT_CACHE, lock
from behaviors import ACTION_KEYS, Behaviors, CONTROL_DT
from policies import POLICIES
from runtime import Runtime


def demo(policy, cache=DEFAULT_CACHE):
    report = {"policy": policy, "status": "failed", "action_success_verified": False}
    runtime = None
    behavior = None
    try:
        mode = POLICIES[policy][1]
        runtime = Runtime(cache, mode=mode, ball=policy in ("kick_left", "kick_right"))
        behavior = Behaviors(runtime)
        report["initial_state"] = runtime.snapshot()
        # 先用基础策略站稳，再运行请求；每次 demo 使用独立初始状态。
        for _ in range(round(2.0 / CONTROL_DT)):
            behavior.step()
        if policy in ("standing", "walking", "roller"):
            if policy != "standing":
                behavior.handle("w")
            for _ in range(round(3.0 / CONTROL_DT)):
                behavior.step()
            behavior.handle("s")
            for _ in range(round(2.0 / CONTROL_DT)):
                behavior.step()
        else:
            key = next(k for k, value in ACTION_KEYS.items() if value == policy)
            report["trigger"] = behavior.handle(key)
            if not report["trigger"].startswith("started:"):
                raise ValueError(report["trigger"])
            if policy == "sitstand":
                # 坐下标志保持 3 s，然后同一策略切换到站起标志。
                for _ in range(round(3.0 / CONTROL_DT)):
                    behavior.step()
                report["rise"] = behavior.handle("y")
            # 10 s 是执行保护上限，不会因失败而重新触发动作。
            for _ in range(round(10.0 / CONTROL_DT)):
                behavior.step()
                if not behavior.active and not behavior.recovery:
                    break
            else:
                raise ValueError("demo action did not finish within 10s")
            for _ in range(round(1.0 / CONTROL_DT)):
                behavior.step()
        report["status"] = "executed"
    except (ValueError, RuntimeError, OSError, KeyboardInterrupt) as error:
        report["reason"] = str(error) or "user interrupt"
    if behavior:
        report["behavior"] = behavior.status()
    if runtime:
        try:
            report["final_state"] = runtime.snapshot()
        except ValueError:
            report["final_state"] = None
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", default=DEFAULT_CACHE)
    parser.add_argument("--policy", choices=tuple(POLICIES) + ("all",), default="all")
    args = parser.parse_args()
    names = POLICIES if args.policy == "all" else [args.policy]
    result = {
        "platform": platform.platform(),
        "python": platform.python_version(),
        "policy_revision": lock()["policy_revision"],
        "scope": "policy execution and upright recovery; not task success",
        "runs": [demo(name, args.cache) for name in names],
    }
    print(json.dumps(result, indent=2, allow_nan=False))
    sys.exit(0 if all(r["status"] == "executed" for r in result["runs"]) else 2)
