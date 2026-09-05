"""运行三次同种子短闭环；不代替完整动作序列验收。"""

import argparse
import json
import platform
import sys
from importlib.metadata import version

import numpy as np

from assets import DEFAULT_CACHE, lock
from runtime import Runtime


def smoke(cache=DEFAULT_CACHE):
    runtime = Runtime(cache, seed=0)
    runs, traces = [], []
    for _ in range(3):
        initial = runtime.reset()
        trace = []
        # 控制频率为 50 Hz：前 250 步站立，后 250 步给低速前进指令。
        for tick in range(500):
            state = runtime.step((0, 0, 0) if tick < 250 else (0.1, 0, 0))
            trace.append(state["qpos"] + state["qvel"])
        traces.append(np.asarray(trace))
        runs.append({"initial": initial, "final": state})
    # 逐控制步比较完整 qpos/qvel 轨迹，验证重置后的可重复性，而非动作达标。
    error = max(float(np.max(np.abs(t - traces[0]))) for t in traces[1:])
    if error > 1e-10:
        raise ValueError(f"reset reproducibility failed: {error}")
    return {
        "status": "smoke_passed",
        "scope": "finite_cpu_closed_loop_only",
        "full_sequence_verified": False,
        "fall_detection_verified": False,
        "platform": platform.platform(),
        "python": platform.python_version(),
        "dependencies": {
            name: version(name)
            for name in ["numpy", "mujoco", "onnxruntime", "better-actuator-models"]
        },
        "model_revision": lock()["model_revision"],
        "bam_revision": lock()["bam_revision"],
        "seed": 0,
        "policy_revision": lock()["policy_revision"],
        "repeat_max_state_error": error,
        "repeat_tolerance": 1e-10,
        "runs": runs,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", default=DEFAULT_CACHE)
    args = parser.parse_args()
    try:
        print(json.dumps(smoke(args.cache), indent=2, allow_nan=False))
    except (OSError, ValueError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
