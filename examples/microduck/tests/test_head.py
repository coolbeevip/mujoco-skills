"""头部指令边界与平滑输入；不下载策略、不调用模型服务。"""

import numpy as np
import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime import Runtime


def check_head_target_bounds_and_roller_rejection():
    runtime = Runtime.__new__(Runtime)
    runtime.mode = "walk"
    runtime.head_target = np.zeros(4, dtype=np.float32)
    runtime.set_head_pitch(0.35)
    np.testing.assert_allclose(runtime.head_target, [0, 0.35, 0, 0])
    for value in (float("nan"), float("inf"), 0.5):
        with unittest.TestCase().assertRaises(ValueError):
            runtime.set_head_pitch(value)
    runtime.mode = "roller"
    with unittest.TestCase().assertRaises(ValueError):
        runtime.set_head_pitch(0)


def check_head_command_is_smoothed_and_cleared_for_non_head_policy():
    runtime = Runtime()
    runtime.set_head_pitch(0.35)
    runtime.step_policy("standing")
    assert 0 < runtime.head_command[1] <= 0.010001
    np.testing.assert_allclose(
        runtime.observation([0, 0, 0])[51:55], runtime.head_command
    )
    runtime.step_policy("kick_left")
    assert not np.any(runtime.head_command)
    assert not np.any(runtime.head_target)


class HeadTests(unittest.TestCase):
    def test_bounds(self):
        check_head_target_bounds_and_roller_rejection()

    def test_smoothing_and_policy_switch(self):
        check_head_command_is_smoothed_and_cleared_for_non_head_policy()


if __name__ == "__main__":
    unittest.main()
