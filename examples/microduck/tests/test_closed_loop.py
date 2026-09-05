"""真实模型测试：必须先显式准备资产，不以 mock 代替物理步进。"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime import Runtime, DT, DECIMATION, validate_policy


@pytest.fixture(scope="module")
def runtime():
    return Runtime()


def test_synchronous_step_and_reset_reproduce_trajectory(runtime):
    traces = []
    for _ in range(2):
        runtime.reset()
        trace = [runtime.step()["qpos"] for _ in range(20)]
        traces.append(trace)
        assert runtime.data.time == pytest.approx(20 * DT * DECIMATION, abs=1e-12)
    np.testing.assert_allclose(traces[0], traces[1], rtol=0, atol=1e-10)


@pytest.mark.parametrize("bad", [[0, 0], [0, np.nan, 0], [9, 0, 0]])
def test_invalid_command_does_not_change_state_or_control(runtime, bad):
    runtime.reset()
    before = runtime.snapshot()
    target = runtime.controller.q_target.copy()
    with pytest.raises(ValueError):
        runtime.step(bad)
    assert runtime.snapshot() == before
    np.testing.assert_array_equal(runtime.controller.q_target, target)


def test_invalid_action_is_atomic(runtime):
    runtime.reset()
    before = runtime.controller.q_target.copy()
    with pytest.raises(ValueError, match="action"):
        runtime.apply_action([0] * 13 + [float("inf")])
    np.testing.assert_array_equal(runtime.controller.q_target, before)
    np.testing.assert_array_equal(runtime.last_action, np.zeros(14))


def test_same_tensor_shape_with_wrong_joint_order_rejected(runtime):
    real = runtime.sessions["walking"]

    class WrongMetadata:
        def get_inputs(self):
            return real.get_inputs()

        def get_outputs(self):
            return real.get_outputs()

        def get_modelmeta(self):
            metadata = real.get_modelmeta()
            from types import SimpleNamespace

            fields = dict(metadata.custom_metadata_map)
            fields["joint_names"] = ",".join(reversed(fields["joint_names"].split(",")))
            return SimpleNamespace(custom_metadata_map=fields)

    with pytest.raises(ValueError, match="semantic"):
        validate_policy(WrongMetadata())


def test_nonfinite_physics_fails_before_applying_action(runtime):
    runtime.reset()
    runtime.data.qpos[0] = np.nan
    target = runtime.controller.q_target.copy()
    with pytest.raises(ValueError):
        runtime.step()
    np.testing.assert_array_equal(runtime.controller.q_target, target)
    assert runtime.data.time == 0


def test_missing_cache_cli_has_nonzero_exit(tmp_path):
    import subprocess

    result = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).resolve().parents[1] / "smoke.py"),
            "--cache",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "missing asset" in result.stderr
    assert "prepare" in result.stderr
    assert "smoke_passed" not in result.stdout
