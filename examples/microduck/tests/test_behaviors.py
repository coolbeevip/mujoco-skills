"""真实策略执行与动作切换测试；时序测试单独记录输入，不以运行完毕代替断言。"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from behaviors import Behaviors
from policies import POLICIES
from policy_demo import demo
from runtime import POSE, Runtime, validate_policy
from sequence import Failure, Sample


@pytest.fixture
def runtime():
    return Runtime()


def recorder(runtime):
    behavior = Behaviors(runtime)
    calls = []

    def step(policy, command=(0, 0, 0), observer=None):
        calls.append((policy, tuple(command)))
        return {}

    runtime.step_policy = step
    return behavior, calls


@pytest.mark.parametrize("policy", POLICIES)
def test_every_official_policy_executes_and_recovers_in_real_physics(policy):
    report = demo(policy)
    assert report["status"] == "executed", report
    assert report["behavior"]["stage"] == "idle"
    assert report["final_state"]["physics_steps"] > 1000
    assert report["action_success_verified"] is False
    if policy not in ("standing", "walking", "roller"):
        assert report["behavior"]["completed_windows"] == [policy]


@pytest.mark.parametrize(
    "key,policy,count,period",
    [
        ("g", "ground_pick", 140, 4.0),
        ("k", "kick_left", 25, None),
        ("l", "kick_right", 25, None),
        ("r", "roulade", 50, None),
        ("c", "crouch", 175, 5.0),
    ],
)
def test_exact_duration_encoding_and_return_to_base(key, policy, count, period):
    r = Runtime(mode="roller" if policy == "crouch" else "walk")
    b, calls = recorder(r)
    assert b.handle(key) == f"started: {policy}"
    for _ in range(count):
        b.step()
    assert len(calls) == count
    assert all(p == policy for p, _ in calls)
    for index, (_, command) in enumerate(calls):
        expected = (
            (0, 0, 0)
            if period is None
            else (
                math.cos(math.tau * index * 0.02 / period),
                math.sin(math.tau * index * 0.02 / period),
                0,
            )
        )
        np.testing.assert_allclose(command, expected, rtol=0, atol=1e-14)
    assert b.active is None and b.recovery == 100
    b.step()
    assert calls[-1] == ("roller" if policy == "crouch" else "standing", (0, 0, 0))
    assert b.recovery == 99


def test_sit_hold_and_rise_use_flags_not_velocity(runtime):
    b, calls = recorder(runtime)
    assert b.handle("y") == "started: sitstand"
    assert "ignored" in b.handle("y")
    for _ in range(150):
        b.step()
    assert b.stage == "seated" and b.active == "sitstand"
    assert all(c == ("sitstand", (1, 0, 0)) for c in calls)
    assert b.handle("y") == "standing up"
    for _ in range(50):
        b.step()
    assert all(c == ("sitstand", (0, 0, 0)) for c in calls[-50:])
    assert b.recovery == 100


def test_busy_keys_do_not_restart_queue_or_chain(runtime):
    b, _ = recorder(runtime)
    b.handle("w")
    assert "stop" in b.handle("r")
    b.handle("s")
    b.handle("r")
    b.step()
    for key in ("r", "k", "y", "w"):
        assert "ignored" in b.handle(key)
    assert b.elapsed == 1 and b.active == "roulade"
    assert "continues" in b.handle(" ")
    assert b.active == "roulade" and b.command == (0, 0, 0)


def test_model_mismatch_and_bad_commands_are_atomic(runtime):
    before = runtime.snapshot()
    for policy, command in [
        ("roller", (0, 0, 0)),
        ("sitstand", (0.5, 0, 0)),
        ("ground_pick", (0, 0, 0)),
        ("kick_left", (0.3, 0, 0)),
    ]:
        with pytest.raises(ValueError):
            runtime.step_policy(policy, command)
        assert runtime.snapshot() == before
    assert "unavailable" in Behaviors(runtime).handle("c")


def test_roller_model_scale_and_heading_contract():
    r = Runtime(mode="roller")
    assert r.model.nq == 25 and r.model.nu == 14
    assert set(r.sessions) == {"roller", "crouch"}
    assert r.data.qpos[2] == pytest.approx(0.1385)
    b = Behaviors(r)
    assert b.handle("a") == "command updated"
    assert b.command == (0, 0, 0.5)
    assert "unavailable" in b.handle("r")
    with pytest.raises(ValueError):
        r.step((0, 0, 1.2))
    r.step_policy("roller")
    np.testing.assert_array_equal(r.controller.q_target, POSE + r.last_action * 0.8)
    for j in range(r.model.njnt):
        if r.model.joint(j).name.startswith("passive_"):
            assert r.model.dof_frictionloss[r.model.jnt_dofadr[j]] == pytest.approx(
                0.003
            )


def test_extra_policy_metadata_is_checked_per_file(runtime):
    real = runtime.sessions["ground_pick"]
    with pytest.raises(ValueError, match="semantic"):
        validate_policy(real)
    validate_policy(real, "twist")


def test_contact_allowed_only_inside_requested_action(runtime):
    b = Behaviors(runtime)
    b.handle("r")
    b.measurement.sample = lambda r: Sample(r.physics_steps, 0, 0, 0, 2.0, "head")
    runtime.step_policy("roulade", observer=b.observe)
    assert b.allowed_contact_steps == 4
    b.active = None
    with pytest.raises(Failure, match="ground contact"):
        runtime.step(observer=b.observe)
    assert runtime.failed


def test_recovery_has_a_fixed_deadline_and_cannot_be_retriggered(runtime):
    b = Behaviors(runtime)
    b._finish("roulade")
    assert "ignored" in b.handle("r")
    b.recovery = 1
    b.measurement.sample = lambda r: Sample(r.physics_steps, 0, 0, 0, 2.0, "head")
    with pytest.raises(Failure, match="recovery"):
        b.step()
    assert runtime.failed


def test_invalid_measurement_during_action_still_fails(runtime):
    b = Behaviors(runtime)
    b.handle("g")
    b.measurement.sample = lambda r: Sample(r.physics_steps, 0, 0, 0, float("nan"))
    with pytest.raises(Failure, match="invalid"):
        b.step()
    assert runtime.failed


def test_kick_only_repositions_ball_not_robot():
    r = Runtime(ball=True)
    b = Behaviors(r)
    before = r.data.qpos.copy()
    assert b.handle("k") == "started: kick_left"
    q = int(r.model.joint("ball_free").qposadr[0])
    np.testing.assert_array_equal(r.data.qpos[:q], before[:q])
    np.testing.assert_allclose(r.data.qpos[q : q + 3], [0.09, 0.042, 0.035])
    assert r.physics_steps == 0


def test_display_and_behavior_path_match_recorded_walk_trace():
    import hashlib
    import json
    import platform
    from keyboard import Display

    if platform.system() != "Darwin" or platform.machine() != "arm64":
        pytest.skip("recorded trajectory is specific to macOS arm64")
    evidence = json.loads(
        (Path(__file__).resolve().parents[1] / "keyboard-verification.json").read_text()
    )
    r = Runtime()
    b = Behaviors(r)
    display = Display(r)
    events = {event["control_step"]: event["key"] for event in evidence["events"]}
    trace = hashlib.sha256()
    for tick in range(evidence["final"]["control_steps"]):
        if tick in events:
            b.handle(events[tick])
        b.step()
        display.copy_from(r)
        trace.update(r.data.qpos.tobytes() + r.data.qvel.tobytes())
    assert trace.hexdigest() == evidence["trace_sha256"]


@pytest.mark.parametrize("mode", ["walk", "roller"])
def test_new_gui_actions_replay_identically_without_viewer(mode):
    import hashlib
    import json
    import platform

    if platform.system() != "Darwin" or platform.machine() != "arm64":
        pytest.skip("recorded trajectory is specific to macOS arm64")
    evidence = json.loads(
        (
            Path(__file__).resolve().parents[1] / "policy-keyboard-verification.json"
        ).read_text()
    )[mode]
    r = Runtime(mode=mode, ball=evidence["ball"])
    b = Behaviors(r)
    events = {event["control_step"]: event for event in evidence["events"]}
    trace = hashlib.sha256()
    for tick in range(evidence["final"]["control_steps"]):
        if tick in events:
            event = events[tick]
            assert b.handle(event["key"]) == event["response"]
        b.step()
        trace.update(r.data.qpos.tobytes() + r.data.qvel.tobytes())
    assert trace.hexdigest() == evidence["trace_sha256"]
    assert b.status() == evidence["behavior"]
    np.testing.assert_array_equal(r.data.qpos, evidence["final"]["qpos"])
