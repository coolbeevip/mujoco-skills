import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sequence import Failure, Monitor, Sample, Measurement, Sequence
from runtime import Runtime
import mujoco
import sequence as sequence_module


def sample(step, **kw):
    return Sample(
        step=step,
        x=kw.get("x", 0),
        y=kw.get("y", 0),
        yaw=kw.get("yaw", 0),
        tilt=kw.get("tilt", 0),
        contact=kw.get("contact"),
    )


def test_contact_fails_immediately_and_latches():
    monitor = Monitor(sample(0))
    with pytest.raises(Failure, match="contact"):
        monitor.update(sample(1, contact="head"))
    with pytest.raises(Failure):
        monitor.update(sample(2))


def test_tilt_requires_40_intervals_since_first_high_sample():
    monitor = Monitor(sample(0))
    for step in range(1, 41):
        monitor.update(sample(step, tilt=math.radians(46)))
    with pytest.raises(Failure, match="tilt"):
        monitor.update(sample(41, tilt=math.radians(46)))


def test_yaw_wrap_is_continuous():
    monitor = Monitor(sample(0, yaw=math.radians(179)))
    yaw = monitor.update(sample(1, yaw=math.radians(-179)))
    assert math.degrees(yaw) == pytest.approx(181)


def test_tilt_at_45_resets_continuous_timer():
    monitor = Monitor(sample(0))
    for step in range(1, 31):
        monitor.update(sample(step, tilt=math.radians(46)))
    monitor.update(sample(31, tilt=math.radians(45)))
    for step in range(32, 72):
        monitor.update(sample(step, tilt=math.radians(46)))
    with pytest.raises(Failure):
        monitor.update(sample(72, tilt=math.radians(46)))


@pytest.fixture(scope="module")
def robot():
    return Runtime()


@pytest.mark.parametrize(
    "height,quat,expected",
    [
        (0.115, [1, 0, 0, 0], None),
        (0.02, [1, 0, 0, 0], "torso"),
        (0.1, [0, 1, 0, 0], "head"),
    ],
)
def test_real_model_floor_contacts(robot, height, quat, expected):
    robot.reset()
    detector = Measurement(robot)
    robot.data.qpos[2] = height
    robot.data.qpos[3:7] = quat
    mujoco.mj_forward(robot.model, robot.data)
    assert robot.data.ncon > 0
    assert detector.sample(robot).contact == expected


def test_missing_head_collision_capability_is_rejected(robot):
    robot.reset()
    detector = Measurement(robot)
    for gid, label in detector.parts.items():
        if label == "head":
            robot.model.geom_contype[gid] = 0
            robot.model.geom_conaffinity[gid] = 0
    with pytest.raises(ValueError, match="missing head"):
        Measurement(robot)


@pytest.mark.parametrize(
    "x,yaw,valid",
    [
        (0.05, 0, True),
        (0.050001, 0, False),
        (0, math.radians(10), True),
        (0, math.radians(10.001), False),
    ],
)
def test_standing_window_boundaries_and_latch(robot, monkeypatch, x, yaw, valid):
    robot.reset()
    sequence = Sequence(robot)
    monkeypatch.setattr(
        sequence.measurement, "sample", lambda r: sample(1, x=x, yaw=yaw)
    )
    if valid:
        sequence.observe(robot)
    else:
        with pytest.raises(Failure):
            sequence.observe(robot)
        monkeypatch.setattr(sequence.measurement, "sample", lambda r: sample(2))
        with pytest.raises(Failure):
            sequence.observe(robot)


def test_physics_observer_runs_at_each_substep(robot):
    robot.reset()
    seen = []
    robot.step(observer=lambda r: seen.append(r.physics_steps))
    assert seen == [1, 2, 3, 4]


def test_observer_failure_stops_remaining_substeps(robot):
    robot.reset()

    def observer(r):
        raise Failure("contact")

    with pytest.raises(Failure):
        robot.step(observer=observer)
    assert robot.physics_steps == 1
    assert robot.data.time == pytest.approx(0.005)
    with pytest.raises(ValueError, match="reset"):
        robot.step()


class ControlledPhysics:
    """仅用于阶段时间/整组控制的受控状态源，不作为物理验收证据。"""

    def __init__(self, *args, **kwargs):
        self.physics_steps = 0

    def reset(self):
        self.physics_steps = 0

    def snapshot(self):
        return {"physics_steps": self.physics_steps}

    def step(self, command, observer):
        for _ in range(4):
            self.physics_steps += 1
            observer(self)


def install_controlled_measurements(monkeypatch, target_step):
    class ControlledMeasurement:
        def __init__(self, runtime):
            pass

        def sample(self, runtime):
            step = runtime.physics_steps
            return sample(
                step,
                x=0.3 if step >= target_step else 0,
                yaw=math.pi / 2 if step >= 3004 else 0,
            )

    monkeypatch.setattr(sequence_module, "Measurement", ControlledMeasurement)


@pytest.mark.parametrize("target_step,passed", [(3000, True), (3001, False)])
def test_forward_deadline_boundary(monkeypatch, target_step, passed):
    install_controlled_measurements(monkeypatch, target_step)
    result = Sequence(ControlledPhysics()).run()
    assert (result["status"] == "passed") is passed
    if passed:
        assert result["stages"][1]["duration_s"] == 10
        assert len(result["stages"]) == 5
    else:
        assert "timeout" in result["reason"]
        assert len(result["stages"]) == 2


def test_failed_group_keeps_failure_and_marks_remaining_runs(monkeypatch):
    install_controlled_measurements(monkeypatch, 99999)
    monkeypatch.setattr(sequence_module, "Runtime", ControlledPhysics)
    report = sequence_module.run_group()
    assert report["status"] == "failed"
    assert [r["status"] for r in report["runs"]] == [
        "failed",
        "not_executed",
        "not_executed",
    ]
    assert "timeout" in report["runs"][0]["reason"]


def test_interrupt_does_not_complete_sequence(robot, monkeypatch):
    robot.reset()

    def interrupt(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(robot, "step", interrupt)
    result = Sequence(robot).run()
    assert result["status"] == "interrupted"
    assert len(result["stages"]) == 1


def test_forward_projection_uses_initial_heading(robot, monkeypatch):
    robot.reset()
    detector = Sequence(robot)
    # 注入朝向为 +Y 的前进阶段，世界 X 负向为左侧漂移。
    detector.index = 1
    detector.monitor = Monitor(sample(0, yaw=math.pi / 2))
    detector.current = detector.monitor.previous
    detector._begin()
    monkeypatch.setattr(
        detector.measurement,
        "sample",
        lambda r: sample(1, x=-0.1, y=0.3, yaw=math.pi / 2),
    )
    detector.observe(robot)
    assert detector.metrics["forward_m"] == pytest.approx(0.3)
    assert detector.metrics["lateral_m"] == pytest.approx(0.1)


def test_nonfinite_measurement_produces_json_failure_without_restart(monkeypatch):
    import json

    class InvalidMeasurement:
        def __init__(self, runtime):
            pass

        def sample(self, runtime):
            return sample(
                runtime.physics_steps, x=math.nan if runtime.physics_steps else 0
            )

    monkeypatch.setattr(sequence_module, "Measurement", InvalidMeasurement)
    physics = ControlledPhysics()
    sequence = Sequence(physics)
    result = sequence.run()
    assert result["status"] == "failed"
    assert "invalid" in result["reason"]
    assert result["stages"][0]["end_position_m"] is None
    json.dumps(result, allow_nan=False)
    assert sequence.run() == result
    assert physics.physics_steps == 1


def test_passive_observer_does_not_change_physics(robot):
    import numpy as np

    robot.reset()
    for _ in range(50):
        robot.step()
    without = robot.data.qpos.copy()
    robot.reset()
    for _ in range(50):
        robot.step(observer=lambda r: None)
    np.testing.assert_array_equal(robot.data.qpos, without)


def test_sequence_cli_initialization_error_is_json_and_nonzero(tmp_path):
    import json
    import subprocess

    result = subprocess.run(
        [
            sys.executable,
            str(Path(__file__).resolve().parents[1] / "sequence.py"),
            "--cache",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    report = json.loads(result.stdout)
    assert report["status"] == "failed"
    assert "missing asset" in report["reason"]
    assert [r["status"] for r in report["runs"]] == ["not_executed"] * 3
