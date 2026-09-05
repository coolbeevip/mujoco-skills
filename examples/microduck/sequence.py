"""固定动作序列的逐物理步测量与失败判定。"""

import math
from dataclasses import dataclass
import argparse
import json
import platform
import sys
from importlib.metadata import version

from assets import DEFAULT_CACHE, lock
from runtime import DT, Runtime


@dataclass
class Sample:
    step: int
    x: float
    y: float
    yaw: float
    tilt: float
    contact: str | None = None


class Failure(ValueError):
    pass


class Monitor:
    def __init__(self, initial):
        self.initial = initial
        self.previous = initial
        self.yaw = initial.yaw
        self.reason = None
        self.tilt_start = initial.step if initial.tilt > math.pi / 4 else None
        self._check(initial)

    def fail(self, reason):
        self.reason = self.reason or reason
        raise Failure(self.reason)

    def _check(self, sample):
        if not all(
            math.isfinite(v) for v in (sample.x, sample.y, sample.yaw, sample.tilt)
        ):
            self.fail("invalid measurement")
        if sample.contact:
            self.fail(f"{sample.contact} ground contact")

    def update(self, sample):
        if self.reason:
            self.fail(self.reason)
        self._check(sample)
        if sample.step != self.previous.step + 1:
            self.fail("missing physics sample")
        delta = math.atan2(
            math.sin(sample.yaw - self.previous.yaw),
            math.cos(sample.yaw - self.previous.yaw),
        )
        self.yaw += delta
        if sample.tilt > math.pi / 4:
            if self.tilt_start is None:
                self.tilt_start = sample.step
            if sample.step - self.tilt_start >= 40:
                self.fail("torso tilt >45deg for >=0.2s")
        else:
            self.tilt_start = None
        self.previous = sample
        return self.yaw


class Measurement:
    def __init__(self, runtime):
        model = runtime.model
        self.root = model.body("trunk_base").id
        self.floor = model.geom("floor").id
        self.feet = {
            model.geom(n).id for n in ("left_foot_collision", "right_foot_collision")
        }
        self.parts = {}
        for body, label in [("trunk_base", "torso"), ("jaw_soft", "head")]:
            bid = model.body(body).id
            ids = {
                i
                for i in range(model.ngeom)
                if model.geom_bodyid[i] == bid
                and (
                    (model.geom_contype[i] & model.geom_conaffinity[self.floor])
                    or (model.geom_conaffinity[i] & model.geom_contype[self.floor])
                )
            }
            if not ids:
                raise ValueError(f"missing {label} ground collision capability")
            self.parts.update({i: label for i in ids})

    def sample(self, runtime):
        data = runtime.data
        rotation = data.xmat[self.root].reshape(3, 3)
        if not all(math.isfinite(float(v)) for v in rotation.flat):
            raise Failure("invalid torso rotation")
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        tilt = math.acos(max(-1.0, min(1.0, float(rotation[2, 2]))))
        contact = None
        for c in data.contact:
            if c.dist <= 0 and self.floor in (c.geom1, c.geom2):
                other = int(c.geom2 if c.geom1 == self.floor else c.geom1)
                if other in self.parts:
                    contact = self.parts[other]
                    break
        x, y = data.xpos[self.root, :2]
        return Sample(runtime.physics_steps, float(x), float(y), yaw, tilt, contact)


class Sequence:
    STAGES = ("initial_stand", "forward", "left_turn", "decelerate", "final_stand")

    def __init__(self, runtime):
        self.runtime = runtime
        self.measurement = Measurement(runtime)
        self.current = self.measurement.sample(runtime)
        self.monitor = Monitor(self.current)
        self.records = []
        self.index = 0
        self.result = None
        self._begin()

    def _begin(self):
        self.start = self.current
        self.start_yaw = self.monitor.yaw
        self.metrics = dict(
            duration_s=0.0,
            forward_m=0.0,
            lateral_m=0.0,
            turn_deg=0.0,
            max_displacement_m=0.0,
            max_yaw_deviation_deg=0.0,
        )

    def observe(self, runtime):
        self.current = self.measurement.sample(runtime)
        yaw = self.monitor.update(self.current)
        dx, dy = self.current.x - self.start.x, self.current.y - self.start.y
        heading = self.start_yaw
        angle = math.degrees(yaw - self.start_yaw)
        self.metrics.update(
            duration_s=(self.current.step - self.start.step) * DT,
            forward_m=dx * math.cos(heading) + dy * math.sin(heading),
            lateral_m=-dx * math.sin(heading) + dy * math.cos(heading),
            turn_deg=angle,
        )
        self.metrics["max_displacement_m"] = max(
            self.metrics["max_displacement_m"], math.hypot(dx, dy)
        )
        self.metrics["max_yaw_deviation_deg"] = max(
            self.metrics["max_yaw_deviation_deg"], abs(angle)
        )
        if self.index in (0, 4):
            if self.metrics["max_displacement_m"] > 0.05:
                self.monitor.fail("standing displacement >0.05m")
            if self.metrics["max_yaw_deviation_deg"] > 10:
                self.monitor.fail("standing yaw deviation >10deg")

    def run(self):
        if self.result is not None:
            return self.result
        try:
            while self.index < len(self.STAGES):
                command = (
                    (0.3, 0, 0)
                    if self.index == 1
                    else (0, 0, 1.2)
                    if self.index == 2
                    else (0, 0, 0)
                )
                self.runtime.step(command, observer=self.observe)
                elapsed = self.current.step - self.start.step
                if self.index in (1, 2):
                    reached = (
                        self.metrics["forward_m"] >= 0.3
                        if self.index == 1
                        else self.metrics["turn_deg"] >= 90
                    )
                    if elapsed > 2000 or (elapsed == 2000 and not reached):
                        self.monitor.fail("stage timeout >10s budget")
                    complete = reached
                    if complete:
                        value = (
                            self.metrics["forward_m"]
                            if self.index == 1
                            else self.metrics["turn_deg"]
                        )
                        low, high = (0.2, 0.4) if self.index == 1 else (75, 105)
                        if not low <= value <= high:
                            self.monitor.fail("handoff outside target tolerance")
                else:
                    complete = elapsed >= (600 if self.index == 3 else 1000)
                if complete:
                    self.records.append(self.record("passed"))
                    self.index += 1
                    if self.index < len(self.STAGES):
                        self._begin()
            self.result = dict(status="passed", stages=self.records)
        except (ValueError, RuntimeError, KeyboardInterrupt) as error:
            self.records.append(self.record("failed"))
            self.result = dict(
                status="interrupted"
                if isinstance(error, KeyboardInterrupt)
                else "failed",
                reason=str(error) or "user interrupt",
                stages=self.records,
                failure_physics_step=self.runtime.physics_steps,
            )
        return self.result

    def record(self, status):
        position = [self.current.x, self.current.y]
        valid = all(
            math.isfinite(v) for v in (*position, self.current.yaw, self.current.tilt)
        )
        return dict(
            stage=self.STAGES[self.index],
            status=status,
            start_step=self.start.step,
            end_step=self.current.step,
            start_position_m=[self.start.x, self.start.y],
            end_position_m=position if valid else None,
            start_yaw_deg=math.degrees(self.start_yaw),
            measurement_valid=valid,
            **{
                **self.metrics,
                "duration_s": (self.runtime.physics_steps - self.start.step) * DT,
            },
        )


def run_group(cache=DEFAULT_CACHE):
    report = dict(
        status="failed",
        platform=platform.platform(),
        python=platform.python_version(),
        seed=0,
        model_revision=lock()["model_revision"],
        policy_revision=lock()["policy_revision"],
        bam_revision=lock()["bam_revision"],
        dependencies={
            name: version(name)
            for name in ("numpy", "mujoco", "onnxruntime", "better-actuator-models")
        },
        runs=[],
    )
    try:
        runtime = Runtime(cache, seed=0)
        for _ in range(3):
            runtime.reset()
            initial = runtime.snapshot()
            result = Sequence(runtime).run()
            result["initial_state"] = initial
            report["runs"].append(result)
            if result["status"] != "passed":
                report["status"] = result["status"]
                report["runs"].extend(
                    [{"status": "not_executed"} for _ in range(3 - len(report["runs"]))]
                )
                return report
        report["status"] = "passed"
    except (ValueError, RuntimeError, OSError, KeyboardInterrupt) as error:
        report["reason"] = str(error) or "user interrupt"
        if isinstance(error, KeyboardInterrupt):
            report["status"] = "interrupted"
        report["runs"].extend(
            [{"status": "not_executed"} for _ in range(3 - len(report["runs"]))]
        )
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", default=DEFAULT_CACHE)
    args = parser.parse_args()
    result = run_group(args.cache)
    print(json.dumps(result, indent=2, allow_nan=False))
    sys.exit(0 if result["status"] == "passed" else 2)
