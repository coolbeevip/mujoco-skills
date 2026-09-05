"""把动作请求转换为逐控制步的策略选择；所有计时使用仿真步数。"""

import math

import mujoco

from runtime import DECIMATION, DT
from sequence import Measurement, Monitor, Sample

CONTROL_DT = DT * DECIMATION
ACTION_KEYS = {
    "y": "sitstand",
    "g": "ground_pick",
    "k": "kick_left",
    "l": "kick_right",
    "r": "roulade",
    "c": "crouch",
}
WALK_COMMANDS = {
    "w": (0.3, 0, 0),
    "a": (0, 0, 1.2),
    "d": (0, 0, -1.2),
    "s": (0, 0, 0),
    " ": (0, 0, 0),
}
ROLLER_COMMANDS = {**WALK_COMMANDS, "a": (0, 0, 0.5), "d": (0, 0, -0.5)}


class Behaviors:
    def __init__(self, runtime):
        self.runtime = runtime
        self.command = (0, 0, 0)
        self.active = None
        self.stage = "idle"
        self.elapsed = 0
        self.recovery = 0
        self.completed = []
        self.measurement = Measurement(runtime)
        self.sample = self.measurement.sample(runtime)
        self.monitor = Monitor(self.sample)
        self.allowed_contact_steps = 0
        self.max_tilt_deg = 0.0

    @property
    def commands(self):
        return ROLLER_COMMANDS if self.runtime.mode == "roller" else WALK_COMMANDS

    def handle(self, key):
        # 动作由一次按键触发；动作执行期间不排队、不重启，也不自动连续翻滚。
        # 空格清除运动目标，但让正在执行的一次性动作完成；q 由入口立即退出。
        if key in self.commands:
            if self.active or self.recovery:
                if key in ("s", " "):
                    self.command = (0, 0, 0)
                    return "movement cleared; current action continues"
                return "ignored: action/recovery in progress"
            self.command = self.commands[key]
            return "command updated"
        if key not in ACTION_KEYS:
            return "ignored: unknown key"
        policy = ACTION_KEYS[key]
        if policy not in self.runtime.sessions:
            return f"ignored: {policy} unavailable in {self.runtime.mode} mode"
        if self.active == "sitstand" and self.stage == "seated" and key == "y":
            self.stage, self.elapsed = "rising", 0
            return "standing up"
        if self.active or self.recovery:
            return "ignored: action/recovery in progress"
        if any(self.command):
            return "ignored: stop with space/s before starting an action"
        # 零指令并不保证已经停止运动。确认躯干直立且速度较低后才启动动作。
        sample = self.measurement.sample(self.runtime)
        speed = math.hypot(*self.runtime.data.qvel[:2])
        if sample.contact or sample.tilt > math.radians(20) or speed > 0.15:
            return "ignored: wait until upright and settled"
        if policy in ("kick_left", "kick_right") and self.runtime.ball:
            self._place_ball(policy)
        self.active = policy
        self.stage = "sitting" if policy == "sitstand" else "executing"
        self.elapsed = 0
        return f"started: {policy}"

    def _place_ball(self, policy):
        # 踢球前仅重置球的位置和速度，不修改机器人位姿。偏移与官方推理入口一致。
        r = self.runtime
        yaw = self.sample.yaw
        side = 0.042 if policy == "kick_left" else -0.042
        x, y = r.data.xpos[r.root_id, :2]
        joint = r.model.joint("ball_free")
        q, v = int(joint.qposadr[0]), int(joint.dofadr[0])
        r.data.qpos[q : q + 7] = [
            x + 0.09 * math.cos(yaw) - side * math.sin(yaw),
            y + 0.09 * math.sin(yaw) + side * math.cos(yaw),
            0.035,
            1,
            0,
            0,
            0,
        ]
        r.data.qvel[v : v + 6] = 0
        mujoco.mj_forward(r.model, r.data)

    def observe(self, runtime):
        self.sample = self.measurement.sample(runtime)
        self.max_tilt_deg = max(self.max_tilt_deg, math.degrees(self.sample.tilt))
        # 翻滚、俯身、坐下包含主动倾斜或触地，不能套用直立行走的跌倒规则。
        # 只在请求的动作及固定恢复窗口内允许；原始接触/倾角仍记录在报告中。
        # NaN、物理警告、缺失采样仍立即失败，窗口结束后恢复严格跌倒检测。
        if self.active or self.recovery:
            if not math.isfinite(self.sample.tilt):
                self.monitor.fail("invalid measurement")
            self.allowed_contact_steps += bool(self.sample.contact)
            guarded = Sample(
                self.sample.step, self.sample.x, self.sample.y, self.sample.yaw, 0.0
            )
        else:
            guarded = self.sample
        self.monitor.update(guarded)

    def step(self):
        policy = self.active
        if policy is None:
            result = self.runtime.step(self.command, observer=self.observe)
            if self.recovery:
                self.recovery -= 1
                if not self.recovery:
                    # 恢复窗口不是成功判据；到期仍触地或倾斜过大则停止。
                    if self.sample.contact or self.sample.tilt > math.pi / 4:
                        self.runtime.failed = True
                        self.monitor.fail("action recovery did not return upright")
                    self.stage = "idle"
            return result
        entry = self.runtime.catalog[policy]
        command = (0, 0, 0)
        if policy == "sitstand":
            # 这里的 1 是「坐下」标志，不是 1 m/s；站起则给同一网络输入 0。
            command = (0 if self.stage == "rising" else 1, 0, 0)
        elif "period_s" in entry.get("command", {}):
            # ground_pick/crouch 使用 cos、sin 表示动作阶段。
            # elapsed 是控制步数，调试暂停或机器变慢不会跳过任何动作阶段。
            phase = self.elapsed * CONTROL_DT / entry["command"]["period_s"]
            command = (math.cos(math.tau * phase), math.sin(math.tau * phase), 0)
        result = self.runtime.step_policy(policy, command, observer=self.observe)
        self.elapsed += 1
        if policy == "sitstand":
            if self.stage == "sitting" and self.elapsed >= round(
                entry["ramp_s"] / CONTROL_DT
            ):
                self.stage = "seated"
            elif self.stage == "rising" and self.elapsed >= round(
                entry["unwind_s"] / CONTROL_DT
            ):
                self._finish(policy)
        elif self.elapsed >= round(entry["duration_s"] / CONTROL_DT):
            self._finish(policy)
        return result

    def _finish(self, policy):
        # 网络执行到规定时长后交还基础策略，并给 2 s 恢复窗口。
        # completed 只表示时序执行完毕，不宣称已成功拾取、踢中或完成翻滚。
        self.completed.append(policy)
        self.active = None
        self.command = (0, 0, 0)
        self.recovery = round(2.0 / CONTROL_DT)
        self.stage = "recovering"

    def status(self):
        return {
            "mode": self.runtime.mode,
            "active": self.active,
            "stage": self.stage,
            "action_control_steps": self.elapsed,
            "recovery_control_steps": self.recovery,
            "completed_windows": list(self.completed),
            "action_success_verified": False,
            "allowed_contact_steps": self.allowed_contact_steps,
            "max_tilt_deg": self.max_tilt_deg,
        }
