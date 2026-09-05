"""视觉任务的状态机：模型给出动作目标，控制器根据实际运动反馈执行。"""

import json
import logging
import math
import time
from concurrent.futures import Future
from dataclasses import dataclass, replace
from motion import Motion

MOVES = {"forward", "left", "right", "wait"}
HEAD_ACTIONS = {"head_down", "head_up", "head_reset"}
SKILLS = {
    "sit",
    "stand",
    "ground_pick",
    "kick_left",
    "kick_right",
    "roulade",
    "crouch",
    "stop",
} | HEAD_ACTIONS


@dataclass(frozen=True)
class Decision:
    action: str
    amount: float
    target_visible: bool
    confidence: float
    evidence: str

    @classmethod
    def parse(cls, value):
        # 不宽松修复模型输出：拼错动作、超长运动或不完整响应都停止本次任务。
        # 特别是不能把任意文本当 Python/shell 代码执行。
        if isinstance(value, str):
            value = json.loads(value)
        if not isinstance(value, dict) or set(value) != {
            "action",
            "amount",
            "target_visible",
            "confidence",
            "evidence",
        }:
            raise ValueError("模型响应字段不符合动作合同")
        if not isinstance(value["action"], str) or value[
            "action"
        ] not in MOVES | SKILLS | {
            "done",
            "abort",
        }:
            raise ValueError("模型请求了未授权动作")
        for key in ("amount", "confidence"):
            if type(value[key]) not in (int, float) or not math.isfinite(value[key]):
                raise ValueError(f"{key} 必须是有限数值")
        if not 0 <= value["confidence"] <= 1:
            raise ValueError("confidence 超出范围")
        if type(value["target_visible"]) is not bool:
            raise ValueError("target_visible 必须是布尔值")
        if (
            not isinstance(value["evidence"], str)
            or not 1 <= len(value["evidence"]) <= 1000
        ):
            raise ValueError("模型必须提供简短可见证据")
        limits = {
            "forward": (5, 50, "厘米"),
            "left": (10, 60, "度"),
            "right": (10, 60, "度"),
            "wait": (0.2, 1, "秒"),
        }
        if value["action"] in limits:
            low, high, unit = limits[value["action"]]
            if not low <= value["amount"] <= high:
                raise ValueError(f"{value['action']} 每次只能执行 {low}～{high} {unit}")
        elif value["amount"] != 0:
            raise ValueError("结束指令的 amount 必须为零")
        if value["action"] == "done" and (
            not value["target_visible"] or value["confidence"] < 0.8
        ):
            raise ValueError("缺少可见目标或置信度不足，不能判定完成")
        return cls(**value)


class Navigation:
    """调用者在主线程调用 tick；submit(task, png, history) 返回异步 Future。

    等待模型时冻结仿真时间，避免网络延迟变成额外移动距离。获得决策后只执行
    实际转角或位移目标，停步复核后再拍新图。连续两次独立观察均判定 done
    才结束；这是视觉判断结果，不是假设物理上一定达到某个距离阈值。
    """

    def __init__(
        self, task, submit, *, max_decisions=40, timeout_s=300, clock=time.monotonic
    ):
        if not isinstance(task, str) or not 1 <= len(task.strip()) <= 1000:
            raise ValueError("任务必须是 1～1000 字的自然语言描述")
        if type(max_decisions) is not int or not 1 <= max_decisions <= 100:
            raise ValueError("决策上限必须是 1～100")
        if (
            type(timeout_s) not in (int, float)
            or not math.isfinite(timeout_s)
            or not 1 <= timeout_s <= 600
        ):
            raise ValueError("任务时间上限必须是 1～600 秒")
        self.task, self.submit, self.clock = task.strip(), submit, clock
        self.max_decisions, self.deadline = max_decisions, clock() + timeout_s
        self.phase = "settling"
        self.remaining = 50
        self.pending = None
        self.history = []
        self.confirmations = 0
        self.reason = "先站稳，再读取头部图像"
        self.observations = 0
        self.motion = None
        self.skill_steps = 0
        self.proximity = None

    @property
    def active(self):
        return self.phase not in {"completed", "cancelled", "failed"}

    def cancel(self, scene, reason="用户中止", *, phase="cancelled"):
        if self.pending is not None:
            # 已经发出的网络请求可能无法撤回，但迟到的响应永远不会再触发动作。
            self.pending.cancel()
            self.pending = None
        if self.motion and self.motion.status == "running":
            try:
                self.motion.measure(scene.motion_pose())
            except ValueError:
                pass  # 反馈损坏时保留最后一次有效测量，仍然执行停止。
            self.motion.status = "cancelled"
            self.history[-1]["execution"] = self.motion.result()
        elif self.phase in {"waiting", "skill", "head"} and self.history:
            self.history[-1]["execution"] = dict(status="cancelled")
        scene.stop()
        self.phase, self.reason = phase, reason
        logging.getLogger("web_control.navigation").info(
            "任务停止 phase=%s observations=%s reason=%s",
            self.phase,
            self.observations,
            reason,
        )

    def fail(self, scene, reason):
        self.cancel(scene, reason, phase="failed")

    def tick(self, scene):
        if not self.active:
            return
        try:
            if self.clock() >= self.deadline:
                raise ValueError("任务超时，已停止")
            if self.phase == "head":
                scene.navigation_hold()
                self.skill_steps += 1
                self.history[-1]["execution"] = dict(
                    status="reached" if self.skill_steps >= 150 else "running",
                    elapsed_s=round(self.skill_steps * 0.02, 2),
                    **scene.head_feedback(),
                )
                if self.skill_steps >= 150:
                    self.phase = "observing"
                return
            if self.phase == "skill":
                scene.step()
                self.skill_steps += 1
                state = scene.state()
                seated = (
                    state.get("active") == "sitstand" and state.get("stage") == "seated"
                )
                finished = (
                    self.history[-1]["action"] in {"sit", "stop"} and seated
                ) or (
                    not state.get("active")
                    and not state.get("recovery_control_steps", 0)
                )
                self.history[-1]["execution"] = dict(
                    status="reached" if finished else "running",
                    phase=state.get("stage"),
                    elapsed_s=round(self.skill_steps * 0.02, 2),
                    action_success_verified=False,
                )
                if finished:
                    self.phase = "observing"
                elif self.skill_steps >= 1500:
                    raise ValueError("原子动作或恢复超过 30 秒，任务未完成")
                return
            if self.phase == "moving":
                self.motion.tick(scene)
                result = self.motion.result()
                record = self.history[-1]
                if record["action"] == "forward":
                    result["segment_target_cm"] = self.motion.amount
                    result["remaining_plan_cm"] = round(
                        max(0, record["amount"] - self.motion.distance), 1
                    )
                    if (
                        self.motion.status == "reached"
                        and self.motion.amount < record["amount"]
                    ):
                        result["status"] = "checkpoint"
                self.history[-1]["execution"] = result
                if self.motion.status == "incomplete":
                    raise ValueError("未达到动作目标，已停止；请查看实际转角和位移")
                if self.motion.status == "reached":
                    self.phase = "observing"
                return
            if self.phase in {"waiting", "settling", "confirming"}:
                if hasattr(scene, "navigation_hold"):
                    scene.navigation_hold()
                else:
                    scene.motion_step((0, 0, 0))
                self.remaining -= 1
                if self.remaining == 0:
                    scene.stop()
                    if self.phase == "waiting":
                        self.history[-1]["execution"] = dict(
                            status="reached", elapsed_s=self.history[-1]["amount"]
                        )
                    self.phase = "observing"
                return
            if self.phase == "observing":
                if len(self.history) >= self.max_decisions:
                    raise ValueError("达到决策次数上限，尚未确认任务完成")
                scene.stop()
                picture = scene.observe()
                self.observations += 1
                # 只传递任务、真实头部图像和已执行动作记录；不暴露仿真坐标。
                context = (
                    {"context": scene.navigation_context()}
                    if hasattr(scene, "navigation_context")
                    else {}
                )
                self.proximity = (
                    scene.proximity(self.task) if hasattr(scene, "proximity") else None
                )
                if self.proximity is not None:
                    context.setdefault("context", {})["proximity"] = self.proximity
                self.pending = self.submit(
                    self.task, picture, tuple(self.history), **context
                )
                if not isinstance(self.pending, Future):
                    raise TypeError("模型接口必须返回 Future")
                self.phase, self.reason = "thinking", "等待模型分析头部画面"
                return
            if self.phase == "thinking" and self.pending.done():
                decision = Decision.parse(self.pending.result())
                self.pending = None
                requested = decision.action
                gate_reason = None
                p = self.proximity
                seated_arrival = False
                if p and p.get("visible") and scene.state().get("stage") == "seated":
                    # 坐下使躯干后移；保留刚在近处坐下的证据，不反复要求站起。
                    last_sit = next(
                        (h for h in reversed(self.history) if h["action"] == "sit"),
                        None,
                    )
                    seated_arrival = bool(
                        last_sit
                        and last_sit.get("proximity", {}).get("near")
                        and p["surface_distance_cm"] <= 38
                        and abs(p["bearing_deg"]) <= 12
                    )
                approach_task = any(
                    word in self.task for word in ("走到", "面前", "靠近", "近距离")
                )
                if p is not None:
                    p = {**p, "seated_arrival_verified": seated_arrival}
                    self.proximity = p
                if p and approach_task:
                    if (
                        decision.action == "sit"
                        and p.get("near")
                        and any(word in self.task for word in ("鞠躬", "俯身"))
                    ):
                        bowed = any(
                            h["action"] == "ground_pick"
                            and h.get("execution", {}).get("status") == "reached"
                            for h in self.history
                        )
                        if not bowed:
                            decision = replace(decision, action="ground_pick", amount=0)
                            gate_reason = "先按任务顺序鞠躬，再坐下"
                    if decision.action == "done" and (p.get("near") or seated_arrival):
                        completed = [
                            h["action"]
                            for h in self.history
                            if h.get("execution", {}).get("status") == "reached"
                        ]
                        if (
                            any(word in self.task for word in ("鞠躬", "俯身"))
                            and "ground_pick" not in completed
                        ):
                            decision = replace(decision, action="ground_pick", amount=0)
                            gate_reason = "近距离已确认，先完成任务要求的鞠躬"
                        elif (
                            "坐" in self.task and scene.state().get("stage") != "seated"
                        ):
                            decision = replace(decision, action="sit", amount=0)
                            gate_reason = "先完成任务要求的坐下，不能站着报告完成"
                    premature = decision.action in {
                        "ground_pick",
                        "sit",
                        "done",
                    } and not (
                        p.get("near", False)
                        or (decision.action == "done" and seated_arrival)
                    )
                    if premature:
                        if not p["visible"]:
                            decision = replace(decision, action="left", amount=30)
                            gate_reason = (
                                "尚无目标深度证据，先搜索，禁止提前执行到达后的动作"
                            )
                        elif abs(p["bearing_deg"]) > 12:
                            decision = replace(
                                decision,
                                action="left" if p["bearing_deg"] > 0 else "right",
                                amount=max(10, min(45, abs(p["bearing_deg"]))),
                            )
                            gate_reason = "目标未正对，先调整方向"
                        elif p["surface_distance_cm"] > 28:
                            decision = replace(
                                decision,
                                action="forward",
                                amount=max(5, min(30, p["surface_distance_cm"] - 24)),
                            )
                            gate_reason = "实测仍未走近，先接近目标，禁止提前鞠躬或坐下"
                        else:
                            raise ValueError(
                                "距目标过近，已停止；请人工调整，不能继续向前或俯身"
                            )
                        if scene.state().get("stage") == "seated":
                            decision = replace(decision, action="stand", amount=0)
                    if p["visible"] and decision.action == "forward":
                        remaining = p["surface_distance_cm"] - 24
                        if p["surface_distance_cm"] <= 28:
                            decision = replace(decision, action="wait", amount=0.2)
                            gate_reason = "已接近目标，停止继续前进并复核后续动作"
                        else:
                            decision = replace(
                                decision, amount=max(5, min(decision.amount, remaining))
                            )
                self.history.append(
                    dict(observation=self.observations, **decision.__dict__)
                )
                if p is not None:
                    self.history[-1]["proximity"] = p
                if gate_reason:
                    self.history[-1].update(
                        model_action=requested, controller_reason=gate_reason
                    )
                    self.history[-1]["evidence"] = (
                        gate_reason + "；模型原判断：" + decision.evidence
                    )
                self.reason = gate_reason or decision.evidence
                if decision.action == "abort":
                    raise ValueError(f"模型无法继续：{decision.evidence}")
                if decision.action == "done":
                    self.confirmations += 1
                    if self.confirmations >= 2:
                        scene.stop()
                        self.phase = "completed"
                    else:
                        # 再走 0.5 s 零指令平衡并获取新图，不用同一张图自我确认。
                        self.phase, self.remaining = "confirming", 25
                    return
                self.confirmations = 0
                if decision.action in SKILLS:
                    available = scene.navigation_context()["available_actions"]
                    if decision.action not in available:
                        raise ValueError("当前场景不支持模型选择的原子动作")
                    scene.stop()
                    scene.navigation_action(decision.action)
                    self.motion = None
                    self.skill_steps = 0
                    self.history[-1]["execution"] = dict(
                        status="running", action_success_verified=False
                    )
                    self.phase = "head" if decision.action in HEAD_ACTIONS else "skill"
                    return
                if decision.action == "wait":
                    self.motion = None
                    scene.stop()
                    self.phase, self.remaining = (
                        "waiting",
                        round(decision.amount / 0.02),
                    )
                    self.history[-1]["execution"] = dict(status="running")
                    return
                amount = decision.amount
                if decision.action == "forward":
                    # 长目标不是一次盲走。每段最多 30 cm，之后停步拍新图；
                    # 未走完的距离仅作为计划反馈，必须由模型重新授权，不能自动续走。
                    # 目标不可见或模型不确定时进一步缩短，不将置信度当作测距。
                    limit = (
                        30
                        if decision.target_visible and decision.confidence >= 0.6
                        else 10
                    )
                    amount = min(amount, limit)
                    self.history[-1]["segment_target_cm"] = amount
                self.motion = Motion(decision.action, amount, scene.motion_pose())
                self.history[-1]["execution"] = self.motion.result()
                self.phase = "moving"
        except Exception as error:
            self.fail(scene, str(error))

    def status(self):
        return dict(
            task=self.task,
            phase=self.phase,
            reason=self.reason,
            decisions=len(self.history),
            observations=self.observations,
            confirmations=self.confirmations,
            remaining_control_steps=self.remaining,
            history=list(self.history),
            motion=self.motion.result() if self.motion else None,
            completion_basis="head_rgbd_and_visual_judgment"
            if self.proximity
            else "head_camera_visual_judgment",
            proximity=self.proximity,
        )
