"""视觉任务的状态机：模型给出动作目标，控制器根据实际运动反馈执行。"""

import json
import logging
import math
import time
import uuid
from concurrent.futures import Future
from collections import deque
from dataclasses import dataclass, replace
from motion import Motion, forward_speed
from office_search import RouteError, POINTS

SEARCH_ACTIONS = {"search_open", "search_101", "search_102", "search_103"}

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
        ] not in MOVES | SKILLS | SEARCH_ACTIONS | {
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
        self.observation_context = {}
        self.image_session = uuid.uuid4().hex
        self.observation_images = {}
        self.observation_snapshots = {}
        self.motion_frames = deque(maxlen=15)
        self.monitor_steps = 0
        self.candidate_visible = False
        self.candidate_absent_samples = 0
        self.trigger_frame = None
        self.search = None
        self.route_failures = 0
        self.target_memory = None

    def recover_route(self, scene, error):
        """局部规划失败先反馈给模型；安全边界不放宽，最多连续恢复三次。"""
        scene.stop()
        self.route_failures += 1
        alternatives, checks = [], []
        pose = scene.motion_pose()
        remembered_target = self.target_memory is not None
        if remembered_target and self.search:
            self.search.reject_approach()
        if self.search and self.search.map.free(pose[:2]):
            for region, points in POINTS.items():
                if region == self.search.region:
                    continue
                for index, point in enumerate(points):
                    if index in self.search.visited[region]:
                        continue
                    try:
                        self.search.map.path(pose[:2], point)
                        alternatives.append("search_" + region)
                        break
                    except RouteError as blocked:
                        checks.append(
                            dict(region=region, point=index, reason=str(blocked))
                        )
        failure = dict(
            code=error.code,
            reason=str(error),
            failed_region=self.search.region if self.search else None,
            pose=list(pose),
            point_failures=error.details,
            alternative_checks=checks,
            reachable_other_actions=alternatives,
            recovery_attempt=self.route_failures,
            instruction="这是局部规划失败，不是任务完成。请依据新图选择 reachable_other_actions 中的其他分区；不要重复刚失败的分区。列表为空时停止并说明限制。",
        )
        if remembered_target:
            failure.update(
                target_memory=self.target_memory,
                instruction="已发现目标的位置仍然保留。当前失败仅排除这条接近路线，控制器将尝试其他接近方向；不要重新全局找目标，也不能凭记忆宣称到达。",
            )
        logging.getLogger("web_control.navigation").warning(
            "路线失败诊断 %s", json.dumps(failure, ensure_ascii=False)
        )
        if self.search:
            self.search.last_failure = failure
            self.search.phase = "blocked"
            self.search.motion = None
            self.search.checked_forward = None
        self.motion = None
        if self.history:
            self.history[-1]["execution"] = dict(status="route_blocked", **failure)
        if error.code == "unsafe_start" or not (
            self.search and self.search.map.free(pose[:2])
        ):
            self.fail(
                scene,
                "当前位置落入障碍安全边界，已停止；需人工检查位置或重置场景，不会强行移动",
            )
        elif not alternatives and not remembered_target:
            self.fail(
                scene, "当前没有其他可达的未采样分区，已停止；请检查场景或重新安排搜索"
            )
        elif self.route_failures > 3:
            self.fail(scene, "连续三次路线恢复后仍失败，已停止；请检查路线与障碍")
        else:
            self.phase, self.remaining = "settling", 50
            self.reason = (
                "目标位置仍已记住，停步复查后尝试另一接近方向"
                if remembered_target
                else "局部路线失败，停步重新观察后请模型选择其他可达分区"
            )

    def monitor_motion(self, scene):
        """5 Hz 仿真采样；只在候选新出现时打断，已经看到的目标不会每帧触发。"""
        if not hasattr(scene, "observation_sample"):
            return False
        self.monitor_steps += 1
        if self.monitor_steps % 10:
            return False
        picture, visible = scene.observation_sample(self.task)
        sample = dict(picture=picture, time_s=scene.state().get("time_s"))
        self.motion_frames.append(sample)
        newly_visible = visible and not self.candidate_visible
        if visible:
            self.candidate_visible = True
            self.candidate_absent_samples = 0
        else:
            self.candidate_absent_samples += 1
            # 边缘目标会随步态反复进出；连续一秒采样不可见才重新武装。
            if self.candidate_absent_samples >= 5:
                self.candidate_visible = False
        periodic = self.phase == "searching" and self.monitor_steps >= 750
        if not newly_visible and not periodic:
            return False
        event = "target_candidate" if newly_visible else "periodic_review"
        self.trigger_frame = {**sample, "event": event}
        if self.phase == "searching":
            self.search.interrupt(scene)
            result = self.search.result()
        else:
            self.motion.measure(scene.motion_pose())
            self.motion.status = "interrupted"
            result = self.motion.result()
        self.history[-1]["execution"] = {
            **result,
            "status": "checkpoint",
            "trigger": event,
        }
        scene.stop()
        self.phase, self.remaining = "settling", 50
        self.reason = (
            "运动中发现颜色候选，停步复查"
            if newly_visible
            else "高层搜索已执行 15 秒，停步交回模型复查"
        )
        return True

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
        if self.phase == "searching" and self.history:
            self.history[-1]["execution"] = {
                **self.search.result(),
                "status": "cancelled",
            }
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
            if self.phase in {"moving", "searching"} and self.monitor_motion(scene):
                return
            if self.phase == "turn_preparing":
                scene.navigation_hold()
                self.turn_steps += 1
                if self.turn_steps >= 150 or (
                    self.turn_steps >= 50
                    and scene.head_command_reached()
                    and scene.body_stable()
                ):
                    self.phase = "moving"
                return
            if self.phase == "searching":
                result = self.search.tick(scene, self.task)
                self.history[-1]["execution"] = self.search.result()
                if result:
                    if result == "point_observed":
                        self.route_failures = 0
                    elif result == "approach_review":
                        p = scene.proximity(self.task) or {}
                        if not p.get("near"):
                            # 到了停靠点却仍不能确认目标，下一轮换角度，不重走同一路线。
                            self.search.reject_approach()
                    scene.stop()
                    self.phase = "observing"
                return
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
                if (
                    self.motion.action == "forward"
                    and self.motion.phase != "settling"
                    and self.motion.steps % 5 == 0
                    and hasattr(scene, "obstacle_clearance")
                ):
                    depth = scene.obstacle_clearance() or {}
                    front = depth.get("front_clearance_cm")
                    if front is not None and front < 20:
                        self.motion.measure(scene.motion_pose())
                        self.motion.status = "interrupted"
                        self.history[-1]["execution"] = {
                            **self.motion.result(),
                            "status": "obstacle_stop",
                        }
                        scene.stop()
                        self.phase, self.remaining = "settling", 50
                        self.reason = "途中检测到近障碍，先停步再观察"
                        return
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
                if hasattr(scene, "observation_sample"):
                    picture, visible = scene.observation_sample(self.task)
                    if visible:
                        self.candidate_visible = True
                        self.candidate_absent_samples = 0
                else:
                    picture = scene.observe()
                self.monitor_steps = 0
                self.observations += 1
                # 保存本次传入 submit 的原始字节，不从后续实时预览重新截图。
                # 只在网页状态中附带地址，不把图片地址或历史图片追加给模型。
                image_id = f"{self.image_session}-{self.observations}"
                self.observation_images[image_id] = picture
                self.observation_snapshots[self.observations] = dict(
                    url=f"/api/observation-image/{image_id}",
                    time_s=scene.state().get("time_s")
                    if hasattr(scene, "state")
                    else None,
                )
                # 传递真实头部图像与反馈；办公场景可提供自身定位和建筑平面图，
                # 但绝不传递目标球的隐藏坐标。
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
                if "office_search" in context.get("context", {}):
                    now = scene.state().get("time_s")
                    position = (self.proximity or {}).get("estimated_position_m")
                    memory = getattr(scene, "object_memory", None)
                    if memory is not None:
                        from proximity import target_color

                        remembered = memory.target(target_color(self.task))
                        if remembered is not None:
                            self.target_memory = remembered
                    if (self.proximity or {}).get("visible") and position is not None:
                        self.target_memory = dict(
                            self.target_memory or {},
                            position=list(position),
                            observed_at_s=now,
                        )
                    # 遮挡和时间流逝只降低位置的新鲜度，不删除“曾经见过”的事实。
                    if self.target_memory is not None:
                        self.target_memory["currently_visible"] = bool(
                            (self.proximity or {}).get("visible")
                        )
                        self.target_memory["age_s"] = (
                            round(max(0, now - self.target_memory["observed_at_s"]), 1)
                            if now is not None
                            and self.target_memory.get("observed_at_s") is not None
                            else None
                        )
                    context["context"]["target_memory"] = self.target_memory
                self.observation_context = context.get("context", {})
                if self.search:
                    self.observation_context["search_plan"] = self.search.context()
                    context.setdefault("context", self.observation_context)
                if self.trigger_frame:
                    trigger, self.trigger_frame = self.trigger_frame, None
                    trigger_id = image_id + "-trigger"
                    self.observation_images[trigger_id] = trigger["picture"]
                    self.observation_snapshots[self.observations]["trigger"] = dict(
                        url=f"/api/observation-image/{trigger_id}",
                        time_s=trigger["time_s"],
                        event=trigger["event"],
                    )
                    context.setdefault("context", {})["trigger_image"] = trigger[
                        "picture"
                    ]
                    context["context"]["motion_observation"] = dict(
                        event=trigger["event"],
                        trigger_time_s=trigger["time_s"],
                        image_order="第一张是停稳后的当前图像；第二张是运动中触发复查的历史图像。颜色候选不证明目标身份或到达，位置判断以当前图像和当前深度为准。",
                    )
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
                obstacle = self.observation_context.get("obstacles") or {}
                if (
                    "office_search" in self.observation_context
                    and not (p or {}).get("visible")
                    and not self.target_memory
                    and decision.action in {"left", "right"}
                ):
                    recent = self.history[-4:]
                    poses = [h.get("observed_from") for h in recent]
                    if (
                        len(recent) == 4
                        and all(h["action"] in {"left", "right"} for h in recent)
                        and all(poses)
                        and max(
                            math.hypot(
                                q["x_m"] - poses[0]["x_m"], q["y_m"] - poses[0]["y_m"]
                            )
                            for q in poses
                        )
                        < 0.25
                    ):
                        from office_search import POINTS

                        remaining = [
                            k
                            for k in POINTS
                            if self.search is None
                            or len(self.search.visited[k]) < len(POINTS[k])
                        ]
                        if not remaining:
                            raise ValueError(
                                "预设观察点已采样但未发现目标，不能继续重复转向"
                            )
                        decision = replace(
                            decision, action="search_" + remaining[0], amount=0
                        )
                        gate_reason = "检测到原地反复转向，改为前往尚未观察的区域"
                # 搜索与接近使用同一套安全地图。目标暂时被挡住时允许沿记忆
                # 绕行，不再强制每一步正对它；到达仍只认可当前 RGB-D 证据。
                if (
                    approach_task
                    and self.target_memory
                    and not (p or {}).get("near")
                    and not seated_arrival
                    and "office_search" in self.observation_context
                    and scene.state().get("stage") != "seated"
                    and decision.action
                    in ({"forward", "left", "right"} | SEARCH_ACTIONS)
                    and (
                        not (p or {}).get("visible")
                        or (p or {}).get("surface_distance_cm", 999) > 40
                        or (
                            obstacle.get("front_clearance_cm") is not None
                            and obstacle["front_clearance_cm"]
                            < (p or {}).get("surface_distance_cm", 0) - 5
                        )
                    )
                ):
                    decision = replace(decision, action="approach_target", amount=0)
                    gate_reason = (
                        "依据已观测目标位置规划安全停靠路线，允许暂时背离目标绕行"
                    )
                clearance_cm = obstacle.get("front_clearance_cm")
                if decision.action == "forward" and clearance_cm is not None:
                    allowed = clearance_cm - 18
                    if allowed < 5:
                        decision = replace(decision, action="wait", amount=0.2)
                        gate_reason = (
                            "前方低位障碍过近，本次前进被拦截；需观察并调整方向"
                        )
                    elif decision.amount > allowed:
                        decision = replace(decision, amount=min(30, allowed))
                        gate_reason = "按头部深度检测到的障碍距离缩短前进段"
                self.history.append(
                    dict(observation=self.observations, **decision.__dict__)
                )
                if "office_search" in self.observation_context:
                    self.history[-1]["observed_from"] = self.observation_context[
                        "office_search"
                    ]["observer_pose"]
                    self.history[-1]["region"] = self.observation_context[
                        "office_search"
                    ]["current_region"]
                    self.history[-1]["obstacles"] = obstacle
                if p is not None:
                    self.history[-1]["proximity"] = p
                if self.target_memory is not None:
                    self.history[-1]["target_memory"] = dict(self.target_memory)
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
                if decision.action == "approach_target":
                    if self.search is None:
                        from office_search import OfficeSearch

                        self.search = OfficeSearch(scene)
                    self.search.start_approach(scene, self.target_memory["position"])
                    self.history[-1]["execution"] = self.search.result()
                    self.phase, self.motion = "searching", None
                    return
                if decision.action in SEARCH_ACTIONS:
                    if "office_search" not in self.observation_context:
                        raise ValueError("只有办公场景支持分区搜索")
                    if scene.state().get("stage") == "seated":
                        raise ValueError("请先站起，再执行分区搜索")
                    if self.search is None:
                        from office_search import OfficeSearch

                        self.search = OfficeSearch(scene)
                    self.search.start(decision.action, scene)
                    self.history[-1]["execution"] = self.search.result()
                    self.phase, self.motion = "searching", None
                    return
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
                speed = (
                    forward_speed(
                        amount,
                        floor_cm=obstacle.get("observed_floor_cm", 0),
                        front_cm=obstacle.get("front_clearance_cm"),
                    )
                    if decision.action == "forward"
                    else 0.3
                )
                self.motion = Motion(
                    decision.action, amount, scene.motion_pose(), forward_speed=speed
                )
                self.history[-1]["execution"] = self.motion.result()
                self.phase = "moving"
                if (
                    decision.action in {"forward", "left", "right"}
                    and hasattr(scene, "head_feedback")
                    and abs(scene.head_feedback()["target_offset_deg"]) > 2
                ):
                    scene.navigation_action("head_reset")
                    self.turn_steps = 0
                    self.phase = "turn_preparing"
                    self.history[-1]["execution"]["phase"] = "head_preparing"
        except RouteError as error:
            self.recover_route(scene, error)
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
            history=[
                {
                    **item,
                    "snapshot": self.observation_snapshots.get(item["observation"]),
                }
                for item in self.history
            ],
            current_snapshot=self.observation_snapshots.get(self.observations),
            motion=self.motion.result() if self.motion else None,
            completion_basis="head_rgbd_and_visual_judgment"
            if self.proximity
            else "head_camera_visual_judgment",
            proximity=self.proximity,
            target_memory=self.target_memory,
        )
