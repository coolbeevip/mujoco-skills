"""办公室分区搜索：建筑碰撞图规划路线，头部 RGB-D 负责发现球。

地图只读取固定家具与墙体；动态球和机器人不进入建筑地图。观察点是房间
覆盖采样点，不是藏球位置。一次高层请求只执行一个观察点及一圈观察。
"""

import heapq
import math
from collections import deque
import numpy as np

from motion import Motion, forward_speed

SEARCH_ACTIONS = {"search_open", "search_101", "search_102", "search_103"}


class RouteError(ValueError):
    """可诊断的规划失败，与策略/物理异常分开处理。"""

    def __init__(self, code, message, details=None):
        super().__init__(message)
        self.code, self.details = code, details or []


POINTS = {
    "open": [(-1.2, -1.8), (-2.15, 0.35), (0.85, -1.5), (1.05, 0.15)],
    "101": [(-1.5, 1.05), (-0.55, 1.2), (-0.55, 2.15)],
    "102": [(0.65, 1.05), (1.55, 2.2), (2.55, 1.05)],
    "103": [(1.72, -0.8), (1.85, -1.95), (2.65, -1.95)],
}


class FloorMap:
    resolution = 0.05
    margin = 0.16

    def __init__(self, boxes):
        self.boxes = boxes
        self.blocked = set()
        self.cache = {}
        self.centers = np.array([b[0] for b in boxes]).reshape(-1, 2)
        self.sizes = np.array([b[1] for b in boxes]).reshape(-1, 2)
        self.rotations = np.array([b[2] for b in boxes]).reshape(-1, 2, 2)

    @classmethod
    def from_scene(cls, scene):
        r = scene.runtime
        boxes = []
        for i in range(r.model.ngeom):
            g = r.model.geom(i)
            # 仅固定 office box；自由关节球、机器人与装饰面均不参与地图。
            if (
                not g.name.startswith("office_")
                or not g.contype[0]
                or int(g.type[0]) != 6
            ):
                continue
            body = int(g.bodyid[0])
            if r.model.body_weldid[body] != 0:
                continue
            center = r.data.geom_xpos[i].copy()
            if center[2] - g.size[2] >= 0.24 or center[2] + g.size[2] <= 0.025:
                continue
            rotation = r.data.geom_xmat[i].reshape(3, 3)[:2, :2].copy()
            boxes.append((center[:2], g.size[:2].copy(), rotation))
        return cls(boxes)

    def cell(self, xy):
        return tuple(round(v / self.resolution) for v in xy)

    def xy(self, cell):
        return np.asarray(cell) * self.resolution

    def free(self, xy):
        cell = self.cell(xy)
        if abs(xy[0]) > 2.8 or abs(xy[1]) > 2.3 or cell in self.blocked:
            return False
        if cell not in self.cache:
            local = np.einsum(
                "bi,bij->bj", self.xy(cell) - self.centers, self.rotations
            )
            self.cache[cell] = not np.any(
                np.all(np.abs(local) <= self.sizes + self.margin + 0.036, axis=1)
            )
        return self.cache[cell]

    def clear_line(self, a, b):
        n = max(2, math.ceil(math.dist(a, b) / 0.025) + 1)
        return all(self.free(p) for p in np.linspace(a, b, n))

    def path(self, start, goal):
        first, last = self.cell(start), self.cell(goal)
        if not self.free(start):
            raise RouteError(
                "unsafe_start",
                "当前位置落入障碍安全边界，不能通过换房间解决；请人工检查位置或重置场景",
            )
        if not self.free(goal):
            raise RouteError("unsafe_goal", "观察点落入障碍安全边界")
        queue = [(0, first)]
        cost, previous = {first: 0}, {}
        while queue:
            _, cell = heapq.heappop(queue)
            if cell == last:
                result = [np.asarray(goal)]
                while cell != first:
                    result.append(self.xy(cell))
                    cell = previous[cell]
                result.append(np.asarray(start))
                result.reverse()
                # 去掉共线中间点，但整条快捷线必须仍处于膨胀后的可通行区域。
                smooth = [result[0]]
                i = 0
                while i < len(result) - 1:
                    j = len(result) - 1
                    while j > i + 1 and not self.clear_line(result[i], result[j]):
                        j -= 1
                    smooth.append(result[j])
                    i = j
                return smooth[1:]
            for dx, dy in (
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ):
                nxt = cell[0] + dx, cell[1] + dy
                if not self.clear_line(self.xy(cell), self.xy(nxt)):
                    continue
                new = cost[cell] + math.hypot(dx, dy)
                if new < cost.get(nxt, math.inf):
                    cost[nxt], previous[nxt] = new, cell
                    heapq.heappush(queue, (new + math.dist(nxt, last), nxt))
        raise RouteError("no_route", "当前建筑地图中没有可通行路线，不能穿越家具")


class OfficeSearch:
    def __init__(self, scene):
        self.map = FloorMap.from_scene(scene)
        self.visited = {key: [] for key in POINTS}
        self.region = None
        self.index = None
        self.phase = "idle"
        self.motion = None
        self.steps = 0
        self.segments = []
        self.scan_angle = 0
        self.replans = 0
        self.recent = []
        self.waiting_for_head = False
        self.head_samples = deque(maxlen=10)
        self.head_wait_steps = 0
        self.checked_forward = None
        self.last_failure = None
        self.approach_target = None
        self.goal_xy = None
        self.rejected_approach_goals = []

    def reject_approach(self):
        """记住失败的是哪个接近方向，而不是把目标本身忘掉。"""
        if self.approach_target is not None and self.goal_xy is not None:
            self.rejected_approach_goals.append(self.goal_xy.copy())
        self.motion = None
        self.checked_forward = None
        self.phase = "blocked"

    def target_seen(self, scene, task):
        p = scene.proximity(task) or {}
        return (
            p.get("near", False)
            if getattr(self, "approach_target", None) is not None
            else p.get("visible", False)
        )

    def start_approach(self, scene, target):
        """目标坐标来自已观测 RGB-D 球面拟合，禁止读取场景球体真值。"""
        pose = scene.motion_pose()
        target = np.asarray(target, dtype=float)
        if target.shape != (2,) or not np.all(np.isfinite(target)):
            raise ValueError("目标观测位置无效")
        if self.approach_target is not None:
            shift = math.dist(target, self.approach_target)
            if shift > 0.2:
                self.rejected_approach_goals.clear()
            elif self.phase == "checkpoint":
                # 暂时看不见目标、或模型定期复查，不代表已有绕行路线失效。
                self.phase = self.resume_phase
                self.prepare_head(scene, "head_reset")
                return
        if not self.map.free(pose[:2]):
            raise RouteError("unsafe_start", "当前位置落入障碍安全边界")
        angle = math.atan2(pose[1] - target[1], pose[0] - target[0])
        # 停在球心约 31 cm 外（表面约 25.5 cm），从多个方向寻找安全停靠点。
        # 视线校验使用未膨胀家具；行走路线仍使用原有机器人安全边界。
        sight = FloorMap(self.map.boxes)
        sight.margin = -0.036
        choices = []
        for radius in (0.31, 0.5, 0.7):
            for offset in (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180):
                theta = angle + math.radians(offset)
                goal = target + radius * np.array([math.cos(theta), math.sin(theta)])
                if any(
                    math.dist(goal, old) < 0.15 for old in self.rejected_approach_goals
                ):
                    continue
                if not self.map.free(goal) or not sight.clear_line(goal, target):
                    continue
                try:
                    path = self.map.path(pose[:2], goal)
                except RouteError:
                    continue
                length = sum(math.dist(a, b) for a, b in zip([pose[:2], *path], path))
                # 优先到近处；近处全被挡住时，允许先到外围换个视角重新确认。
                choices.append((length + 4 * (radius - 0.31), goal, path))
        if not choices:
            raise RouteError(
                "target_unreachable", "已观察到目标，但周围没有可安全抵达的停靠点"
            )
        _, self.goal_xy, self.path = min(choices, key=lambda item: item[0])
        self.approach_target = target
        self.region, self.index = "target", None
        self.phase, self.motion = "route", None
        self.steps, self.scan_angle, self.replans = 0, 0, 0
        self.segments, self.recent = [], []
        self.head_mode = "unknown"
        self.settling = self.head_wait_steps = 0
        self.checked_forward = None
        self.prepare_head(scene, "head_reset")

    def context(self):
        return dict(
            phase=self.phase,
            selected_region=self.region,
            selected_point=self.index,
            observed_points={k: list(v) for k, v in self.visited.items()},
            point_counts={k: len(v) for k, v in POINTS.items()},
            coverage_note="仅证明指定观察点的一圈采样，不保证家具遮挡后或整个房间已找遍",
            available_search_actions=sorted(SEARCH_ACTIONS),
            last_route_failure=self.last_failure,
            target_estimate_m=self.approach_target.tolist()
            if self.approach_target is not None
            else None,
            rejected_approach_count=len(self.rejected_approach_goals),
        )

    def start(self, action, scene):
        if self.phase == "checkpoint" and action == "search_" + self.region:
            # 模型复查后继续同一个观察点，保留已扫描角度，不能从头反复扫同一方向。
            self.phase = self.resume_phase
            # 观察中断没有改变建筑地图，沿保留的下一航点继续；仍重新低头
            # 检查当前方向。不要把转身的短暂侧移当成需要重新选观察点。
            self.prepare_head(scene, "head_reset")
            return
        self.approach_target = None
        self.region = action.removeprefix("search_")
        candidates = [
            i
            for i in range(len(POINTS[self.region]))
            if i not in self.visited[self.region]
        ]
        if not candidates:
            raise RouteError(
                "region_exhausted", "该分区预设观察点已经采样，请选择尚未观察的分区"
            )
        pose = scene.motion_pose()
        paths = []
        failures = []
        for i in candidates:
            try:
                path = self.map.path(pose[:2], POINTS[self.region][i])
                length = sum(math.dist(a, b) for a, b in zip([pose[:2], *path], path))
                paths.append((length, i, path))
            except RouteError as error:
                if error.code == "unsafe_start":
                    raise
                failures.append(dict(point=i, code=error.code, reason=str(error)))
        if not paths:
            raise RouteError(
                "region_unreachable",
                "该分区剩余观察点当前不可达，请选择其他分区",
                failures,
            )
        _, self.index, self.path = min(paths, key=lambda p: p[0])
        self.goal_xy = POINTS[self.region][self.index]
        self.phase, self.motion = "route", None
        self.steps, self.scan_angle, self.replans = 0, 0, 0
        self.segments, self.recent = [], []
        # 行走时低头确认脚前连续地面，不能因为默认视角没看到障碍就加长步长。
        # 到达观察点后再回正，扩大寻找远处小球的视野。
        self.head_mode = (
            "head_down"
            if scene.head_feedback()["target_offset_deg"] > 18
            else "unknown"
        )
        self.settling = 0
        self.head_wait_steps = 0
        self.checked_forward = None
        self.prepare_head(scene, "head_reset")

    def prepare_head(self, scene, mode):
        if self.head_mode != mode:
            scene.navigation_action(mode)
            self.head_mode = mode
            self.settling = 150
            self.waiting_for_head = True
            self.wait_elapsed = 0
            self.head_samples.clear()

    def interrupt(self, scene):
        self.resume_phase = self.phase
        if self.motion:
            self.motion.measure(scene.motion_pose())
            if self.phase == "scan":
                self.scan_angle += abs(self.motion.angle)
            self.segments.append({**self.motion.result(), "status": "checkpoint"})
        self.motion = None
        self.checked_forward = None
        self.settling = 0
        self.waiting_for_head = False
        self.phase = "checkpoint"

    def tick(self, scene, task):
        self.steps += 1
        if self.steps > 15000:
            raise ValueError("单个观察点搜索超过 300 秒仿真时间，已停止")
        if self.settling:
            scene.navigation_hold()
            self.settling -= 1
            if self.waiting_for_head:
                self.head_wait_steps += 1
                self.wait_elapsed += 1
                self.head_samples.append(scene.head_feedback()["camera_pitch_deg"])
                if (
                    self.wait_elapsed >= 35
                    and len(self.head_samples) == 10
                    and max(self.head_samples) - min(self.head_samples) < 0.6
                    and scene.head_command_reached()
                    and scene.body_stable()
                ):
                    self.settling = 0
            if not self.settling:
                self.waiting_for_head = False
            return None
        if self.phase == "target_visible":
            return "target_visible"
        if self.checked_forward is not None:
            # 低头检查只授权眼前这一段；回正期间的位移也占用安全距离。
            # 身体明显转向或挪动后，旧图像不能继续证明新方向可通行。
            checked, self.checked_forward = self.checked_forward, None
            pose = scene.motion_pose()
            drift = math.dist(pose[:2], checked["pose"][:2]) * 100
            turn = abs(
                math.degrees(math.remainder(pose[2] - checked["pose"][2], math.tau))
            )
            if drift > 3 or turn > 5:
                return None
            if self.target_seen(scene, task):
                self.phase = "target_visible"
                return "target_visible"
            amount = checked["amount"] - drift
            if amount < 5:
                return None
            self.motion = Motion(
                "forward",
                amount,
                pose,
                heading_offset_deg=checked["bearing"],
                forward_speed=checked["speed"],
            )
        if self.motion:
            # 每 0.1 秒仿真时间复查一次前方深度。遇障碍先停步，不继续整段盲走。
            if self.motion.action == "forward" and self.steps % 5 == 0:
                depth = scene.obstacle_clearance()
                front = depth.get("front_clearance_cm")
                if front is not None and front < 20:
                    self.motion.measure(scene.motion_pose())
                    self.segments.append(
                        {**self.motion.result(), "status": "obstacle_stop"}
                    )
                    self.motion = None
                    scene.stop()
                    self.settling = 50
                    self.waiting_for_head = False
                    if self.target_seen(scene, task):
                        self.phase = "target_visible"
                        return None
                    self.replans += 1
                    if self.replans > 3:
                        raise RouteError(
                            "obstacle_blocked",
                            "前方障碍连续阻断路线，请选择另一观察区域",
                        )
                    x, y, yaw = scene.motion_pose()
                    hit = np.array(
                        [
                            x + front / 100 * math.cos(yaw),
                            y + front / 100 * math.sin(yaw),
                        ]
                    )
                    for dx in np.arange(-0.15, 0.151, 0.05):
                        for dy in np.arange(-0.15, 0.151, 0.05):
                            self.map.blocked.add(self.map.cell(hit + [dx, dy]))
                    self.path = self.map.path((x, y), self.goal_xy)
                    return None
                # 平视未看到障碍不等于脚前可通行，不能据此续走。
                # 本段结束后重新低头检查；途中可见的近障碍仍立即停车。
            self.motion.tick(scene)
            if self.motion.status == "incomplete":
                self.segments.append(
                    {
                        **self.motion.result(),
                        "action": self.motion.action,
                        "amount": self.motion.amount,
                    }
                )
                self.replans += 1
                if self.replans > 3:
                    raise ValueError("观察点路线的运动段多次未达到目标")
                self.motion = None
                self.settling = 50
                self.waiting_for_head = False
                self.path = self.map.path(scene.motion_pose()[:2], self.goal_xy)
                return None
            if self.motion.status != "reached":
                return None
            if self.phase == "scan":
                self.scan_angle += abs(self.motion.angle)
            self.segments.append(self.motion.result())
            self.motion = None
            # 每次停步均用真实 RGB-D 搜索目标；发现后立即交回模型，不自动宣称到达。
            if self.target_seen(scene, task):
                self.phase = "target_visible"
                return "target_visible"
        pose = scene.motion_pose()
        if self.phase == "route":
            while self.path and math.dist(pose[:2], self.path[0]) < (
                0.055 if self.approach_target is not None else 0.09
            ):
                self.path.pop(0)
            if not self.path:
                self.phase = "scan"
                self.prepare_head(scene, "head_reset")
                return None
            else:
                target = self.path[0]
                dx, dy = target - pose[:2]
                bearing = math.degrees(
                    math.remainder(math.atan2(dy, dx) - pose[2], math.tau)
                )
                if abs(bearing) > 15:
                    if self.head_mode != "head_reset":
                        self.prepare_head(scene, "head_reset")
                        return None
                    self.motion = Motion(
                        "left" if bearing > 0 else "right",
                        max(10, min(60, abs(bearing))),
                        pose,
                    )
                else:
                    if self.head_mode != "head_down":
                        self.prepare_head(scene, "head_down")
                        return None
                    depth = scene.obstacle_clearance()
                    front = depth.get("front_clearance_cm")
                    # 建筑地图不能证明动态通道为空。较长段还要求相机看见连续地面。
                    known = depth.get("observed_floor_cm", 0)
                    limit = 30 if known >= 50 else 10
                    amount = min(limit, known - 20, math.hypot(dx, dy) * 100)
                    if front is not None:
                        amount = min(amount, front - 18)
                    if amount < 5:
                        raise RouteError(
                            "insufficient_clearance",
                            "观察点路线前方空间不足，已停止等待重新选择",
                        )
                    # 直线两侧在原地图安全边界之外还需各有 8 cm 余量，
                    # 才使用最快档；门洞、桌腿旁的窄通道不加速。
                    unit = np.asarray([dx, dy]) / math.hypot(dx, dy)
                    lateral = np.array([-unit[1], unit[0]]) * 0.08
                    start = np.asarray(pose[:2])
                    end = start + unit * (amount / 100)
                    roomy = math.hypot(dx, dy) >= 0.5 and all(
                        self.map.clear_line(start + offset, end + offset)
                        for offset in (-lateral, lateral)
                    )
                    narrow = not all(
                        self.map.clear_line(start + offset, end + offset)
                        for offset in (-lateral / 2, lateral / 2)
                    )
                    # 先保存检查结果并回正头部，待稳定后才真正开始前进。
                    self.checked_forward = dict(
                        amount=amount,
                        pose=tuple(pose),
                        bearing=bearing,
                        speed=forward_speed(
                            amount,
                            floor_cm=known,
                            front_cm=front,
                            bearing_deg=bearing,
                            roomy=roomy,
                            narrow=narrow,
                        ),
                    )
                    self.prepare_head(scene, "head_reset")
                self.recent.append(pose[:2])
                if (
                    len(self.recent) >= 8
                    and max(math.dist(pose[:2], p) for p in self.recent[-8:]) < 0.15
                ):
                    self.replans += 1
                    if self.replans > 3:
                        raise RouteError(
                            "route_stalled", "路线多次停滞，已停止，不能反复左右转"
                        )
                    self.path = self.map.path(pose[:2], self.goal_xy)
                    self.recent.clear()
                return None
        if self.approach_target is not None:
            delta = self.approach_target - np.asarray(pose[:2])
            bearing = math.degrees(
                math.remainder(math.atan2(delta[1], delta[0]) - pose[2], math.tau)
            )
            if abs(bearing) <= 10:
                self.phase = "point_observed"
                return "approach_review"
            self.motion = Motion(
                "left" if bearing > 0 else "right", max(10, min(45, abs(bearing))), pose
            )
            return None
        if self.scan_angle >= 350:
            self.visited[self.region].append(self.index)
            self.phase = "point_observed"
            return "point_observed"
        self.motion = Motion("left", min(60, max(10, 360 - self.scan_angle)), pose)
        return None

    def result(self):
        return dict(
            status="running"
            if self.phase in {"route", "scan"}
            else "checkpoint"
            if self.phase == "checkpoint"
            else "reached",
            search_phase=self.phase,
            region=self.region,
            point=self.index,
            segments=list(self.segments),
            current_segment=self.motion.result() if self.motion else None,
            elapsed_s=round(self.steps * 0.02, 2),
            head_wait_s=round(self.head_wait_steps * 0.02, 2),
            action_success_verified=False,
            target_estimate_m=self.approach_target.tolist()
            if self.approach_target is not None
            else None,
        )
