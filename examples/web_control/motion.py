"""按实际转角和位移执行一个动作；只使用机器人自身位姿，不读取目标物体坐标。"""

import math
from collections import deque

DT = 0.02


def forward_speed(
    amount, *, floor_cm=0, front_cm=None, bearing_deg=0, roomy=False, narrow=False
):
    """只在当前深度证明通畅时提速；远目标或模型高置信度本身不能证明安全。"""
    if (
        narrow
        or amount < 20
        or floor_cm < 60
        or (front_cm is not None and front_cm < 70)
    ):
        return 0.3
    if abs(bearing_deg) > 5:
        return 0.3
    return 0.4 if roomy else 0.35


class Motion:
    def __init__(
        self, action, amount, pose, *, heading_offset_deg=0, forward_speed=0.3
    ):
        low, high = (5, 30) if action == "forward" else (10, 60)
        if (
            action not in {"forward", "left", "right"}
            or type(amount) not in (int, float)
            or not math.isfinite(amount)
            or not low <= amount <= high
        ):
            raise ValueError("动作目标超出支持范围")
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError("机器人运动反馈无效")
        self.action, self.amount = action, amount
        self.origin = tuple(pose)
        self.previous_yaw = pose[2]
        self.angle = 0.0
        self.distance = 0.0
        self.drift = 0.0
        self.steps = 0
        self.phase = "moving"
        self.remaining = 0
        self.direction = -1 if action == "right" else 1
        self.target = amount * self.direction
        self.aim = self.target
        self.before_settle = 0.0
        self.status = "running"
        if not math.isfinite(heading_offset_deg) or abs(heading_offset_deg) > 15:
            raise ValueError("前进航向修正不得超过 15 度")
        self.heading_offset_deg = heading_offset_deg
        if (
            type(forward_speed) not in (int, float)
            or not math.isfinite(forward_speed)
            or not 0.1 <= forward_speed <= 0.4
        ):
            raise ValueError("步行速度指令必须在 0.1～0.4 m/s 内")
        self.forward_speed = forward_speed
        self.settle_samples = deque(maxlen=10)
        self.settle_steps = 0
        self.total_settle_steps = 0
        self.extensions = 0

    def extend(self, centimeters):
        """仅供已授权的高层直线路线使用；延长运动目标，不插入一次停步。

        调用方必须重新确认地图、当前深度和转弯距离。累计上限 60 cm，
        原有 10 秒运动超时与漂移限制仍然有效。
        """
        if (
            self.action != "forward"
            or self.phase != "moving"
            or self.status != "running"
            or not 5 <= centimeters <= 30
            or self.amount + centimeters > 60
        ):
            raise ValueError("当前运动不能延长")
        self.amount += centimeters
        self.target = self.aim = self.amount
        self.extensions += 1

    def settled_early(self, scene):
        self.settle_samples.append(scene.motion_pose())
        if self.settle_steps < 20 or len(self.settle_samples) < 10:
            return False
        first = self.settle_samples[0]
        stable = all(
            math.dist(p[:2], first[:2]) < 0.003
            and abs(math.remainder(p[2] - first[2], math.tau)) < math.radians(0.6)
            for p in self.settle_samples
        )
        return stable and (not hasattr(scene, "body_stable") or scene.body_stable())

    def measure(self, pose):
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError("机器人运动反馈无效")
        self.angle += math.degrees(
            math.remainder(pose[2] - self.previous_yaw, math.tau)
        )
        self.previous_yaw = pose[2]
        dx, dy = pose[0] - self.origin[0], pose[1] - self.origin[1]
        # 前进量是沿动作开始时朝向的净位移，不把原地晃动累加成前进距离。
        self.distance = 100 * (
            dx * math.cos(self.origin[2]) + dy * math.sin(self.origin[2])
        )
        self.drift = 100 * math.hypot(dx, dy)

    @property
    def progress(self):
        return self.distance if self.action == "forward" else self.angle

    def tick(self, scene):
        if self.status != "running":
            return
        self.measure(scene.motion_pose())
        drift_limit = self.amount + 15 if self.action == "forward" else 20
        if self.steps >= 500 or self.drift > drift_limit:
            self.status = "incomplete"
            scene.stop()
            return
        tolerance = 2.5 if self.action == "forward" else 5
        error = self.aim - self.progress
        if self.phase == "settling":
            scene.motion_step((0, 0, 0))
            self.remaining -= 1
            self.steps += 1
            self.settle_steps += 1
            self.total_settle_steps += 1
            self.measure(scene.motion_pose())
            if self.remaining == 0 or self.settled_early(scene):
                error = self.target - self.progress
                if abs(error) <= tolerance:
                    self.status = "reached"
                    scene.stop()
                elif self.action == "forward" and error < 0:
                    # 不擅自倒退补偿前进超调；让调用方报告真实结果。
                    self.status = "incomplete"
                    scene.stop()
                else:
                    self.phase = "moving"
                    # 根据刚才实测的停步回摆补偿下一次指令，不反复停在同一个
                    # 只扭髋、不换脚的位置。补偿量有上限，最终仍按原目标验收。
                    limit = 3 if self.action == "forward" else 15
                    recoil = max(-limit, min(limit, self.before_settle - self.progress))
                    self.aim = self.target + recoil
                    self.direction = 1 if self.aim > self.progress else -1
            return
        # 补偿目标用于抵消停步回摆，不是新的用户目标。接近时间上限且实际
        # 目标已在容差内时，留出最后一段停步复核时间，避免继续追逐旧补偿量。
        final_check = (
            self.steps >= 450 and abs(self.target - self.progress) <= tolerance
        )
        if final_check or self.direction * error <= (
            0.8 if self.action == "forward" else 2
        ):
            self.before_settle = self.progress
            self.phase, self.remaining = "settling", min(50, 499 - self.steps)
            self.settle_steps = 0
            self.settle_samples.clear()
            scene.stop()
            return
        if self.action == "forward":
            command = (
                # 每段末尾先降回原速度，降低高速停步时的超调和回摆。
                min(self.forward_speed, 0.3) if error <= 8 else self.forward_speed,
                0,
                max(
                    -0.5,
                    min(0.5, math.radians(self.heading_offset_deg - self.angle) * 2),
                ),
            )
        else:
            # 纯角速度的小指令容易只扭动髋部。轻微迈步使步行策略真正换脚，
            # 转身会伴随少量位移，因此同时记录并限制净位移，而不宣称原地转动。
            command = (0.15, 0, 1.2 * self.direction)
        scene.motion_step(command)
        self.steps += 1
        self.measure(scene.motion_pose())

    def result(self):
        return dict(
            status=self.status,
            phase=self.phase,
            actual_angle_deg=round(self.angle, 1),
            actual_distance_cm=round(self.distance, 1),
            drift_cm=round(self.drift, 1),
            elapsed_s=round(self.steps * DT, 2),
            settling_s=round(self.total_settle_steps * DT, 2),
            extensions=self.extensions,
            speed_command_m_s=self.forward_speed if self.action == "forward" else None,
        )
