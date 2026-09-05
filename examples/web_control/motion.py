"""按实际转角和位移执行一个动作；只使用机器人自身位姿，不读取目标物体坐标。"""

import math

DT = 0.02


class Motion:
    def __init__(self, action, amount, pose):
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
            self.measure(scene.motion_pose())
            if self.remaining == 0:
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
        if self.direction * error <= (0.8 if self.action == "forward" else 2):
            self.before_settle = self.progress
            self.phase, self.remaining = "settling", 50
            scene.stop()
            return
        if self.action == "forward":
            command = (0.3, 0, max(-0.5, min(0.5, -math.radians(self.angle) * 2)))
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
        )
