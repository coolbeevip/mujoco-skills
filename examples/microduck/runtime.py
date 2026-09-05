"""无窗口、同步的 MicroDuck CPU 运行核心。

BAM 配置和默认位姿改编自 pollen-robotics/microduck_rl 的 infer_policy.py
（Apache-2.0）；固定来源见 assets.lock.json 与 README。
"""

import json
from importlib.metadata import distribution, version

import mujoco
import numpy as np
import onnxruntime as ort
from bam.model import load_model
from bam.mujoco import MujocoController

from assets import DEFAULT_CACHE, lock, verify
from policies import POLICIES, catalog

# 阅读主线：从 step() 开始，再看它调用的 observation() 和 apply_action()。
# 例如按下 w 后，keyboard.py 会持续传入 (0.3, 0, 0)：
#   期望速度 + 当前状态 → 策略给出关节目标 → 电机产生力矩 → 物理状态改变。
# 下一轮再读取改变后的状态，这就是闭环控制。策略是已训练好的神经网络，
# 本文件只执行推理，不训练网络，也不播放一段预先录制的关节动画。

# 策略的 14 个动作、关节观测和电机目标都按这个顺序排列。
NAMES = (
    "left_hip_yaw,left_hip_roll,left_hip_pitch,left_knee,left_ankle,"
    "neck_pitch,head_pitch,head_yaw,head_roll,right_hip_yaw,right_hip_roll,"
    "right_hip_pitch,right_knee,right_ankle"
).split(",")
# 默认关节角（弧度）：既是初始化姿态，也是策略输入和输出共同使用的基准。
# observation() 用「当前角度 - POSE」描述姿态，apply_action() 用
# 「POSE + action」得到目标角度；二者必须使用训练时的同一个基准。
POSE = np.array(
    [
        0,
        -0.0873,
        -0.4579,
        -0.0049,
        0.4530,
        0.3491,
        0.3491,
        0,
        0,
        0,
        0.0873,
        0.4579,
        0.0049,
        -0.4530,
    ],
    dtype=np.float32,
)
OBS_NAMES = "base_ang_vel,projected_gravity,joint_pos,joint_vel,actions,command,head_command,body_command"
# 物理以 200 Hz 积分；每 4 个物理步推理一次策略，即 50 Hz 控制。
DT = 0.005
DECIMATION = 4


def vector(value, length, label):
    try:
        result = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{label}: expected finite vector[{length}]") from error
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{label}: expected finite vector[{length}]")
    return result.copy()


def validate_policy(session, command_names="twist,head_pose,body_pose"):
    # 除张量维度外，还校验关节顺序和观测语义，避免形状相同但含义不同。
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if (
        len(inputs) != 1
        or inputs[0].name != "obs"
        or inputs[0].shape != [1, 61]
        or inputs[0].type != "tensor(float)"
        or len(outputs) != 1
        or outputs[0].name != "actions"
        or outputs[0].shape != [1, 14]
        or outputs[0].type != "tensor(float)"
    ):
        raise ValueError("policy tensor contract mismatch")
    meta = session.get_modelmeta().custom_metadata_map
    if (
        meta.get("joint_names") != ",".join(NAMES)
        or meta.get("observation_names") != OBS_NAMES
        or meta.get("action_scale") != "1.0"
        or meta.get("command_names") != command_names
    ):
        raise ValueError("policy semantic contract mismatch")
    reference = np.fromstring(meta.get("default_joint_pos", ""), sep=",")
    if reference.shape != (14,) or not np.allclose(
        reference, POSE, atol=0.00051, rtol=0
    ):
        raise ValueError("policy reference pose mismatch")


class Runtime:
    def __init__(self, cache=DEFAULT_CACHE, seed=0, *, mode="walk", ball=False):
        if mode not in ("walk", "roller") or (ball and mode != "walk"):
            raise ValueError("mode must be walk or roller; ball requires walk mode")
        self.mode = mode
        self.ball = ball
        # 启动只验证本地资产；下载由 assets.py prepare 显式完成。
        self.cache = verify(cache)
        self.catalog = catalog(self.cache)
        for name, expected in [
            ("numpy", "2.3.2"),
            ("mujoco", "3.7.0"),
            ("onnxruntime", "1.23.2"),
            ("better-actuator-models", "1.0.1"),
        ]:
            if version(name) != expected:
                raise ValueError(
                    f"dependency mismatch: {name}; install requirements.txt"
                )
        provenance = json.loads(
            distribution("better-actuator-models").read_text("direct_url.json") or "{}"
        )
        if provenance.get("vcs_info", {}).get("commit_id") != lock()["bam_revision"]:
            raise ValueError("BAM revision mismatch; install requirements.txt")
        self.seed = seed
        options = ort.SessionOptions()
        # 固定 CPU 推理线程数，减少重复运行时的执行差异。
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.sessions = {}
        # 只加载当前模型支持的策略；轮式模型与步行模型不能在运行中互换。
        for key, (filename, policy_mode, metadata_command) in POLICIES.items():
            if policy_mode != self.mode:
                continue
            session = ort.InferenceSession(
                str(self.cache / "policies" / filename),
                sess_options=options,
                providers=["CPUExecutionProvider"],
            )
            validate_policy(session, metadata_command)
            self.sessions[key] = session
        self.reset()

    def reset(self):
        # 重建模型及 BAM 对象，同时清除其可变摩擦、控制历史和求解器历史。
        self.rng = np.random.default_rng(self.seed)
        motor = load_model(motor_name="xl330", model="m6")
        motor.actuator.kp = 200.0
        motor.actuator.vin = 7.4
        motor.actuator.max_current = None
        force = 7.4 * motor.kt.value / motor.R.value
        # BAM 根据目标角度计算电机力矩，MuJoCo 执行器接收力矩而非角度。
        scene = (
            "scene_rollers.xml"
            if self.mode == "roller"
            else ("scene_ball.xml" if self.ball else "scene.xml")
        )
        spec = mujoco.MjSpec.from_file(str(self.cache / "model" / scene))
        for actuator in spec.actuators:
            actuator.set_to_motor()
            actuator.forcelimited = True
            actuator.forcerange = (-force, force)
            actuator.ctrllimited = False
            actuator.gear = [1.0, 0, 0, 0, 0, 0]
        for joint in spec.joints:
            if joint.name in NAMES:
                joint.damping = np.zeros((3, 1))
                joint.frictionloss = 0.0
                joint.solref_friction = (-5e4, -2e2)
                joint.solimp_friction = (0.99, 0.9999, 0.001, 0.5, 2.0)
        self.model = spec.compile()
        self.model.opt.timestep = DT
        if self.mode == "roller":
            # 轮子是被动关节，不接收策略动作；按官方推理入口设置轴承摩擦。
            for j in range(self.model.njnt):
                if self.model.joint(j).name.startswith("passive_"):
                    self.model.dof_frictionloss[self.model.jnt_dofadr[j]] = 0.003
        if [self.model.actuator(i).name for i in range(self.model.nu)] != NAMES:
            raise ValueError("model actuator order mismatch")
        self.data = mujoco.MjData(self.model)
        self.controller = MujocoController(
            motor, NAMES, self.model, self.data, vin_drop_gain=0.1, vin_min=6.0
        )
        self.qids = self.model.jnt_qposadr[[self.model.joint(n).id for n in NAMES]]
        # qpos 与 qvel 的索引分别查询：自由基座的位姿占 7 维，速度占 6 维。
        self.vids = self.model.jnt_dofadr[[self.model.joint(n).id for n in NAMES]]
        self.root_id = self.model.body("trunk_base").id
        self.gyro = int(self.model.sensor("imu_ang_vel").adr[0])
        root_qpos = self.model.joint("trunk_base_freejoint").qposadr[0]
        height = 0.1385 if self.mode == "roller" else 0.125
        self.data.qpos[root_qpos : root_qpos + 7] = [0, 0, height, 1, 0, 0, 0]
        # 前 7 维是基座位置和 wxyz 四元数，其余关节按策略顺序写入默认角度。
        self.data.qpos[self.qids] = POSE
        self.controller.reset(self.data.qpos)
        self.last_action = np.zeros(14, dtype=np.float32)
        self.ticks = 0
        self.physics_steps = 0
        self.failed = False
        self.active_policy = "roller" if self.mode == "roller" else "standing"
        mujoco.mj_forward(self.model, self.data)
        return self.snapshot()

    def observation(self, command):
        # 策略需要同时知道「我现在是什么姿态」和「我希望怎样运动」。
        # 此函数把这些信息按训练时约定的顺序拼成 61 个数；顺序也是接口的一部分。
        rotation = self.data.xmat[self.root_id].reshape(3, 3)
        # 把世界中的向下方向变换到躯干坐标系，让策略感知身体的倾斜方向。
        # 身体直立时约为 (0, 0, -1)，倾斜后会出现水平分量。
        # rotation 把躯干坐标转到世界坐标，因此反向变换使用转置 rotation.T。
        gravity = rotation.T @ np.array([0, 0, -1])
        # 观测归一化已包含在 ONNX 图中，此处只组装输入。
        return vector(
            np.concatenate(
                (
                    # [0:3] 躯干角速度（rad/s）：身体正在怎样旋转。
                    self.data.sensordata[self.gyro : self.gyro + 3],
                    # [3:6] 重力单位方向：身体朝哪边倾斜。
                    gravity,
                    # [6:20] 14 个关节相对默认角度的偏移（rad）：当前姿态。
                    self.data.qpos[self.qids].astype(np.float32) - POSE,
                    # [20:34] 14 个关节速度（rad/s）：关节正在怎样运动。
                    self.data.qvel[self.vids],
                    # [34:48] 上一次策略输出：为网络提供上一轮控制信息。
                    self.last_action,
                    # [48:51] 策略命令：步行时为期望速度，w 对应 (0.3, 0, 0)；
                    # 动作策略在相同位置接收姿态标志或阶段信号，见 behaviors.py。
                    command,
                    # [51:61] 头部指令 4 维、身体指令 6 维，本样例固定为零。
                    np.zeros(10),
                )
            ),
            61,
            "observation",
        )

    def apply_action(self, action, scale=1.0):
        action = vector(action, 14, "action")
        # 网络输出不是电机力矩，而是 14 个关节相对默认角度的偏移。
        # 步行策略的动作缩放为 1，轮式策略为 0.8。例如缩放为 1 时，
        # 默认角度 0.35 rad、偏移 0.10 rad，
        # 则目标是 0.45 rad。这里只设置目标，不会把真实关节瞬移到该角度。
        targets = vector(POSE + action * scale, 14, "target")
        self.controller.q_target[:] = targets
        # 保存本次输出，下一次 observation() 会把它作为输入的一部分。
        self.last_action = action

    def step(self, velocity=(0, 0, 0), observer=None):
        # 一次调用完成 20 ms 仿真：推理一次，随后执行四个 5 ms 物理步。
        # velocity 是期望速度，不是直接写入机器人的实际速度。
        command = vector(velocity, 3, "velocity")
        if self.mode == "roller":
            return self.step_policy("roller", command, observer)
        if self.failed:
            raise ValueError("runtime failed; explicit reset required")
        if (
            not np.isfinite(self.data.qpos).all()
            or not np.isfinite(self.data.qvel).all()
        ):
            self.failed = True
            raise ValueError("invalid physics state before action")
        # 本样例只开放保守指令范围；这不是动作成功判定。
        if np.any(np.abs(command) > np.array([0.5, 0.3, 1.5])):
            raise ValueError("velocity outside supported range")
        # 1. 根据指令选择策略。w 的向量长度为 0.3，使用 walking；
        #    空格的指令为零，使用 standing，仍会不断推理以维持平衡。
        mode = "standing" if np.linalg.norm(command) <= 0.05 else "walking"
        if mode == "standing":
            command[:] = 0
        return self.step_policy(mode, command, observer)

    def step_policy(self, policy, command=(0, 0, 0), observer=None):
        # 指定策略时，命令的含义由策略定义，而不是一律当作速度。
        # 校验完成前不修改物理状态；高层时序由 behaviors.py 管理。
        command = vector(command, 3, "policy command")
        if policy not in self.sessions:
            raise ValueError(f"policy {policy} is unavailable in {self.mode} mode")
        if self.failed:
            raise ValueError("runtime failed; explicit reset required")
        if (
            not np.isfinite(self.data.qpos).all()
            or not np.isfinite(self.data.qvel).all()
        ):
            self.failed = True
            raise ValueError("invalid physics state before action")
        if policy in ("standing", "walking"):
            valid = np.all(np.abs(command) <= [0.5, 0.3, 1.5])
            valid = valid and (policy != "standing" or not np.any(command))
        elif policy == "roller":
            # 第三项是相对航向误差（rad），不是步行策略中的角速度（rad/s）。
            valid = (
                -0.5 <= command[0] <= 0.6 and command[1] == 0 and abs(command[2]) <= 1
            )
        elif policy == "sitstand":
            valid = command[0] in (0, 1) and not np.any(command[1:])
        elif policy in ("ground_pick", "crouch"):
            valid = command[2] == 0 and np.isclose(
                np.linalg.norm(command[:2]), 1, atol=1e-5
            )
        else:
            valid = not np.any(command)
        if not valid:
            raise ValueError(f"invalid command for policy {policy}")
        mode = policy
        # 2. 收集当前状态和目标。即使一直保持 w，关节和身体状态也在变化，
        #    因此每轮都要重新读取观测，策略输出也可能随之改变。
        obs = self.observation(command)
        # 3. 执行已训练的网络：61 个输入数 → 14 个关节角偏移。
        #    obs[None] 把 (61,) 变成 (1, 61)，表示一次处理一个样本。
        #    run() 返回输出列表；第一个 [0] 取 actions 张量，
        #    第二个 [0] 取该张量中唯一的样本，最终得到形状 (14,)。
        action = self.sessions[mode].run(["actions"], {"obs": obs[None]})[0][0]
        # 4. 将角度偏移换算成目标角度，交给 BAM 电机控制器。
        self.apply_action(action, self.catalog[policy].get("action_scale", 1.0))
        self.active_policy = mode
        # 5. 让物理系统执行这个目标。策略以 50 Hz 决策，电机与物理以 200 Hz 更新。
        for _ in range(DECIMATION):
            # 四个子步内目标角度相同，但关节已在运动；BAM 每步结合最新状态
            # 计算力矩并写入 data.ctrl，相当于回答「现在该用多大的力去接近目标」。
            self.controller.update()
            if not np.isfinite(self.data.ctrl).all():
                self.failed = True
                raise ValueError("invalid actuator torque")
            # MuJoCo 根据力矩、重力、地面接触等推进 5 ms，得到新的位置和速度。
            # 是否真的向前移动取决于这些物理结果，而不是直接赋值为 0.3 m/s。
            mujoco.mj_step(self.model, self.data)
            self.physics_steps += 1
            if (
                not np.isfinite(self.data.qpos).all()
                or not np.isfinite(self.data.qvel).all()
                or np.any(self.data.warning.number)
            ):
                self.failed = True
                raise ValueError("invalid physics state or MuJoCo warning")
            # 更新积分后的位姿、传感器和接触等派生量，不推进仿真时间。
            mujoco.mj_forward(self.model, self.data)
            if observer is not None:
                # 每 5 ms 检查一次状态；测量抛出异常时立即停止剩余子步。
                try:
                    observer(self)
                except (Exception, KeyboardInterrupt):
                    self.failed = True
                    raise
        # 这轮结束；调用者再次调用 step() 时，会用新的状态重新开始上述流程。
        self.ticks += 1
        mujoco.mj_forward(self.model, self.data)
        return self.snapshot()

    def snapshot(self):
        # 核心只导出状态；是否跌倒由 sequence.py 的测量和监测器判断。
        state = {
            "time_s": float(self.data.time),
            "control_steps": self.ticks,
            "physics_steps": self.physics_steps,
            "position_m": self.data.xpos[self.root_id].tolist(),
            "quaternion_wxyz": self.data.xquat[self.root_id].tolist(),
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "policy": self.active_policy,
            "fall_verdict": "not_implemented",
        }
        # JSON 拒绝 NaN/Infinity，不能生成伪造的有限状态。
        json.dumps(state, allow_nan=False)
        return state
