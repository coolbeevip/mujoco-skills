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

NAMES = (
    "left_hip_yaw,left_hip_roll,left_hip_pitch,left_knee,left_ankle,"
    "neck_pitch,head_pitch,head_yaw,head_roll,right_hip_yaw,right_hip_roll,"
    "right_hip_pitch,right_knee,right_ankle"
).split(",")
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


def validate_policy(session):
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
        or meta.get("command_names") != "twist,head_pose,body_pose"
    ):
        raise ValueError("policy semantic contract mismatch")
    reference = np.fromstring(meta.get("default_joint_pos", ""), sep=",")
    if reference.shape != (14,) or not np.allclose(
        reference, POSE, atol=0.00051, rtol=0
    ):
        raise ValueError("policy reference pose mismatch")


class Runtime:
    def __init__(self, cache=DEFAULT_CACHE, seed=0):
        self.cache = verify(cache)
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
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.sessions = {}
        for key, filename in [
            ("standing", "alpha_stand.onnx"),
            ("walking", "alpha_walking.onnx"),
        ]:
            session = ort.InferenceSession(
                str(self.cache / "policies" / filename),
                sess_options=options,
                providers=["CPUExecutionProvider"],
            )
            validate_policy(session)
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
        spec = mujoco.MjSpec.from_file(str(self.cache / "model" / "scene.xml"))
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
        if [self.model.actuator(i).name for i in range(self.model.nu)] != NAMES:
            raise ValueError("model actuator order mismatch")
        self.data = mujoco.MjData(self.model)
        self.controller = MujocoController(
            motor, NAMES, self.model, self.data, vin_drop_gain=0.1, vin_min=6.0
        )
        self.qids = self.model.jnt_qposadr[[self.model.joint(n).id for n in NAMES]]
        self.vids = self.model.jnt_dofadr[[self.model.joint(n).id for n in NAMES]]
        self.root_id = self.model.body("trunk_base").id
        self.gyro = int(self.model.sensor("imu_ang_vel").adr[0])
        self.data.qpos[:7] = [0, 0, 0.125, 1, 0, 0, 0]
        self.data.qpos[self.qids] = POSE
        self.controller.reset(self.data.qpos)
        self.last_action = np.zeros(14, dtype=np.float32)
        self.ticks = 0
        self.failed = False
        self.active_policy = "standing"
        mujoco.mj_forward(self.model, self.data)
        return self.snapshot()

    def observation(self, command):
        rotation = self.data.xmat[self.root_id].reshape(3, 3)
        gravity = rotation.T @ np.array([0, 0, -1])
        return vector(
            np.concatenate(
                (
                    self.data.sensordata[self.gyro : self.gyro + 3],
                    gravity,
                    self.data.qpos[self.qids].astype(np.float32) - POSE,
                    self.data.qvel[self.vids],
                    self.last_action,
                    command,
                    np.zeros(10),
                )
            ),
            61,
            "observation",
        )

    def apply_action(self, action):
        action = vector(action, 14, "action")
        targets = vector(POSE + action, 14, "target")
        self.controller.q_target[:] = targets
        self.last_action = action

    def step(self, velocity=(0, 0, 0)):
        command = vector(velocity, 3, "velocity")
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
        mode = "standing" if np.linalg.norm(command) <= 0.05 else "walking"
        if mode == "standing":
            command[:] = 0
        obs = self.observation(command)
        action = self.sessions[mode].run(["actions"], {"obs": obs[None]})[0][0]
        self.apply_action(action)
        self.active_policy = mode
        for _ in range(DECIMATION):
            self.controller.update()
            if not np.isfinite(self.data.ctrl).all():
                self.failed = True
                raise ValueError("invalid actuator torque")
            mujoco.mj_step(self.model, self.data)
            if (
                not np.isfinite(self.data.qpos).all()
                or not np.isfinite(self.data.qvel).all()
                or np.any(self.data.warning.number)
            ):
                self.failed = True
                raise ValueError("invalid physics state or MuJoCo warning")
        self.ticks += 1
        mujoco.mj_forward(self.model, self.data)
        return self.snapshot()

    def snapshot(self):
        state = {
            "time_s": float(self.data.time),
            "control_steps": self.ticks,
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
