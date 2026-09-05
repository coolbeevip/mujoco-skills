"""固定官方策略的运行接口；文件和时序来自已校验的 manifest.json。"""

# metadata_command 是 ONNX 内的元数据，不等于全部观测槽位的名称。
# 新增策略仍输入 61 维，但前 3 个命令槽可能表示速度、姿态标志或动作阶段。
POLICIES = {
    "standing": ("alpha_stand.onnx", "walk", "twist,head_pose,body_pose"),
    "walking": ("alpha_walking.onnx", "walk", "twist,head_pose,body_pose"),
    "sitstand": ("alpha_sitstand.onnx", "walk", "twist,head_pose"),
    "ground_pick": ("alpha_ground_pick.onnx", "walk", "twist"),
    "kick_left": ("ball_kick_left.onnx", "walk", "twist"),
    "kick_right": ("ball_kick_right.onnx", "walk", "twist"),
    "roulade": ("roulade.onnx", "walk", "twist"),
    "roller": ("roller.onnx", "roller", "twist"),
    "crouch": ("roller_crouch.onnx", "roller", "twist"),
}


def catalog(cache):
    import json

    entries = json.loads((cache / "policies" / "manifest.json").read_text())["policies"]
    by_file = {entry["file"]: entry for entry in entries}
    return {name: by_file[spec[0]] for name, spec in POLICIES.items()}
