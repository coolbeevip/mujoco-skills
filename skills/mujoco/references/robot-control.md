# MuJoCo Robot Control

只在任务涉及运行、检查、viewer、actuator、抓夹动作或最小控制实验时读取本文件。

## 统一入口

优先使用这两个脚本的组合：

- `scripts/mujoco_viewer.py`
- `scripts/mujoco_cli.py`

补充辅助脚本：

- `scripts/env_bootstrap.py`

当用户只是要求列出所有场景时，不要走控制流程。直接读取 `~/Documents/mujoco` 的一级目录名，或调用：

```bash
python - <<'PY'
from path_utils import list_scene_groups
print("\n".join(list_scene_groups()))
PY
```

## 默认工作流

控制任务按这个顺序推进：

1. `bootstrap`
2. 启动 `mujoco_viewer.py`
3. 用 `mujoco_cli.py` 查询 actuator / 当前 ctrl
4. 用 `mujoco_cli.py` 下发控制

不要跳过前两步，直接进入复杂控制。

## 环境检查优先

在执行 viewer 或控制命令之前，先确认：

- 有可用的 Python 3
- 能导入 `mujoco` Python 包

默认行为不是“等报错再处理”，而是：

1. 先做环境检查
2. 缺库就自动安装
3. 安装完成后再继续执行控制任务

显式检查可直接调用：

```bash
python scripts/env_bootstrap.py
```

如果缺少 `mujoco`，应自动安装并继续，不要把 `ModuleNotFoundError: No module named 'mujoco'` 直接暴露给用户作为第一反馈。

## 动作请求默认视为执行请求

当用户说这些话时，不要停在解释层：

- 打开抓夹
- 闭合夹爪
- 抬一下肩膀
- 弯一下手肘
- 把机械臂回零
- 让某个 actuator 到某个值

默认动作：

1. 先启动或连接到 `mujoco_viewer.py`
2. 通过 `mujoco_cli.py actuators` / `info` 获取最小必要结构信息
3. 信息足够后默认接到可视化 viewer 执行 `set` + `step`
4. 执行后再汇报结果

## actuator 优先，不按 joint 猜

对用户来说，“让机械臂动起来”是动作请求；对 MuJoCo 来说，通常是对 actuator 发控制值。

默认规则：

- 按 actuator 名控制，不按 joint 名臆测
- 从小幅度开始，不要一上来打满
- 先单轴，再多轴
- 先短时，再长时

## viewer 去重

机器人操作默认应接到可视化 viewer，不要默认走离线 headless。

- `mujoco_viewer.py` 启动的 viewer 应由 Python 脚本通过 `mujoco.viewer` 持有，而不是把场景直接丢给 `MuJoCo.app`
- `mujoco_cli.py` 应通过 `CLI -> socket -> 运行中的 viewer 进程 -> data.ctrl` 下发控制
- 只有用户明确要求无窗口、批处理、离线执行时，才用单独的离线脚本或一次性 Python 片段
- 不要因为多次控制请求反复开新窗口

- 在启动 viewer 前，先检查是否已有对应 scene 的 viewer socket
- 如果已有，就直接复用，不再开新窗口
- 结果里说明当前 viewer 已复用

## 常用命令

```bash
python scripts/mujoco_viewer.py ~/Documents/mujoco/so101/scene.xml
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml ping
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml info
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml actuators
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml set shoulder_lift 0.1
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml set-batch shoulder_lift 0.1 elbow_flex 0.6 wrist_roll -0.2
python scripts/mujoco_cli.py --scene ~/Documents/mujoco/so101/scene.xml step 120
```

检查时重点看：

- actuator 是否存在
- `ctrlrange` 是否合理
- joint 拓扑是否符合预期
- 末端 `site` 是否定义好

## 简单动作序列

如果目标只是执行简单动作，不要每次现写一段大 Python。优先流程是：

1. 用 `mujoco_cli.py actuators` 和 `info` 读取最小结构信息
2. 生成一串 `set` + `step` 命令
3. 在运行中的 viewer 上执行

如果需要让多关节或多机械臂“同时起跳”，优先把同一时刻的控制量合并成一次 `set-batch`，不要连续发多个 `set`。

## 抓夹语义

### 打开抓夹

默认顺序：

1. 优先找名字接近 `gripper`、`grip`、`finger` 的 actuator
2. 如果开合方向已知，直接打到张开端
3. 如果方向未知，先看 `ctrlrange`
4. 默认把“打开抓夹”解释为较大一端；若命名或历史结果显示相反，再修正

### 闭合抓夹

默认顺序：

1. 定位抓夹 actuator
2. 把控制值打到与张开相反的一端
3. 执行后汇报实际控制值与结果

不要只把 actuator 名称抛给用户，让用户自己试。

## 运行场景

不要把场景直接交给 `Applications/MuJoCo.app`。默认应启动 skill 的 Python 脚本，由它内部调用 `mujoco.viewer`：

```bash
python scripts/mujoco_viewer.py /absolute/path/to/scene.xml
```

不要用：

```bash
/Applications/MuJoCo.app/Contents/MacOS/simulate /absolute/path/to/scene.xml
```

## 交付要求

- 明确模型路径
- 明确执行的是 `inspect`、`run` 还是 `control`
- 如果只做了结构检查、还没真正驱动 actuator，要直说
