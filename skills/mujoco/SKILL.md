---
name: mujoco
description: "统一处理 MuJoCo 建模与运行控制。适用于用户提到 MuJoCo、MJCF、scene.xml、机器人/机械臂场景搭建、viewer 打开、actuator 控制、最小控制实验、抓夹开合、模型结构检查等需求。触发后先分流到两类渐进式加载内容：MuJoCo Scene Builder 与 MuJoCo Robot Control。"
---

# MuJoCo

这个 skill 统一覆盖 MuJoCo 的两类任务，但不要把两类上下文一次性全读进来。

先判断用户属于哪一类，再按需加载对应 reference：

- **MuJoCo Scene Builder**: 新建或修改 `MJCF` / `XML`、搭场景、放机器人/桌面/物体、修 body/joint/geom/site/contact/inertial 结构、编译调试模型、打开场景做可视化检查。
  读取 [references/scene-builder.md](references/scene-builder.md)。
- **MuJoCo Robot Control**: 检查模型结构、启动 viewer、执行 actuator 控制、做最小控制实验、打开/闭合抓夹、区分模型问题与控制链路问题。
  读取 [references/robot-control.md](references/robot-control.md)。

如果任务同时包含两部分，顺序固定为：

1. 先走 Scene Builder，把模型或场景改到可编译、可打开。
2. 再走 Robot Control，启动 viewer 服务并下发控制。

## 共享规则

- 不要臆造 MuJoCo 标签、属性名或默认行为。语法不确定时，以官方文档和当前工作区已有模型为准。
- 默认输出目录是 `~/Documents/mujoco`。如果用户没给绝对路径，优先在这里落盘、查找或解析模型。
- 当用户要求“列出所有场景”时，默认只从 `~/Documents/mujoco` 读取，并且只返回一级目录名作为场景名称，不递归展开文件。
- 用户只说“打开 MuJoCo / 打开某个 scene.xml”时，先判断意图：
  - 如果是检查场景能否正常打开，走 Scene Builder。
  - 如果后续还要操作机器人，补读 Robot Control。
- 报告结果时，必须明确给出最终模型路径、你实际执行的命令或脚本入口，以及还没验证的风险。

## 官方参考

- MuJoCo modeling: `https://mujoco.readthedocs.io/en/stable/modeling.html`
- MuJoCo python: `https://mujoco.readthedocs.io/en/stable/python.html`
- MuJoCo api: `https://mujoco.readthedocs.io/en/stable/APIreference/index.html`

## 脚本与资源

- Viewer 服务入口：`scripts/mujoco_viewer.py`
- 控制与查询入口：`scripts/mujoco_cli.py`
- 环境与路径辅助：`scripts/env_bootstrap.py`、`scripts/path_utils.py`
- 不要预读整个 `scripts/`。只有需要执行或修改脚本时才打开对应文件。
