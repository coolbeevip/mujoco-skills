# MuJoCo Skill

一个面向 Codex 的 MuJoCo 技能，用来帮助 AI agent 更稳定地处理 MJCF 场景搭建、模型检查、viewer 启动、actuator 查询和最小控制实验。

演示视频：[使用演示视频](https://www.youtube.com/watch?v=G2hwzWDg8Js&t=15s)

## 快速指南

### 1. 安装到 Codex

推荐用 `npx skills` 安装：

```bash
npx skills@latest add coolbeevip/mujoco-skills
```

安装时按提示选择 Codex。如果安装器询问要安装哪个 skill，选择 `mujoco`。

如果要显式指定这个 skill，可以使用：

```bash
npx skills@latest add coolbeevip/mujoco-skills --skill mujoco
```

也可以在 Codex 里让内置安装技能处理：

```text
用 $skill-installer 安装 https://github.com/coolbeevip/mujoco-skills/tree/main/skills/mujoco
```

安装后重启 Codex，让新的 skill metadata 被重新加载。

### 2. 在 Codex 中使用

在对话里显式调用 `$mujoco`，然后说明你要处理的 MuJoCo 任务。

```text
用 $mujoco 打开 ~/Documents/mujoco/so101/scene.xml，检查模型能不能正常加载。
```

```text
用 $mujoco 给这个 MJCF 场景加一个桌面、一个红色方块和一个俯视相机。
```

```text
用 $mujoco 查询这个机械臂的 actuator，然后小幅闭合抓夹并 step 120 帧。
```

```text
用 $mujoco 列出 ~/Documents/mujoco 下面已有的所有场景。
```

## 这个技能做什么

- 按任务类型分流到场景搭建或机器人控制，避免一次性加载过多上下文。
- 指导 agent 新建或修改 `MJCF` / `XML`、补桌面、物体、相机、灯光和抓取 site。
- 使用内置脚本打开 `mujoco.viewer`，并通过本地 UNIX socket 控制同一个 viewer 进程。
- 对抓取任务强制检查末端执行器、碰撞几何、抓取参考位、开合语义和最小抓取验证。
- 默认把 MuJoCo 场景放在 `~/Documents/mujoco`，便于跨任务复用。

## 仓库结构

```text
skills/
└── mujoco/
    ├── SKILL.md
    ├── references/
    │   ├── robot-control.md
    │   └── scene-builder.md
    └── scripts/
        ├── env_bootstrap.py
        ├── mujoco_cli.py
        ├── mujoco_viewer.py
        └── path_utils.py
```

## 依赖

- Python 3
- MuJoCo Python 包：`mujoco`
- macOS 图形 viewer 需要 MuJoCo 提供的 `mjpython` launcher

技能内的 `scripts/env_bootstrap.py` 会检查 `mujoco` 包；缺失时会尝试通过当前 Python 的 `pip` 安装。

## 说明

这个仓库只包含 agent skill、参考文档和辅助脚本，不包含机器人模型资产。实际抓取和控制效果取决于本地 MuJoCo 环境、MJCF 模型质量、actuator 定义和接触几何。对抓放任务，模型可编译和 viewer 可打开不等于抓取链路已经验证。

## License

MIT License. See [LICENSE](LICENSE).
