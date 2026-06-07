# MuJoCo Skill

[English](README.md)

一个面向 AI Agent 的 MuJoCo 技能，用于更可靠地处理 MJCF 场景构建、模型检查、viewer 启动、执行器检查和最小控制实验。

本 README 中的示例和场景检查均在 Codex GPT-5.5 下验证。

## 快速开始

### 1. 安装技能

在终端中运行：

```bash
npx skills@latest add coolbeevip/mujoco-skills
```

安装完成后，重启 Codex，让新的技能元数据重新加载。

### 2. 创建仿真场景

在 Codex 中输入：

> 使用 $mujoco 创建一个 MuJoCo 仿真场景，包含 franka_panda 机器人、一张桌子和一个可抓取物体。保存到 ~/Documents/mujoco/franka_pick/scene.xml，然后打开 viewer 检查场景是否能正确加载。

![创建仿真场景](images/build-scene.png)

### 3. 操作机器人

在 Codex 中输入：

> 使用 $mujoco 打开 ~/Documents/mujoco/franka_pick/scene.xml，检查 franka_panda 的执行器和抓取 site，然后操作机器人抓取桌面上的物体。

![抓取方块](images/pick-cube.png)

## 更多场景

> 使用 $mujoco 创建一个 Universal Robots UR5e 分拣场景，包含 UR5e、传送带、两个不同颜色的方块和两个料箱。保存到 ~/Documents/mujoco/ur5e_sort/scene.xml，然后打开 viewer 检查场景。

![UR5e 分拣场景](images/scene-ur5e.png)

> 使用 $mujoco 创建一个 Unitree Go1 四足机器人越障场景，包含 Go1、地面、低台阶和若干障碍物。保存到 ~/Documents/mujoco/go1_obstacle/scene.xml，然后打开 viewer 检查场景。

![Go1 越障场景](images/scene-go1.png)

> 使用 $mujoco 创建一个 Hello Robot Stretch 移动操作场景，包含 Stretch、桌面、杯子和目标托盘。保存到 ~/Documents/mujoco/stretch_tabletop/scene.xml，然后打开 viewer 检查场景。

![Stretch 桌面操作场景](images/scene-stretch.png)

> 使用 $mujoco 创建一个人形机器人平衡和行走场景，包含人形机器人、地面、低矮踏步方块和标记的行走目标。保存到 ~/Documents/mujoco/humanoid_walk/scene.xml，然后打开 viewer 检查场景。

![Unitree H1 人形机器人行走场景](images/scene-unitree-h1.png)

## 许可证

MIT License。详见 [LICENSE](LICENSE)。
