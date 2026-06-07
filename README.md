# MuJoCo Skill

一个面向 Codex 的 MuJoCo 技能，用来帮助 AI agent 更稳定地处理 MJCF 场景搭建、模型检查、viewer 启动、actuator 查询和最小控制实验。

演示视频：[使用演示视频](https://www.youtube.com/watch?v=G2hwzWDg8Js&t=15s)

## 快速指南

### 1. 安装技能

在终端执行：

```bash
npx skills@latest add coolbeevip/mujoco-skills
```

安装后重启 Codex，让新的 skill metadata 被重新加载。

### 2. 创建仿真场景

在 Codex 中输入：

```text
用 $mujoco 创建一个 MuJoCo 仿真场景，包含 franka_panda 机器人、一张桌子和一个可抓取物体。保存到 ~/Documents/mujoco/franka_pick/scene.xml，并打开 viewer 检查场景能否正常加载。
```

![Create simulation scene](images/build-scene.png)

### 3. 操作机器人

在 Codex 中输入：

```text
用 $mujoco 打开 ~/Documents/mujoco/franka_pick/scene.xml，查询 franka_panda 的 actuator 和抓取 site，然后操作机器人抓取桌面上的物体。
```

![Pick cube](images/pick-cube.png)

## 更多场景

```text
用 $mujoco 创建一个 Universal Robots UR5e 分拣场景，包含 UR5e、传送带、两个不同颜色的方块和两个收纳盒。保存到 ~/Documents/mujoco/ur5e_sort/scene.xml，并打开 viewer 检查。
```

```text
用 $mujoco 创建一个 Unitree Go1 四足机器人越障场景，包含 Go1、地面、低矮台阶和几个障碍物。保存到 ~/Documents/mujoco/go1_obstacle/scene.xml，并打开 viewer 检查。
```

```text
用 $mujoco 创建一个 Hello Robot Stretch 移动操作场景，包含 Stretch、桌面、杯子和目标托盘。保存到 ~/Documents/mujoco/stretch_tabletop/scene.xml，并打开 viewer 检查。
```

```text
用 $mujoco 创建一个 Shadow Hand 灵巧手操作场景，包含 Shadow Hand、桌面、球体和圆柱体。保存到 ~/Documents/mujoco/shadow_hand_dexterous/scene.xml，并打开 viewer 检查。
```

## License

MIT License. See [LICENSE](LICENSE).
