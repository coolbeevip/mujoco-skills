# MuJoCo Scene Builder

只在任务涉及建模、改模、场景搭建或场景可视化检查时读取本文件。

## 负责什么

- 新建或修改 `MJCF` / `XML`
- 调整 `body` / `joint` / `geom` / `site` / `sensor` / `contact` / `equality` / `inertial`
- 为现有机器人补 `scene.xml`、桌面、障碍物、相机、灯光、抓取物
- 修复编译失败、层级错误、惯性异常、接触穿透、姿态错乱
- 直接打开指定场景做可视化检查

不负责：

- actuator 控制
- 最小控制实验
- 抓夹开合这类运行时动作

这些任务交给 `MuJoCo Robot Control`。

## 默认工作流

1. 明确系统边界：这次是改机器人本体，还是只改顶层场景。
2. 能复用就复用：已有真实机器人时，优先沿用上游或官方模型，不要先手搓替身。
3. 先做最小可编译版本，再补细节。
4. 落盘到 `~/Documents/mujoco`，文件名稳定可读。
5. 做 XML / 编译 / viewer 检查，再向用户汇报。

如果任务包含抓取、抓放、放入盒子、堆叠、拾取这类 manipulation 语义，不要把“模型能编译”和“机械臂能动”误判为“任务可执行”。对这类任务，末端执行器必须额外通过抓取链路检查。

## 场景列表

当用户要求列出所有场景时：

- 默认只读取 `~/Documents/mujoco`
- 只把该目录下的一级目录名当作场景名称
- 不递归列出 `scene.xml`、`assets`、`meshes` 或更深层文件

如果需要脚本化列出，优先使用：

```bash
python - <<'PY'
from path_utils import list_scene_groups
print("\n".join(list_scene_groups()))
PY
```

## 复用优先级

对真实机器人，默认顺序是：

1. MuJoCo 官方 `Model Gallery`
2. `MuJoCo Menagerie`
3. 当前仓库已有相似模型
4. 最后才是从零建模

如果用户真正要的是“把某机器人放进一个操作场景里”，优先保留机器人本体不动，只改顶层 `scene.xml`。

典型最小侵入修改：

- 调整 base 的摆放高度或朝向
- 添加桌面、盒子、木块、地面、障碍物
- 增加 `camera`、`light`、`site`、`keyframe`

不要把“加几个场景物体”升级成“重写机器人本体”。

## 建模铁律

- 一切先看 body 树，别把独立刚体和附属几何混为一谈。
- 优先用局部坐标理解 `body pos`、`joint pos`、`geom pos`、`site pos`。
- 先追求可编译，再考虑 `default class`、抽象复用、include 拆分。
- 惯性和质量不能糊弄；抖动、翻转、力矩离谱时先怀疑惯性。
- 接触问题同时检查 `contype` / `conaffinity` / `solref` / `solimp` / `friction` / 初始穿插。
- 只使用确认存在的属性名；不确定时查官方文档或当前仓库样例。

## 场景视觉默认值

- 默认不要交付高亮、近白的 `skybox`、`haze`、地面棋盘或强头灯组合。
- 除非用户明确要求“明亮展台风格”，背景优先用低饱和、偏暗的蓝灰或中性灰。
- `visual/rgba haze` 不要轻易设到接近 `1 1 1 1`；`texture type="skybox"` 的渐变顶色也不要接近纯白。
- 地面和背景应服务于看清机器人与物体，而不是让大面积高亮背景喧宾夺主、造成刺眼或发白。

## 抓手建模硬约束

当用户要的是抓取、夹取、抓放时，默认按下面的约束检查或建模。少一条都可能出现“机械臂会动，但块抓不起来”。

### 1. 不能只看 actuator，要看完整抓取链路

必须同时具备：

- 合理的末端 frame 或 grasp site
- 至少两个能形成夹持约束的接触面
- 可解释的开合语义
- 夹爪和待抓物之间真实发生接触的碰撞几何

不要把“有一个 `gripper` actuator”误判成“抓手可用”。

### 2. 不要交付单侧运动、无对侧接触面的假抓手

如果模型只有一个 moving jaw，而另一侧没有固定夹持面或对向 jaw，默认认为它不具备稳定夹取能力。

对夹爪类末端，至少满足以下之一：

- 两个对向 jaw，且开合由 joint / equality / tendon 保持几何一致
- 一个 moving jaw + 一个明确的固定 jaw / palm 接触面，并验证确实能把物体夹在中间

### 3. 视觉 mesh 不能直接当抓取碰撞基准

对 gripper / fingertip：

- 可以保留 mesh 作为 visual
- 抓取接触优先补简单 primitive collision geom，例如 `box`、`capsule`
- 指尖接触面要明确、稳定、可推理，不要完全依赖复杂 STL 网格

### 4. 必须定义抓取参考位

至少提供以下 site 中的一种：

- `gripperframe`
- `grasp_center`
- 左右指尖 site

如果任务会用到抓取规划，优先同时提供：

- 一个中心抓取 site
- 两个指尖 site

### 5. 必须给出开合语义

不要只给 joint range / ctrlrange。

必须能回答：

- 哪一端是 open
- 哪一端是 close
- 闭合时夹爪之间理论剩余间距大概多少

如果上游模型没定义清楚，补注释、site、命名或 README 说明；不要把不清楚的抓手直接交付成“可抓取模型”。

### 6. 抓取任务必须做最小抓取验证

如果用户目标是抓放，不要只做这些验证：

- XML 能编译
- viewer 能打开
- actuator 能驱动

必须至少补一个最小抓取检查：

- 闭合时物体是否真正离开桌面
- 抬起后物体 `z` 是否持续增加
- 松手后物体是否从夹爪中脱离

如果没做这一步，交付时必须明确写“运动链路已验证，但抓取链路未验证”。

## 抓放任务的默认分解

当用户要求“把方块放进盒子”时，不要把它当成单一动作。默认拆成：

1. 场景与目标物检查
2. 末端执行器抓取能力检查
3. 接近位 / 抓取位 / 抬起位 / 放置位定义
4. 控制或规划执行

如果第 2 步失败，应先修 gripper 模型，而不是继续堆动作脚本或 IK。

## 打开场景

当任务是“打开场景看看”时：

- 如果用户给了明确路径，直接用明确路径。
- 如果只给文件名，先在当前工作区找，再回退到 `~/Documents/mujoco`。
- 如果用户没给路径，默认打开最近修改的 XML。

为了让后续控制命令能够复用同一个 viewer 进程，不要把场景交给 `Applications/MuJoCo.app` 直接打开。优先通过 skill 自带脚本启动 `mujoco.viewer`：

```bash
python scripts/mujoco_viewer.py /absolute/path/to/scene.xml
```

不要用：

```bash
/Applications/MuJoCo.app/Contents/MacOS/simulate /absolute/path/to/scene.xml
```

因为那样启动的窗口不受当前 skill 的脚本进程管理，后续也无法稳定挂接 `CLI -> socket -> viewer 进程 -> data.ctrl` 这条控制链路。

## 交付要求

- 明确写出最终文件路径
- 说明是改了机器人本体还是只改了顶层场景
- 如果只做了静态建模、还没做运行验证，要直说
