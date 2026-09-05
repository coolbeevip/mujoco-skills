# MicroDuck CPU 场景样例

面向希望在本机运行 MicroDuck 的开发者：显式获取官方模型与预训练权重，使用 MuJoCo、ONNX Runtime 和 BAM 电机模型运行无窗口短闭环，无需 CUDA 或自行训练。

当前已提供无窗口固定序列验证：站立、前进、左转、减速和最终站稳。键盘窗口入口尚未实现。`sequence.py` 的 `passed` 表示完整序列通过；`smoke_passed` 仍只表示基础短闭环和重置重复性通过。

## 快速开始

需要 Python **3.12**、Git 和首次安装/下载时的网络。实测平台：macOS 15.7.3、Apple Silicon；Linux 尚未验证。BAM 固定版本要求 Python `>=3.12,<3.13`。

从仓库根目录执行：

```sh
cd examples/microduck
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python assets.py prepare
python sequence.py
```

成功时退出码为 0，JSON 中包含：

完整 JSON 包含总体 `status: passed`、三个 `runs` 以及每阶段测量。任一次失败返回非零，保留失败原因，剩余轮次标为 `not_executed`，不筛选成功结果。

## 完整序列与判断方式

```sh
python sequence.py
python sequence.py > sequence-result.json
```

每次从同一初始状态和 seed=0 开始：

| 阶段 | 目标与失败边界 |
| --- | --- |
| 初始站立 | 5 秒，全程平面偏移 ≤0.05 m、航向偏差 ≤10° |
| 前进 | 实际投影达到名义目标 0.3 m 时交接；交接值须在 0.2–0.4 m，最多 10 秒 |
| 左转 | 连续航向变化达到名义目标 90° 时交接；交接值须在 75–105°，最多 10 秒 |
| 减速 | 零运动指令固定 3 秒 |
| 最终站稳 | 重新记录窗口起点，连续 5 秒使用与初始站立相同阈值 |

前进量投影到阶段起始朝向，另报左向横移；航向展开 ±180° 绕回，左转为正。每个 5 ms 物理步检测接触、倾角和窗口越界；阶段交接在 20 ms 控制边界检查实际状态。时间使用整数物理步，10 秒恰好达标可通过，晚一个物理步不能补算成功。窗口越界后回到原位仍失败。

躯干/头部实际触地（模型接触距离 ≤0）立即失败；躯干竖直轴倾角 >45° 连续至少 0.2 秒也失败，回到 ≤45° 即清零。计时从首次观测到超限的采样时刻开始，采样精度 5 ms。足部正常接触不算跌倒。无效物理状态、超时和 Ctrl+C 均非零退出，不自动复位继续同组。

实测三次结果相同：前进 0.30120 m / 3.22 s，左转 90.0705° / 2.92 s；初始窗口最大偏移 3.913 mm、最大航向偏差 1.5281°，最终窗口 0.328 mm / 0.01722°。[sequence-verification.json](sequence-verification.json) 保存完整结果。这仅验证固定条件，不证明其他地形或扰动下的泛化。

基础排障可运行 `python smoke.py`：每次 10 秒，前 5 秒零指令、后 5 秒 0.1 m/s 指令，重复三次并比较 qpos/qvel（容差 `1e-10`）。低速指令几乎不动；完整序列使用 0.3 m/s 前进指令和 1.2 rad/s 转向指令，仍以实际测量而非指令值判定。

## 资产与常见错误

资产保存在本目录 `.cache/`，不会加入 Git。`prepare` 显式联网下载，复用校验通过的文件；对缺失或损坏文件下载到临时文件，哈希通过后才替换。两份权重、模型 XML、所有引用网格与许可文件均由 [assets.lock.json](assets.lock.json) 固定来源和 SHA-256。

```sh
python assets.py verify
python assets.py prepare --cache /path/to/microduck-cache
python sequence.py --cache /path/to/microduck-cache
```

普通运行只校验本地文件，不自动下载、升级或使用替代策略。默认缓存相对脚本而非当前目录解析；从其他目录使用脚本绝对路径也可运行。自定义缓存应指定绝对路径。

- `missing asset`：先运行 `assets.py prepare`。
- `checksum/version mismatch`：本地文件不属于固定组合；显式运行 `prepare` 恢复。不会接受手工修改后的模型或权重。
- 下载失败：检查 GitHub/Hugging Face 连接，再重跑 `prepare`；已验证文件无需重下。
- `dependency mismatch` / `BAM revision mismatch`：在 Python 3.12 环境重新安装 `requirements.txt`，不能只安装 PyPI 上同版本号的 BAM。
- `policy ... mismatch`：观测布局、关节顺序或模型不匹配；检查固定资产，不用随机策略绕过。
- `invalid physics state`：物理失败，返回非零；不可当作动作成功。无窗口入口不需要 viewer、显示服务器或后台 socket。

## 运行接口与物理边界

[runtime.py](runtime.py) 提供 `Runtime(cache, seed)`、`reset()`、`step([vx, vy, yaw_rate])` 和 `snapshot()`。可选 `observer` 在每个物理步后收到当前 Runtime；异常立即停止余下子步，开启测量不改变物理更新路径。一次 step 同步推进 4 个 5 ms 物理步，即 50 Hz 控制。单位分别为 m/s、m/s、rad/s，开放范围为 ±0.5、±0.3、±1.5；这些是输入限制，不是速度或动作验收阈值。向量范数不超过 0.05 时使用站立策略并清零 twist，其余使用 walking；这个切换规则来自官方推理入口。

保留自由基座、重力、原始场景碰撞。模型在初始化/显式 reset 时设定根部高度 0.125 m、单位四元数、官方默认关节位姿、零速度；无隐藏预热。reset 重建 MuJoCo/BAM 对象，清除可变摩擦和控制历史。运行时不覆盖位姿、不焊接、不自动起身。

执行器按官方 CPU 路径转换为 BAM XL330 M6 扭矩电机：7.4 V、固件 P 增益 200、电压降系数 0.1 V/Nm、电压下限 6 V、无固件电流限制；启用同源惯量和刚性摩擦约束。不能改用普通 PD 并声称同一物理契约。

[contract.json](contract.json) 记录观测区间、默认位姿、命名测量对象和碰撞候选映射。权重内置观测归一化，运行时不重复归一化。头部碰撞体挂在 `jaw_soft` 上，躯干为 `trunk_base`；`sequence.py` 已用实际模型验证头部/躯干触地检测和足部接触排除。原始 Runtime 快照不单独执行判定，其 `fall_verdict: not_implemented` 不能当作序列结论；完整结果以 sequence 报告为准。此样例不是完整 RL 环境，不含奖励、训练、随机化或真实机器人控制。

## 验证与维护

```sh
python -m pip install pytest==8.4.2
python -m pytest -q tests
```

真实模型测试要求先完成 `assets.py prepare`，不跳过缺失资产。覆盖缓存错误、输入拒绝、原子控制更新、关节语义不匹配、精确步进、重置重复性、真实接触、阈值边界、阶段时限和失败锁存。无既有旧接口需要兼容。

实测运行组合：Python 3.12.12、NumPy 2.3.2、MuJoCo 3.7.0、ONNX Runtime 1.23.2 CPUExecutionProvider、BAM 1.0.1（固定 Git 提交）。[verification.json](verification.json) 保存三次短闭环证据。依赖与资产准备完成后普通运行无需下载；操作系统级断网验收及 GUI 验收留待完整入口交付。

## 来源与许可

- [Pollen Robotics 仿真模型](https://github.com/pollen-robotics/microduck_rl/tree/29e887ecfbf5d37144759e5a9f8a176dfb83d547)，Apache-2.0。运行核心的位姿和执行器配置改编自该提交的 `scripts/infer_policy.py`，删去交互、训练依赖和扩展行为，改为同步入口与完整 reset。
- [官方权重](https://huggingface.co/pollen-robotics/microduck-policies/tree/088524a64e2557dc453256b6071dbb9d23888802)，此固定提交的 README 单独声明 Apache-2.0。保留缓存中的 README 许可声明，不以代码仓库许可替代权重许可。
- [Rhoban BAM](https://github.com/Rhoban/bam/tree/62bd8ce12154340be97e06f7f41a0ca8f116d967)，Apache-2.0，版权 Marc Duclusaud & Grégoire Passault。固定到官方仿真仓库 lock 指定的提交，不跟随浮动分支。

许可正文见 [LICENSE-Apache-2.0](LICENSE-Apache-2.0)。模型、权重和 BAM 保持各自许可；升级任一固定版本都需要重新核对控制语义和运行证据。
