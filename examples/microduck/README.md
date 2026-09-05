# MicroDuck CPU 场景样例

面向希望在本机运行 MicroDuck 的开发者：显式获取官方模型与预训练权重，使用 MuJoCo、ONNX Runtime 和 BAM 电机模型完成无窗口动作验证或键盘控制，无需 CUDA 或自行训练。

三个入口共用物理控制核心：`sequence.py` 自动验证站立、前进、左转、减速和最终站稳；`keyboard.py` 提供九份官方策略的场景交互；`policy_demo.py` 无窗口检查各策略执行和直立恢复。完整序列通过为 `passed`，键盘正常退出为 `stopped`，策略演示执行完毕为 `executed`。

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

成功时退出码为 0；JSON 包含总体 `status: passed`、三个 `runs` 以及每阶段测量。任一次失败返回非零，保留失败原因，剩余轮次标为 `not_executed`，不筛选成功结果。

然后启动键盘体验（保持虚拟环境激活，在同一终端执行）：

```sh
mjpython keyboard.py
```

macOS 的 MuJoCo 窗口通过安装依赖时生成的 `mjpython` 启动。Linux 尽力兼容但未实测，不列为通过平台。

## 键盘操作

请把输入焦点放在**启动程序的终端**，不是 MuJoCo 窗口。使用小写按键：

| 按键 | 指令 |
| --- | --- |
| `w` | 前进（0.3 m/s） |
| `a` / `d` | 左转 / 右转（±1.2 rad/s） |
| 空格 / `s` | 清除运动指令，基础模式下站立平衡 |
| `q` | 正常退出 |

每次移动按键替换完整指令；**松键不会停止**，指令保持至下一次有效按键。未知键（含大写字母）忽略，不支持按住组合键。关闭窗口也会结束运行。Ctrl+C、物理异常或当前模式的安全检查失败以非零状态终止，并恢复终端输入设置。

鼠标可调整视图，但窗口只持有模型/数据副本，不会拖动物理机器人或修改实际控制参数。停止后允许平衡摆动，并不冻结关节。终端输出 `INPUT` 记录按键发生的物理状态；正常退出输出 `RESULT`（`status: stopped`）及轨迹摘要。键盘体验不是固定序列的替代验收。

如提示 GUI 启动失败，检查是否使用 `mjpython`；无图形环境可运行 `python sequence.py`。如果输入无响应，先把焦点切回启动终端。

## 官方策略与动作

策略来自 [Pollen Robotics 的 Hugging Face 仓库](https://huggingface.co/pollen-robotics/microduck-policies/tree/088524a64e2557dc453256b6071dbb9d23888802)，九份权重均固定版本并校验 SHA-256。更新样例后执行 `python assets.py prepare`，补齐新增权重、带轮模型与球场景；已校验的资产会复用。

### 步行模型

```sh
mjpython keyboard.py --mode walk
mjpython keyboard.py --mode walk --ball
```

`--ball` 加入官方球模型。触发踢球时，球被放到对应脚前方并清零球速；机器人位姿保持不变。未加该参数时执行踢腿动作，场景中没有球。

| 策略 | 按键 | 执行方式 |
| --- | --- | --- |
| `alpha_stand.onnx` | 空格 / `s` | 零运动指令下持续站立平衡 |
| `alpha_walking.onnx` | `w/a/d` | 根据期望速度持续行走或转向 |
| `alpha_sitstand.onnx` | `y` | 坐下后保持，再按 `y` 站起 |
| `alpha_ground_pick.onnx` | `g` | 俯身拾取动作，执行 2.8 秒 |
| `ball_kick_left.onnx` | `k` | 左脚踢球动作，执行 0.5 秒 |
| `ball_kick_right.onnx` | `l` | 右脚踢球动作，执行 0.5 秒 |
| `roulade.onnx` | `r` | 单次翻滚动作，执行 1 秒 |

先按空格或 `s`，等待机器人站稳，再触发动作。启动条件为零运动指令、无头部/躯干触地、躯干倾角 ≤20°、基座水平速度 ≤0.15 m/s。未满足时，终端 `INPUT.response` 会说明拒绝原因。

坐下使用姿态标志 `1`，保持至少 2 秒后可再次按 `y`；站起使用同一策略的标志 `0`，执行 1 秒后交回站立策略。这里的标志不是前进速度。俯身动作使用周期 4 秒的 cos/sin 阶段信号，执行到阶段 0.7 时结束；踢球和翻滚使用全零命令。

### 带轮模型

```sh
mjpython keyboard.py --mode roller
```

| 策略 | 按键 | 执行方式 |
| --- | --- | --- |
| `roller.onnx` | `w` | 期望前向速度 0.3 m/s |
| `roller.onnx` | `a/d` | 零前向速度，持续给出 ±0.5 rad 相对航向误差 |
| `roller.onnx` | 空格 / `s` | 零命令平衡 |
| `roller_crouch.onnx` | `c` | 蹲伏动作，周期 5 秒，执行到阶段 0.7，共 3.5 秒 |

带轮模型包含四个被动轮关节，仍由策略控制 14 个电机关节。初始基座高度为 0.1385 m，轮轴摩擦参数为 0.003，两份轮式策略动作缩放为 0.8。第三个命令分量表示相对航向误差（rad），步行模式中则表示角速度（rad/s）。切换模型需退出后重新启动；本样例的轮式模式只开放这两份轮式策略。

### 动作切换与检查范围

动作和恢复期间，新动作与移动请求会被忽略，不排队，也不连续重复翻滚。空格或 `s` 清除运动目标，正在执行的动作继续到规定时长；`q` 或关闭窗口立即结束程序。坐下保持期间通过 `y` 发起站起。

一次性动作或站起结束后，交回基础策略执行 2 秒零命令恢复。终端的 `BEHAVIOR` 输出显示阶段变化，`idle` 表示可以接收下一次动作。动作和恢复窗口内允许主动倾斜及头部/躯干触地，并记录实际接触步数和最大倾角；物理数值异常、MuJoCo 警告或缺失采样仍立即失败。恢复到期仍触地或倾角 >45° 时停止程序，此后恢复基础模式的严格跌倒检测。坐下保持阶段持续使用动作检查规则。

`sequence.py` 始终使用原有步行模型和严格站立/跌倒判据，不受动作模式影响。策略演示中的 `completed_windows` 只表示规定执行窗口结束；`action_success_verified` 为 `false`，不代表已踢中球、拾起物体或完整翻滚。俯身场景没有可拾取物体，也未模拟夹持。

### 无窗口策略演示

```sh
python policy_demo.py --policy all
python policy_demo.py --policy sitstand
python policy_demo.py --policy crouch
```

每份策略从独立初始状态开始，自动选择对应模型；踢球演示包含球。`executed` 表示执行及恢复检查完成，任一演示失败则进程返回非零。九份策略的进程级禁网运行结果见 [policy-verification.json](policy-verification.json)。两个模型的真实窗口按键、动作切换和退出记录见 [policy-keyboard-verification.json](policy-keyboard-verification.json)。

## 在 PyCharm 中调试（macOS）

先完成快速开始中的依赖安装与资产准备。调试采用本机 **Python Debug Server**：终端通过 `mjpython` 启动程序，PyCharm 提供断点、单步执行和变量查看。

### 1. 配置调试服务器

在 PyCharm 中打开当前仓库，将项目解释器设为 `examples/microduck/.venv/bin/python`，然后进入 **Run → Edit Configurations → + → Python Debug Server**：

- Name：例如 `MicroDuck Debug`。
- IDE host name：`127.0.0.1`。
- Port：`5678`。
- Suspend after connect：勾选。
- Redirect output to console：取消勾选，输入输出保留在启动终端。
- Path mappings：本机使用同一份源码时留空。IDE 与终端使用相同的项目目录。

### 2. 安装匹配当前 PyCharm 的调试包

配置页面的 **Update your script** 区域会给出 `pip install pydevd-pycharm~=...` 命令。使用页面提供的版本约束，将调试包安装到样例 `.venv`。版本随 PyCharm 版本变化，参见 [JetBrains 官方配置说明](https://www.jetbrains.com/help/pycharm/run-debug-configuration-python-remote-debug.html)。

从仓库根目录执行以下安装模板；**先把 `VERSION_FROM_PYCHARM` 替换为配置页面显示的版本号**。若页面给出的运算符是 `==`，也一并按页面替换：

```sh
cd examples/microduck
.venv/bin/python -m pip install "pydevd-pycharm~=VERSION_FROM_PYCHARM"
.venv/bin/python -m pip show pydevd-pycharm
.venv/bin/python -c "import pydevd_pycharm; print(pydevd_pycharm.__file__)"
```

包名是 `pydevd-pycharm`，Python 导入名是 `pydevd_pycharm`。最后两条命令应显示安装版本及 `.venv` 内的模块路径。它是可选开发依赖，没有加入固定的运行依赖；不启用调试参数时无需安装。升级 PyCharm 后，重新按配置页面核对版本。

### 3. 启动与断点调试

先在 PyCharm 中选择 `MicroDuck Debug` 配置并点击 **Debug**，等待服务器监听。然后在 PyCharm 的 **Terminal** 或系统终端中，从 `examples/microduck` 执行：

```sh
.venv/bin/mjpython keyboard.py --pycharm-debug
```

调试选项可与场景参数组合，例如 `.venv/bin/mjpython keyboard.py --mode roller --pycharm-debug`。

默认连接 `127.0.0.1:5678`。如果服务器配置为其他端口，例如 `6789`，使用：

```sh
.venv/bin/mjpython keyboard.py --pycharm-debug 6789
```

连接成功后，PyCharm 显示调试会话，程序在创建 `Runtime` 和窗口前暂停。在 `keyboard.py` 的 `Runtime(...)` 或 `runtime.py` 的 `step()` 中设置断点，再点击 **Resume Program** 或单步执行。

继续运行后，在**启动终端**输入 `w/a/d`、空格或 `s`、`q`。断点暂停期间控制循环暂停，恢复后继续执行。调试完成后去掉 `--pycharm-debug` 即恢复普通运行。

### 调试排障

- `PyCharm debugging requires pydevd-pycharm`：按上面的安装步骤将调试包安装到 `.venv`，并检查模块路径。
- `Cannot connect to PyCharm`：先启动 **Python Debug Server**，核对端口是否与命令一致，必要时按当前 IDE 提示重新安装匹配版本。
- `keyboard requires a terminal`：从 PyCharm Terminal 或系统终端启动。
- 找不到 **Python Debug Server** 配置：确认当前 PyCharm 安装提供该功能。
- 网络限制：调试需要允许本机 TCP 连接。下文的禁网验证适用于未启用调试的普通运行。

如果只调试无窗口控制逻辑，可使用普通 Python Debug 配置：脚本选 `sequence.py`，解释器选本目录 `.venv/bin/python`，工作目录选 `examples/microduck`，无需 `mjpython` 或上述调试连接参数。

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

资产保存在本目录 `.cache/`，不会加入 Git。`prepare` 显式联网下载，复用校验通过的文件；对缺失或损坏文件下载到临时文件，哈希通过后才替换。九份权重、模型 XML、所有引用网格与许可文件均由 [assets.lock.json](assets.lock.json) 固定来源和 SHA-256。

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

保留自由基座、重力、原始场景碰撞。默认步行模型在初始化/显式 reset 时设定根部高度 0.125 m、单位四元数、官方默认关节位姿、零速度；无隐藏预热。reset 重建 MuJoCo/BAM 对象，清除可变摩擦和控制历史。运行中不覆盖机器人位姿；动作恢复通过策略和力矩执行，检测失败后停止。

`Runtime(..., mode="roller")` 选择带轮模型，`Runtime(..., ball=True)` 在步行模型中加入球。`step_policy(name, command)` 显式选择策略，输入须满足该策略的命令约束；[behaviors.py](behaviors.py) 负责按键请求、阶段编码、定时结束和恢复窗口。学习控制流程可从 `keyboard.py → Behaviors.step() → Runtime.step_policy()` 阅读，普通移动由 `Runtime.step()` 自动选择基础策略。

执行器按官方 CPU 路径转换为 BAM XL330 M6 扭矩电机：7.4 V、固件 P 增益 200、电压降系数 0.1 V/Nm、电压下限 6 V、无固件电流限制；启用同源惯量和刚性摩擦约束。不能改用普通 PD 并声称同一物理契约。

[contract.json](contract.json) 记录观测区间、默认位姿、命名测量对象和碰撞候选映射。权重内置观测归一化，运行时不重复归一化。头部碰撞体挂在 `jaw_soft` 上，躯干为 `trunk_base`；`sequence.py` 已用实际模型验证头部/躯干触地检测和足部接触排除。原始 Runtime 快照不单独执行判定，其 `fall_verdict: not_implemented` 不能当作序列结论；完整结果以 sequence 报告为准。此样例不是完整 RL 环境，不含奖励、训练、随机化或真实机器人控制。

## 验证与维护

```sh
python -m pip install pytest==8.4.2
python -m pytest -q tests
```

真实模型测试要求先完成 `assets.py prepare`，不跳过缺失资产。覆盖缓存错误、输入拒绝、原子控制更新、关节语义不匹配、精确步进、重置重复性、真实接触、阈值边界、阶段时限和失败锁存。无既有旧接口需要兼容。

实测运行组合：Python 3.12.12、NumPy 2.3.2、MuJoCo 3.7.0、ONNX Runtime 1.23.2 CPUExecutionProvider、BAM 1.0.1（固定 Git 提交）。[verification.json](verification.json) 保存短闭环证据，[keyboard-verification.json](keyboard-verification.json) 保存真实窗口按键记录；同一输入的无窗口重放与有窗口运行，全部 qpos/qvel 轨迹哈希完全一致。该精确轨迹回归只在 macOS arm64 执行，其他平台明确跳过。

已使用 macOS `sandbox-exec` 对测试进程禁止所有网络操作，确认网络调用返回 PermissionError；两个入口仍可运行，见 [offline-verification.json](offline-verification.json)。隔离不改变整机网络配置。可在资产已准备后复现：

```sh
sandbox-exec -p '(version 1)(allow default)(deny network*)' python sequence.py
sandbox-exec -p '(version 1)(allow default)(deny network*)' mjpython keyboard.py
```

以上是 macOS 专用验证命令，不是普通启动的必要条件。首次依赖安装和 `assets.py prepare` 不应放在禁网环境中。

窗口关闭路径可用 `mjpython tests/gui_close_smoke.py` 复测：它打开真实窗口，约一秒后调用公开的 `viewer.close()`，检查正常退出和终端恢复。按键退出、启动失败和终端设置恢复也已验证；未依赖操作系统辅助功能权限自动点击关闭按钮。

## 来源与许可

- [Pollen Robotics 仿真模型](https://github.com/pollen-robotics/microduck_rl/tree/29e887ecfbf5d37144759e5a9f8a176dfb83d547)，Apache-2.0。运行核心的位姿和执行器配置改编自该提交的 `scripts/infer_policy.py`，删去交互、训练依赖和扩展行为，改为同步入口与完整 reset。
- [官方权重](https://huggingface.co/pollen-robotics/microduck-policies/tree/088524a64e2557dc453256b6071dbb9d23888802)，此固定提交的 README 单独声明 Apache-2.0。保留缓存中的 README 许可声明，不以代码仓库许可替代权重许可。
- [Rhoban BAM](https://github.com/Rhoban/bam/tree/62bd8ce12154340be97e06f7f41a0ca8f116d967)，Apache-2.0，版权 Marc Duclusaud & Grégoire Passault。固定到官方仿真仓库 lock 指定的提交，不跟随浮动分支。

许可正文见 [LICENSE-Apache-2.0](LICENSE-Apache-2.0)。模型、权重和 BAM 保持各自许可；升级任一固定版本都需要重新核对控制语义和运行证据。
