# G1 踢足球 · MuJoCo 3.12.0

本工程提供 G1 机器人的**走位、转向、抬脚射门、物理足球和进球判定**。既可以在浏览器里用实体键盘和屏幕按键操作，也可以打开 MuJoCo 窗口观看一次完整演示。

网页显示的是本机 MuJoCo 渲染的实时画面，需要先启动 Python 服务。它不是可直接双击 HTML 运行的静态动画。

## Windows 最简单的运行方法

1. 先安装 **64 位 Python 3.11**，安装时勾选 **Add Python to PATH**。你已有的 Python 3.11.6 可以用于本工程。
2. **完整解压 ZIP**，保留 `assets`、`g1soccer`、`web` 等文件夹。
3. 打开 `G1_Soccer_MuJoCo_3_12_0` 文件夹，双击 **`run_windows.bat`**。
4. 第一次会建立项目内 `.venv` 并安装依赖，随后自动打开浏览器。

默认本机地址为 `http://127.0.0.1:8877`。如果没有自动打开，在运行仿真的同一台电脑浏览器中输入这个地址。运行期间保持命令窗口打开；结束时在命令窗口按 **Ctrl+C**。

依赖已包含 **Pillow**，它提供 `PIL` 模块，启动脚本会一起安装。`mujoco==3.12.0` 已固定在 `requirements.txt`。

要看自动演示，也可以双击 **`run_demo_windows.bat`**：机器人会走位、停步、踢球，动作结束后保留 MuJoCo 窗口，关闭窗口查看报告。

## 键盘与屏幕按键

| 按键 | 功能 |
| --- | --- |
| W / ↑ | 前进 |
| S / ↓ | 后退 |
| A / D | 向左 / 向右横移 |
| Q / E 或 ← / → | 向左 / 向右转向 |
| Shift | 按住加速 |
| Space / K | 踢球，根据球的左右位置选择脚 |
| B | 将球放回脚前，便于反复射门 |
| P | 暂停 / 继续物理仿真 |
| X | 停止走位并退出自动演示；已开始的踢球动作会收回脚 |
| R | 全部复位，清空进球数和射门次数 |

网页右侧有相同的屏幕键盘，移动键支持按住和多点触摸。松开移动键会减速停止；切换窗口、隐藏页面和关闭页面会清除按键。连续 0.45 秒未收到按键更新时，服务端也会清空移动输入并减速停止。

左上方可切换**全场、跟随、后方**三种视角。点击「演示一次射门」会复位场景并执行自动走位和射门。默认步行速度 0.24 m/s，加速 0.36 m/s，转向上限 0.6 rad/s。移动方向相对于机器人朝向。

射门准备提示表示球在脚前的近似范围。它不保证命中；球离得太远、偏在脚侧或正在运动时，机器人可能踢空。先按 B，再按空格，可以快速练习。机器人走位限制在场内，足球可滚出边线，按 B 或 R 重新开始。

## 仿真的实现与范围

机器人采用 Unitree 官方 29 自由度 G1 MJCF 和 STL 资源，保留原始文件、来源记录和 BSD-3-Clause 许可。默认足球半径 **0.105 m**、质量 **0.43 kg**，球门宽 1.8 m、高 0.9 m。

本项目采用**虚拟底座辅助的教学控制**：一个 mocap/weld 约束支撑并移动浮动骨盆，预设足端轨迹由腿部逆运动学转换为关节目标，关节使用力矩限幅的 PD 和重力/科氏项补偿。它没有自主平衡控制或强化学习步态。机器人只增加脚底碰撞垫，不提供完整的机身/自碰撞处理。

足球是独立自由刚体。**踢球动作不直接设置球的速度、不对球施加额外推力，也不把球焊接到脚上**；球的运动来自 MuJoCo 求解的脚球接触。仅显式调用 B/R 复位或初始化场景时会设置球的位姿和速度。球门柱、横梁和后网碰撞由 MuJoCo 处理。

计分要求整个球越过球门线，并在越线时位于门柱之间、横梁下方；同一球只计一次，球反弹回来不会重复计分。B 复位球保留累计进球数，R 复位清空数据。这里的计分是训练场逻辑，不包含完整足球比赛规则。

网页采用本机 HTTP + MJPEG 实时图像。物理计算与渲染使用独立状态和线程，慢渲染不会阻塞按键处理。帧率取决于电脑显卡和驱动，页面显示实际渲染 FPS；未达到设定 FPS 时会显示较低数值。

本工程不包含 Unitree SDK、DDS、ROS 或硬件连接，不向真机发命令。

## 命令行运行

下面的命令输入 Windows 命令提示符/PowerShell 或 Linux 终端，**不要输入 Python 的 `>>>` 解释器**。先进入实际解压目录，路径有空格时加引号。

```bat
cd /d "D:\你的解压位置\G1_Soccer_MuJoCo_3_12_0"
python -m pip install -r requirements.txt
python start_web.py
```

PowerShell 不使用 `cd /d`，改为 `cd "D:\你的解压位置\G1_Soccer_MuJoCo_3_12_0"`。若在 `>>>` 中，先输入 `exit()` 返回终端。

```bash
# 降低画面分辨率与刷新率；也可作为 run_windows.bat 的参数。
python start_web.py --width 640 --height 360 --fps 15

# 8877 被其他程序占用时。
python start_web.py --port 8878

# 只做数值仿真和验证，不创建图形窗口。
python g1_soccer.py --headless --demo --verify

# 实际 MuJoCo 画面录像。
python g1_soccer.py --headless --demo --verify --video results/my_demo.mp4
```

Linux 无桌面环境时，入口脚本默认使用 EGL；如驱动需要，可自行设置合适的 `MUJOCO_GL`。Windows 使用 MuJoCo 的默认 OpenGL 后端。

## 文件说明

| 文件 | 用途 |
| --- | --- |
| `run_windows.bat` | Windows 一键安装依赖并打开网页 |
| `run_demo_windows.bat` | Windows MuJoCo 自动射门窗口 |
| `start_web.py` | 本机网页服务入口 |
| `g1_soccer.py` | 数值演示、原生窗口、录像与报告入口 |
| `g1soccer/simulation.py` | 行走、踢球、IK/PD、物理状态与计分 |
| `g1soccer/scene.py` | 球场、球门、足球和辅助骨盆场景 |
| `g1soccer/webserver.py` | 控制队列、物理线程、渲染快照与 HTTP 接口 |
| `web/` | 中文界面、实体键盘和屏幕键盘 |
| `assets/g1/` | 原始 G1 模型、36 个网格、来源与许可 |
| `assets/football.png` | 程序生成的足球纹理 |
| `scene.xml` | 可独立加载的场景 MJCF，含 ready 初始关键帧；控制器仍需 Python 入口 |
| `results/g1_soccer_demo.mp4` | 本次录制的实际 MuJoCo 演示 |
| `results/validation_*.json` | 本次执行的验证报告 |

## 已执行的验证

本次使用 **MuJoCo 3.12.0 / Python 3.12 / Linux EGL** 完成：

- 默认自动演示：行走约 **0.578 m**，脚球接触 10 个物理步，球速峰值约 **5.126 m/s**，进球 1 次，未出现关节越限或非有限状态。
- 单独的右脚和左脚射门均产生物理接触并进球。
- 关闭脚部碰撞的反例：踢腿后足球没有水平位移，未进球。
- 计分检查：门外、横梁上方、地面下方不计分；使用越线时的插值位置；整球越线、单次计分和左右球门成立。
- 9 项本机网页检查：页面与屏幕键盘资源、真实 JPEG、MJPEG、走位与心跳超时、暂停、组合横移/转向与停止、过期指令过滤、错误输入/来源处理、网页发起的脚球接触和进球。
- JavaScript 输入逻辑：同向别名按键释放、多点触摸、组合输入、加速、清空及动作映射。
- 导出的 MJCF 可加载：29 个执行器、1 个辅助骨盆约束，足球没有 mocap 或焊接约束。

当前环境没有执行 Windows 窗口操作或浏览器界面视觉测试。脚本、网页接口、服务器控制与 MuJoCo 实际图像已验证；本机显示效果仍取决于显卡、驱动和浏览器。

复查命令：

```bash
python tools/verify_physics.py
python tools/verify_web.py
node tools/verify-input.mjs
```

`verify_web.py` 会创建一个临时本机端口的仿真服务，测试结束自动停止。每次正常退出网页服务会更新 `results/web_last_run.json`。

## 常见问题

| 提示 | 处理方式 |
| --- | --- |
| 找不到 requirements.txt / Python 脚本 | 完整解压并进入本项目文件夹，推荐直接双击 `run_windows.bat` |
| `ModuleNotFoundError: No module named PIL` | 用运行入口的同一个 Python 执行 `python -m pip install -r requirements.txt`，或运行一键脚本 |
| 地址打不开 | 保持启动命令窗口运行，在运行仿真的同一台电脑浏览器里打开打印的地址 |
| 端口被占用 | 关闭旧实例，或使用 `run_windows.bat --port 8878` |
| OpenGL / GLFW 初始化失败 | 更新显卡驱动，确认本机可运行 MuJoCo；可先使用 `--headless --verify` 验证物理部分 |
| 画面刷新较慢 | 选择 640×360、15 FPS；实际帧率由电脑渲染能力决定，物理与按键仍独立运行 |
| 机器人踢空 | 先按 B 将足球放到脚前，待球停稳后按空格 |

官方资料：[MuJoCo 3.12.0](https://pypi.org/project/mujoco/3.12.0/)、[Unitree MuJoCo 模型](https://github.com/unitreerobotics/unitree_mujoco)。新增代码采用 MIT 许可，Unitree 模型许可见 `assets/g1/UNITREE_LICENSE`。
