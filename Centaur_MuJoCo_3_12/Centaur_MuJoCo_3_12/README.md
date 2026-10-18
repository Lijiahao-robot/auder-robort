# 半人马机器人 · MuJoCo 3.12.0

原创参数化教学机器人：四足底盘上安装人形躯干、双臂和头部，包含 25 个关节执行器、6 自由度浮动基座、足底接触和网页控制台。支持四足站立、慢走、挥手、小幅律动及键盘控制。

**默认启用可关闭的辅助稳定：外部力和力矩帮助基座保持姿态、高度与参考速度。关节和接触由 MuJoCo 物理步进求解。当前交付验证的是辅助步态演示，无辅助自主行走尚未验证。**

这是可修改结构和控制参数的教学模型，没有对应某款商业机器人的实测惯性或电机参数。

## Windows 一键启动

需要已安装 Python，建议 Python 3.11/3.12。解压后双击：

```text
START_CENTAUR.bat
```

首次启动会在工程目录创建 `.venv`，安装固定版本 MuJoCo 3.12.0，以及 Pillow、NumPy 等依赖，随后自动打开浏览器。首次安装需要网络；后续启动会先检查已有环境。

也可在包含 `bootstrap.py` 的文件夹打开 PowerShell/CMD：

```bash
python bootstrap.py
```

页面默认地址为 `http://127.0.0.1:8700/`。终端需要保持运行，Ctrl+C 退出。端口占用时：

```bash
python bootstrap.py --port 8701
```

如果出现 `ModuleNotFoundError: No module named 'PIL'`，运行 `python bootstrap.py`，由启动器安装到本工程使用的环境。依赖包名称是 Pillow，代码导入名称是 PIL。

若提示符为 Python 的 `>>>`，先执行 `exit()` 回到系统终端，再输入上述命令。

## 网页操作

1. 选择动作模式和慢走参考速度，保留“辅助稳定”勾选。
2. 点击“开始 / 应用设置”。
3. 观察机器人、仿真时间、基座高度、位移和四足接触力。
4. 点击“暂停”冻结物理时钟；“重置”恢复初始姿态并清空累计指标。

| 键盘 | 动作 |
| --- | --- |
| W / ↑ | 前进 |
| S / ↓ | 后退 |
| A / ← | 左移 |
| D / → | 右移 |
| Q / E | 左转 / 右转 |
| Space | 暂停仿真 |

也可长按网页方向按钮。键盘输入自动切换为“键盘控制”，松开后目标速度归零并平滑减速。网页每 0.18 秒刷新按键指令；超过 0.65 秒未收到刷新，执行线程会将键盘速度归零。网页失去焦点且有按键活动时请求暂停。

慢走滑块是周期步态的参考速度，不保证实际速度完全跟随。实际位移、地面反力和跟踪误差见网页与运行报告。

关闭辅助后，仿真继续使用浮动基座和真实重力。检测到基座过低或明显倾倒时暂停，点击重置可重新开始。这是动力学实验入口，不代表已提供无辅助平衡控制器。

## 原生 MuJoCo 窗口与无窗口运行

通过同一启动器使用已经准备好的环境：

```bash
python bootstrap.py --app sim --mode walk --seconds 20
python bootstrap.py --app sim --headless --mode stand --seconds 12
python bootstrap.py --app sim --headless --mode walk --speed 0.12 --seconds 12
```

只准备环境、不启动应用：

```bash
python bootstrap.py --prepare-only
```

Linux/macOS 也可使用这些入口；Linux 原生窗口需要图形会话。macOS passive viewer 按 MuJoCo 官方要求使用环境内的 `mjpython`，例如 `.venv/bin/mjpython run_sim.py --mode walk`。本次没有进行 macOS 或 Windows 窗口实测。

## 视频与结果

```bash
python bootstrap.py --app sim --render --demo --seconds 14
```

按仿真时间依次生成站立、慢走、挥手和小幅律动的演示视频。模型动作由力矩控制与 `mj_step` 产生，渲染离线运行，视频为 30 fps。

| 输出 | 内容 |
| --- | --- |
| `results/centaur_demo.mp4` | 960×640、H.264 演示视频 |
| `results/centaur_preview.png` | 视频中的一帧 |
| `results/run_report.json` | 位移、倾角、接触、关节误差与外部辅助统计 |

安装包已附一段视频。新运行会覆盖同目录的结果；`--output results_wave` 可指定其他目录。CLI 检测到倒地后保留报告，并以退出码 2 结束。

Linux 无显示的网页/视频入口默认使用 EGL，需要可用的 EGL/OpenGL 驱动。`--headless` 不创建渲染器，可先用于检查物理模型。网页画面约 10–12 fps，实际取决于机器；慢机器上的网页不保证与墙钟严格同步。

## 结构与控制方法

| 部分 | 关节数 |
| --- | ---: |
| 四足，每足外展 / 髋 / 膝 | 12 |
| 腰部旋转 / 侧倾 / 俯仰 | 3 |
| 双臂，每臂肩俯仰 / 肩侧摆 / 肘 / 腕 | 8 |
| 头部转向 / 俯仰 | 2 |
| 合计 | 25 |

模型在 `assets/centaur.xml`，不需要外部网格文件。默认大腿、小腿各 0.30 m，模型总质量约 44.07 kg；这些是工程设定值。

`centaur/model.py` 可重新生成 MJCF。可以修改腿长、几何体、质量、关节范围和力矩限制；重新生成会覆盖 XML。若改变腿部尺寸，还需同步检查逆运动学、目标高度与控制增益。直接编辑 XML 后不必重新生成。

```bash
.venv/bin/python -m centaur.model
```

Windows 对应 `.venv\Scripts\python.exe -m centaur.model`。

当前算法包括：三关节腿解析逆运动学、占空比 0.8 的四足慢速爬行参考、PD 关节跟踪及偏置力补偿。控制周期 0.002 s，`implicitfast` 积分。参考位置保留关节范围余量，电机指令受模型的力矩范围限制。

辅助稳定使用 `xfrc_applied` 向基座施加外部力和力矩，报告中包含峰值；关闭辅助后该外部力清零。运行过程不直接指定基座位置，基座状态由动力学积分得到，重置时恢复初始状态。

足部参与地面接触；教学模型关闭机器人自身碰撞，尚不适合用于自碰撞验证、硬件选型或真机控制。工程不包含真实机器人的通信或驱动接口。

## 验证与工程目录

```bash
.venv/bin/python -m unittest discover -s tests -v
node tests/test_input.mjs
```

Node 只用于开发时测试键盘映射，运行仿真无需 Node。Windows 使用 `.venv\Scripts\python.exe` 运行 Python 测试。

验证覆盖固定版本、模型自由度、腿部逆运动学、站立/挥手/律动、多个参考速度、键盘运动与输入超时、暂停/重置、网页请求校验、渲染帧和报告。验证记录在 `validation/`。

本次实测为 Linux、Python 3.12 和 MuJoCo 3.12.0；网页接口和真实离屏渲染已测试，浏览器视觉/点击与 Windows 双击启动尚未实测。自动安装与依赖导入在 Linux 独立环境中验证。

| 路径 | 工作 |
| --- | --- |
| `bootstrap.py`、`START_CENTAUR.bat` | 独立环境、一键启动 |
| `start_web.py`、`web/` | 网页与键盘控制 |
| `run_sim.py` | 原生窗口、无窗口运行、视频导出 |
| `assets/centaur.xml` | 原创半人马 MJCF |
| `centaur/model.py` | 参数化模型生成 |
| `centaur/simulation.py` | 逆运动学、力矩控制、辅助稳定、接触统计 |
| `centaur/web.py` | 单线程物理/渲染与网页接口 |
| `tests/`、`validation/` | 自动化测试与交付验证 |

参考：[MuJoCo 3.12.0 Python 接口](https://mujoco.readthedocs.io/en/3.12.0/python.html)、[MJCF 格式](https://mujoco.readthedocs.io/en/3.12.0/XMLreference.html)。
