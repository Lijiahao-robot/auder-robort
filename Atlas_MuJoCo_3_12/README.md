# Atlas · MuJoCo 3.12.0 仿真工程

历史液压款 Boston Dynamics Atlas 模型，包含 27 个关节执行器、浮动基座、真实地面接触和教学用全身控制器。支持站立平衡、挥手、下蹲、双脚舞蹈、慢速前进/后退与转向。

模型源自 LocoMuJoCo v0.3.0 中的历史 Atlas 描述，不是新款电动 Atlas。控制代码在此工程中独立实现，不是 Boston Dynamics 的控制系统。

## Windows：直接启动

请先把整个压缩包解压到一个文件夹，再运行批处理；不要在压缩软件预览窗口里运行。

1. 双击 **`1_安装依赖.bat`**。优先使用已安装的 Python 3.11，也支持 Python 3.10 以上版本；脚本会建立项目自己的 `.venv`，检查并修复缺失的 pip，再安装依赖。首次安装需要联网。已有环境不会被删除。
2. 双击 **`2_启动Atlas.bat`** 打开原生 MuJoCo 窗口。
3. 或双击 **`3_自动演示.bat`**，观看约 49 秒的站立、挥手、下蹲、舞蹈、行走和停步演示。
4. **`4_无窗口自检.bat`** 运行可复现的物理检查，并生成 `results/verification_report.json`。

如果你已经在 Python 3.11 环境中安装 MuJoCo 3.12.0，可在工程文件夹运行：

```bat
python -m pip install -r requirements.txt
python run_atlas.py
```

依赖只有 `mujoco==3.12.0`、NumPy 和 Pillow。不需要安装 ROS、PyTorch、CUDA、LocoMuJoCo 或下载模型权重。STL 网格已包含在工程中。

## 键盘操作

请先点击 MuJoCo 画面，使窗口获得键盘焦点。

| 按键 | 功能 |
| --- | --- |
| `1` | 返回站立平衡；行走时先完成当前步 |
| `2` | 右手挥手 |
| `3` | 循环下蹲、起立 |
| `4` | 双脚支撑下的摆臂和身体律动 |
| `5` | 慢速向前行走 |
| `W` / `↑` | 切换为向前行走，指令约 0.03 m/s |
| `S` / `↓` | 切换为向后行走，指令约 −0.025 m/s |
| `A` / `←` | 慢速向左转 |
| `D` / `→` | 慢速向右转 |
| `X` | 完成当前步后停下并恢复双脚站立 |
| `Space` | 暂停 / 继续物理时间 |
| `R` | 复位机器人、计时和步态状态 |
| `P` | 施加一次 35 N、持续 0.12 s 的侧向推力 |
| `Esc` / 关闭窗口 | 退出，并保存运行报告 |

W/S/A/D 是持续的运动指令，松开按键后仍会执行；用 X 停止。慢速步态先移动重心，再抬脚，需要耐心等待约一个步态周期。暂停冻结仿真时间。

鼠标可旋转视角并用滚轮缩放；相机位置会跟随机器人。左上角显示当前模式、步态阶段、模拟时间和已完成步数。

## 命令行

```bash
# 挥手
python run_atlas.py --mode wave

# 慢速前进
python run_atlas.py --mode walk --speed 0.03

# 没有图形桌面的机器也能运行物理仿真
python run_atlas.py --headless --mode walk --seconds 30

# 运行完整自动演示，并输出真实 MuJoCo 渲染图
python run_atlas.py --headless --demo --snapshot results/demo_final.png

# Linux 无显示器环境：有可用 EGL 驱动时可选
python run_atlas.py --headless --mode wave --seconds 6 --render-backend egl --snapshot results/wave.png

# 自检
python verify_simulation.py
```

Linux / macOS 也可使用 `sh run.sh`。macOS 的交互窗口需要 `mjpython`，`run.sh` 会自动选择；例如 `sh run.sh --mode wave`。纯物理 `--headless` 不要求 OpenGL；`--snapshot` 会额外要求可用的图形渲染后端。

## 控制与物理

- 物理步长 **0.002 s**，控制更新周期 **0.008 s**。
- 浮动基座 6 个速度自由度，27 个关节自由度：`nq=34`、`nv=33`、`nu=27`。
- 上游 Atlas 描述使用了 6 个基座滑动/旋转关节；本工程改为四元数自由关节，并恢复左右踝关节的侧倾轴。
- 全身控制器以双脚/支撑脚约束、质心位置、骨盆姿态、关节姿态为加权任务，通过最小二乘计算关节加速度，再用完整质量矩阵和偏置力做逆动力学。
- 接触力按目标地面压力中心分配支撑载荷，并限制摩擦和足底力矩。电机控制量是有上下限的关节力矩，单位 N·m。
- 步态为慢速准静态教程：转移重心 1.6 s、摆腿 1.4 s、落脚停留 0.3 s，抬脚高度约 0.055 m。目标位置和姿态只是控制参考，实际位置由 `mj_step` 和地面接触决定。
- 站立、动作和行走没有基座焊接约束，没有悬挂支撑力，没有逐帧写入基座位置来假装移动。只有初始化和用户复位时设置 `qpos`；P 键的短时推力是明确的扰动实验。
- 自碰撞在此教程中关闭，脚底碰撞形状作了简化。碰撞网格、质量和惯性沿用历史描述并做兼容修改；它们不是新款 Atlas 的已校准动力学参数。

本工程用于平地、低速学习与算法修改。控制器没有强化学习权重，未实现跑步、跳跃或翻跟斗。当前实现是加权逆动力学控制，并非带完整摩擦锥、关节限制和接触互补约束的工业级 QP-WBC。

## 检查结果与文件

`results/verification_report.json` 保存交付前的实际检查：站立、挥手、下蹲、双脚舞蹈、前进、后退、左右转向、步行到站立的切换、侧向扰动、暂停与复位。报告还记录实际脚底法向力、脚离地次数、骨盆高度、倾角、力矩上限与是否施加基座外力。

交付时在 Linux、Python 3.12 和 MuJoCo 3.12.0 下运行物理检查，并使用 EGL 生成了真实模型渲染。Windows 批处理和原生图形窗口未在 Windows 实机验收。

每次正常退出后，`results/run_report.json` 记录最终状态与事件，`results/trajectory.csv` 记录 20 Hz 轨迹和脚底法向力。报告中的 `distance_m` 是骨盆平面运动路径长度，包含左右重心移动，不等于前进距离；净位移请比较骨盆坐标。

```text
run_atlas.py                     原生查看器、自动演示、无窗口运行
verify_simulation.py             可复现的物理检查
atlas_sim/simulation.py          步态规划与全身控制
atlas_sim/model_loader.py        支持中文路径的内存模型加载
verify_model_loading.py          模型路径与缺失文件检查
models/atlas.xml                可直接加载的 MuJoCo 3.12 场景
models/atlas_source.xml         保留作者注释的历史模型源文件
models/meshes/                  模型所需 STL 网格
build_model.py                  从源文件重建兼容场景
third_party/                    上游许可、来源与网格校验值
results/                        检查报告、轨迹与渲染图
```

## 常见问题

**提示 `No module named pip`**：重新运行修复版 `1_安装依赖.bat`，脚本会先执行虚拟环境中的 `ensurepip`。也可在命令提示符 CMD 中依次运行：

```bat
cd /d "D:\开发\Atlas_MuJoCo_3_12"
".venv\Scripts\python.exe" -m ensurepip --upgrade --default-pip
".venv\Scripts\python.exe" -m pip install -r requirements.txt
".venv\Scripts\python.exe" run_atlas.py
```

`ensurepip` 使用 Python 自带组件补装 pip；后续安装 MuJoCo 等依赖仍需要联网。如果这个 Python 连 `ensurepip` 也没有，安装脚本会尝试用基础 Python 的 pip 修复目标环境。若两种方式都不可用，请用完整的 Python 安装程序执行“Modify / 修改”并启用 pip 后重试。

**提示找不到 `mujoco` 或 `numpy`**：重新运行 `1_安装依赖.bat`，或使用 `.venv\Scripts\python.exe run_atlas.py`，确保安装和运行使用同一个 Python。

**提示 `ParseXML: Error opening file ... atlas.xml`**：本版已改用 Python 读取模型和 STL 字节，再通过 MuJoCo 内存 VFS 编译。这样无需让引擎直接打开 `D:\开发\...` 这类中文磁盘路径。请将压缩包解压到 `D:\开发\`，覆盖同名的工程文件夹，并确认结构如下，避免在工程目录中又多套一层同名文件夹：

```text
D:\开发\Atlas_MuJoCo_3_12\
  run_atlas.py
  atlas_sim\
    simulation.py
    model_loader.py
  models\
    atlas.xml
    meshes\
      pelvis.stl
      ...
```

若文件缺失，程序会明确列出缺少的 XML 或 STL 文件。目录里 `run_atlas.py`、`models` 和 `atlas_sim` 应处于同一层。需要验证加载时可执行 `python verify_model_loading.py`。测试报告为 `results/model_loading_verification.json`，包括中文和空格路径、不同工作目录、实际启动后行走，以及缺失文件的提示检查；测试环境为 Linux，Windows 实机仍未验收。

**窗口闪退**：在工程文件夹打开终端，运行 `.venv\Scripts\python.exe run_atlas.py` 查看完整错误。先运行无窗口自检区分物理与图形驱动问题。

**提示版本不匹配**：执行 `python -m pip install -r requirements.txt`，程序会检查实际导入的 `mujoco.__version__` 是否为 3.12.0。

**机器人被鼠标拖拽后失去平衡**：按 R 复位。教学控制器的扰动能力有限；跌倒检测会暂停运行，避免把失稳状态当作成功结果。

**想看新版圆形头部的电动 Atlas**：本包是历史液压款网格；需要另行提供新版模型的 URDF/MJCF 与网格才能替换。

## 模型来源与归属

- [LocoMuJoCo v0.3.0 Atlas 模型](https://github.com/robfiras/loco-mujoco/tree/v0.3.0/loco_mujoco/environments/data/atlas)
- [LocoMuJoCo Atlas 文档](https://loco-mujoco.readthedocs.io/en/v0.2.0/source/humanoids/atlas.html)
- [MuJoCo 3.12.0](https://pypi.org/project/mujoco/3.12.0/)

模型原始注释列出 OSRF DRCsim、Boston Dynamics Atlas v5、Vikash Kumar / Roboti LLC，以及 Guoping Zhao 的后续修改。上游项目许可随包保留；`third_party/model_provenance.json` 记录所用提交树和各网格的 SHA-256。新增 Python 程序按根目录 LICENSE 提供，第三方模型保留其原有归属。
