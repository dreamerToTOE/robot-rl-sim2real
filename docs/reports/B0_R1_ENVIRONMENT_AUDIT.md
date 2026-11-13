# B0-R1A：Isaac Lab 环境恢复审查

日期：2026-11-13（KST）。输入基线：`e68b6fd956c3b660d469b176e29d34958e39725f`。

**结论：最小启动 PASS；环境总体 PARTIAL。** 已通过只读复用此前 Factory 的隔离 Python、Isaac Lab v2.1.1 和现存 Isaac Sim 4.5，恢复本任务的 Lab 启动路径。未修补旧 Conda，未安装、升级或卸载软件；这不表示全部 Isaac Lab 功能或机器人动力学验证通过。

## 1. 本次实际检测与证据

原始记录位于本任务工作目录 `work/r1_environment/`；空场景启动在 `work/r1_smoke/empty_stage.log`。这些路径相对本次 Codex 工作目录，原始日志不含厂商模型，默认不提交整份本机路径清单。下表来自本次执行结果，不沿用旧报告作为当前实验。

经审查的失败/成功输出摘录、空场景 summary、Factory 文件 hash 对照及导入重试记录另随报告提交为 [B0_R1_RUNTIME_EVIDENCE.json](data/B0_R1_RUNTIME_EVIDENCE.json)。保留了失败，没有把 shell 退出0当作充分成功条件。

| 项目 | 当前检测 | 判定 |
|---|---|---|
| 宿主 | Ubuntu 22.04.5 LTS；kernel `6.8.0-138-generic`；系统 Python `3.10.12` | PASS |
| GPU / 驱动 | RTX 3070，8,192 MiB；驱动 `580.178.04`；检测时占用 2,277 MiB、空闲 5,542 MiB（瞬时值） | PASS（仅资源存在） |
| CUDA | `nvcc` 不在当前 PATH；复用 Torch wheel 为 `cu128`；未编译 CUDA 扩展 | PARTIAL |
| ROS 2 | `/opt/ros/humble`；当前 shell 继承 Humble 的 Python/动态库路径，未启动或控制机器人 | PARTIAL |
| Isaac Sim 独立安装 | `/home/ubuntu2004/isaacsim-4.5.0/VERSION` 实读 `4.5.0-rc.36+release.19112.f59b3005.gl` | PASS（本次最小启动） |
| Factory 隔离环境 | Python 3.10.12，Torch `2.7.0+cu128`，torchvision `0.22.0+cu128`，NumPy `1.26.4`，Gymnasium `1.2.0`，RL-Games `1.6.1` | PASS（运行时核心包核对） |
| Factory Lab checkout | tag `v2.1.1`，SHA `90b79bb2d44feb8d833f260f2bf37da3487180ba`；`git status --short` 空；有效 `_isaac_sim` 指向现存 Sim 4.5 | PASS |
| Factory 声明依赖检查 | `uv pip check` 检查 107 包；缺 `pin-pink==3.1.0`、`dex-retargeting==0.4.6`、`moviepy`，退出 1 | PARTIAL；不补装本次未用依赖 |

官方 [v2.1.1 安装文档](https://isaac-sim.github.io/IsaacLab/v2.1.1/source/setup/installation/index.html) 列出 Isaac Sim 4.5 的配对支持；本次采用该配对，没有把其他版本的 Conda 环境混入。Lab checkout 的项目 `VERSION=2.1.1`；editable 分发元数据 `isaaclab==0.41.3` 是其扩展包版本，不能据此误判 checkout 不是 v2.1.1。

### 现存环境清单

`conda env list --json` 本次列出 miniconda base、`isaac_env`、`isaaclab`、`unitree_rl_mjlab`；另从文件系统找到 anaconda 的旧 `isaaclab` 和 Factory 的独立 venv。

| 环境 | Python / 关键现状 | 本次选择 |
|---|---|---|
| miniconda base | 3.10.14；Torch 元数据 `2.7.0+cu128`；无 Lab/Sim 分发 | 不选 |
| miniconda `isaac_env` | 3.10.20；Sim 分发 `4.5.0.0`；Torch 元数据 `2.11.0`；没有安装 Lab；继承用户 NumPy `1.24.4` | 不选 |
| miniconda `isaaclab` | 3.11.15；Lab 元数据 `0.54.3`；Torch `2.7.0+cu128`；editable 指向已经不存在的 Sim 5.1 / Lab 目录 | BLOCKED；保持原样 |
| miniconda `unitree_rl_mjlab` | 3.11.15；Torch 元数据 `2.13.0`、NumPy `2.4.6`；无 Lab/Sim 分发 | 不选 |
| anaconda 旧 `isaaclab` | 3.10.17；Lab 元数据 `0.36.23` 的 editable 指向 `/home/ubuntu2004/isaac-sim4.5/IsaacLab/source/isaaclab`；未做启动验证 | UNKNOWN；不选 |
| Factory `factory-g01` | 3.10.12；Lab editable、Sim 链接和固定核心依赖均存在；通过既有 launcher 工作 | 复用 |

除 Factory 核心包有启动前后实际导入证据外，其他环境的版本主要为分发元数据，不代表已验证可运行。WorkBuddy 的早期 Factory 目录仍存在，但其 README 记录的是待启动阶段、相邻 Lab 没有 `_isaac_sim`；本次复用的是后来的独立验证环境，不据早期 README 宣布稳定。

## 2. `import isaaclab` 直接失败原因

复现命令：

```bash
/home/ubuntu2004/miniconda3/envs/isaaclab/bin/python -c 'import isaaclab'
```

本次两次测试（包括清除 ROS 路径、禁用用户 site 的复测）均退出 1：`ModuleNotFoundError: No module named 'isaaclab'`。

**已确认根因：失效的 editable 安装映射。** 当前 Python 3.11 环境仍有 Lab 元数据和 `.pth`，但 `lib/python3.11/site-packages/__editable___isaaclab_0_54_3_finder.py:9` 的 `MAPPING` 指向不存在的 `/home/ubuntu2004/isaacsim-5.1.0/isaac-sim-standalone-5.1.0-linux-x86_64/IsaacLab/source/isaaclab/isaaclab`；同环境 `isaaclab-0.54.3.dist-info/direct_url.json:1` 也声明该旧 editable 源目录。本次 `Path.exists()` 为 false，因此“有 dist-info”不能证明模块文件可定位。这是 Python editable 映射失效，不是把某个实际 `.so` 文件缺失误写成根因。

其他区别：

- 清除 `PYTHONPATH`、`LD_LIBRARY_PATH` 并设 `PYTHONNOUSERSITE=1` 后，该旧 Conda 仍然相同失败，故 ROS 路径污染不是本次 `ModuleNotFoundError` 的直接原因；污染仍会影响后续复现隔离。
- `isaac_env` 本来没有 Lab 分发，直接导入失败不能靠改几个动态库路径修好。
- Factory 裸 Python 可以导入 `isaaclab`，但 `from isaaclab.app import AppLauncher` 因未加载独立 Sim 的路径而报 `No module named 'isaacsim'`；使用既有 Factory launcher 后，两项导入均成功，退出 0。
- 不把 Python 3.11 的旧 Conda 强行接入要求 Python 3.10 的这套 Sim 4.5 运行路径。

原始失败与成功对照：`conda_isaaclab_import.log`、`conda_isaaclab_clean_paths.log`、`conda_isaac_env_import.log`、`factory_bare_import.log`、`factory_launcher_import.log`；各命令另保存在同名 `.command.json`。

## 3. 无安装变更的恢复方案及影响

本机既有 Factory 工作目录：

```text
FACTORY_ROOT=/home/ubuntu2004/Documents/Codex/2026-10-23/the-github-repository-has-now-been
FACTORY_VENV=$FACTORY_ROOT/work/envs/factory-g01
ISAACLAB_PATH=$FACTORY_ROOT/work/upstream/IsaacLab
ISAACSIM_PATH=/home/ubuntu2004/isaacsim-4.5.0
FACTORY_LAUNCHER=$FACTORY_ROOT/outputs/isaac-factory-repro/scripts/run_sim.sh
```

既有 `run_sim.sh:6-16` 在子进程内清理继承路径、禁用用户 site、加载现存 Sim 的 `setup_conda_env.sh`，前置 venv 包目录并预加载同版 `libcarb.so`。`python_entry.py:6-30` 预导入并断言核心包版本，在 AppLauncher 启动后再次核对版本和模块身份，防止 Kit 内置包覆盖目标依赖。Lab 的 editable 已指向有效 checkout，无需额外 `ISAACLAB_PATH` 注入。

本次复用这些现存脚本和目录，未执行 `pip install`、`uv pip install`、`conda install` 或 `isaaclab.sh --install`，未修改 `.pth`、符号链接、旧工程源码或系统驱动。影响限于本次短命进程、当前任务日志，以及 Kit 自身常规缓存/日志；恢复不依赖永久激活 shell，也没有把“修好旧 Conda”作为既成事实。

对 Factory 的 107 个包 `METADATA`、`pyvenv.cfg` 和两个启动脚本共 110 文件做 SHA-256 前后比对，未发现变化（`factory_environment_fingerprint_check.json`）。这不是全文件系统快照，不用于声称 Kit 缓存或运行日志完全未写入。

如以后需要项目独立环境，应先提交审批：新建本项目专用 Python 3.10 venv，锁定核心包、绑定 v2.1.1 checkout 和现存 Sim 4.5，估算 Torch 下载/磁盘占用；不在 Factory 或旧 Conda 中重装。当前无需执行该方案。

## 4. 本次最小启动验证

入口代码：本仓库 `scripts/smoke_isaaclab.py`。通过现存 Factory launcher 执行：

```bash
FACTORY_ROOT=/home/ubuntu2004/Documents/Codex/2026-10-23/the-github-repository-has-now-been
env FACTORY_VENV="$FACTORY_ROOT/work/envs/factory-g01" \
  ISAACSIM_PATH=/home/ubuntu2004/isaacsim-4.5.0 \
  PYTHONDONTWRITEBYTECODE=1 \
  bash "$FACTORY_ROOT/outputs/isaac-factory-repro/scripts/run_sim.sh" \
  "$PWD/scripts/smoke_isaaclab.py" --headless --device cpu \
  --summary "$PWD/../r1_smoke/recheck_summary.json"
```

上述命令在本仓库根目录执行；原始实际执行记录为 `work/r1_smoke/empty_stage.log` 和 `empty_stage_summary.json`；重跑示例另写 `recheck_summary.json`。

实测：headless AppLauncher 启动、创建空 stage、导入 `SimulationCfg` / `SimulationContext` / `UrdfConverter` 成功，执行 3 次 Kit `app.update()`，关闭进程，退出 0。日志同时有 `PINNED_PACKAGES`、`PACKAGES_AFTER_APP_LAUNCH` 和 `B0_R1_SMOKE={"status":"PASS", ...}`；核心包启动前后版本不变。

**范围严格限定：0 次 physics step、没有机器人、没有策略、没有动作、没有实机连接。** `--device cpu` 指物理设备选择，不表示 Kit 完全不使用 GPU 图形初始化。日志保留了 OmniHub 不可达、旧扩展弃用等警告；它们没有阻止本次空场景测试，不推断所有联网资产或可选扩展可用。

| 检查 | 结果 |
|---|---|
| Lab / AppLauncher 在正确启动路径导入 | PASS |
| Headless 空场景启动、3 次更新、正常关闭 | PASS |
| Lab simulation / URDF converter Python API 导入 | PASS |
| 全部声明依赖、可选重定向/视频功能 | PARTIAL（3 个缺项，本次未用） |
| 模型动态稳定性、驱动/增益、控制周期、PPO | UNKNOWN / 未测试 |
| 修复所有旧 Conda 环境 | 未执行；无需为本任务修改 |

## 5. 决策门

环境已允许 B0-R1A 的静态导入准备；仅凭该测试不能放行 B1 运动验证或训练。URDF→USD 的静态 smoke 与模型检查见 `B0_R1_CANONICAL_MODEL_PLAN.md`；真实动态参数、关节映射和安全门仍以模型及硬件证据为准。最终项目状态由 `docs/STATUS.md` 记录。

后续静态转换实测完成：7 刚体、6 revolute、14 mesh、固定 articulation root、轴/零位/惯量检查通过，但 `UsdUtils` 的3个 `OmniPBR.mdl` 材质解析项保留 PARTIAL。排查时使用 `fast_shutdown=False` 的一次失败运行关闭后段错误（exit139），恢复既有默认快速关闭后的最终运行有新 summary 和完成标记并退出0；没有因此修改驱动、依赖或已有环境。空场景最小启动 PASS 与这些完整转换/关闭路径局限分别记录，不扩大为“所有功能均已恢复”。

理解验收：① 为什么 `isaaclab` 元数据存在却会导入失败？② 为什么同一个 Factory venv 必须经过启动器加载 Sim 路径？③ 为什么空 stage 的 PASS 不能证明机器人模型或实机闭环可用？研究者尚未作答，学习验收未 PASS。
