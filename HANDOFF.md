# 交接文档 — FanGe Microduck 仿真与 3D 打印

> 生成时间：2026-09-15（中国标准时间）
> 面向：下一个接手本仓库的 agent / 会话
> 敏感信息已剔除（无密码、无 token、无 API key）

---

## 一句话现状

**FanGe 版 Microduck（`G:\Microduck-build-tutorial`）已在 WSL 跑起 MuJoCo 仿真，能站立/转头/走路；官方 Pollen 版复刻仿真此前也已跑通。用户正在用 A1 打印机打印 FanGe 版实体件。**

---

## 1. 目标

复刻 Pollen Robotics Microduck（25cm 双足机器鸭）软件栈，并把它 3D 打印出来。分两条线：

- **官方线**（Pollen 官方几何）：`G:\microduck`（Rust 守护进程）+ `G:\microduck_rl`（MuJoCo twin）。
- **FanGe 线**（第三方复刻，用户最终选择打印的版本）：`G:\Microduck-build-tutorial`。

用户明确：**打印 FanGe 版**，仿真也应对齐 FanGe 版。

---

## 2. 仓库布局（都在 G 盘，WSL 见 `/mnt/g/...`）

| 路径 | 内容 | 状态 |
|---|---|---|
| `G:\microduck` | 官方 Rust 守护进程仓 | `cargo check/test` 通过；仿真入口 `scripts/duck-sim` |
| `G:\microduck_rl` | 官方 MuJoCo/RL 仓（branch `develop`） | 最小 venv 已装；`duck-sim` 曾跑通 |
| `G:\Microduck-build-tutorial` | **FanGe 教程仓**（本次工作主线） | 仿真已跑通（见下） |
| `G:\microduck-replica` | fanhao375 官方几何复刻（中文，含 BOM/装配图） | 仅作参考，未打印 |
| `G:\rustdl` | 离线轮子/策略缓存 | `wheels/`、`policies/` |

---

## 3. FanGe 仿真（本次核心成果）

### 3.1 环境

- WSL Ubuntu 24.04，用户 `deng`；Rust 1.98.1 已装在 `~/.cargo`。
- `uv` 0.12.12 在 `~/.local/bin`。
- **WSL 的 HTTPS/TLS 会挂**（Clash TUN `198.18.0.2`）：PyPI/rustup HTTPS 超时，HTTP 镜像可用。
  → 轮子一律在 **Windows** 用 HTTP 阿里云镜像下到 `G:/rustdl/wheels/`，再在 WSL 离线装。
- 已建 venv：**`~/fange-sim-venv`**（在 Linux 盘，避免 `/mnt/g` 复制超慢；用 `UV_LINK_MODE=copy`）。

### 3.2 已装依赖（离线 wheel）

numpy 2.4.1、mujoco 3.10.0、onnxruntime 1.29.0、glfw 2.10.0、better_actuator_models(bam) 1.0.2、colorama、placo 0.9.4、pin 3.7/4.1、eigenpy、coal、hpp_fcl、cmeel-*（boost 1.90）等。

### 3.3 启动仿真

```bash
export PATH="$HOME/.local/bin:$PATH"
export DISPLAY=:0
export PYTHONPATH=/mnt/g/Microduck-build-tutorial/microduck/src
export LD_LIBRARY_PATH="$HOME/fange-sim-venv/lib/python3.12/site-packages/cmeel.prefix/lib:$LD_LIBRARY_PATH"
cd /mnt/g/Microduck-build-tutorial/microduck
rm -f /tmp/microduck_scheduler.stop
$HOME/fange-sim-venv/bin/python -u /tmp/fange_sim_aligned.py
```

（一键脚本：`C:/Users/deng_/AppData/Local/Temp/wsl-fange-aligned.sh`）

### 3.4 关键：仓库自己打架，启动器已做适配

上游 `src/sim/sim_main.py` **对不上当前 MJCF**，故自写启动器 `/tmp/fange_sim_aligned.py`
（源码副本：`C:/Users/deng_/AppData/Local/Temp/fange_sim_aligned.py`，**建议拷进仓库保存**）。

它处理了 4 处不一致：

1. **关节名**：`src/constants.py` 是 14 名（`right_ankle`、`neck_pitch`…），而
   `src/model/mjcf/robot.xml` 是 **19 个执行器**（`right_ankle_pitch/roll`、双肩、只有 `head`）。
   启动器用 `ALIAS` 映射：`right_ankle→right_ankle_pitch`、`left_ankle→left_ankle_pitch`、
   `neck_pitch/head_pitch/head_yaw/head_roll→head`。
2. **BAM API**：PyPI `better-actuator-models==1.0.2` 的 `MujocoController` 只接受
   `vin_drop_resistance`/`vin_min`，**没有** 仓库代码用的 `vin_drop_gain`/`max_current`，
   也**没有 `reset()`**。启动器改用它实际有的签名，并用 `set_q_target` 逐关节初始化。
3. **placo 加载即崩**：`scheduler.py` 顶层 `from moves.squat import SquatMove` → `import placo`，
   而 placo 的 cmeel 原生库版本拧着（`libboost_python312.so.1.87.0` 缺失）。
   启动器在 import scheduler 前 stub 掉 `placo` 模块，并**不注册 SquatMove**（深蹲 `s` 不可用）。
4. **电流限幅**：bam 1.0.2 无 `max_current`，启动器手动用 `kt` 把 `data.ctrl` 夹到 `±BAM_MAX_CURRENT*kt`。

### 3.5 已验证运行结果

- MJCF：`nq=26, nu=19, dt=0.002`；19 个执行器全部被 BAM 驱动。
- 控制环 50 Hz 运行，`walk.onnx` 加载成功（`Walk ONNX: True`）。
- MuJoCo 窗口在 WSLg（`DISPLAY=:0`）打开。
- 键位：`v` 走路、`h` 转头、方向键 `vx/vtheta`、`x` 清零、`r` 复位、`q` 退出。

### 3.6 已知问题

- **控制环偶发 overrun**（`/mnt/g` 上 BAM+渲染慢，几十 ms），窗口仍可用，走路能跑。
- **深蹲 `s` 不可用**：placo/pinocchio 原生库没对齐。要修需把 cmeel 全套版本对齐，
  或从 FanGe 的预构建镜像 `microduck.img.xz` 里抠出原始 venv。
- **`make sim` 不能用**：直接用上面的 `fange_sim_aligned.py`。

---

## 4. 官方线（Pollen）仿真 — 已完成，供参考

- 仓库：`G:\microduck`（`scripts/duck-sim`）+ `G:\microduck_rl`（`duck-body` MuJoCo）。
- 最小 venv 在 `G:\microduck_rl\.venv`（numpy/mujoco/onnxruntime/glfw）。
- 策略 ONNX 在 `G:\microduck\policies\`（9 个，从 HuggingFace `pollen-robotics/microduck-policies` v1 拉）。
- 启动：
  ```bash
  export PATH="$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
  export DUCK_SIM_RL=/mnt/g/microduck_rl
  export PYTHONPATH=/mnt/g/microduck_rl/src
  export DISPLAY=:0
  cd /mnt/g/microduck && scripts/duck-sim
  ```
- 曾验证：`robotd` healthy、50.5 Hz、鸭子站立（trunk ~0.116 m）、9 策略加载。
- 行尾问题已修：`.gitattributes` 已 staged（**未提交、未 push**），脚本转 LF，工作区干净。
- ⚠️ 不要把 Windows 密码当 WSL sudo；sudo 装包由用户手动完成。

---

## 5. 3D 打印状态

- 打印机：**Bambu Lab A1**（256mm 热床；固件 `product_name` 报 "Bambu Lab A1"，
  但 MQTT 字段写 `A1M`，config.yaml 的 `BAMBU_MODEL` 也误写 `a1mini`——**切片要显式传 `bambu_model=a1`**）。
- 无 AMS，外置 PLA 绿色（`GACC38`/`GFA00` PLA Basic）料，已确认有余量。
- **用户已手动开始打印 FanGe 版**（不是通过 agent 远程下发）。
- FanGe 打印件：`G:\Microduck-build-tutorial\microduck\cad\stl\`（**毫米**，PLA，0.12mm 层高，15% 填充）。
- 现成工程：`G:\Microduck-build-tutorial\microduck3D打印.3mf`（约 5.3MB，热床约 123×83mm，A1 一盘可放）。
- 参考 Onshape 装配体 + YouTube 教程见仓库 README。

### ⚠️ 打印踩坑（重要）

- **Bambu MCP 远程下发打印失败**：`err_code 84033543` / HMS `65543`，
  外置料被当成 AMS 槽 0（`ams_mapping:[0,...]`），而本机外置料是槽 **254**。
  清 HMS、传 `ams_slots=[254]`、`use_ams=false`、改 `bambu_model` 均无效。
  → **结论：这台机器的远程打印走不通，改由用户在 Bambu Studio / 机身屏手动打。**
- 官方 `microduck_rl/.../assets/*.stl` 是**仿真网格（米制，含舵机/PCB）**，
  要 ×1000 才能当 mm，且**不是可装配打印件**，别直接打。
- `G:\microduck-replica`（fanhao375）是官方几何的中文复刻，含 BOM/装配图，作参考。

---

## 6. 三条候选路线的成本对比（供决策）

| | FanGe 教程 | 官方几何复刻 |
|---|---|---|
| 舵机 | 14× XL330（走路够用） | 15× + 超压 |
| 驱动 | 成品 OpenRB-150 | 4 层 HAT 必须 SMT + 未开源 `imu_to_dxl` |
| 主控 | Pi Zero 2 W | Radxa Zero 3W + IMX219 + ToF |
| 机械 | mm STL + 现成 3MF + 手册 | 仿真网格 ×1000，公差未验证 |
| 自估整机 | ~$567（或按最低配 ~$400 出头） | 舵机就 $359–412，整机远超 $399 零售 |

→ **FanGe 更便宜**，用户已选它。

---

## 7. 建议下一步（按优先级）

1. ~~**把 `fange_sim_aligned.py` 拷进仓库**~~ ✅ **已完成（2026-09-26）**：
   - 源码从会话数据库完整恢复（含 placo stub、bam 1.0.2 无 `reset()` 的两处 patch），
     落仓为 **`microduck/src/sim/sim_aligned.py`**（336 行）。
   - 启动脚本落仓为 **`scripts/run-sim-aligned.sh`**（原 Temp 版丢失，已重写为绝对路径）。
   - 已验证：MJCF nq=26/nu=19、19 执行器全部映射、walk.onnx 加载、50Hz 控制环运行 ✓。
   - 启动：`bash /mnt/g/Microduck-build-tutorial/scripts/run-sim-aligned.sh`（WSL）。
2. **修 placo**（让深蹲 `s` 可用）：对齐 cmeel 全套版本，或从 `microduck.img.xz` 抠原始 venv。
3. 打印：按 `microduck/docs/printing.md` 的件表与朝向打齐，参照 `assembly.md` 装配。
4. （可选）官方线：交叉编译 / `cargo board` / 真机 `dev-push` 排在仿真之后。

---

## 8. 建议加载的技能（skill_view）

- `microduck-dev` — Microduck 栈构建/仿真的总纲（含 `references/sim-loop.md`）。
- `wsl-development` — WSL 构建、CRLF、离线装轮子（含 `references/offline-python-wheels.md`、`references/offline-rust-install.md`）。

---

## 9. 关键文件索引

**FanGe 仓**
- `microduck/Makefile`（`sim`/`viewer`/`run` 目标）
- `microduck/src/sim/sim_aligned.py` ← 对齐启动器（已落仓，2026-09-26 从会话库恢复）
- `scripts/run-sim-aligned.sh` ← 一键启动脚本（已落仓）
- `microduck/src/sim/sim_main.py`、`mujoco_controller.py`、`mujoco_input.py`
- `microduck/src/constants.py`（14 名关节表）、`microduck/src/model/mjcf/robot.xml`（19 轴）
- `microduck/src/moves/walk.py`、`rotate_head.py`、`squat.py`
- `microduck/src/agents/walk.onnx`
- `microduck/cad/stl/`、`microduck3D打印.3mf`
- `microduck/docs/{printing,assembly,bom,usage}.md`

**本机脚本（Temp，重启易丢）**
- `C:/Users/deng_/AppData/Local/Temp/fange_sim_aligned.py` ← 启动器源码
- `.../wsl-fange-aligned.sh`、`wsl-fange-venv2.sh`、`wsl-fange-homevenv.sh`
- `.../fetch_fange_wheels.py`、`fetch_cmeel.py`、`fetch_pin37.py`、`fetch_colorama.py`
- WSL：`~/fange-sim-venv/`、`~/fange-sim-venv/lib/python3.12/site-packages/cmeel.prefix/lib/`

**官方线**
- `G:/microduck/scripts/duck-sim`、`G:/microduck/.gitattributes`（staged）
- `G:/microduck/policies/*.onnx`、`G:/microduck_rl/.venv/`
- `G:/rustdl/wheels/`、`G:/rustdl/policies/`
