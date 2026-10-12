# Microduck-build-tutorial（FanGe 树）

FanGe 路线 Microduck 复刻仓库：用 3D 打印件 + 国产总线舵机复刻 Pollen Robotics Microduck（25cm 双足机器鸭）。

## Language

### 形态与硬件

**无臂形态**：
官方 Pollen 的 14 舵机布局（腿 10 + 头颈 4 + 嘴被砍掉由 walk 策略外的嘴舵机承担——本仓库为腿 10 + 头颈 4）。本仓库当前目标形态；手臂打印件封存不装。
_Avoid_: 14 舵机版（与 README 的 BOM 语境混淆时用「无臂形态」）

**带臂形态**：
FanGe CAD/MJCF 实际建模的 19 舵机变体（多肩/上臂/前臂 6 舵机）。用户已打印该版全套件，但装配按无臂形态执行。
_Avoid_: 19 舵机版

**HD-1910**：
飞特 HD-1910-C001 总线舵机，本项目的 XL330 替代执行器。SCS 协议、1:320 减速比、回差 ≤0.5°、金属齿、¥123/颗。
_Avoid_: HL-1910（另一型号，机械兼容但已无价格优势）

**URT-2**：
飞特舵机 USB 调试板，用于台架写 ID、调寄存器、验证舵机。注意区分于装机用的半双工转接板。

### 软件栈

**FanGe 树**：
本仓库（AI-FanGe/Microduck-build-tutorial）。Python + MuJoCo + BAM 路线。与 Pollen 官方树（`G:/microduck` + `G:/microduck_rl`，Rust 守护进程）是两套不可混用的栈。
_Avoid_: 官方树、Pollen 树（指本仓库时）

**对齐启动器**：
`scripts/run-sim-aligned.sh` → `microduck/src/sim/sim_aligned.py`。绕过上游 `make sim` 的 14/19 DoF 常量表不一致，是本仓库唯一可用的仿真入口。
_Avoid_: make sim（已坏，勿尝试修复替代）

**BAM**：
better-actuator-models，Rhoban 开源的执行器物理辨识/建模库。仿真中用 `bam_load_model(motor_name="xl330", model="m6")`；换 HD-1910 后需重辨识。
_Avoid_: 执行器模型（泛称时）

**单摆辨识**：
用备用舵机做单摆台架实验采集输入-输出数据，拟合 BAM 参数。本特性收尾阶段执行（v1 先用规格书占位值）。

### 流程

**首批试装**：
采购分批策略的第一步：先买 2 只 HD-1910 + URT-2，验证腔体适配与舵盘配合后，再补购余量。对应 ADR-0004。
_Avoid_: 一次买齐

**分级实机验收**：
实机测试顺序：站立 → 原地小步 → 平整地面直线行走。每级稳定后才能升下一级。
