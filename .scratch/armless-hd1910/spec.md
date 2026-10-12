# Spec：混搭 14 舵机版 Microduck（HD-1910，无臂形态）

> 源自 `混搭方案-14舵机HD1910.md` + 2026-10-12 grill 五问答。
> 决策已定：Q1=C 无臂起步不封死、Q2=A 分批采购、Q3=A 保留 OpenRB-150、Q4=A 本地 2× RTX 5090D、Q5=A 起步（规格书 BAM 训「能走」）→ B 收尾（单摆辨识精调）。

## Problem Statement

官方 Dynamixel XL330 舵机因机器鸭热潮断货炒价（淘宝现货 ¥456–498/颗），按 19 舵机带臂版买齐成本不可接受。用户已有 FanGe 打印件（19 舵机带臂版几何，沉没成本 ¥50 耗材），需要一条能走、可控成本、且不浪费已有打印件的路线。

## Solution

按 FanGe README 的 14 舵机无臂版装机：腿部 10 + 头颈 4 全部用飞特 HD-1910-C001（¥123/颗），手臂 6 舵机直接不买、手臂打印件封存（保留以后升级可能）。执行器更换导致官方 walk 策略作废，走完整 sim2real 重训管线（本地 2× RTX 5090D 训练）。驱动层从 Dynamixel Protocol 2.0 换成飞特 SCS，物理链路保留 OpenRB-150 USB 串口透传。

## User Stories

1. As 建造者，I want 用 ¥123/颗的国产舵机装出一只能走的鸭，so that 不为炒价的 XL330 支付 4 倍价钱
2. As 建造者，I want 手臂舵机完全不采购、手臂打印件封存不装，so that 省 6 舵机（≈¥738）且保留日后补臂的升级路径
3. As 建造者，I want 先买 2 只 HD-1910 + URT-2 做台架验证，so that 在补购剩余 14 只之前用实物否掉「薄 3mm + 舵盘凸出」两个最大适配风险
4. As 建造者，I want 保留 OpenRB-150 当 USB-串口透传，so that 硬件零改动、软件风险集中到协议层一处
5. As 建造者，I want 用本地 2× RTX 5090D 重训 walk 策略，so that 不租云、训练成本可忽略、迭代不受配额限制
6. As 建造者，I want 第一版重训用规格书 BAM 参数走通全管线（能走标准），so that 最快见到走路；单摆辨识作为收尾迭代而非前置阻塞
7. As 操作者，I want 14 只舵机按 constants 表烧好 ID / 1Mbps / 响应等级并打开过流过压保护，so that 装机后总线一次点亮
8. As 操作者，I want 实机先站立后行走（平整地面、可随时抱起），so that 首次上电风险可控
9. As 操作者，I want 收尾阶段用备用舵机做单摆台架辨识并重训 v2，so that 步态质量逼近原版且辨识参数今后每只鸭复用
10. As 未来会话的 agent，I want ADR 和 GLOSSARY 记录执行器/形态/链路三个决策及其理由，so that 不会建议「换回 XL330」或「把手臂装上」这种已被否决的路线
11. As 未来会话的 agent，I want 票据图（blocking edges），so that 快递等待期可以并行推进纯软件票（MJCF、驱动层、训练环境）
12. As 观看者，I want 鸭子以官方无臂形态走路，so that 外形与 Pollen 官方一致

## Implementation Decisions

- **执行器**：14 × 飛特 HD-1910-C001（腿 10 + 头颈 4），单协议飞特 SCS；出「模式 4：纯位置 PD（Sim2Real）」直接对应 RL 落地；KP/KD 起步值参照 Xgoduck 运行时（KP 6 / KD 20）
- **形态**：无臂（官方 Pollen 形态）。手臂打印件（shoulder/humerus/radius/arm_bearing_support ×2）不装配、不改造、妥善封存
- **采购节奏**：分两批。首批 2 只 + URT-2（¥291）做台架与试装验证；通过后补 14 只（含备用）+ 主控/IMU/电池
- **物理链路**：保留 OpenRB-150 作 USB-串口透传；UART 直连 + 半双工缓冲留给 v2
- **ID 表**：以 constants 表为准（1–14：右腿 1–5、左腿 6–10、头颈 11–14），不是 assembly.md 的带臂 19-ID 表
- **仿真**：从现有 MJCF 派生无臂 14-DoF 版（删 6 臂 body/actuator，头颈从 1 个执行器扩成 4 个，与 constants 的 4 头颈关节一一对应）；删除而非质量归零，使质量分布与实机一致
- **BAM**：先规格书占位（减速比 1:320、回差 ≤0.5°、12bit 磁编码、vin_range 5.0–6.5V 对应 6V 电池带载区间）；收尾阶段单摆辨识替换精确值
- **训练**：本地 2× RTX 5090D（Blackwell sm_120，需 CUDA ≥12.8 的 PyTorch）；域随机化按无臂质量分布 + HD-1910 电压区间重设
- **驱动层**：Dynamixel SDK → 飞特 SCS（scservo_sdk 或手写帧协议）；需实现 ping、sync_write 目标位置、sync_read 位置/速度/电压；串口探测路径改为 OpenRB-150 实际路径
- **已核实事实（2026-10-12）**：walk.onnx 输入 obs [1,51] / 输出 actions [1,14]；constants 表 14 电机含头颈 4 关节——head 4 关节确实是策略动作空间的一部分，训练 MJCF 必须带 4 头颈执行器。注意：无臂 MJCF 会改变 obs 维度，重训配置的 obs/action 维度必须自洽，导出 ONNX 由驱动栈实测加载

## Testing Decisions

- 驱动层用 mock 串口传输做单元测试（帧构造/解析、校验和、sync_read 组包），不依赖实物；台架上用 2 只首批舵机做集成验证
- 仿真验收走对齐启动器（aligned launcher）：MJCF nq/nu 与 14 电机映射关系打印核对 + 站立不倒 + 行走速度参考
- 试装验收是物理判定：femur/tibia 腔体装得进 HD-1910、舵盘配合件能锁紧；失败则记录并触发 fanhao375 v2.1 `-FT` 改件补打印
- 重训验收：策略在仿真内稳定行走（步速参考社区公开值量级）+ 导出 ONNX 能被驱动栈加载推理
- 实机验收分级：站立 → 原地小步 → 平整地面直线行走，每级稳定再升下一级

## Out of Scope

- 手臂 6 舵机（采购、装配、驱动、进策略全部不做；打印件封存）
- UART 直连主控链路（v2 议题）
- 摄像头/ToF 等感知扩展（本次只要「能走」）
- 官方 Pollen 软件栈（duck-sim/microduck_rl）的改动——本次全部工作在 FanGe 树
- 商用/发布考虑（CC BY-SA-NC 非商用约束照旧）

## Further Notes

- 方案原文（BOM、ID 表、装配差异、风险清单）见 `混搭方案-14舵机HD1910.md`；其 ⚠️ 项随事实核实在原文件更新
- HD-1910 若试装失败：fanhao375/microduck-replica-cad v2.1 `-FT` 已提供 8 个舵盘配合改动件（STEP/3MF）
- 快递等待期（首批舵机在途）与纯软件票完全并行：票 01/02/03 无阻塞可直接开工
