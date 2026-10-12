# Microduck-build-tutorial（FanGe 树）

FanGe 路线 Microduck 复刻仓库（本地 fork：`dengchenlong/Microduck-build-tutorial`，上游 `AI-FanGe/Microduck-build-tutorial` 只读）。当前活跃特性：混搭 14 舵机版（HD-1910，无臂形态），spec 在 `.scratch/armless-hd1910/spec.md`，票据在本仓库 GitHub Issues。

## Agent skills

### Issue tracker

GitHub Issues（fork 仓库，gh CLI）。阻塞关系用平台原生 issue dependencies。上游仓库不开 issue。See `docs/agents/issue-tracker.md`.

### Triage labels

默认五枚角色标签：`needs-triage`、`needs-info`、`ready-for-agent`、`ready-for-human`、`wontfix`。See `docs/agents/triage-labels.md`.

### Domain docs

Single-context：根目录 `GLOSSARY.md` + `docs/adr/`。See `docs/agents/domain.md`.

## 硬约束（改代码前必读）

- 构建与仿真在 WSL Ubuntu 24.04（`/mnt/g/Microduck-build-tutorial`），不在 Windows 侧
- 上游仓库（`upstream` remote）只读：不 push、不开 issue；日常工作提交并推送到自己的 fork（`origin`）
- 仿真入口是对齐启动器 `scripts/run-sim-aligned.sh`（仓库内已提交）；上游 `make sim` 是坏的，不要试图"修好"它来替代
