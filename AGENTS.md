# Microduck-build-tutorial（FanGe 树）

FanGe 路线 Microduck 复刻仓库的本地工作副本。当前活跃特性：混搭 14 舵机版（HD-1910，无臂形态），见 `.scratch/armless-hd1910/`。

## Agent skills

### Issue tracker

Local markdown：spec 与票据存于 `.scratch/<feature-slug>/`（spec.md + issues/NN-slug.md，每票一文件）。本仓库 remote 是上游作者的仓库，不往 GitHub 开 issue。See `docs/agents/issue-tracker.md`.

### Triage labels

默认五枚角色标签：`needs-triage`、`needs-info`、`ready-for-agent`、`ready-for-human`、`wontfix`。See `docs/agents/triage-labels.md`.

### Domain docs

Single-context：根目录 `GLOSSARY.md` + `docs/adr/`。See `docs/agents/domain.md`.

## 硬约束（改代码前必读）

- 构建与仿真在 WSL Ubuntu 24.04（`/mnt/g/Microduck-build-tutorial`），不在 Windows 侧
- 本地 checkout 是对上游仓库的学习/改造，不向上游 push
- 仿真入口是对齐启动器 `scripts/run-sim-aligned.sh`（仓库内已提交）；上游 `make sim` 是坏的，不要试图"修好"它来替代
