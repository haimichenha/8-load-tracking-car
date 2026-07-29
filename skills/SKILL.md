---
name: stm32f103zet6-ecomp
description: Use when working on the STM32F103ZET6 electronic-competition car: pin planning, peripheral multiplexing, TB6612 motor drive, encoder, sensors, UART links, staged bring-up, logging, or project documentation. Always read this skill before changing hardware-related code or documentation.
---

# STM32F103ZET6 电赛小车 Skill

本目录是当前电赛小车的硬件规则、测试流程与文档基线。当前目标 MCU 为 **STM32F103ZET6**；旧 `lidarMSP` / MSPM0 内容只允许作为历史算法或雷达经验参考，**不得再作为当前引脚或外设配置依据**。

## Mandatory Entry Rule

0. 如果用户消息严格匹配 `/switch <provider>` 或明确要求切换 Codex/cc-switch provider，直接执行仓库根目录的 `scripts/codex-switch.ps1 <provider>`；完成后检查输出的 `diag`，确认 `base_url`、`model` 和当前 provider 标记。
1. 每次处理本项目任务前，先读取本文件。
2. 涉及引脚、串口、定时器、PWM、编码器、传感器、GPIO 或外设复用时，必须读取 `references/rules.md`。
3. 涉及上电、烧录、单元测试、联调、日志或阶段验收时，必须读取 `references/workflow.md`。
4. 涉及资料优先级、风险、工程边界或文档同步时，必须读取 `references/core.md`。
5. 涉及历史雷达调参与大转弯经验时，才读取 `references/radar_bigturn_good_baseline_20260515.md`；它不定义当前 MCU 引脚。
6. 一旦实测确认了新接线、引脚冲突、外设复用、方向极性或稳定测试方法，先同步 `references/rules.md`，再改代码与其他文档。

## Reference Map

| Reference | Use when |
| --- | --- |
| `references/core.md` | 项目范围、资料优先级、硬件风险、文档同步规则 |
| `references/rules.md` | 当前唯一有效的 STM32F103ZET6 引脚、外设复用和模块边界定义 |
| `references/workflow.md` | 上电、烧录、单元测试、联调和日志验收顺序 |
| `references/docs.md` | 当前工程、原理图、芯片手册、历史资料的查阅入口 |
| `references/turning-points.md` | 需要长期保留的接线/测试决策与反冲突原则 |
| `references/radar_bigturn_good_baseline_20260515.md` | 旧雷达/树莓派侧算法基线；仅按历史参考使用 |

## Non-Negotiable Rules

1. `references/rules.md` 的“当前主用引脚分配”是唯一 pin-map 基线；一根引脚不能被两个当前主用功能同时占用。
2. 标注为“备用/复用/条件可用”的脚，只有在主用功能明确关闭、完成外设重映射并记录后才能启用。
3. `PA2/PA3` 已留给 TB6612 的 `TIM2_CH3/CH4` PWM；不能同时启用 USART2 默认引脚。
4. 先验证电源、共地、STBY、方向、PWM 和编码器，再进入闭环、循迹、雷达或路径控制。
5. 任何电机测试都必须先架空轮子或保证安全场地；先低占空比、短脉冲、单轮，再双轮。
6. 外设代码按 `driver -> service -> app/stage` 分层；UART 解析、编码器读取、速度环和最终 PWM 不允许互相跨层直调。
7. 文档、原理图/接线表、代码宏定义与实际焊线有冲突时，先停止扩展功能，完成冲突定位并更新基线。

## Current Directory Shape

```text
skills/
├── SKILL.md
├── references/
│   ├── core.md
│   ├── rules.md
│   ├── workflow.md
│   ├── docs.md
│   ├── turning-points.md
│   └── radar_bigturn_good_baseline_20260515.md
└── scripts/
```

## Maintenance Rules

1. 保持本入口简短；详细 pin map 只写入 `references/rules.md`。
2. 每次改引脚先更新 `rules.md`，然后同步工程配置、代码宏、接线图和测试记录。
3. 历史项目资料必须标注“历史参考”，不得混入当前主用表。
4. 不在 skill 文件中保存密钥、Token、串口日志中的敏感信息或未经脱敏的设备标识。
