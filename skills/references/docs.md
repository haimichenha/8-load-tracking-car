# Docs Skill

本文件是 STM32F103ZET6 电赛小车的资料入口和归档规则。它不重复 pin-map；当前硬件定义一律查看 `rules.md`。

## 1. 资料查阅顺序

| 需要确认的问题 | 首选资料 | 说明 |
| --- | --- | --- |
| 现在每根线怎么接、哪个功能优先 | `skills/references/rules.md` + 实际接线 | `rules.md` 为当前基线，实物接线用于最终复核。 |
| 某脚能否配置 UART/TIM/ADC/I2C 或重映射 | STM32F103xE 数据手册、参考手册 | 必须核对封装、通道和 AFIO 重映射，不能照搬其他 STM32 型号。 |
| 当前工程具体宏、时钟和启动文件 | `F:/keil5/stm/ZET6 HAl/lidar_car/` | 重点看 `Src/`、`Hardware/`、`System/`、`Start/` 和构建配置。 |
| 电机/编码器带起顺序 | `references/workflow.md` | 先单元测试，后闭环。 |
| 旧雷达/树莓派经验 | `radar_bigturn_good_baseline_20260515.md` | 仅算法和运维经验，不作为当前引脚依据。 |

## 2. 当前本地关键位置

| 名称 | 路径 | 用途 | 可信度 |
| --- | --- | --- | --- |
| Skill 入口 | `F:/keil5/stm/skills/SKILL.md` | 当前项目入口与阅读顺序 | 高 |
| 硬件规则 | `F:/keil5/stm/skills/references/rules.md` | STM32F103ZET6 pin-map、复用冲突、模块边界 | 最高 |
| 测试流程 | `F:/keil5/stm/skills/references/workflow.md` | 带起与验收阶段 | 高 |
| 当前工程 | `F:/keil5/stm/ZET6 HAl/lidar_car/` | 当前 STM32F103ZET6 代码与构建产物 | 高 |
| 历史雷达基线 | `F:/keil5/stm/skills/references/radar_bigturn_good_baseline_20260515.md` | 树莓派/雷达侧历史算法参考 | 历史参考 |

## 3. 官方资料清单

1. **STM32F103xE datasheet**：确认 LQFP144 封装、引脚复用、模拟脚和电气限制。
2. **STM32F10x reference manual**：确认 RCC、GPIO、AFIO、USART、TIM、ADC、DAC 和中断配置。
3. **TB6612FNG datasheet**：确认 VM/VCC/GND、`STBY`、IN、PWM、续流和最大电流。
4. **编码器/电机/超声波/OLED/雷达/无线模块各自 datasheet**：确认电平、供电、波特率和协议。

阅读官方资料时，记录“器件型号 + 页码/章节 + 结论”，不要只保存截图或二手博客结论。

## 4. 当前硬件资料归档建议

建议在电赛工程目录中保存以下资料，文件名包含日期和板卡版本：

```text
docs/
├── schematic/          # 主控、TB6612、编码器、传感器接线图
├── pinmap/             # 导出的当前 pin-map 与变更记录
├── datasheets/         # 官方 PDF 或其来源链接
├── test-evidence/      # 串口日志、示波器截图、单轮视频说明
└── competition/        # 题目、评分项、答辩材料
```

若某资料只存在聊天记录或照片，先转成能检索的 Markdown/图片索引，并注明采集时间和是否已实测。

## 5. 代码定位建议

| 需求 | 优先模块 |
| --- | --- |
| 时钟、启动、烧录 | `Start/`、链接脚本、构建配置 |
| 延时与系统基准 | `System/` |
| TB6612、电机与 PWM | `Hardware/bsp_motor.*` 或后续 `motor_driver.*` |
| 编码器 | `Hardware/bsp_encoder.*` 或后续 `encoder.*` |
| OLED/蜂鸣器/按键/LED | 对应 `Hardware/bsp_*` 模块 |
| 串口协议 | 对应 `Hardware/bsp_usart.*`、`protocol.*` |
| 阶段测试 | `Src/main.c` 或后续 `stage_runner.*` |

当前工程若仍存在旧 TB6612、L298N 或标准库/HAL 混用文件，必须以本轮 `rules.md` 的 TB6612/STM32F103ZET6 口径重建配置，不能直接复制旧引脚宏。

## 6. 文档与版本留痕

每次修改硬件相关代码时，在提交说明或测试记录中最少写：

```text
日期：
固件版本 / commit：
涉及引脚与外设：
接线或重映射变化：
测试步骤：
观察证据：
结论：
回退方式：
```

当一次改动同时涉及 `rules.md`、工程配置和实物接线时，必须在同一记录中交叉引用，防止文档和固件分别漂移。

## 7. 历史资料使用边界

- 旧 `lidarMSP`、MSPM0、树莓派雷达、A*、Cartographer 和旧无线协议文档可以保留为算法/经验资料。
- 其中出现的 `PBxx`、`PAxx`、`UARTx`、按键、PWM 或测试编号均不自动迁移到 STM32F103ZET6。
- 任何历史资料引用当前工程前，必须先经过 `rules.md` 的 pin-map 审核。
