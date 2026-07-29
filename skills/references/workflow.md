---
name: stm32f103zet6-workflow
description: STM32F103ZET6 电赛小车的上电、烧录、单元测试、联调、日志和验收流程。
---

# STM32F103ZET6 电赛小车 Workflow

## 1. 总原则

1. 每次硬件改动前先读 `rules.md`，确认当前主用脚、冲突脚和备用脚启用条件。
2. 从供电和单元测试逐级向上推进；没有前一阶段证据，不进入后一阶段。
3. 一轮测试只改变一个变量：接线、时钟/重映射、方向、占空比、采样周期或控制参数只能改一个。
4. 电机测试必须有安全条件：车轮架空或安全场地、低占空比起步、具备立即断电/停机方式。
5. 每一阶段在 USB 串口、OLED、LED 或采集工具中至少保留一种可观测输出。

## 2. 测试前固定检查

| 检查项 | 必须确认 |
| --- | --- |
| 板卡与电源 | STM32F103ZET6、电机电源、逻辑 3.3 V 和所有模块 GND 共地。 |
| 烧录 | SWD 可连、下载可校验、复位后程序正常运行。 |
| 引脚表 | 本轮会触及的脚均与 `rules.md` 一致，未占用备用/冲突脚。 |
| 日志 | USART1 `PA9/PA10` USB 串口或等效日志观察口可用。 |
| 安全 | TB6612 `STBY=PE6` 默认关闭；测试前确认轮子架空或场地安全。 |

## 3. 分阶段硬件带起流程

### B0：上电、时钟、烧录与 USB 串口

1. 只启动最小系统和 USART1 `PA9/PA10`。
2. 输出固件版本、系统时钟、复位原因和当前测试阶段。
3. ST-LINK 下载必须出现 program/verify 成功；仅显示 ST-LINK 版本不代表程序已烧入。
4. 通过条件：可重复烧录、复位后出现同一条启动日志。

### B1：按键和 LED

1. 逐个读取 `PG9..PG13` 五个按键，记录按下电平和去抖后事件。
2. 分别点亮 `PG14` 红灯、`PG15` 绿灯、`PE1` 黄灯，确认有效电平。
3. 通过条件：每个按键与 LED 都有唯一、可重复的日志/可见反应。

### B2：TB6612 单轮开环

1. 先把 `PE6(STBY)` 保持低电平，确认电机不会意外转动。
2. 按顺序测试左电机：方向脚 `PE2/PE3`，再 `PA2(TIM2_CH3)` 低占空 PWM。
3. 再测试右电机：方向脚 `PE4/PE5`，再 `PA3(TIM2_CH4)` 低占空 PWM。
4. 每一轮均执行：正转短脉冲 -> 停止 -> 反转短脉冲 -> 停止。
5. 通过条件：左右电机可独立正反转，停止时无持续驱动；记录每侧最小稳定可动占空比。

### B3：左右编码器

1. 左轮使用 `PA0/PA1` 的 TIM5 编码器模式；右轮使用 `PA6/PA7` 的 TIM3 编码器模式。
2. 手动转动和低速电机转动各验证一次。
3. 日志至少输出：`time_ms,wheel,cmd_pwm,enc_count,enc_delta,sign,event`。
4. 通过条件：每个轮子转动时只对应本侧计数变化；正反转符号稳定且可解释。

### B3.5：扩展 TB6612 与双 A4988（无编码器）

1. 本轮扩展 TB6612 临时测试映射为：`PF1/PF2`=G513-A 的 ENA/ENB、`PF3/PF4`=G513-B 的 ENA/ENB（TIM7 软件 PWM、同占空比），`PA4/PA5`=AIN1/AIN2，`PD8/PD9`=BIN1/BIN2，`PB9(STBY)=低` 为默认关闭。确认数字循迹 `x1..x4` 已断开/禁用后，才以 25% PWM 的 `G` 序列单路正转、停止、反转。
2. 两块 A4988 当前使用：A4988-1 `STEP/DIR/EN=PE14/PE13/PC4`，A4988-2 `STEP/DIR/EN=PD14/PD13/PD15`；`EN` 默认高（禁用），`MS1/MS2/MS3` 由硬件固定为全步进，驱动器电流限流先按电机额定值设置。
3. 先执行 `E` 保持力测试：两块驱动器只使能线圈 5 s、不发 STEP，人工轻拧轴确认有保持阻力；没有阻力时停止，核对 `VDD`、`VMOT`、共地、`RESET/SLEEP` 和线圈配对。
4. 单步测试：每次只发一个 STEP 脉冲并观察方向；随后用低速单独 `+16 -> -16` 步测试。
5. 整圈测试：在轴端贴零位标记，发送 `full_steps_per_rev × microstep_divider` 脉冲正转，再发送等量反向脉冲，人工确认回到标记。标准 1.8° 电机的全步进起始值是 200 steps/rev，必须以电机铭牌和实际细分为准。
6. 双机测试：在单机均通过后，才允许两台同向和反向的同步整圈测试。当前 `ExpansionActuatorTestDebug` 用 `C` 同时发送两路正向 200 脉冲、`D` 同时发送两路反向 200 脉冲，固定为 20 steps/s；先观察两轴是否同时、持续且无异响，再用零位标记确认反向回零。
7. 日志至少输出：`time_ms,stage,driver,dir,requested_steps,issued_steps,remaining_steps,step_rate,enabled,event`。无编码器时，这些字段仅为命令台账，不能当作真实位置反馈。
8. 通过条件：扩展 G513 左右方向可重复、停止时 `STBY` 关闭；两块步进在保持力、低速单步/16步/正反整圈/双机同步测试中均无异常发热、异响或丢步迹象，并经人工零位标记确认回零。

### B4：基础外设

| 外设 | 测试动作 | 通过条件 |
| --- | --- | --- |
| 超声波 | `PF0` 发触发脉冲，`PA8/TIM1_CH1` 捕获回波 | 距离随真实目标变化，超时能识别。 |
| 蜂鸣器 | `PB8/TIM4_CH3` 输出短音 | 可控响/停，不影响 TIM4_CH4 备用状态。 |
| OLED | 使用 `PB10/PB11` 刷新固定内容 | 上电、刷新、错误状态均可显示。 |
| 激光笔 | `PE15` 输出开/关 | 三极管开关极性与安全默认状态确认。 |

### B5：循迹输入

1. 先测 ADC 主组 `PC0/PC1/PC2`，输出原始值、滤波值、阈值和最终状态。
2. 需要备用模拟组时再启用 `PC3/PB0/PB1`；不要同时把 PB0/PB1 当 TIM3 PWM 输出。
3. 数字循迹依次核对 `PF1..PF4`、`PF6..PF9` 与 `x1..x8`。
4. `PG0/PG1` 为灰度 GPIO，若要把 PG1 用作普通 IO 必须先在规则表登记释放。

### B6：串口和扩展链路

1. 默认调试日志：USART1 `PA9/PA10`。
2. 雷达：UART4 `PC10/PC11`，先记录帧头、长度、校验和超时，不直接接入控制环。
3. ESP32/无线：UART5 `PC12/PD2`，先完成回环、收发和断链超时。
4. 陀螺仪：USART2 重映射 `PD5/PD6`；不得动用 PA2/PA3。
5. MaixCam/临时日志：USART3 重映射 `PD8/PD9`；OLED 保持 PB10/PB11 优先。
6. 循迹串口：只有在明确切换 USART1 重映射时才使用 `PB6/PB7`。

### B6.5：陀螺仪直线串级控制（待接线/待地面实测）

1. 保持 `PA9/PA10` 为 115200 日志，陀螺仪只使用 USART2 重映射 `PD5/PD6`；不得启用 USART2 默认 `PA2/PA3`。
2. 烧录 `GyroStraight400Debug` 后，确认静止时连续出现有效 `0x55 0x53` 角度帧，`gyro_age_ms < 350`、校验错误不持续增长且 `rx_overflows=0`；无有效帧时不得开始电机测试。
3. 小车置于安全地面，先发 `O` 做同速观察；再发 `N` 做 650 ms 小差速 yaw 响应标定。`N` 未通过或 yaw 变化不足 0.8° 时，停止排查安装方向、帧格式和地面摩擦。
4. 仅在 `N` 已记录 `yaw_response_sign` 后发 `C`，进行 6 s 外环 PID → 双速度内环的闭环直行。日志必须保存 yaw 基线/误差、PID 输出、左右目标 counts/s、编码器速度与 PWM；采集后用 `scripts/analyze_gyro_straight_log.py <log>` 做首轮完整性检查。
5. 通过条件：无 gyro 超时/校验故障/堵转，PID 输出未持续顶在 ±240 counts/s 限幅，停止后 `PE6(STBY)` 关闭。未测真实轮径前，只对 counts/s 和航向误差下结论。

### B7：闭环与比赛功能

进入条件：B0~B6 均有日志证据、pin-map 冲突为零、电机/编码器方向已固定。

执行顺序：

```text
motor + encoder open-loop
  -> speed loop
  -> straight / turn motion test
  -> line/ultrasonic assist
  -> radar / wireless protocol intake
  -> competition state machine
```

## 4. 推荐日志字段

| 阶段 | 最小字段 |
| --- | --- |
| 启动 | `time_ms,fw_version,clock_hz,reset_reason,stage,event` |
| 电机 | `time_ms,wheel,dir,pwm,stby,event` |
| 编码器 | `time_ms,wheel,enc_count,enc_delta,speed,sign,event` |
| 循迹 | `time_ms,adc0,adc1,adc2,x1,x2,x3,x4,x5,x6,x7,x8,line_state,event` |
| 超声波 | `time_ms,echo_us,distance_mm,valid,event` |
| UART | `time_ms,port,rx_len,frame_type,crc_ok,timeout,event` |
| 速度环 | `time_ms,target_l,target_r,speed_l,speed_r,pwm_l,pwm_r,err_l,err_r,event` |

## 5. 失败时的回退规则

| 现象 | 首先回退/检查 |
| --- | --- |
| 烧录成功但无启动反应 | USB 串口、时钟、复位、BOOT0、最小 LED。 |
| 四轮同转但不能单轮控制 | TB6612 IN/PWM 对应、STBY、PA2/PA3 定时器通道和接线。 |
| 电机转但编码器无数据 | PA0/PA1、PA6/PA7 的 A/B 接线、供电和编码器模式。 |
| 串口乱码或无数据 | TX/RX 交叉、共地、波特率/帧格式、是否误启用了冲突复用。 |
| OLED 与 USART3 异常 | 检查 PB10/PB11 是否被 UART3 默认功能占用，必要时改走 PD8/PD9 重映射。 |
| PB0/PB1 行为异常 | 检查是否把 ADC/IO 备用脚错误配置成 TIM3_CH3/CH4。 |

## 6. 变更与验收

一次阶段通过必须满足：

1. 有可复现步骤。
2. 有至少一条日志、波形、照片或可见输出证据。
3. 已说明使用的引脚和外设复用。
4. 与 `rules.md` 无冲突。
5. 有回退方式，例如恢复上一份固件或关闭新增外设初始化。
