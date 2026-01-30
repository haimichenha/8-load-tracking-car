# PID 调参计划

## 一、系统参数总结

### 硬件参数
| 参数 | 值 |
|------|-----|
| 电机型号 | MG310 (1:20 减速比) |
| 编码器 | 11线霍尔，4倍频 = 880脉冲/转 |
| 轮子直径 | 48mm (周长 150.7mm) |
| 最大速度 | ~2000 mm/s (100% PWM) |
| 死区 | 左 4%，右 5% |
| 控制周期 | 20ms (50Hz) |
| 循迹传感器 | 8通道，60mm宽度 |

### 编码器方向配置
| 轮子 | 反转设置 | 说明 |
|------|----------|------|
| 左轮 | `countL = -countL` | 需要反转 |
| 右轮 | 不反转 | 原始方向正确 |

---

## 二、PID 架构

```
┌─────────────────────────────────────────────────────────────┐
│                    循迹 PID (外环)                           │
│  文件: app_control.c                                        │
│  输入: 循迹误差 (-30 ~ +30)                                  │
│  输出: 转向速度 Vz                                           │
│  频率: 50Hz                                                 │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                    速度 PID (内环)                           │
│  文件: app_motor.c                                          │
│  输入: 目标速度 vs 实际速度 (mm/s)                           │
│  输出: PWM 占空比 (0-100%)                                   │
│  频率: 50Hz                                                 │
└─────────────────────────────────────────────────────────────┘
```

**调参原则：先调内环（速度环），再调外环（循迹环）**

---

## 三、第一阶段：速度环调参

### 3.1 目标
让电机能准确跟踪目标速度，响应快、无超调

### 3.2 参数位置
文件：`Hardware/app_motor.c`

```c
#define MOTOR_SPEED_MAX_MMS     2000.0f     // 最大速度
#define MOTOR_DEADZONE_L_PCT    4           // 左轮死区
#define MOTOR_DEADZONE_R_PCT    5           // 右轮死区
#define MOTOR_PID_KP            0.08f       // 比例系数
#define MOTOR_PID_KI            0.02f       // 积分系数
#define MOTOR_PID_KD            0.0f        // 微分系数
```

### 3.3 调参步骤

#### 步骤 1：纯 P 控制
```c
#define MOTOR_PID_KP  0.05f
#define MOTOR_PID_KI  0.0f
#define MOTOR_PID_KD  0.0f
```

**测试方法：**
1. 设置目标速度 500 mm/s
2. 观察实际速度是否接近目标
3. 如果响应慢，增大 Kp
4. 如果震荡，减小 Kp

**Kp 调整参考：**
| 现象 | 调整 |
|------|------|
| 响应慢，达不到目标 | Kp × 1.5 |
| 轻微震荡 | Kp × 0.8 |
| 剧烈震荡 | Kp × 0.5 |

#### 步骤 2：加入 I 控制
当 P 控制稳定后，加入 I 消除静差：

```c
#define MOTOR_PID_KI  0.01f  // 从小值开始
```

**调整方法：**
- 如果静差消除太慢，增大 Ki
- 如果出现震荡，减小 Ki
- Ki 通常是 Kp 的 1/4 ~ 1/10

#### 步骤 3：D 控制（可选）
速度环通常不需要 D，但如果需要抑制超调：

```c
#define MOTOR_PID_KD  0.001f  // 非常小的值
```

### 3.4 速度环验证
创建测试程序，验证速度跟踪效果：

```c
// 在 main.c 中测试
void SpeedLoopTest(void)
{
    static uint32_t lastMs = 0;
    static int step = 0;
    uint32_t now = SysTick_GetMs();
    
    if (now - lastMs > 2000)  // 每2秒切换
    {
        lastMs = now;
        step = (step + 1) % 4;
    }
    
    switch (step)
    {
        case 0: Motion_Car_Control(300, 0, 0); break;  // 低速
        case 1: Motion_Car_Control(600, 0, 0); break;  // 中速
        case 2: Motion_Car_Control(300, 0, 0); break;  // 低速
        case 3: Motion_Car_Control(0, 0, 0);   break;  // 停止
    }
}
```

**验证标准：**
- [ ] 速度误差 < 10%
- [ ] 无明显震荡
- [ ] 启动/停止平滑

---

## 四、第二阶段：循迹环调参

### 4.1 目标
让小车能平稳跟踪黑线，转弯流畅

### 4.2 参数位置
文件：`Hardware/app_control.c`

```c
#define TRACK_BASE_SPEED    300     // 基础速度 (mm/s)
#define TRACK_PID_KP        15.0f   // 比例系数
#define TRACK_PID_KI        0.0f    // 积分系数
#define TRACK_PID_KD        5.0f    // 微分系数
```

### 4.3 调参步骤

#### 步骤 1：确定基础速度
从低速开始：
```c
#define TRACK_BASE_SPEED  200  // 先用低速
```

#### 步骤 2：纯 P 控制
```c
#define TRACK_PID_KP  10.0f
#define TRACK_PID_KI  0.0f
#define TRACK_PID_KD  0.0f
```

**测试方法：**
1. 放在直线上，观察是否能保持直行
2. 放在弯道上，观察能否跟踪

**Kp 调整：**
| 现象 | 调整 |
|------|------|
| 转弯不够，冲出线 | Kp × 1.5 |
| 左右摇摆 | Kp × 0.7 |
| 剧烈抖动 | Kp × 0.5 |

#### 步骤 3：加入 D 控制
D 控制对循迹非常重要，能抑制摇摆：

```c
#define TRACK_PID_KD  3.0f  // 从 Kp/3 开始
```

**调整方法：**
- 如果还在摇摆，增大 Kd
- 如果响应变慢，减小 Kd
- Kd 通常是 Kp 的 1/3 ~ 1/2

#### 步骤 4：I 控制（通常不需要）
循迹一般不需要 I 控制，因为：
- 误差是位置误差，不是速度误差
- I 会导致积分饱和问题

### 4.4 循迹验证

**测试场景：**
1. 直线：应该平稳直行
2. 缓弯：应该平滑转弯
3. 急弯：应该能跟上
4. S弯：应该流畅通过

**验证标准：**
- [ ] 直线无明显摇摆
- [ ] 弯道不冲出线
- [ ] 无剧烈抖动

---

## 五、参数推荐值

### 5.1 保守参数（稳定优先）
```c
// 速度环
#define MOTOR_PID_KP  0.05f
#define MOTOR_PID_KI  0.01f
#define MOTOR_PID_KD  0.0f

// 循迹环
#define TRACK_BASE_SPEED  200
#define TRACK_PID_KP  10.0f
#define TRACK_PID_KI  0.0f
#define TRACK_PID_KD  3.0f
```

### 5.2 中等参数（平衡）
```c
// 速度环
#define MOTOR_PID_KP  0.08f
#define MOTOR_PID_KI  0.02f
#define MOTOR_PID_KD  0.0f

// 循迹环
#define TRACK_BASE_SPEED  300
#define TRACK_PID_KP  15.0f
#define TRACK_PID_KI  0.0f
#define TRACK_PID_KD  5.0f
```

### 5.3 激进参数（速度优先）
```c
// 速度环
#define MOTOR_PID_KP  0.12f
#define MOTOR_PID_KI  0.03f
#define MOTOR_PID_KD  0.001f

// 循迹环
#define TRACK_BASE_SPEED  500
#define TRACK_PID_KP  20.0f
#define TRACK_PID_KI  0.0f
#define TRACK_PID_KD  8.0f
```

---

## 六、调试技巧

### 6.1 使用 OLED 显示
```c
// 显示关键变量
OLED_ShowNum(0, 0, targetSpeed, 4);
OLED_ShowNum(0, 2, actualSpeed, 4);
OLED_ShowNum(0, 4, trackError, 3);
OLED_ShowNum(0, 6, pwmOutput, 3);
```

### 6.2 使用串口输出
```c
// 每 100ms 输出一次
printf("T:%d A:%d E:%d P:%d\r\n", 
       targetSpeed, actualSpeed, trackError, pwmOutput);
```

### 6.3 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 原地打转 | 编码器方向错误 | 检查 ENCODER_x_INVERT |
| 速度不稳 | Kp 太大 | 减小速度环 Kp |
| 跟不上线 | Kp 太小 | 增大循迹环 Kp |
| 左右摇摆 | Kd 太小 | 增大循迹环 Kd |
| 冲出弯道 | 速度太快 | 减小 BASE_SPEED |

---

## 七、调参记录表

| 日期 | Kp | Ki | Kd | 速度 | 效果 | 备注 |
|------|-----|-----|-----|------|------|------|
| | | | | | | |
| | | | | | | |
| | | | | | | |

---

## 八、下一步计划

1. [ ] 关闭测试模式，恢复正常循迹
2. [ ] 从保守参数开始测试
3. [ ] 逐步调整速度环
4. [ ] 验证速度环后调整循迹环
5. [ ] 记录每次调整的效果
