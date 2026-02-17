#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "bsp_usart.h"
#include "bsp_key.h"
#include "bsp_key2.h"
#include "bsp_systick.h"
#include "bsp_led_pwm.h"
#include "bsp_buzzer.h"
#include "app_motor.h"
#include "bsp_ir_gpio.h"
#include "HCSR04.h"
#include <stdio.h>

/*====================================================================================*/
/*                          GPIO 循迹 + 高频闭环 (V7)                                  */
/*====================================================================================*/
/*
 * GPIO 直接读取循迹 (< 1μs vs I2C 500μs)
 * 控制周期 2ms (500Hz)
 * 非线性Kp + 弯道自动减速
 * 速度PI闭环 + 循迹PD外环
 */

/*====================================================================================*/
/*====================================================================================*/
/*                    【干净版】速度PI闭环 + 循迹PD                                   */
/*====================================================================================*/
/*
 * 调参顺序：
 * 
 * 【第一步】速度环（内环）：先P后I，不要D
 *   1. Ki=0，从Kp=0.01开始翻倍，直到电机能转
 *   2. 继续增加Kp，直到出现"嗡嗡"震动，记录此值
 *   3. 取50%作为最终Kp
 *   4. 加Ki（通常是Kp的1/10~1/100），消除静差
 * 
 * 【第二步】循迹环（外环）：先P后D
 *   1. Kd=0，从小到大调Kp，直到出现明显波动
 *   2. 取50%~60%作为最终Kp
 *   3. 加Kd抑制振荡，让过弯更丝滑
 */

/*==================== 速度环参数（内环）====================*/
/* MG310 电机动力很强，PWM=30 就能跑 650mm/s
 * Kp=0.18/Ki=0.12 在500mm/s时OK但320mm/s时PWM振荡(0~33跳动)
 * Kp=0.12/Ki=0.04 不振荡但太慢：目标364实际220，差速建立不起来
 * 折中：Kp=0.15/Ki=0.06，兼顾平滑与响应
 */
#define SPEED_KP            0.15f       /* 速度P - 折中值 */
#define SPEED_KI            0.08f       /* 速度I - 竞速提高：更快跟踪高目标速度 */
#define INTEGRAL_MAX        200.0f      /* 积分限幅 - 竞速放宽：高速需更大积分空间 */

/*==================== 循迹环参数（外环）====================*/
/* 调试步骤：
 * 1. 先设 Kd=0，从小到大调 Kp
 * 2. Kp 太小：转弯迟钝，跟不上线
 * 3. Kp 太大：左右摆动（蛇形）
 * 4. 找到临界点后，加 Kd 抑制摆动
 */
#define TRACK_KP            12.00f       /* 循迹P - 竞速650：高速需更强响应 */
#define TRACK_KD            2.50f        /* 循迹D - 竞速650：高速需更强阻尼 */

/*==================== 速度设定 ====================*/
/* 注意：PWM=30 对应约 650 mm/s
 * 基础速度要根据实际电机能力设置
 */
#define BASE_SPEED_MMS      650.0f      /* 竞速模式 650mm/s */
#define MIN_SPEED_MMS        60.0f      /* 最低速度 mm/s */
#define MAX_SPEED_MMS       1100.0f     /* 提高上限：650+400(turnOut)=1050 */

/*==================== PWM限制 ====================*/
/* 注意：电机驱动接受 0~100 的百分比值
 * PWM_MAX 设太低会导致 Kp 调不出震动（输出被限幅）
 * 建议设为 80~100，给 PI 足够的调节空间
 */
#define PWM_MAX             100.0f
#define PWM_MIN             0.0f

/*==================== 滤波参数 ====================*/
#define SPEED_FILT_ALPHA    0.3f        /* 速度滤波 - 增大减少滞后 */

/*==================== 辅助宏 ====================*/
#define ABSF(x)             (((x) >= 0.0f) ? (x) : -(x))

#define DISPLAY_PAGES       2

/*==================== 日志缓冲系统 ====================*/
/* 运行时存到RAM，停下来后通过串口导出
 * 每条日志 8 字节，可存约 2000 条（约 20 秒 @ 10ms周期）
 */
#define LOG_ENABLE          1           /* 1=启用日志，0=关闭 */
#define LOG_PERIOD_MS       10          /* 日志记录周期（ms）*/
#define LOG_MAX_ENTRIES     1400        /* 最大日志条数（扩展诊断字段后降低容量以控RAM） */

typedef struct {
    uint16_t timeMs;        /* 相对时间（ms） */
    uint16_t seq;           /* 行序号（导出完整性校验） */
    uint8_t  testId;        /* 每行都带测试编号，避免分片时丢上下文 */

    int8_t   trackError;    /* 循迹误差 */
    uint8_t  sensorRaw;     /* 传感器原始值 */
    int8_t   pwmL;          /* 左轮PWM */
    int8_t   pwmR;          /* 右轮PWM */
    int8_t   speedL;        /* 左轮速度/10（实际=值*10 mm/s） */
    int8_t   speedR;        /* 右轮速度/10 */
    int16_t  tgtL;          /* 目标左轮速度 mm/s */
    int16_t  tgtR;          /* 目标右轮速度 mm/s */

    uint8_t  flags;         /* bit0:edgeTurn, bit1:edgeL, bit2:edgeR,
                               bit3:recovery, bit4:protection,
                               bit5:postEtLock, bit6:kpBoost, bit7:stabilize */
    uint8_t  edgeTurnSlow;  /* 边缘退出缓行剩余帧 */
    uint8_t  satInfo;       /* bit0:turnSat, bit1:pwmLsat, bit2:pwmRsat */
    int8_t   kpScale10;     /* 动态Kp倍率*10（10=1.0x） */
    int8_t   dtMs;          /* 控制周期ms */
    int8_t   errSpdL10;     /* (tgtL-speedL)/10 */
    int8_t   errSpdR10;     /* (tgtR-speedR)/10 */

    int16_t  turnOutput;    /* 转向输出（合成后） */
    int16_t  dynBaseSpd;    /* 动态基础速度 mm/s */
} LogEntry_t;

#if LOG_ENABLE
static LogEntry_t s_logBuffer[LOG_MAX_ENTRIES];
static uint16_t s_logIndex = 0;
static uint16_t s_logSeq = 0;
static uint32_t s_logStartMs = 0;
static uint32_t s_lastLogMs = 0;
static uint8_t s_logRunning = 0;
static uint8_t s_testNum = 0;          /* 当前测试编号（1,2,3...） */

/* 添加一条日志 */
static void Log_Add(uint32_t nowMs, int8_t err, uint8_t raw,
                    float pwmL, float pwmR, float spdL, float spdR,
                    float tgtL, float tgtR,
                    uint8_t flags, uint8_t etSlow,
                    uint8_t satInfo, float kpScale, float dtMs,
                    float errSpdL, float errSpdR,
                    float turnOut, float dynBase)
{
    if (s_logIndex >= LOG_MAX_ENTRIES) return;

    LogEntry_t *entry = &s_logBuffer[s_logIndex];
    float kp10 = kpScale * 10.0f;
    float eL10 = errSpdL / 10.0f;
    float eR10 = errSpdR / 10.0f;
    entry->timeMs = (uint16_t)(nowMs - s_logStartMs);
    entry->seq = s_logSeq++;
    entry->testId = s_testNum;
    entry->trackError = err;
    entry->sensorRaw = raw;
    entry->pwmL = (int8_t)pwmL;
    entry->pwmR = (int8_t)pwmR;
    entry->speedL = (int8_t)(spdL / 10.0f);
    entry->speedR = (int8_t)(spdR / 10.0f);
    entry->tgtL = (int16_t)tgtL;
    entry->tgtR = (int16_t)tgtR;
    entry->flags = flags;
    entry->edgeTurnSlow = etSlow;
    entry->satInfo = satInfo;
    if (kp10 > 127.0f) kp10 = 127.0f;
    if (kp10 < -128.0f) kp10 = -128.0f;
    entry->kpScale10 = (int8_t)kp10;
    if (dtMs > 127.0f) dtMs = 127.0f;
    if (dtMs < -128.0f) dtMs = -128.0f;
    entry->dtMs = (int8_t)dtMs;
    if (eL10 > 127.0f) eL10 = 127.0f;
    if (eL10 < -128.0f) eL10 = -128.0f;
    if (eR10 > 127.0f) eR10 = 127.0f;
    if (eR10 < -128.0f) eR10 = -128.0f;
    entry->errSpdL10 = (int8_t)eL10;
    entry->errSpdR10 = (int8_t)eR10;
    entry->turnOutput = (int16_t)turnOut;
    entry->dynBaseSpd = (int16_t)dynBase;
    s_logIndex++;
}

/* 插入测试分隔标记：trackError=127 表示这是一个标记条目，testNum存在sensorRaw中 */
static void Log_InsertTestMarker(uint32_t nowMs)
{
    if (s_logIndex >= LOG_MAX_ENTRIES) return;
    LogEntry_t *entry = &s_logBuffer[s_logIndex];
    entry->timeMs = 0;
    entry->seq = s_logSeq++;
    entry->testId = s_testNum;
    entry->trackError = 127;        /* 特殊标记值，正常范围不会出现 */
    entry->sensorRaw = s_testNum;   /* 测试编号 */
    entry->pwmL = 0;
    entry->pwmR = 0;
    entry->speedL = 0;
    entry->speedR = 0;
    entry->tgtL = 0;
    entry->tgtR = 0;
    entry->flags = 0xFF;            /* 全1标志，便于识别 */
    entry->edgeTurnSlow = 0;
    entry->satInfo = 0;
    entry->kpScale10 = 10;
    entry->dtMs = 0;
    entry->errSpdL10 = 0;
    entry->errSpdR10 = 0;
    entry->turnOutput = 0;
    entry->dynBaseSpd = 0;
    s_logIndex++;
}

/* 开始记录（追加模式：不清空缓冲区，插入测试标记） */
static void Log_Start(uint32_t nowMs)
{
    s_testNum++;
    Log_InsertTestMarker(nowMs);
    s_logStartMs = nowMs;
    s_lastLogMs = nowMs;
    s_logRunning = 1;
}

/* 停止记录 */
static void Log_Stop(void)
{
    s_logRunning = 0;
}

/* 清空日志缓冲区 */
static void Log_Clear(void)
{
    s_logIndex = 0;
    s_logSeq = 0;
    s_testNum = 0;
    s_logRunning = 0;
}

/* 统计8位中1的个数（active触发点数） */
static uint8_t PopCount8(uint8_t x)
{
    uint8_t c = 0;
    while (x) {
        c += (uint8_t)(x & 1U);
        x >>= 1;
    }
    return c;
}

/* 导出日志到串口（CSV格式，改进版：每行延时防截断，测试标记分隔）*/
static void Log_Export(void)
{
    char buf[180];
    uint16_t i;

    /* 使用 UART4 (PC10/PC11) - DAPLink 连接到这里 */
    UART4_init(115200);
    Delay_ms(50);

    /* 导出标记 */
    sprintf(buf, "#LOG_BEGIN,%u,%u\r\n", s_logIndex, s_testNum);
    UART4_Send_String(buf);
    Delay_ms(5);

    /* 打印表头 */
    UART4_Send_String("seq,test_id,time_ms,error,sensor,active,cnt,pwmL,pwmR,speedL,speedR,tgtL,tgtR,flags,etSlow,sat,kpScale10,dtMs,errSpdL10,errSpdR10,turnOut,dynBase\r\n");
    Delay_ms(5);

    /* 打印数据 */
    for (i = 0; i < s_logIndex; i++) {
        LogEntry_t *e = &s_logBuffer[i];

        /* 测试分隔标记：trackError==127 && flags==0xFF */
        if (e->trackError == 127 && e->flags == 0xFF) {
            sprintf(buf, "#TEST,%u\r\n", e->sensorRaw);
            UART4_Send_String(buf);
            Delay_ms(3);
            continue;
        }

        {
            uint8_t active = (uint8_t)(~e->sensorRaw);
            uint8_t cnt = PopCount8(active);
            sprintf(buf, "%u,%u,%u,%d,0x%02X,0x%02X,%u,%d,%d,%d,%d,%d,%d,0x%02X,%u,0x%02X,%d,%d,%d,%d,%d,%d\r\n",
                    (unsigned)e->seq,
                    (unsigned)e->testId,
                    (unsigned)e->timeMs,
                    e->trackError,
                    e->sensorRaw,
                    active,
                    cnt,
                    e->pwmL,
                    e->pwmR,
                    e->speedL * 10,
                    e->speedR * 10,
                    (int)e->tgtL,
                    (int)e->tgtR,
                    e->flags,
                    e->edgeTurnSlow,
                    e->satInfo,
                    e->kpScale10,
                    e->dtMs,
                    e->errSpdL10,
                    e->errSpdR10,
                    (int)e->turnOutput,
                    (int)e->dynBaseSpd);
            UART4_Send_String(buf);
        }

        /* 每行加2ms延时，降低串口拥塞导致的拼行风险 */
        Delay_ms(2);
    }

    sprintf(buf, "# Total: %u entries, %u tests\r\n", s_logIndex, s_testNum);
    UART4_Send_String(buf);
    UART4_Send_String("#LOG_END\r\n");
}
#endif /* LOG_ENABLE */

/* 传感器权重 - 非线性分布（1,3,9,27模式）
 * 边缘传感器权重更大，让弯道反应更激烈
 * 线在左边 → 误差为负 → 左轮减速
 * 线在右边 → 误差为正 → 右轮减速
 */
static const int8_t SENSOR_WEIGHTS_V5[8] = {
    -12,    /* X1: 最左边 - 超大权重！ */
    -8,    /* X2 */
    -3,    /* X3 */
    -1,     /* X4: 中心左 */
    1,      /* X5: 中心右 */
    3,     /* X6 */
    8,     /* X7 */
    12      /* X8: 最右边 - 超大权重！ */
};

/*==================== 速度PI结构体 ====================*/
typedef struct {
    float Kp, Ki;
    float integral;
    float integralMax;
    float outMin, outMax;
} PI_t;

static PI_t s_piL, s_piR;           /* 左右轮速度PI */
static float s_pwmL = 0, s_pwmR = 0; /* 当前PWM */
static float s_speedL_f = 0, s_speedR_f = 0;  /* 滤波后速度 */
static uint8_t s_speedFiltInit = 0;  /* 速度滤波初始化标志 */
static int8_t s_lastDir = 1;         /* 上次方向：1=右，-1=左 */
static uint8_t s_dirConfident = 0;   /* 方向置信度：1=来自|err|>=3的深弯，0=来自|err|<3的浅偏 */
static int8_t s_lastErr = 0;         /* 上次误差（显示用） */
static uint32_t s_runStartMs = 0;    /* 本次运行启动时刻 */
static uint16_t s_finishVoteCnt = 0; /* 终点检测：连续全触发帧计数 */

/*==================== 超声波避障 ====================*/
/*
 * 40cm 预警(不干涉), 20cm 介入(接管), 30cm 退出(迟滞)
 * 梯度法: 下次距离 > 上次 → 方向正确; 反之翻转
 * 有线时仅降速, 无线时转向避障
 */
#define OBS_PERIOD_MS       60      /* 测量周期 ms */
#define OBS_WARN_CM         40.0f
#define OBS_AVOID_CM        20.0f
#define OBS_EXIT_CM         30.0f   /* 退出迟滞 */
#define OBS_SPEED_FLOOR     0.25f   /* 最低速度比 */
#define OBS_TURN_BASE       180.0f  /* 避障基础转向 */
#define OBS_FILT_ALPHA      0.4f    /* 距离一阶滤波 */
#define OBS_DIR_DEADBAND    1.0f    /* 梯度翻转死区 cm */
#define OBS_DIR_LOCK_CNT    2       /* 翻转后锁定次数 */

typedef enum { OBS_NONE = 0, OBS_WARNING, OBS_AVOID } ObsState_t;

static ObsState_t s_obsState      = OBS_NONE;
static float      s_obsDist       = -1.0f;   /* 滤波后距离 */
static float      s_obsPrevDist   = -1.0f;   /* 上次距离(梯度用) */
static int8_t     s_obsTurnDir    = 0;        /* +1 右转, -1 左转 */
static uint8_t    s_obsDirLock    = 0;        /* 方向锁定倒计数 */
static uint32_t   s_lastUltrasonicMs = 0;

static void Obs_Reset(void)
{
    s_obsState    = OBS_NONE;
    s_obsDist     = -1.0f;
    s_obsPrevDist = -1.0f;
    s_obsTurnDir  = 0;
    s_obsDirLock  = 0;
}

/*==================== PI函数 ====================*/
static void PI_Init(PI_t *pi, float kp, float ki, float intMax, float outMin, float outMax)
{
    pi->Kp = kp;
    pi->Ki = ki;
    pi->integral = 0;
    pi->integralMax = intMax;
    pi->outMin = outMin;
    pi->outMax = outMax;
}

static float PI_Compute(PI_t *pi, float target, float actual)
{
    float err = target - actual;
    float out;

    /* 积分累加 - 注意：这里没有乘以dt，所以Ki要设得很小 */
    /* 控制周期是2ms，每秒累加500次 */
    pi->integral += err;

    /* 积分限幅 - 防止积分饱和！ */
    if (pi->integral > pi->integralMax) pi->integral = pi->integralMax;
    if (pi->integral < -pi->integralMax) pi->integral = -pi->integralMax;

    /* PI输出 */
    out = pi->Kp * err + pi->Ki * pi->integral;

    /* 输出限幅 */
    if (out > pi->outMax) out = pi->outMax;
    if (out < pi->outMin) out = pi->outMin;
    
    return out;
}

static void PI_Reset(PI_t *pi)
{
    pi->integral = 0;
}

int main(void)
{
    uint8_t ir_raw = 0xFF;
    uint32_t lastControlMs = 0;
    uint32_t lastOledMs = 0;
    char buf[24];
    KeyEvent_t keyEvent;
    Key2Event_t key2Event;
    uint8_t running = 0;
    uint8_t displayPage = 0;
    uint32_t c5PressStartMs = 0;
    uint8_t c5Pressing = 0;
    /* LED 亮度档位 (PWM周期=50, 值域0-50): 48/30/15/3 */
    static const uint8_t ledBrightTable[] = {48, 30, 15, 3};
    uint8_t ledBrightIdx = 0;
    uint32_t lastPeriphMs = 0;

    float speedL = 0, speedR = 0;
    float targetL = 0, targetR = 0;
    
    /* 初始化 */
    SysTick_Init();
    Key_Init();
    Key2_Init();
    
    /* GPIO 循迹初始化 */
    IR_GPIO_Init();

    /* 超声波初始化 */
    HCSR04_Init();
    
    /* OLED 初始化 */
    OLED_Init();

#if LOG_ENABLE
    /* 启动标记：用于验证PC10日志串口是否有输出 */
    UART4_init(115200);
    Delay_ms(10);
    UART4_Send_String("#BOOT\r\n");
#endif
    Delay_ms(100);

    /* LED PWM (TIM6 自驱动) + 蜂鸣器 */
    LED_PWM_Init();
    LED_Switch(LED_GREEN, 1);
    LED_Switch(LED_RED, 1);
    Buzzer_Init();

    /* 电机和编码器 */
    Set_Motor(0);
    Encoder_Init();
    Encoder_Reset();
    
    /* 速度 PI 初始化 */
    PI_Init(&s_piL, SPEED_KP, SPEED_KI, INTEGRAL_MAX, PWM_MIN, PWM_MAX);
    PI_Init(&s_piR, SPEED_KP, SPEED_KI, INTEGRAL_MAX, PWM_MIN, PWM_MAX);
    
    OLED_Clear();
    OLED_ShowString(1, 1, "V7 Track+Obs");
    OLED_ShowString(2, 1, "650mm/s GPIO");
    OLED_ShowString(3, 1, "C5L:Start/Stop");
    OLED_ShowString(4, 1, "C5S:Page C4:Stp");
    Buzzer_PlayBeep(BEEP_STARTUP);
    
    lastControlMs = SysTick_GetMs();
    lastOledMs = lastControlMs;
    s_lastUltrasonicMs = lastControlMs;
    lastPeriphMs = lastControlMs;

    while (1)
    {
        uint32_t nowMs = SysTick_GetMs();

        /* 按键扫描 — Key2 由 TIM1 ISR 每1ms扫描，此处仅读取事件 */
        keyEvent = Key_Scan();
        key2Event = Key2_GetEvent();
        if (key2Event != KEY2_EVENT_NONE) Key2_ClearEvent();

        /* C5 按压追踪 — running 时跳过 */
        if (!running) {
            if (Key_GetLevel() == 0) {
                if (!c5Pressing) { c5PressStartMs = nowMs; c5Pressing = 1; }
            } else {
                c5Pressing = 0;
            }
        }

        /* PC4 按键处理 */
        if (running) {
            /* 运行中: 任意按 = 紧急停止 */
            if (key2Event == KEY2_EVENT_SHORT || key2Event == KEY2_EVENT_LONG) {
                Motor_SetSpeedBoth(0, 0);
                running = 0;
                s_pwmL = 0; s_pwmR = 0;
                PI_Reset(&s_piL);
                PI_Reset(&s_piR);
                s_speedFiltInit = 0;
                Obs_Reset();
                TIM_Cmd(TIM6, ENABLE);
                LED_Switch(LED_GREEN, 1);
                LED_Switch(LED_RED, 1);
                Buzzer_PlayBeep(BEEP_KEY_PRESS);
#if LOG_ENABLE
                Log_Stop();
#endif
            }
        } else {
            /* 待机: 短按或长按都循环LED亮度档位 */
            if (key2Event == KEY2_EVENT_SHORT || key2Event == KEY2_EVENT_LONG) {
                ledBrightIdx = (ledBrightIdx + 1) % (sizeof(ledBrightTable)/sizeof(ledBrightTable[0]));
                LED_SetBrightness(LED_GREEN, ledBrightTable[ledBrightIdx]);
                LED_SetBrightness(LED_RED,   ledBrightTable[ledBrightIdx]);
            }
        }
        
        /* C5 短按=切页 (仅待机), C5 长按=启停 */
        if (keyEvent == KEY_EVENT_SHORT && !running) {
            displayPage = (displayPage + 1) % DISPLAY_PAGES;
            OLED_Clear();
        }
        if (keyEvent == KEY_EVENT_LONG) {
            running = running ? 0 : 1;
            if (running) {
                /* 启动 */
                Buzzer_PlayBeep(BEEP_ACTIVATE);
                LED_Switch(LED_GREEN, 0);
                LED_Switch(LED_RED, 0);
                LED_Switch(LED_INDICATOR, 0);
                TIM_Cmd(TIM6, DISABLE);
                s_piL.integral = 150.0f;
                s_piR.integral = 150.0f;
                s_speedFiltInit = 0;
                s_runStartMs = nowMs;
                s_finishVoteCnt = 0;
                Obs_Reset();
#if LOG_ENABLE
                Log_Start(nowMs);
#endif
            } else {
                /* 停止 */
                Motor_SetSpeedBoth(0, 0);
                s_pwmL = 0; s_pwmR = 0;
                PI_Reset(&s_piL);
                PI_Reset(&s_piR);
                s_speedFiltInit = 0;
                Obs_Reset();
                TIM_Cmd(TIM6, ENABLE);
                LED_Switch(LED_GREEN, 1);
                LED_Switch(LED_RED, 1);
                Buzzer_PlayBeep(BEEP_KEY_PRESS);
#if LOG_ENABLE
                Log_Stop();
#endif
            }
            s_lastErr = 0;
        }

        /* 低频外设更新 — running 100ms 仅蜂鸣器, 待机 20ms */
        {
            uint32_t periphInterval = running ? 100 : 20;
            if ((nowMs - lastPeriphMs) >= periphInterval) {
                Buzzer_Update(nowMs - lastPeriphMs);
                lastPeriphMs = nowMs;
            }
        }

        /* ===== 超声波非阻塞轮询 + 避障状态机 ===== */
        HCSR04_Poll();
        if ((nowMs - s_lastUltrasonicMs) >= OBS_PERIOD_MS) {
            s_lastUltrasonicMs = nowMs;
            HCSR04_StartMeasure();
        }
        if (running && HCSR04_IsNewReady()) {
            float rawDist = HCSR04_GetDistance();
            if (rawDist > 0.0f) {
                /* 一阶低通滤波 */
                if (s_obsDist < 0.0f) s_obsDist = rawDist;
                else s_obsDist += OBS_FILT_ALPHA * (rawDist - s_obsDist);

                /* --- 状态转换 --- */
                if (s_obsDist < OBS_AVOID_CM) {
                    if (s_obsState != OBS_AVOID) {
                        /* 首次进入避障: 初始方向取循迹方向 */
                        s_obsTurnDir = s_lastDir;
                        s_obsDirLock = OBS_DIR_LOCK_CNT;
                        s_obsState = OBS_AVOID;
                    } else if (s_obsDirLock > 0) {
                        s_obsDirLock--;  /* 刚翻转，锁定等稳定 */
                    } else if (s_obsPrevDist > 0.0f) {
                        /* 梯度判断: 越来越近 → 方向错了，翻转 */
                        if (s_obsDist < s_obsPrevDist - OBS_DIR_DEADBAND) {
                            s_obsTurnDir = -s_obsTurnDir;
                            s_obsDirLock = OBS_DIR_LOCK_CNT;
                        }
                        /* 越来越远或不变 → 保持 */
                    }
                } else if (s_obsState == OBS_AVOID) {
                    if (s_obsDist >= OBS_EXIT_CM) {
                        /* 退出避障: 设搜索方向为避障反方向（线在那一侧） */
                        s_lastDir = -s_obsTurnDir;
                        s_dirConfident = 1;
                        s_obsState = OBS_NONE;
                    }
                    /* 20~30cm 迟滞区: 保持避障 */
                } else if (s_obsDist < OBS_WARN_CM) {
                    s_obsState = OBS_WARNING;
                } else {
                    s_obsState = OBS_NONE;
                }
                s_obsPrevDist = s_obsDist;
            }
            /* 测量失败(-1): 保持当前状态 */
        }

        /* 编码器更新 + 控制周期 2ms（高频控制，快速响应）
         * 500mm/s 下每2ms移动1mm，必须快速反应
         */
        if ((nowMs - lastControlMs) >= 2)
        {
            float dt = (float)(nowMs - lastControlMs) / 1000.0f;
            if (dt < 0.002f) dt = 0.002f;
            
            /* 第一时间读取传感器！不要等！ */
            ir_raw = IR_GPIO_Read();
            
            /* 更新编码器 */
            Encoder_Update(nowMs - lastControlMs);
            speedL = Encoder_GetSpeedMMS(ENCODER_LEFT);
            speedR = Encoder_GetSpeedMMS(ENCODER_RIGHT);

            if (!s_speedFiltInit) {
                s_speedL_f = speedL;
                s_speedR_f = speedR;
                s_speedFiltInit = 1;
            } else {
                s_speedL_f += SPEED_FILT_ALPHA * (speedL - s_speedL_f);
                s_speedR_f += SPEED_FILT_ALPHA * (speedR - s_speedR_f);
            }

            lastControlMs = nowMs;
            
            if (!running) {
                Motor_SetSpeedBoth(0, 0);
                continue;
            }
            
            /*==================== 串级控制：循迹PD(外环) + 速度PI(内环) ====================*/
            /* V7 极简版：砍掉所有复杂状态机，回归"看到线就跟"的核心
             * 原则：
             * 1. 传感器有线 → PD 立刻跟，不要任何延迟/锁定
             * 2. 丢线 → 按上次方向低速转，一旦看到线立刻恢复PD
             * 3. 弯道 → 靠非线性Kp + 自动减速，不需要ET模式
             */
            {
                uint8_t active = (uint8_t)(~ir_raw);  /* 1=触发 */
                int16_t sum = 0;
                uint8_t count = 0;
                uint8_t i;
                float trackError = 0;
                float turnOutput;
                float spdL_filt, spdR_filt;
                float pwmL, pwmR;
                float logTurnOutput = 0.0f;
                float logDynBaseSpd = 0.0f;
                uint8_t logFlags = 0;
                uint8_t logSatInfo = 0;
                float logKpScale = 1.0f;
                float errSpdL = 0.0f;
                float errSpdR = 0.0f;
                static float lastTrackError = 0;
                static uint8_t lostLineFrames = 0;  /* 连续丢线帧数 */

                /*---------- 第一步：计算循迹误差 ----------*/
                for (i = 0; i < 8; i++) {
                    if (active & (1U << (7 - i))) {
                        sum += SENSOR_WEIGHTS_V5[i];
                        count++;
                    }
                }

                /*---------- 终点检测（投票机制）----------
                 * 全部8个传感器触发 → 投票+1，否则归零
                 * 连续20帧(40ms)全触发 → 判定为终点停车线
                 * 封闭图形交叉路口全触发通常仅3-8帧(6-16ms)，不会误触发
                 * 前5秒不检测：避免起跑线误判
                 */
                if (count == 8 && (nowMs - s_runStartMs) > 5000) {
                    s_finishVoteCnt++;
                    if (s_finishVoteCnt >= 40) {
                        /* 终点！停车 */
                        Motor_SetSpeedBoth(0, 0);
                        running = 0;
                        s_pwmL = 0; s_pwmR = 0;
                        PI_Reset(&s_piL);
                        PI_Reset(&s_piR);
                        Obs_Reset();
                        /* 恢复 LED 后播终点效果 */
                        TIM_Cmd(TIM6, ENABLE);
                        LED_Switch(LED_GREEN, 1);
                        LED_Switch(LED_RED, 1);
                        Buzzer_BeepTriple();
                        LED_StartFinishEffect();
#if LOG_ENABLE
                        Log_Stop();
#endif
                        OLED_Clear();
                        OLED_ShowString(1, 1, "FINISH!");
                        {
                            char tbuf[24];
                            sprintf(tbuf, "Time: %.1fs",
                                    (float)(nowMs - s_runStartMs) / 1000.0f);
                            OLED_ShowString(2, 1, tbuf);
                        }
                        OLED_ShowString(3, 1, "Well Done!");
                        OLED_ShowString(4, 1, "C5L=Restart");
                        continue;
                    }
                } else {
                    s_finishVoteCnt = 0;
                }

                if (count == 0) {
                    /* 丢线：保持上次方向，给最大误差 */
                    trackError = (s_lastDir > 0) ? 25.0f : -25.0f;
                    if (lostLineFrames < 255) lostLineFrames++;
                    logFlags |= 0x08U; /* bit3: 丢线中 */
                } else {
                    /* 有线！立刻跟踪，不管之前是什么状态 */
                    if (count == 8) {
                        /* 全触发（十字路口）：保持上一帧轨迹
                         * 不能设0直行！如果赛道在路口处有弯，直行会跟错线
                         */
                        trackError = lastTrackError;
                        logFlags |= 0x40U; /* bit6: 交叉路口 */
                    } else if (count >= 4
                               && (active & 0xC0)   /* 左侧有触发 */
                               && (active & 0x03)) { /* 右侧也有触发 */
                        /* 左右两侧同时触发 = 交叉路口穿越中
                         * 保持上一帧轨迹，避免被交叉线的加权平均拉偏方向
                         */
                        trackError = lastTrackError;
                        logFlags |= 0x40U; /* bit6: 交叉路口 */
                    } else {
                        trackError = (float)sum / (float)count;
                    }

                    /* 丢线刚恢复：重置PI积分器防暴冲
                     * 同时把 lastTrackError 对齐到当前误差，避免 D 项因"25→小误差"突变
                     * 造成反向尖峰（T5 出现 +12/-12 来回翻的一种常见诱因）。
                     */
                    if (lostLineFrames > 5) {
                        PI_Reset(&s_piL);
                        PI_Reset(&s_piR);
                        lastTrackError = trackError;
                        logFlags |= 0x80U; /* bit7: 刚从丢线恢复 */
                    }

                    /* 交叉路口方向锁定：
                     * 丢线后短时间内找到反方向的线，说明碰到了交叉线路
                     * 忽略这条线，继续按原方向搜索，避免在封闭图形中反复纠缠
                     * 条件：丢线帧数 3~8
                     *        且找到的线方向与搜索方向相反
                     *        且误差较小(|err|<5)——大误差是真实轨迹
                     *        且方向置信度高——低置信(来自error=±1)可能是交叉线误导的
                     */
                    if (lostLineFrames >= 3 && lostLineFrames <= 8
                        && s_dirConfident) {
                        int foundDir = (trackError >= 1) ? 1 : ((trackError <= -1) ? -1 : 0);
                        if (foundDir != 0 && foundDir != s_lastDir
                            && ABSF(trackError) < 5.0f) {
                            /* 方向矛盾：这是交叉线，忽略它，继续搜索 */
                            trackError = (s_lastDir > 0) ? 25.0f : -25.0f;
                            /* 不重置 lostLineFrames，继续搜索 */
                            logFlags |= 0x40U; /* bit6: 交叉路口忽略 */
                            goto skip_line_found;
                        }
                    }

                    lostLineFrames = 0;

                    /* 更新方向记忆 — 连续帧确认 + 置信度标记
                     * 同方向：立即保持（计数器归零），大误差时提升置信度
                     * 反方向：需连续5帧(10ms)确认才翻转
                     * 翻转时标记置信度：|error|>=3 → 高置信，|error|<3 → 低置信
                     * 交叉路口拒绝逻辑只信任高置信度方向，避免被交叉线误导
                     */
                    {
                        static uint8_t s_dirFlipCnt = 0;
                        int8_t curDir = 0;
                        if (trackError >= 1) curDir = 1;
                        else if (trackError <= -1) curDir = -1;

                        if (curDir == s_lastDir || curDir == 0) {
                            s_dirFlipCnt = 0;  /* 同方向或居中，重置 */
                            /* 同方向时，大误差升级置信度 */
                            if (curDir == s_lastDir && ABSF(trackError) >= 3.0f)
                                s_dirConfident = 1;
                        } else {
                            /* 反方向：累计确认 */
                            s_dirFlipCnt++;
                            if (s_dirFlipCnt >= 6) {
                                s_lastDir = curDir;
                                s_dirFlipCnt = 0;
                                /* 翻转时标记置信度 */
                                s_dirConfident = (ABSF(trackError) >= 3.0f) ? 1 : 0;
                            }
                        }
                    }
                }
                skip_line_found:

                /*---------- 第二步：PD计算转向 ----------*/
                {
                    float absErr = ABSF(trackError);
                    float dynamicKp = TRACK_KP;
                    float dynamicBaseSpeed = BASE_SPEED_MMS;
                    float dErr;

                    /* 非线性Kp：误差越大，Kp越大（竞速650版）
                     * 小误差(0~1)：1.0x — 直线稳定
                     * 中误差(1~8)：1.10~2.50x — 高速下更早更强的弯道响应
                     *   err=3: 1.50x turnOut=54
                     *   err=5: 1.90x turnOut=114
                     *   err=7: 2.30x turnOut=193
                     * 大误差(8~25)：2.5~4.0x — 急弯时暴力摆车身
                     *   err=10: 2.70x turnOut=324
                     *   err=12: 2.90x turnOut=400(cap)
                     */
                    if (absErr > 8.0f) {
                        dynamicKp = TRACK_KP * (2.5f + (absErr - 8.0f) * 0.10f);
                        if (dynamicKp > TRACK_KP * 4.0f) dynamicKp = TRACK_KP * 4.0f;
                    } else if (absErr > 1.0f) {
                        dynamicKp = TRACK_KP * (1.10f + (absErr - 1.0f) * 0.20f);
                    }
                    logKpScale = dynamicKp / TRACK_KP;

                    /* 弯道自动减速（竞速650版）：误差越大速度越低
                     * absErr=0: 100% (650)
                     * absErr=2: 97% (630)
                     * absErr=3: 90% (588)
                     * absErr=5: 77% (503)
                     * absErr=7: 64% (418)
                     * absErr=10: 45% (291)
                     * absErr=12: 32% (207)
                     * absErr=15+: clamp 15% (98)
                     */
                    if (absErr > 1.5f) {
                        float speedFactor = 1.0f - (absErr - 1.5f) * 0.065f;
                        if (speedFactor < 0.12f) speedFactor = 0.12f;
                        dynamicBaseSpeed *= speedFactor;
                    }

                    /* 直角弯预减速：误差变化率大 → 即将进入急弯
                     * 正常S弯每帧误差变化1~2，直角弯入口可达3~7
                     * 检测到快速变化时立即额外减速，给外侧轮留足转向空间
                     * 同时为内侧轮快速降零创造条件（dynBase低 + turnOut大 → inner < 0 → 0）
                     */
                    if (dt < 0.015f && lostLineFrames == 0) {
                        float errDelta = ABSF(trackError - lastTrackError);
                        if (errDelta > 2.0f && absErr > 1.5f) {
                            float decelExtra = 1.0f - (errDelta - 2.0f) * 0.15f;
                            if (decelExtra < 0.40f) decelExtra = 0.40f;
                            dynamicBaseSpeed *= decelExtra;
                        }
                    }

                    /* 丢线时：分阶段搜索（降低初始速度，减少角动量） */
                    if (lostLineFrames > 0) {
                        if (lostLineFrames <= 15) {
                            /* 刚丢线：适中速度搜索 */
                            dynamicBaseSpeed = 180.0f;
                        } else if (lostLineFrames <= 50) {
                            /* 中期：持续搜索 */
                            dynamicBaseSpeed = 150.0f;
                        } else {
                            /* 长时间丢线：低速搜索 */
                            dynamicBaseSpeed = 120.0f;
                        }
                        logFlags |= 0x10U; /* bit4: 丢线降速 */
                    }

                    /* OLED gap 后误差限速：
                     * gap期间车身物理移动，误差可能跳变很大
                     * 如果不限速，P项直接响应跳变 → 方向翻转 → 乱拐弯
                     * 限制每gap帧最大变化量4，且禁止大误差时符号翻转
                     */
                    if (dt > 0.015f && lostLineFrames == 0) {
                        float errDiff = trackError - lastTrackError;
                        if (errDiff > 4.0f)  trackError = lastTrackError + 4.0f;
                        if (errDiff < -4.0f) trackError = lastTrackError - 4.0f;
                        /* 防止符号翻转：误差>1.5时不允许跳过零点 */
                        if (lastTrackError >  1.5f && trackError < 0.0f) trackError = 0.5f;
                        if (lastTrackError < -1.5f && trackError > 0.0f) trackError = -0.5f;
                        logFlags |= 0x20U; /* bit5: OLED gap 误差限速 */
                    }

                    /* PD计算 */
                    dErr = (trackError - lastTrackError) / dt;

                    /* OLED gap 期间 dt 远大于正常 2ms，D 项计算不可靠
                     * dt > 15ms 说明经历了 OLED 刷新，抑制 D 项避免尖峰抖动
                     */
                    if (dt > 0.015f) {
                        dErr = 0.0f;
                    }

                    /* 限制微分项，防止噪声放大
                     * clamp=15: 竞速模式需更大D项空间，最大D贡献=2.5*15=37.5
                     */
                    if (dErr > 15.0f) dErr = 15.0f;
                    if (dErr < -15.0f) dErr = -15.0f;

                    turnOutput = dynamicKp * trackError + TRACK_KD * dErr;
                    lastTrackError = trackError;

                    /* 转向限幅：竞速模式需更大转向空间 */
                    {
                        float turnLimit = 420.0f;
                        if (lostLineFrames > 0) {
                            if (lostLineFrames <= 30) {
                                /* 丢线前期(60ms)：最大转向，紧凑搜索弧线
                                 * 锐角弯后惯性大，必须快速旋转才能找回线
                                 * 400的turnOut + 180的dynBase → 外轮580, 内轮0 */
                                turnLimit = 420.0f;
                            } else {
                                /* 丢线后期：适度降低，避免过度旋转 */
                                turnLimit = 340.0f;
                            }
                        }
                        if (turnOutput > turnLimit) {
                            turnOutput = turnLimit;
                            logSatInfo |= 0x01U;
                        }
                        if (turnOutput < -turnLimit) {
                            turnOutput = -turnLimit;
                            logSatInfo |= 0x01U;
                        }
                    }

                    /* ===== 超声波避障接管 ===== */
                    if (s_obsState == OBS_AVOID) {
                        float distRatio = s_obsDist / OBS_AVOID_CM;
                        if (distRatio < OBS_SPEED_FLOOR) distRatio = OBS_SPEED_FLOOR;
                        if (distRatio > 1.0f) distRatio = 1.0f;

                        /* 距离权重降速 */
                        dynamicBaseSpeed *= distRatio;
                        if (dynamicBaseSpeed < MIN_SPEED_MMS)
                            dynamicBaseSpeed = MIN_SPEED_MMS;

                        /* 无线 → 避障转向接管; 有线 → 保持循迹仅降速 */
                        if (count == 0 || lostLineFrames > 3) {
                            turnOutput = (float)s_obsTurnDir *
                                (OBS_TURN_BASE + (1.0f - distRatio) * 120.0f);
                        }
                        logFlags |= 0x04U; /* bit2: 避障激活 */
                    }

                    /* 保存日志数据 */
                    logTurnOutput = turnOutput;
                    logDynBaseSpd = dynamicBaseSpeed;

                    /*---------- 第三步：计算目标速度 ----------*/
                    targetL = dynamicBaseSpeed + turnOutput;
                    targetR = dynamicBaseSpeed - turnOutput;
                }

                /* 目标速度限幅
                 * 关键修复：弯道/丢线时不要强行把内侧轮抬到 MIN_SPEED_MMS。
                 * 否则会“推着走”，转弯半径变大（T2/T5 的典型症状）。
                 */
                {
                    float minSpeed = MIN_SPEED_MMS;

                    /* 丢线搜索/中大误差：允许内侧轮降到 0，提高转向能力
                     * 直角弯初期err=5-7时内侧轮还在推着走，转不过去
                     * 降低阈值从8到5，让直角弯一侧触发时内侧轮及时刹停
                     */
                    if (lostLineFrames > 0 || ABSF(trackError) > 5.0f) {
                        minSpeed = 0.0f;
                    }

                    /* 目标速度不允许为负（本工程不支持反转），负值按 0 处理 */
                    if (targetL < 0.0f) targetL = 0.0f;
                    if (targetR < 0.0f) targetR = 0.0f;

                    if (targetL < minSpeed) targetL = minSpeed;
                    if (targetR < minSpeed) targetR = minSpeed;
                    if (targetL > MAX_SPEED_MMS) targetL = MAX_SPEED_MMS;
                    if (targetR > MAX_SPEED_MMS) targetR = MAX_SPEED_MMS;
                }

                /*---------- 第四步：速度PI闭环 ----------*/
                spdL_filt = s_speedFiltInit ? s_speedL_f : speedL;
                spdR_filt = s_speedFiltInit ? s_speedR_f : speedR;

                errSpdL = targetL - spdL_filt;
                errSpdR = targetR - spdR_filt;

                pwmL = PI_Compute(&s_piL, targetL, spdL_filt);
                pwmR = PI_Compute(&s_piR, targetR, spdR_filt);

                if (pwmL >= (PWM_MAX - 0.5f) || pwmL <= (PWM_MIN + 0.5f)) logSatInfo |= 0x02U;
                if (pwmR >= (PWM_MAX - 0.5f) || pwmR <= (PWM_MIN + 0.5f)) logSatInfo |= 0x04U;

                /* 设置电机 */
                Motor_SetSpeedBoth((int16_t)pwmL, (int16_t)pwmR);

                /* 更新显示变量 */
                s_pwmL = pwmL;
                s_pwmR = pwmR;
                s_lastErr = (int8_t)trackError;

#if LOG_ENABLE
                /* 日志记录 */
                if (s_logRunning && (nowMs - s_lastLogMs) >= LOG_PERIOD_MS) {
                    s_lastLogMs = nowMs;
                    Log_Add(nowMs, (int8_t)trackError, ir_raw, pwmL, pwmR, s_speedL_f, s_speedR_f, targetL, targetR,
                            logFlags, (uint8_t)lostLineFrames, logSatInfo, logKpScale, dt * 1000.0f,
                            errSpdL, errSpdR, logTurnOutput, logDynBaseSpd);
                }
#endif
            }
        }
        
        /* OLED 分帧刷新 — running 1000ms, 待机 800ms */
        {
            static uint8_t oledLine = 0;
            uint8_t c5Held = !running && c5Pressing && ((nowMs - c5PressStartMs) > 50);
            uint32_t oledInterval = c5Held ? 150 : (running ? 1000 : 800);

            if ((nowMs - lastOledMs) >= oledInterval)
            {
                lastOledMs = nowMs;

                if (c5Held) {
                    /* C5 按压进度条 */
                    uint32_t pt = nowMs - c5PressStartMs;
                    uint8_t filled = (uint8_t)(pt * 8 / KEY_LONG_TIME);
                    uint8_t j;
                    if (filled > 8) filled = 8;
                    buf[0] = '[';
                    for (j = 0; j < 8; j++)
                        buf[1 + j] = (j < filled) ? '=' : ' ';
                    buf[9] = ']';
                    sprintf(buf + 10, " %u.%us",
                            (unsigned)(pt / 1000),
                            (unsigned)((pt % 1000) / 100));
                    OLED_ShowString(1, 1, buf);
                } else if (displayPage == 0) {
                    /* 页面0: 循迹实时 */
                    switch (oledLine) {
                    case 0: sprintf(buf, "E:%+3d Spd%3.0f", s_lastErr,
                                    (s_speedL_f + s_speedR_f) / 2);
                            OLED_ShowString(1, 1, buf); break;
                    case 1: sprintf(buf, "Tgt%3.0f/%3.0f", targetL, targetR);
                            OLED_ShowString(2, 1, buf); break;
                    case 2: sprintf(buf, "PWM%2.0f/%2.0f", s_pwmL, s_pwmR);
                            OLED_ShowString(3, 1, buf); break;
                    case 3:
                        if (s_obsState == OBS_AVOID) {
                            sprintf(buf, "OBS%4.1fcm %c  ",
                                    s_obsDist, s_obsTurnDir > 0 ? 'R' : 'L');
                        } else {
                            uint8_t act = (uint8_t)(~ir_raw);
                            char ss[9]; int si;
                            for (si = 0; si < 8; si++)
                                ss[si] = (act & (1 << (7-si))) ? '*' : '-';
                            ss[8] = '\0';
                            sprintf(buf, "%s %s", ss, running ? "RUN" : "STP");
                        }
                        OLED_ShowString(4, 1, buf); break;
                    }
                    oledLine = (oledLine + 1) & 3;
                } else {
                    /* 页面1: 参数 */
                    switch (oledLine) {
                    case 0: sprintf(buf, "Kp%.1f Kd%.1f", TRACK_KP, TRACK_KD);
                            OLED_ShowString(1, 1, buf); break;
                    case 1: sprintf(buf, "Base:%3.0f mm/s", BASE_SPEED_MMS);
                            OLED_ShowString(2, 1, buf); break;
                    case 2: {
                            float runSec = running ?
                                (float)(nowMs - s_runStartMs) / 1000.0f : 0.0f;
                            sprintf(buf, "T:%.1fs %s  ", runSec,
                                    running ? "GO" : "STOP");
                            OLED_ShowString(3, 1, buf);
                            } break;
                    case 3:
                        if (s_obsDist > 0.0f)
                            sprintf(buf, "US:%.1fcm      ", s_obsDist);
                        else
                            sprintf(buf, "US:---         ");
                        OLED_ShowString(4, 1, buf); break;
                    }
                    oledLine = (oledLine + 1) & 3;
                }
            }
        }

        /* running 时 1ms 节流至 ~1kHz */
        if (running) Delay_ms(1);
    }
}
