/**
 ******************************************************************************
 * @file    app_line_follow_mission.c
 * @brief   Field-ready line-following run: PG10 gate, gray/direct-JY901 observer,
 *          front-wheel LADRC plus four-wheel physical-forward actuation.
 *
 * Front encoders close the speed loop. The rear TB6612 receives the matching
 * signed raw command only; it is a four-wheel open-loop follower until its
 * TIM4/TIM8 encoder feedback is calibrated. The directly wired JY901 yaw is
 * the bounded yaw/rate feedback source; gray remains the primary path-error
 * source. UART4 radar pose parsing is retained as a separate future back-up
 * path and is intentionally not mixed into this field controller.
 ******************************************************************************
 */

#include "app_line_follow_mission.h"

#include "app_line_observer.h"
#include "bsp_aux_tb6612.h"
#include "bsp_diag_uart.h"
#include "bsp_encoder.h"
#include "bsp_four_wheel_direction.h"
#include "bsp_motor.h"
#include "bsp_gyro_wit.h"
#include "bsp_robot_uart.h"
#include "bsp_v22_protocol.h"
#include "speed_ladrc.h"

#define LINE_CONTROL_PERIOD_MS               20U
#define LINE_LOG_PERIOD_MS                  100U
#define LINE_BUTTON_DEBOUNCE_MS              50U
#define LINE_START_GYRO_WAIT_MS            5000U
#define LINE_LEAVE_A_TIMEOUT_MS            3000U
#define LINE_LOST_TIMEOUT_MS                650U
#define LINE_COMPLETE_MARK_MS               200U
#define LINE_RUN_WATCHDOG_MS              45000U
#define LINE_GYRO_RUN_STALE_MS              250U
#define LINE_ENCODER_LIVE_AFTER_MS         1000U
#define LINE_ENCODER_LIVE_CMD_PERCENT        35
#define LINE_ENCODER_LIVE_MIN_CPS            200

#ifndef LINE_BASE_SPEED_MM_S
#define LINE_BASE_SPEED_MM_S                 600L
#endif
#ifndef LINE_REQUIRE_GYRO_FOR_START
#define LINE_REQUIRE_GYRO_FOR_START              1
#endif

#if (LINE_REQUIRE_GYRO_FOR_START == 0)
#define LINE_LEAVE_A_SPEED_MM_S              300L
#define LINE_CURVE_SPEED_MM_S                300L
#define LINE_HARD_CURVE_SPEED_MM_S           220L
#define LINE_TIGHT_CURVE_SPEED_MM_S          180L
#define LINE_LOST_HOLD_SPEED_MM_S            180L
#define LINE_WIDE_SPEED_MM_S                 LINE_BASE_SPEED_MM_S
#else
#define LINE_LEAVE_A_SPEED_MM_S              400L
#define LINE_CURVE_SPEED_MM_S                500L
#define LINE_HARD_CURVE_SPEED_MM_S           420L
#define LINE_TIGHT_CURVE_SPEED_MM_S          280L
#define LINE_LOST_HOLD_SPEED_MM_S            360L
#define LINE_WIDE_SPEED_MM_S                 LINE_BASE_SPEED_MM_S
#endif

#define LINE_SPEED_B0                       800.0f
#define LINE_SPEED_WC                         8.0f
#define LINE_SPEED_WO                        40.0f
#define LINE_SPEED_OUT_MAX                   45.0f
#define LINE_SPEED_OUT_STEP                   5.0f
#define LINE_TARGET_MIN_CPS                  500
#define LINE_TARGET_MAX_CPS                 4200
#define LINE_LOST_CORNER_THRESHOLD_CPS       500
#define LINE_LOST_CORNER_DIFFERENTIAL_CPS   2200
#define LINE_VELOCITY_WINDOW_COUNT             4U
#define LINE_FROZEN_RECORD_COUNT             256U

#ifndef GYRO_BAUDRATE
#define GYRO_BAUDRATE 9600U
#endif
#ifndef LORA_UART_BAUDRATE
#define LORA_UART_BAUDRATE 115200U
#endif

typedef enum
{
    LINE_MISSION_IDLE = 0,
    LINE_MISSION_START_GATE,
    LINE_MISSION_LEAVE_A,
    LINE_MISSION_TRACK,
    LINE_MISSION_LOST_HOLD,
    LINE_MISSION_COMPLETE,
    LINE_MISSION_SAFE_STOP,
    LINE_MISSION_FAULT
} LineMissionState_t;

typedef struct
{
    int16_t delta[LINE_VELOCITY_WINDOW_COUNT];
    uint16_t periodMs[LINE_VELOCITY_WINDOW_COUNT];
    int32_t deltaSum;
    uint32_t periodSumMs;
    uint8_t writeIndex;
    uint8_t usedCount;
} LineVelocityWindow_t;

typedef struct
{
    uint32_t timeMs;
    int16_t errorX100;
    int16_t grayDiffCps;
    int16_t yawRefTenthsPerSec;
    int16_t yawTenthsDeg;
    int16_t yawRateTenthsPerSec;
    int16_t headingErrorTenthsDeg;
    int16_t gyroDiffCps;
    int16_t totalDiffCps;
    int16_t targetLeftCps;
    int16_t targetRightCps;
    int16_t measuredLeftCps;
    int16_t measuredRightCps;
    int16_t commandLeftPercent;
    int16_t commandRightPercent;
    uint8_t rawMask;
    uint8_t activeMask;
    uint8_t stableMask;
    uint8_t lineClass;
    uint8_t missionState;
    uint8_t gyroFresh;
} LineFrozenRecord_t;

static LineObserver_t s_observer;
static SpeedLadrc_t s_leftSpeed;
static SpeedLadrc_t s_rightSpeed;
static LineVelocityWindow_t s_leftVelocityWindow;
static LineVelocityWindow_t s_rightVelocityWindow;
static LineFrozenRecord_t s_frozenRecord[LINE_FROZEN_RECORD_COUNT];

static LineMissionState_t s_state;
static uint32_t s_lastControlMs;
static uint32_t s_lastLogMs;
static uint32_t s_stateStartMs;
static uint32_t s_runStartMs;
static uint32_t s_lostStartMs;
static uint32_t s_buttonCandidateSinceMs;
static uint8_t s_buttonStablePressed;
static uint8_t s_buttonCandidatePressed;
static uint8_t s_normalLineCount;
static uint8_t s_completeMarkCount;
static uint8_t s_radioSequence;
static uint16_t s_frozenWriteIndex;
static uint16_t s_frozenCount;
static uint8_t s_frozen;
static uint32_t s_lastButtonPressMs;
static const char *s_lastReason;
static int16_t s_holdDifferentialCps;
static int16_t s_lastTrackingDifferentialCps;
static int16_t s_targetLeftCps;
static int16_t s_targetRightCps;
static int16_t s_measuredLeftCps;
static int16_t s_measuredRightCps;
static int16_t s_commandLeftPercent;
static int16_t s_commandRightPercent;
static int16_t s_rearLeftRawPercent;
static int16_t s_rearRightRawPercent;

static int16_t LineMission_Clamp(int32_t value, int16_t lower, int16_t upper)
{
    if (value > upper)
    {
        return upper;
    }
    if (value < lower)
    {
        return lower;
    }
    return (int16_t)value;
}

static int32_t LineMission_Abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t LineMission_IsActive(void)
{
    return ((s_state == LINE_MISSION_LEAVE_A) ||
            (s_state == LINE_MISSION_TRACK) ||
            (s_state == LINE_MISSION_LOST_HOLD)) ? 1U : 0U;
}

static const char *LineMission_StateName(void)
{
    switch (s_state)
    {
        case LINE_MISSION_START_GATE: return "START_GATE";
        case LINE_MISSION_LEAVE_A:    return "LEAVE_A";
        case LINE_MISSION_TRACK:      return "TRACK";
        case LINE_MISSION_LOST_HOLD:  return "LOST_HOLD";
        case LINE_MISSION_COMPLETE:   return "COMPLETE";
        case LINE_MISSION_SAFE_STOP:  return "SAFE_STOP";
        case LINE_MISSION_FAULT:      return "FAULT";
        case LINE_MISSION_IDLE:
        default:                      return "IDLE";
    }
}

static void LineMission_VelocityWindowReset(LineVelocityWindow_t *window)
{
    uint8_t index;

    window->deltaSum = 0;
    window->periodSumMs = 0U;
    window->writeIndex = 0U;
    window->usedCount = 0U;
    for (index = 0U; index < LINE_VELOCITY_WINDOW_COUNT; ++index)
    {
        window->delta[index] = 0;
        window->periodMs[index] = 0U;
    }
}

static int16_t LineMission_VelocityWindowPush(LineVelocityWindow_t *window,
                                              int16_t delta,
                                              uint32_t periodMs)
{
    uint16_t storedPeriodMs;
    int32_t cps;

    if (periodMs == 0U)
    {
        return 0;
    }

    storedPeriodMs = (periodMs > 65535U) ? 65535U : (uint16_t)periodMs;
    if (window->usedCount >= LINE_VELOCITY_WINDOW_COUNT)
    {
        window->deltaSum -= window->delta[window->writeIndex];
        window->periodSumMs -= window->periodMs[window->writeIndex];
    }
    else
    {
        ++window->usedCount;
    }

    window->delta[window->writeIndex] = delta;
    window->periodMs[window->writeIndex] = storedPeriodMs;
    window->deltaSum += delta;
    window->periodSumMs += storedPeriodMs;
    ++window->writeIndex;
    if (window->writeIndex >= LINE_VELOCITY_WINDOW_COUNT)
    {
        window->writeIndex = 0U;
    }

    if (window->periodSumMs == 0U)
    {
        return 0;
    }

    cps = (window->deltaSum * 1000L) / (int32_t)window->periodSumMs;
    return LineMission_Clamp(cps, -32768, 32767);
}

static int16_t LineMission_MmToCps(int32_t speedMmPerSec)
{
    if (WHEEL_CIRCUMFERENCE_MM <= 0)
    {
        return 0;
    }
    return LineMission_Clamp((speedMmPerSec * ENCODER_4X_PPR) /
                             WHEEL_CIRCUMFERENCE_MM,
                             -32768, 32767);
}

static int16_t LineMission_RoundPercent(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

static void LineMission_MotorOff(void)
{
    s_commandLeftPercent = 0;
    s_commandRightPercent = 0;
    s_rearLeftRawPercent = 0;
    s_rearRightRawPercent = 0;
    Motor_SetSpeedBoth(0, 0);
    Motor_Enable(0U);
    AuxTb6612_StopAll();
}

static void LineMission_WriteEvent(const char *event,
                                   const char *reason,
                                   uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",state=");
    DiagUart_WriteString(LineMission_StateName());
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",mask=");
    DiagUart_WriteUInt32(s_observer.stableMask);
    DiagUart_WriteString(",gyro_fresh=");
    DiagUart_WriteUInt32(s_observer.gyroFresh);
    DiagUart_WriteString(",gyro_required=");
    DiagUart_WriteUInt32(LINE_REQUIRE_GYRO_FOR_START);
    DiagUart_WriteString(",yaw_tenths=");
    DiagUart_WriteInt32(s_observer.yawTenthsDeg);
    DiagUart_WriteString(",gyro_age_ms=");
    DiagUart_WriteUInt32(s_observer.gyroAgeMs);
    DiagUart_WriteString(",motors_enabled=");
    DiagUart_WriteUInt32(Motor_IsEnabled());
    DiagUart_WriteString(",rear_enabled=");
    DiagUart_WriteUInt32(AuxTb6612_IsEnabled());
    DiagUart_WriteString("\r\n");
}

static void LineMission_WriteStatus(uint32_t nowMs)
{
    const volatile GyroWitState_t *gyro = GyroWit_GetState();
    uint8_t rawPressed = (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == Bit_RESET) ?
                         1U : 0U;

    DiagUart_WriteString("LF,event=status,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",state=");
    DiagUart_WriteString(LineMission_StateName());
    DiagUart_WriteString(",last_reason=");
    DiagUart_WriteString((s_lastReason != 0) ? s_lastReason : "NONE");
    DiagUart_WriteString(",pg10_raw_pressed=");
    DiagUart_WriteUInt32(rawPressed);
    DiagUart_WriteString(",pg10_stable_pressed=");
    DiagUart_WriteUInt32(s_buttonStablePressed);
    DiagUart_WriteString(",last_button_ms=");
    DiagUart_WriteUInt32(s_lastButtonPressMs);
    DiagUart_WriteString(",stable_mask=");
    DiagUart_WriteUInt32(s_observer.stableMask);
    DiagUart_WriteString(",line_class=");
    DiagUart_WriteString(LineObserver_ClassName(s_observer.lineClass));
    DiagUart_WriteString(",gyro_fresh=");
    DiagUart_WriteUInt32(s_observer.gyroFresh);
    DiagUart_WriteString(",gyro_age_ms=");
    DiagUart_WriteUInt32(s_observer.gyroAgeMs);
    DiagUart_WriteString(",yaw_tenths=");
    DiagUart_WriteInt32(s_observer.yawTenthsDeg);
    DiagUart_WriteString(",yaw_rate_tenths_s=");
    DiagUart_WriteInt32(s_observer.yawRateTenthsPerSec);
    DiagUart_WriteString(",heading_err_tenths=");
    DiagUart_WriteInt32(s_observer.headingErrorTenthsDeg);
    DiagUart_WriteString(",gyro_angle_frames=");
    DiagUart_WriteUInt32(gyro->angleFrameCount);
    DiagUart_WriteString(",gyro_rate_frames=");
    DiagUart_WriteUInt32(gyro->angularVelocityFrameCount);
    DiagUart_WriteString(",gyro_raw_bytes=");
    DiagUart_WriteUInt32(gyro->rawByteCount);
    DiagUart_WriteString(",gyro_frame_heads=");
    DiagUart_WriteUInt32(gyro->frameHeadCount);
    DiagUart_WriteString(",gyro_checksum_errors=");
    DiagUart_WriteUInt32(gyro->checksumErrorCount);
    DiagUart_WriteString(",motors_enabled=");
    DiagUart_WriteUInt32(Motor_IsEnabled());
    DiagUart_WriteString(",rear_enabled=");
    DiagUart_WriteUInt32(AuxTb6612_IsEnabled());
    DiagUart_WriteString(",gyro_required=");
    DiagUart_WriteUInt32(LINE_REQUIRE_GYRO_FOR_START);
    DiagUart_WriteString("\r\n");
}

static void LineMission_Freeze(uint32_t nowMs, const char *reason)
{
    s_frozen = 1U;
    DiagUart_WriteString("LF,event=freeze,t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString(reason);
    DiagUart_WriteString(",records=");
    DiagUart_WriteUInt32(s_frozenCount);
    DiagUart_WriteString(",motors_enabled=");
    DiagUart_WriteUInt32(Motor_IsEnabled());
    DiagUart_WriteString("\r\n");
}

static void LineMission_EnterState(LineMissionState_t state,
                                   uint32_t nowMs,
                                   const char *reason)
{
    s_state = state;
    s_stateStartMs = nowMs;
    s_lastReason = reason;
    LineMission_WriteEvent("state", reason, nowMs);
}

static void LineMission_Stop(LineMissionState_t state,
                             uint32_t nowMs,
                             const char *reason)
{
    LineMission_MotorOff();
    LineMission_EnterState(state, nowMs, reason);
    LineMission_Freeze(nowMs, reason);
}

static void LineMission_Record(uint32_t nowMs)
{
    LineFrozenRecord_t *record = &s_frozenRecord[s_frozenWriteIndex];

    record->timeMs = nowMs;
    record->errorX100 = s_observer.grayErrorX100;
    record->grayDiffCps = s_observer.grayDifferentialCps;
    record->yawRefTenthsPerSec = s_observer.yawRateReferenceTenthsPerSec;
    record->yawTenthsDeg = s_observer.yawTenthsDeg;
    record->yawRateTenthsPerSec = s_observer.yawRateTenthsPerSec;
    record->headingErrorTenthsDeg = s_observer.headingErrorTenthsDeg;
    record->gyroDiffCps = s_observer.gyroDifferentialCps;
    record->totalDiffCps = s_observer.totalDifferentialCps;
    record->targetLeftCps = s_targetLeftCps;
    record->targetRightCps = s_targetRightCps;
    record->measuredLeftCps = s_measuredLeftCps;
    record->measuredRightCps = s_measuredRightCps;
    record->commandLeftPercent = s_commandLeftPercent;
    record->commandRightPercent = s_commandRightPercent;
    record->rawMask = s_observer.rawMask;
    record->activeMask = s_observer.activeMask;
    record->stableMask = s_observer.stableMask;
    record->lineClass = (uint8_t)s_observer.lineClass;
    record->missionState = (uint8_t)s_state;
    record->gyroFresh = s_observer.gyroFresh;

    ++s_frozenWriteIndex;
    if (s_frozenWriteIndex >= LINE_FROZEN_RECORD_COUNT)
    {
        s_frozenWriteIndex = 0U;
    }
    if (s_frozenCount < LINE_FROZEN_RECORD_COUNT)
    {
        ++s_frozenCount;
    }
}

static void LineMission_WriteRecord(const char *event, uint32_t nowMs)
{
    DiagUart_WriteString("LF,event=");
    DiagUart_WriteString(event);
    DiagUart_WriteString(",t_ms=");
    DiagUart_WriteUInt32(nowMs);
    DiagUart_WriteString(",state=");
    DiagUart_WriteString(LineMission_StateName());
    DiagUart_WriteString(",line_class=");
    DiagUart_WriteString(LineObserver_ClassName(s_observer.lineClass));
    DiagUart_WriteString(",raw_mask=");
    DiagUart_WriteUInt32(s_observer.rawMask);
    DiagUart_WriteString(",active_mask=");
    DiagUart_WriteUInt32(s_observer.activeMask);
    DiagUart_WriteString(",stable_mask=");
    DiagUart_WriteUInt32(s_observer.stableMask);
    DiagUart_WriteString(",err_x100=");
    DiagUart_WriteInt32(s_observer.grayErrorX100);
    DiagUart_WriteString(",gray_diff_cps=");
    DiagUart_WriteInt32(s_observer.grayDifferentialCps);
    DiagUart_WriteString(",yaw_ref_tenths_s=");
    DiagUart_WriteInt32(s_observer.yawRateReferenceTenthsPerSec);
    DiagUart_WriteString(",yaw_tenths=");
    DiagUart_WriteInt32(s_observer.yawTenthsDeg);
    DiagUart_WriteString(",yaw_rate_tenths_s=");
    DiagUart_WriteInt32(s_observer.yawRateTenthsPerSec);
    DiagUart_WriteString(",heading_err_tenths=");
    DiagUart_WriteInt32(s_observer.headingErrorTenthsDeg);
    DiagUart_WriteString(",gyro_diff_cps=");
    DiagUart_WriteInt32(s_observer.gyroDifferentialCps);
    DiagUart_WriteString(",total_diff_cps=");
    DiagUart_WriteInt32(s_observer.totalDifferentialCps);
    DiagUart_WriteString(",target_l_cps=");
    DiagUart_WriteInt32(s_targetLeftCps);
    DiagUart_WriteString(",target_r_cps=");
    DiagUart_WriteInt32(s_targetRightCps);
    DiagUart_WriteString(",meas_l_cps=");
    DiagUart_WriteInt32(s_measuredLeftCps);
    DiagUart_WriteString(",meas_r_cps=");
    DiagUart_WriteInt32(s_measuredRightCps);
    DiagUart_WriteString(",cmd_l_pct=");
    DiagUart_WriteInt32(s_commandLeftPercent);
    DiagUart_WriteString(",cmd_r_pct=");
    DiagUart_WriteInt32(s_commandRightPercent);
    DiagUart_WriteString(",rear_raw_l_pct=");
    DiagUart_WriteInt32(s_rearLeftRawPercent);
    DiagUart_WriteString(",rear_raw_r_pct=");
    DiagUart_WriteInt32(s_rearRightRawPercent);
    DiagUart_WriteString(",gyro_age_ms=");
    DiagUart_WriteUInt32(s_observer.gyroAgeMs);
    DiagUart_WriteString(",gyro_fresh=");
    DiagUart_WriteUInt32(s_observer.gyroFresh);
    DiagUart_WriteString("\r\n");
}

static void LineMission_DumpFrozenLog(void)
{
    uint16_t index;
    uint16_t count;
    uint16_t cursor;
    const LineFrozenRecord_t *record;

    if (LineMission_IsActive() != 0U)
    {
        DiagUart_WriteString("LF,event=dump_denied,reason=RUNNING\r\n");
        return;
    }

    DiagUart_WriteString("LF,event=dump_begin,frozen=");
    DiagUart_WriteUInt32(s_frozen);
    DiagUart_WriteString(",reason=");
    DiagUart_WriteString((s_lastReason != 0) ? s_lastReason : "NONE");
    DiagUart_WriteString(",records=");
    DiagUart_WriteUInt32(s_frozenCount);
    DiagUart_WriteString("\r\n");

    count = s_frozenCount;
    cursor = (count < LINE_FROZEN_RECORD_COUNT) ? 0U : s_frozenWriteIndex;
    for (index = 0U; index < count; ++index)
    {
        record = &s_frozenRecord[cursor];
        DiagUart_WriteString("LF,rec,t_ms,");
        DiagUart_WriteUInt32(record->timeMs);
        DiagUart_WriteString(",state,");
        DiagUart_WriteUInt32(record->missionState);
        DiagUart_WriteString(",class,");
        DiagUart_WriteUInt32(record->lineClass);
        DiagUart_WriteString(",raw,");
        DiagUart_WriteUInt32(record->rawMask);
        DiagUart_WriteString(",active,");
        DiagUart_WriteUInt32(record->activeMask);
        DiagUart_WriteString(",stable,");
        DiagUart_WriteUInt32(record->stableMask);
        DiagUart_WriteString(",err_x100,");
        DiagUart_WriteInt32(record->errorX100);
        DiagUart_WriteString(",gray_diff,");
        DiagUart_WriteInt32(record->grayDiffCps);
        DiagUart_WriteString(",yaw_ref,");
        DiagUart_WriteInt32(record->yawRefTenthsPerSec);
        DiagUart_WriteString(",yaw,");
        DiagUart_WriteInt32(record->yawTenthsDeg);
        DiagUart_WriteString(",yaw_rate,");
        DiagUart_WriteInt32(record->yawRateTenthsPerSec);
        DiagUart_WriteString(",heading_err,");
        DiagUart_WriteInt32(record->headingErrorTenthsDeg);
        DiagUart_WriteString(",gyro_diff,");
        DiagUart_WriteInt32(record->gyroDiffCps);
        DiagUart_WriteString(",total_diff,");
        DiagUart_WriteInt32(record->totalDiffCps);
        DiagUart_WriteString(",target_l,");
        DiagUart_WriteInt32(record->targetLeftCps);
        DiagUart_WriteString(",target_r,");
        DiagUart_WriteInt32(record->targetRightCps);
        DiagUart_WriteString(",meas_l,");
        DiagUart_WriteInt32(record->measuredLeftCps);
        DiagUart_WriteString(",meas_r,");
        DiagUart_WriteInt32(record->measuredRightCps);
        DiagUart_WriteString(",cmd_l,");
        DiagUart_WriteInt32(record->commandLeftPercent);
        DiagUart_WriteString(",cmd_r,");
        DiagUart_WriteInt32(record->commandRightPercent);
        DiagUart_WriteString(",gyro_fresh,");
        DiagUart_WriteUInt32(record->gyroFresh);
        DiagUart_WriteString("\r\n");

        ++cursor;
        if (cursor >= LINE_FROZEN_RECORD_COUNT)
        {
            cursor = 0U;
        }
    }
    DiagUart_WriteString("LF,event=dump_end\r\n");
}

static void LineMission_SendStartPlaceholder(uint32_t nowMs)
{
    V22Frame_t frame;
    uint8_t encoded[V22_MAX_FRAME_BYTES];
    uint16_t length;

    /*
     * This is a deliberately non-authoritative link proof: one V2.2
     * HEARTBEAT.  It cannot start the aircraft.  Replace only after the
     * selected TaskType, MissionId and non-zero CalibrationId are available
     * for the required 0x81 CAR_TASK_REQUEST payload.
     */
    frame.version = V22_VERSION;
    frame.type = V22_TYPE_HEARTBEAT;
    frame.source = V22_ADDR_CAR_RADIO;
    frame.destination = V22_ADDR_BROADCAST;
    frame.sequence = s_radioSequence++;
    frame.flags = 0U;
    frame.length = 0U;
    length = V22Protocol_Encode(encoded, sizeof(encoded), &frame);
    if (length != 0U)
    {
        RobotUart_NanoWriteBuffer(encoded, length);
        DiagUart_WriteString("LF,event=radio_start_placeholder,t_ms=");
        DiagUart_WriteUInt32(nowMs);
        DiagUart_WriteString(",type=3,seq=");
        DiagUart_WriteUInt32(frame.sequence);
        DiagUart_WriteString(",bytes=");
        DiagUart_WriteUInt32(length);
        DiagUart_WriteString(",not_task_request=1\r\n");
    }
    else
    {
        DiagUart_WriteString("LF,event=radio_start_placeholder_fail,t_ms=");
        DiagUart_WriteUInt32(nowMs);
        DiagUart_WriteString(",reason=ENCODE\r\n");
    }
}

static void LineMission_ButtonInit(uint32_t nowMs)
{
    GPIO_InitTypeDef gpio;
    uint8_t pressed;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10; /* KEY2: PG10, active low. */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOG, &gpio);

    pressed = (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == Bit_RESET) ? 1U : 0U;
    s_buttonStablePressed = pressed;
    s_buttonCandidatePressed = pressed;
    s_buttonCandidateSinceMs = nowMs;
}

static uint8_t LineMission_ButtonPressedEvent(uint32_t nowMs)
{
    uint8_t rawPressed = (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_10) == Bit_RESET) ?
                         1U : 0U;

    if (rawPressed != s_buttonCandidatePressed)
    {
        s_buttonCandidatePressed = rawPressed;
        s_buttonCandidateSinceMs = nowMs;
    }

    if ((s_buttonCandidatePressed != s_buttonStablePressed) &&
        ((uint32_t)(nowMs - s_buttonCandidateSinceMs) >= LINE_BUTTON_DEBOUNCE_MS))
    {
        s_buttonStablePressed = s_buttonCandidatePressed;
        if (s_buttonStablePressed != 0U)
        {
            s_lastButtonPressMs = nowMs;
            return 1U;
        }
    }
    return 0U;
}

static void LineMission_ResetRunControllers(void)
{
    Encoder_Reset();
    LineMission_VelocityWindowReset(&s_leftVelocityWindow);
    LineMission_VelocityWindowReset(&s_rightVelocityWindow);
    s_measuredLeftCps = 0;
    s_measuredRightCps = 0;
    s_targetLeftCps = 0;
    s_targetRightCps = 0;
    s_commandLeftPercent = 0;
    s_commandRightPercent = 0;
    SpeedLadrc_Reset(&s_leftSpeed, 0.0f);
    SpeedLadrc_Reset(&s_rightSpeed, 0.0f);
    s_holdDifferentialCps = 0;
    s_lastTrackingDifferentialCps = 0;
    s_normalLineCount = 0U;
    s_completeMarkCount = 0U;
    s_frozenWriteIndex = 0U;
    s_frozenCount = 0U;
    s_frozen = 0U;
}

static int16_t LineMission_SelectBaseCps(void)
{
    int32_t absError = LineMission_Abs(s_observer.grayErrorX100);

    if (s_state == LINE_MISSION_LEAVE_A)
    {
        return LineMission_MmToCps(LINE_LEAVE_A_SPEED_MM_S);
    }
    if (s_state == LINE_MISSION_LOST_HOLD)
    {
        return LineMission_MmToCps(LINE_LOST_HOLD_SPEED_MM_S);
    }
    if (s_observer.lineClass == LINE_OBSERVER_WIDE)
    {
        return LineMission_MmToCps(LINE_WIDE_SPEED_MM_S);
    }
    if (absError >= 500L)
    {
        return LineMission_MmToCps(LINE_TIGHT_CURVE_SPEED_MM_S);
    }
    if (absError > 300L)
    {
        return LineMission_MmToCps(LINE_HARD_CURVE_SPEED_MM_S);
    }
    if (absError > 100L)
    {
        return LineMission_MmToCps(LINE_CURVE_SPEED_MM_S);
    }
    return LineMission_MmToCps(LINE_BASE_SPEED_MM_S);
}

static void LineMission_ApplySpeedControl(uint32_t samplePeriodMs)
{
    int16_t baseCps;
    int16_t differentialCps;
    float samplePeriodSeconds = (float)samplePeriodMs / 1000.0f;

    baseCps = LineMission_SelectBaseCps();
    differentialCps = (s_state == LINE_MISSION_LOST_HOLD) ?
                      s_holdDifferentialCps : s_observer.totalDifferentialCps;
    s_targetLeftCps = LineMission_Clamp((int32_t)baseCps -
                                        ((int32_t)differentialCps / 2L),
                                        LINE_TARGET_MIN_CPS, LINE_TARGET_MAX_CPS);
    s_targetRightCps = LineMission_Clamp((int32_t)baseCps +
                                         ((int32_t)differentialCps / 2L),
                                         LINE_TARGET_MIN_CPS, LINE_TARGET_MAX_CPS);

    s_commandLeftPercent = LineMission_RoundPercent(SpeedLadrc_Update(
        &s_leftSpeed, (float)s_targetLeftCps, (float)s_measuredLeftCps,
        samplePeriodSeconds));
    s_commandRightPercent = LineMission_RoundPercent(SpeedLadrc_Update(
        &s_rightSpeed, (float)s_targetRightCps, (float)s_measuredRightCps,
        samplePeriodSeconds));

    if (s_commandLeftPercent < 0)
    {
        s_commandLeftPercent = 0;
    }
    if (s_commandRightPercent < 0)
    {
        s_commandRightPercent = 0;
    }

    /*
     * Apply the physical signs measured by FourWheelTb6612Debug. The LADRC
     * command remains positive for logical vehicle-forward motion; writing it
     * directly to the front TB6612 was the regression that made the car run
     * backward. Rear motors are followers, not part of the speed feedback.
     */
    s_rearLeftRawPercent = (int16_t)(s_commandLeftPercent *
                                     REAR_LEFT_FORWARD_SIGN);
    s_rearRightRawPercent = (int16_t)(s_commandRightPercent *
                                      REAR_RIGHT_FORWARD_SIGN);

    Motor_SetSpeed(MOTOR_LEFT, (int16_t)(s_commandLeftPercent *
                                         FRONT_LEFT_FORWARD_SIGN));
    Motor_SetSpeed(MOTOR_RIGHT, (int16_t)(s_commandRightPercent *
                                          FRONT_RIGHT_FORWARD_SIGN));
    AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_A, s_rearLeftRawPercent);
    AuxTb6612_SetRawSpeed(AUX_TB6612_MOTOR_B, s_rearRightRawPercent);
    Motor_Enable(1U);
    AuxTb6612_Enable(1U);
}

static void LineMission_HandleButton(uint32_t nowMs)
{
    if (LineMission_ButtonPressedEvent(nowMs) == 0U)
    {
        return;
    }

    LineMission_WriteEvent("button_press", "PG10_EDGE", nowMs);

    if (LineMission_IsActive() != 0U)
    {
        LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs, "PG10_MANUAL_STOP");
        return;
    }

    if (s_observer.lineClass != LINE_OBSERVER_A_MARK)
    {
        LineMission_WriteEvent("start_rejected", "REQUIRE_A_FULL_BLACK", nowMs);
        return;
    }
    LineMission_EnterState(LINE_MISSION_START_GATE, nowMs, "PG10_START_REQUEST");
}

static void LineMission_UpdateRunState(uint32_t nowMs)
{
    if (s_state == LINE_MISSION_START_GATE)
    {
        if ((LINE_REQUIRE_GYRO_FOR_START == 0) ||
            (s_observer.gyroFresh != 0U))
        {
            LineMission_ResetRunControllers();
            LineObserver_ResetHeadingReference(&s_observer);
            s_runStartMs = nowMs;
            LineMission_SendStartPlaceholder(nowMs);
            LineMission_EnterState(LINE_MISSION_LEAVE_A, nowMs, "START_OK");
        }
        else if ((uint32_t)(nowMs - s_stateStartMs) >= LINE_START_GYRO_WAIT_MS)
        {
            LineMission_EnterState(LINE_MISSION_IDLE, nowMs, "GYRO_NOT_FRESH");
        }
        return;
    }

    if (LineMission_IsActive() == 0U)
    {
        return;
    }

    if ((LINE_REQUIRE_GYRO_FOR_START != 0) &&
        ((s_observer.gyroFresh == 0U) ||
         (s_observer.gyroAgeMs > LINE_GYRO_RUN_STALE_MS)))
    {
        LineMission_Stop(LINE_MISSION_FAULT, nowMs, "GYRO_STALE");
        return;
    }
    if ((uint32_t)(nowMs - s_runStartMs) >= LINE_RUN_WATCHDOG_MS)
    {
        LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs, "RUN_WATCHDOG");
        return;
    }

    if ((s_state == LINE_MISSION_TRACK) &&
        (s_observer.lineClass == LINE_OBSERVER_TRACK))
    {
        s_lastTrackingDifferentialCps = s_observer.totalDifferentialCps;
    }

    if (s_state == LINE_MISSION_LEAVE_A)
    {
        if (s_observer.lineClass == LINE_OBSERVER_TRACK)
        {
            ++s_normalLineCount;
            if (s_normalLineCount >= 3U)
            {
                LineMission_EnterState(LINE_MISSION_TRACK, nowMs, "LEFT_A_MARK");
            }
        }
        else
        {
            s_normalLineCount = 0U;
        }
        if ((uint32_t)(nowMs - s_stateStartMs) >= LINE_LEAVE_A_TIMEOUT_MS)
        {
            LineMission_Stop(LINE_MISSION_FAULT, nowMs, "A_EXIT_TIMEOUT");
        }
        return;
    }

    if (s_state == LINE_MISSION_LOST_HOLD)
    {
        if (s_observer.lineClass == LINE_OBSERVER_TRACK)
        {
            LineMission_EnterState(LINE_MISSION_TRACK, nowMs, "LINE_REACQUIRED");
            return;
        }
        if ((uint32_t)(nowMs - s_lostStartMs) >= LINE_LOST_TIMEOUT_MS)
        {
            LineMission_Stop(LINE_MISSION_FAULT, nowMs, "LINE_LOST_TIMEOUT");
        }
        return;
    }

    if (s_observer.lineClass == LINE_OBSERVER_LOST)
    {
        /* The observer clears its error on a white mask; retain the last
         * measured turn rather than blindly driving straight off a corner. */
        s_holdDifferentialCps = s_lastTrackingDifferentialCps;
        if (LineMission_Abs(s_holdDifferentialCps) >=
            LINE_LOST_CORNER_THRESHOLD_CPS)
        {
            s_holdDifferentialCps = (s_holdDifferentialCps < 0) ?
                                      -LINE_LOST_CORNER_DIFFERENTIAL_CPS :
                                      LINE_LOST_CORNER_DIFFERENTIAL_CPS;
        }
        s_lostStartMs = nowMs;
        LineMission_EnterState(LINE_MISSION_LOST_HOLD, nowMs, "LINE_LOST");
        return;
    }

    if (s_observer.lineClass == LINE_OBSERVER_A_MARK)
    {
        ++s_completeMarkCount;
        if ((uint32_t)s_completeMarkCount * LINE_CONTROL_PERIOD_MS >=
            LINE_COMPLETE_MARK_MS)
        {
            LineMission_Stop(LINE_MISSION_COMPLETE, nowMs, "A_MARK_RETURN");
        }
    }
    else
    {
        s_completeMarkCount = 0U;
    }
}

void LineFollowMission_Init(uint32_t nowMs)
{
    Motor_Init();
    AuxTb6612_Init();
    Encoder_Init();
    GyroWit_Init(GYRO_BAUDRATE);
    RobotUart_NanoInit(LORA_UART_BAUDRATE);
    LineObserver_Init(&s_observer, nowMs);
    LineMission_ButtonInit(nowMs);
    SpeedLadrc_Init(&s_leftSpeed, LINE_SPEED_B0, LINE_SPEED_WC, LINE_SPEED_WO,
                    0.0f, LINE_SPEED_OUT_MAX, LINE_SPEED_OUT_STEP);
    SpeedLadrc_Init(&s_rightSpeed, LINE_SPEED_B0, LINE_SPEED_WC, LINE_SPEED_WO,
                    0.0f, LINE_SPEED_OUT_MAX, LINE_SPEED_OUT_STEP);
    LineMission_VelocityWindowReset(&s_leftVelocityWindow);
    LineMission_VelocityWindowReset(&s_rightVelocityWindow);
    s_state = LINE_MISSION_IDLE;
    s_lastControlMs = nowMs;
    s_lastLogMs = nowMs;
    s_stateStartMs = nowMs;
    s_runStartMs = nowMs;
    s_lostStartMs = nowMs;
    s_normalLineCount = 0U;
    s_completeMarkCount = 0U;
    s_radioSequence = 0U;
    s_frozenWriteIndex = 0U;
    s_frozenCount = 0U;
    s_frozen = 0U;
    s_lastButtonPressMs = 0U;
    s_lastReason = "BOOT";
    s_holdDifferentialCps = 0;
    s_targetLeftCps = 0;
    s_targetRightCps = 0;
    s_measuredLeftCps = 0;
    s_measuredRightCps = 0;
    s_commandLeftPercent = 0;
    s_commandRightPercent = 0;
    s_rearLeftRawPercent = 0;
    s_rearRightRawPercent = 0;
    LineMission_MotorOff();

    DiagUart_WriteString("LF,boot,mode=COMPETITION_LINE_FOLLOW,start_key=PG10_active_low,");
    DiagUart_WriteString("gray=pc0_pc1_pc2_pg0,white_raw=0,center_mask=24,");
    DiagUart_WriteString("front=pa2_pa3_pe2_pe6,rear=pe13_pe14_pf1_pf4_pb9,enc_front=tim5_tim3,rear=open_loop_follower,");
    DiagUart_WriteString("forward_sign=fl-1_fr-1_rl+1_rr-1,gyro=usart2_remap_pd5_tx_pd6_rx_9600,radar=uart4_backup_not_control,");
    DiagUart_WriteString("radio=uart5_pc12_pd2,base_mm_s=");
    DiagUart_WriteInt32(LINE_BASE_SPEED_MM_S);
    DiagUart_WriteString(",base_cps=");
    DiagUart_WriteInt32(LineMission_MmToCps(LINE_BASE_SPEED_MM_S));
    DiagUart_WriteString(",gyro_required=");
    DiagUart_WriteUInt32(LINE_REQUIRE_GYRO_FOR_START);
    DiagUart_WriteString(",task_request=disabled,placeholder=heartbeat\r\n");
    DiagUart_WriteString("LF,commands,P=status,F=dump_frozen,S=manual_stop,H=help; start_only=PG10\r\n");
}

void LineFollowMission_Update(uint32_t nowMs)
{
    uint32_t samplePeriodMs;

    LineMission_HandleButton(nowMs);

    samplePeriodMs = nowMs - s_lastControlMs;
    if (samplePeriodMs < LINE_CONTROL_PERIOD_MS)
    {
        return;
    }
    s_lastControlMs = nowMs;

    Encoder_Update(samplePeriodMs);
    s_measuredLeftCps = LineMission_VelocityWindowPush(&s_leftVelocityWindow,
                                                        g_encoderL.count,
                                                        samplePeriodMs);
    s_measuredRightCps = LineMission_VelocityWindowPush(&s_rightVelocityWindow,
                                                         g_encoderR.count,
                                                         samplePeriodMs);
    LineObserver_Update(&s_observer, nowMs);
    LineMission_UpdateRunState(nowMs);

    if (LineMission_IsActive() != 0U)
    {
        LineMission_ApplySpeedControl(samplePeriodMs);
        if (((uint32_t)(nowMs - s_runStartMs) >= LINE_ENCODER_LIVE_AFTER_MS) &&
            (((s_commandLeftPercent >= LINE_ENCODER_LIVE_CMD_PERCENT) &&
              (LineMission_Abs(s_measuredLeftCps) < LINE_ENCODER_LIVE_MIN_CPS)) ||
             ((s_commandRightPercent >= LINE_ENCODER_LIVE_CMD_PERCENT) &&
              (LineMission_Abs(s_measuredRightCps) < LINE_ENCODER_LIVE_MIN_CPS))))
        {
            LineMission_Stop(LINE_MISSION_FAULT, nowMs, "ENCODER_NOT_LIVE");
        }
    }
    else
    {
        LineMission_MotorOff();
    }

    if (((uint32_t)(nowMs - s_lastLogMs) >= LINE_LOG_PERIOD_MS) &&
        (LineMission_IsActive() != 0U))
    {
        s_lastLogMs = nowMs;
        LineMission_Record(nowMs);
        LineMission_WriteRecord("sample", nowMs);
    }
}

void LineFollowMission_HandleCommand(char command, uint32_t nowMs)
{
    switch (command)
    {
        case 'F':
        case 'f':
            LineMission_DumpFrozenLog();
            break;
        case 'S':
        case 's':
            if (LineMission_IsActive() != 0U)
            {
                LineMission_Stop(LINE_MISSION_SAFE_STOP, nowMs, "UART_MANUAL_STOP");
            }
            else
            {
                LineMission_WriteEvent("stop_ignored", "NOT_RUNNING", nowMs);
            }
            break;
        case 'P':
        case 'p':
            LineMission_WriteStatus(nowMs);
            break;
        case 'H':
        case 'h':
        case '?':
            DiagUart_WriteString("LF,commands,P=status,F=dump_frozen,S=manual_stop,H=help; start_only=PG10\r\n");
            break;
        default:
            break;
    }
}
