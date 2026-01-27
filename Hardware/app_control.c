#include "app_control.h"
#include "bsp_systick.h"
#include "bsp_led_pwm.h"
#include "bsp_buzzer.h"
#include "app_motor.h"
#include "app_ui.h"
#include <string.h>
#include <stdlib.h>

#define CONTROL_SAMPLE_MS 20U
#define CONTROL_HISTORY_SECONDS 6U
#define CONTROL_HISTORY_SIZE (CONTROL_HISTORY_SECONDS * 1000U / CONTROL_SAMPLE_MS)

#define CONTROL_VX_MAX 40
#define CONTROL_VX_MIN 6
#define CONTROL_VX_BOOST 45
#define CONTROL_VX_CURVE_DROP 15
#define CONTROL_VX_SCALE_PERCENT 20
#define CONTROL_VZ_SCALE_PERCENT 35

#define CONTROL_GYRO_STRAIGHT_THRESHOLD 90.0f
#define CONTROL_GYRO_TURN_THRESHOLD 120.0f
#define CONTROL_GYRO_SLOPE_THRESHOLD 18.0f

#define CONTROL_TRACK_LOST_COUNT 12U
#define CONTROL_LOST_IDLE_COUNT 200U
#define CONTROL_FINISH_COUNT 15U
#define CONTROL_STUCK_COUNT 15U

#define CONTROL_REPEAT_WINDOW 200U
#define CONTROL_REPEAT_MIN_MATCH 140U

#define CONTROL_STABLE_COUNT 8U

#define CONTROL_LIFTED_HOLD_MS 1500U
#define CONTROL_LIFTED_GYRO_THRESHOLD 40.0f

#define CONTROL_HANDHELD_DISTANCE_CM 20.0f
#define CONTROL_HANDHELD_VX_LIMIT 250

#define CONTROL_LIFTED_VZ_SLOW 150

#define CONTROL_ARM_COUNT 5U
#define CONTROL_PREDICT_K 0.60f

#define CONTROL_OBS_DISTANCE_CM 35.0f
#define CONTROL_OBS_EXIT_DISTANCE_CM 45.0f
#define CONTROL_OBS_SAMPLE_COUNT 7U
#define CONTROL_OBS_K 3U
#define CONTROL_OBS_VX_MAX 800
#define CONTROL_OBS_VX_MIN 200
#define CONTROL_OBS_PRE_VX_MAX 750
#define CONTROL_OBS_TURN 600
#define CONTROL_OBS_NEAR_CM 18.0f
#define CONTROL_OBS_GYRO_MAX 180.0f

#define CONTROL_OBS_ENTRY_TRACK_MAX 2U
#define CONTROL_OBS_EXIT_TRACK_COUNT 2U
#define CONTROL_OBS_EXIT_HOLD_COUNT 5U
#define CONTROL_OBS_INVALID_MAX 10U

#define CONTROL_OBS_BOOST_STEP 60

#define LINE_ERROR_MAX 14

#define PID_LINE_KP 45.0f
#define PID_LINE_KI 0.0f
#define PID_LINE_KD 12.0f
#define PID_LINE_I_LIMIT 120.0f
#define PID_LINE_OUT_LIMIT 800.0f

typedef struct {
    int8_t error;
    uint8_t trackCount;
    uint8_t cross;
    float gyroZ;
} ControlSample_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prevError;
} LinePid_t;

static ControlSample_t s_history[CONTROL_HISTORY_SIZE];
static uint16_t s_historyIndex = 0;
static uint8_t s_historyFilled = 0;

static LinePid_t s_linePid;
static uint32_t s_lastUpdateMs = 0;
static uint8_t s_lostCount = 0;
static uint8_t s_finishCount = 0;
static uint8_t s_stuckCount = 0;
static uint32_t s_lastTrackChangeMs = 0;
static uint32_t s_lastMotionMs = 0;
static int8_t s_lastError = 0;
static uint8_t s_lastTrackData = 0xFF;
static int8_t s_lostDirection = 1;
static uint8_t s_finishTriggered = 0;
static uint8_t s_referenceSet = 0;
static uint8_t s_refTrackData = 0xFF;
static float s_refAnglePitch = 0.0f;
static uint8_t s_stableCount = 0;
static uint8_t s_armedCount = 0;
static uint8_t s_motionArmed = 0;
static uint32_t s_liftedStartMs = 0;
static uint8_t s_obstacleMode = 0;
static uint8_t s_obstacleActive = 0;
static uint8_t s_obstacleExitCount = 0;
static int16_t s_obstacleBoostVx = 0;
static float s_obsSamples[CONTROL_OBS_SAMPLE_COUNT];
static uint8_t s_obsIndex = 0;
static uint8_t s_obsFilled = 0;
static uint8_t s_obsInvalidCount = 0;

static ControlState_t s_state = CONTROL_STATE_IDLE;

static void Control_ObstacleReset(void);
static void Control_ObstaclePush(float distanceCm);
static uint8_t Control_ObstacleDetect(void);
static uint8_t Control_ObstacleDetectExit(void);

static float clamp_f(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float clamp_abs_f(float value, float maxAbs)
{
    if (value > maxAbs) return maxAbs;
    if (value < -maxAbs) return -maxAbs;
    return value;
}

static int16_t clamp_i16(int16_t value, int16_t min, int16_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static void LinePid_Init(LinePid_t *pid)
{
    pid->kp = PID_LINE_KP;
    pid->ki = PID_LINE_KI;
    pid->kd = PID_LINE_KD;
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

static void LinePid_Reset(LinePid_t *pid)
{
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

static float LinePid_Update(LinePid_t *pid, float error, float dt)
{
    float derivative;
    float output;

    if (dt < 0.0005f) dt = 0.0005f;

    pid->integral += error * dt;
    pid->integral = clamp_f(pid->integral, -PID_LINE_I_LIMIT, PID_LINE_I_LIMIT);

    derivative = (error - pid->prevError) / dt;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->prevError = error;

    return clamp_f(output, -PID_LINE_OUT_LIMIT, PID_LINE_OUT_LIMIT);
}

static uint8_t Track_CountBits(uint8_t data)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & (1U << i)) count++;
    }
    return count;
}

static int8_t Track_CalcError(uint8_t data)
{
    static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int16_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & (1U << (7 - i)))
        {
            sum += weights[i];
            count++;
        }
    }

    if (count == 0)
    {
        return 0;
    }

    return (int8_t)clamp_i16((int16_t)(sum / (int16_t)count), -LINE_ERROR_MAX, LINE_ERROR_MAX);
}

static uint8_t Track_IsCross(uint8_t data, uint8_t count)
{
    if (count >= 6)
    {
        return 1;
    }
    if ((data == 0x3C) || (data == 0x7E))
    {
        return 1;
    }
    return 0;
}

static void ControlHistory_Push(int8_t error, uint8_t count, uint8_t cross, float gyroZ)
{
    s_history[s_historyIndex].error = error;
    s_history[s_historyIndex].trackCount = count;
    s_history[s_historyIndex].cross = cross;
    s_history[s_historyIndex].gyroZ = gyroZ;

    s_historyIndex++;
    if (s_historyIndex >= CONTROL_HISTORY_SIZE)
    {
        s_historyIndex = 0;
        s_historyFilled = 1;
    }
}

static void ControlHistory_Analyze(uint8_t *isStraight, uint8_t *isRepeat, uint8_t gyroValid)
{
    uint16_t total = s_historyFilled ? CONTROL_HISTORY_SIZE : s_historyIndex;
    float gyroSum = 0.0f;
    uint16_t crossCount = 0;
    uint16_t repeatMatches = 0;
    uint16_t repeatWindow = CONTROL_REPEAT_WINDOW;
    uint16_t lowErrorCount = 0;

    if (total == 0)
    {
        *isStraight = 0;
        *isRepeat = 0;
        return;
    }

    if (repeatWindow > total)
    {
        repeatWindow = total;
    }

    for (uint16_t i = 0; i < total; i++)
    {
        uint16_t idx = (s_historyIndex + CONTROL_HISTORY_SIZE - 1 - i) % CONTROL_HISTORY_SIZE;
        gyroSum += (s_history[idx].gyroZ >= 0.0f) ? s_history[idx].gyroZ : -s_history[idx].gyroZ;
        if (s_history[idx].cross)
        {
            crossCount++;
        }
        int8_t err = s_history[idx].error;
        if (err >= -1 && err <= 1)
        {
            lowErrorCount++;
        }
    }

    for (uint16_t i = 0; i < repeatWindow; i++)
    {
        uint16_t idxRecent = (s_historyIndex + CONTROL_HISTORY_SIZE - 1 - i) % CONTROL_HISTORY_SIZE;
        uint16_t idxPast = (s_historyIndex + CONTROL_HISTORY_SIZE - 1 - i - repeatWindow) % CONTROL_HISTORY_SIZE;
        if (s_history[idxRecent].error == s_history[idxPast].error && s_history[idxRecent].cross == s_history[idxPast].cross)
        {
            repeatMatches++;
        }
    }

    if (gyroValid)
    {
        *isStraight = (gyroSum < CONTROL_GYRO_STRAIGHT_THRESHOLD && crossCount == 0) ? 1 : 0;
    }
    else
    {
        *isStraight = (lowErrorCount > (total * 3 / 4) && crossCount == 0) ? 1 : 0;
    }
    *isRepeat = (repeatMatches >= CONTROL_REPEAT_MIN_MATCH) ? 1 : 0;
}

static uint8_t Control_DetectSlope(float gyroZ, float anglePitch, float speedLeft, float speedRight)
{
    float speed = (speedLeft + speedRight) * 0.5f;
    float pitchAbs = anglePitch >= 0.0f ? anglePitch : -anglePitch;
    float pitchDelta = anglePitch - s_refAnglePitch;

    if (speed < 0.0f) speed = -speed;

    if ((pitchAbs > CONTROL_GYRO_SLOPE_THRESHOLD) && speed < 30.0f)
    {
        return 1;
    }
    if ((pitchDelta > CONTROL_GYRO_SLOPE_THRESHOLD) || (pitchDelta < -CONTROL_GYRO_SLOPE_THRESHOLD))
    {
        return 1;
    }
    if ((gyroZ > CONTROL_GYRO_TURN_THRESHOLD) || (gyroZ < -CONTROL_GYRO_TURN_THRESHOLD))
    {
        return 1;
    }
    return 0;
}

static uint8_t Control_CheckStuck(float speedLeft, float speedRight, uint32_t nowMs)
{
    float speed = (speedLeft + speedRight) * 0.5f;
    if (speed < 0.0f) speed = -speed;

    if (speed < 10.0f)
    {
        if ((nowMs - s_lastMotionMs) > 300U)
        {
            s_stuckCount++;
            s_lastMotionMs = nowMs;
        }
    }
    else
    {
        s_lastMotionMs = nowMs;
        s_stuckCount = 0;
    }

    if (s_stuckCount > CONTROL_STUCK_COUNT)
    {
        s_stuckCount = CONTROL_STUCK_COUNT;
        return 1;
    }
    return 0;
}

static uint8_t Control_CheckFinish(uint8_t trackData, uint8_t count, float gyroZ, float speedLeft, float speedRight, uint8_t gyroValid)
{
    float speed = (speedLeft + speedRight) * 0.5f;
    if (speed < 0.0f) speed = -speed;

    uint8_t gyroOk = 1;
    if (gyroValid)
    {
        gyroOk = ((gyroZ < 3.0f && gyroZ > -3.0f)) ? 1 : 0;
    }

    if (count >= 6 && (trackData & 0x7EU) == 0x7EU && speed < 5.0f && gyroOk)
    {
        if (s_finishCount < 200) s_finishCount++;
    }
    else
    {
        s_finishCount = 0;
    }

    return (s_finishCount >= CONTROL_FINISH_COUNT) ? 1 : 0;
}

void Control_Init(void)
{
    memset(s_history, 0, sizeof(s_history));
    s_historyIndex = 0;
    s_historyFilled = 0;
    s_lastUpdateMs = 0;
    s_lostCount = 0;
    s_finishCount = 0;
    s_stuckCount = 0;
    s_lastTrackChangeMs = 0;
    s_lastMotionMs = 0;
    s_lastError = 0;
    s_lastTrackData = 0xFF;
    s_lostDirection = 1;
    s_finishTriggered = 0;
    s_referenceSet = 0;
    s_refTrackData = 0xFF;
    s_refAnglePitch = 0.0f;
    s_stableCount = 0;
    s_armedCount = 0;
    s_motionArmed = 0;
    s_liftedStartMs = 0;
    s_obstacleMode = 0;
    Control_ObstacleReset();
    s_state = CONTROL_STATE_IDLE;
    LinePid_Init(&s_linePid);
}

void Control_Reset(void)
{
    Control_Init();
}

void Control_SetObstacleMode(uint8_t enable)
{
    s_obstacleMode = enable ? 1 : 0;
    Control_ObstacleReset();
    s_obstacleExitCount = 0;
    s_obstacleBoostVx = 0;
}

uint8_t Control_GetObstacleMode(void)
{
    return s_obstacleMode;
}

uint8_t Control_IsObstacleActive(void)
{
    return s_obstacleActive;
}

ControlOutput_t Control_Update(uint8_t trackData, float gyroZ, float anglePitch, float distanceCm, float speedLeftMms, float speedRightMms, uint8_t gyroValid, uint32_t nowMs)
{
    ControlOutput_t output;
    uint32_t deltaMs;
    float dt;
    uint8_t count;
    int8_t error;
    uint8_t cross;
    uint8_t isStraight = 0;
    uint8_t isRepeat = 0;
    uint8_t isSlope = 0;
    uint8_t isLifted = 0;
    uint8_t obsDetect = 0;
    uint8_t obsPre = 0;
    int16_t vx;
    int16_t vz;

    memset(&output, 0, sizeof(output));

    if (s_lastUpdateMs == 0)
    {
        s_lastUpdateMs = nowMs;
    }

    deltaMs = nowMs - s_lastUpdateMs;
    if (deltaMs < CONTROL_SAMPLE_MS)
    {
        output.state = s_state;
        return output;
    }
    s_lastUpdateMs = nowMs;

    dt = (float)deltaMs / 1000.0f;
    trackData = (uint8_t)~trackData;
    count = Track_CountBits(trackData);
    error = Track_CalcError(trackData);
    cross = Track_IsCross(trackData, count);

    if (!s_motionArmed)
    {
        if (count > 0)
        {
            if (s_armedCount < 255) s_armedCount++;
        }
        else
        {
            s_armedCount = 0;
        }

        if (s_armedCount >= CONTROL_ARM_COUNT)
        {
            s_motionArmed = 1;
        }
    }

    if (!s_referenceSet && count > 0)
    {
        s_refTrackData = trackData;
        s_refAnglePitch = anglePitch;
        s_referenceSet = 1;
    }

    if (trackData != s_lastTrackData)
    {
        s_lastTrackChangeMs = nowMs;
        s_lastTrackData = trackData;
    }

    if (count == 0)
    {
        if (s_lostCount < 255) s_lostCount++;
    }
    else
    {
        s_lostCount = 0;
    }

    if (count > 0)
    {
        if (error > 0) s_lostDirection = 1;
        if (error < 0) s_lostDirection = -1;

        if (trackData == s_refTrackData)
        {
            if (s_stableCount < 200) s_stableCount++;
        }
        else
        {
            s_stableCount = 0;
        }
    }
    else
    {
        s_stableCount = 0;
    }

    if (!gyroValid)
    {
        gyroZ = 0.0f;
        anglePitch = 0.0f;
    }

    ControlHistory_Push(error, count, cross, gyroZ);
    ControlHistory_Analyze(&isStraight, &isRepeat, gyroValid);
    isSlope = gyroValid ? Control_DetectSlope(gyroZ, anglePitch, speedLeftMms, speedRightMms) : 0;

    s_obstacleActive = 0;
    if (s_obstacleMode)
    {
        Control_ObstaclePush(distanceCm);
        obsDetect = Control_ObstacleDetect();
        if (!s_obstacleActive && count > 0 && obsDetect)
        {
            obsPre = 1;
        }
    }

    /* Lifted detection: no track signal for a while with low gyro motion */
    if (!s_obstacleMode)
    {
        if (count == 0)
        {
            if (s_liftedStartMs == 0)
            {
                s_liftedStartMs = nowMs;
            }
        }
        else
        {
            s_liftedStartMs = 0;
        }

        if (s_liftedStartMs != 0 && (nowMs - s_liftedStartMs) >= CONTROL_LIFTED_HOLD_MS)
        {
            if (!gyroValid || (gyroZ > -CONTROL_LIFTED_GYRO_THRESHOLD && gyroZ < CONTROL_LIFTED_GYRO_THRESHOLD))
            {
                isLifted = 1;
            }
        }
    }

    if (!s_motionArmed)
    {
        s_state = CONTROL_STATE_IDLE;
    }
    else if (s_obstacleMode && gyroValid)
    {
        if (s_obstacleActive)
        {
            uint8_t exitOk = 0;

            if (count >= CONTROL_OBS_EXIT_TRACK_COUNT)
            {
                if (Control_ObstacleDetectExit())
                {
                    exitOk = 1;
                }
                else if (s_obsInvalidCount >= CONTROL_OBS_INVALID_MAX)
                {
                    /* Ultrasonic invalid for long: allow exit when track recovered. */
                    exitOk = 1;
                }
            }
            else if (count > 0 && obsDetect)
            {
                s_obstacleActive = 1;
                s_obstacleBoostVx = 0;
                s_obstacleExitCount = 0;
                s_state = CONTROL_STATE_OBSTACLE;
            }

            if (exitOk)
            {
                if (s_obstacleExitCount < 255) s_obstacleExitCount++;
            }
            else
            {
                s_obstacleExitCount = 0;
            }

            if (s_obstacleExitCount >= CONTROL_OBS_EXIT_HOLD_COUNT)
            {
                s_obstacleActive = 0;
                s_obstacleExitCount = 0;
            }
            else
            {
                s_state = CONTROL_STATE_OBSTACLE;
            }
        }

        if (!s_obstacleActive)
        {
            s_obstacleExitCount = 0;
            if (count <= CONTROL_OBS_ENTRY_TRACK_MAX && Control_ObstacleDetect())
            {
                s_state = CONTROL_STATE_OBSTACLE;
                s_obstacleActive = 1;
                s_obstacleBoostVx = 0;
                s_obstacleExitCount = 0;
            }
            else if (isLifted)
            {
                s_state = CONTROL_STATE_LIFTED;
            }
            else if (Control_CheckFinish(trackData, count, gyroZ, speedLeftMms, speedRightMms, gyroValid))
            {
                s_state = CONTROL_STATE_FINISH;
            }
            else if (Control_CheckStuck(speedLeftMms, speedRightMms, nowMs))
            {
                s_state = CONTROL_STATE_STUCK;
            }
            else if (s_lostCount > CONTROL_TRACK_LOST_COUNT)
            {
                s_state = CONTROL_STATE_LOST_LINE;
            }
            else
            {
                s_state = CONTROL_STATE_LINE_FOLLOW;
            }
        }
    }
    else if (isLifted)
    {
        s_state = CONTROL_STATE_LIFTED;
    }
    else if (Control_CheckFinish(trackData, count, gyroZ, speedLeftMms, speedRightMms, gyroValid))
    {
        s_state = CONTROL_STATE_FINISH;
    }
    else if (Control_CheckStuck(speedLeftMms, speedRightMms, nowMs))
    {
        s_state = CONTROL_STATE_STUCK;
    }
    else if (s_lostCount > CONTROL_TRACK_LOST_COUNT)
    {
        s_state = CONTROL_STATE_LOST_LINE;
    }
    else
    {
        s_state = CONTROL_STATE_LINE_FOLLOW;
    }

    if (s_state == CONTROL_STATE_FINISH && !s_finishTriggered)
    {
        s_finishTriggered = 1;
        Buzzer_BeepTriple();
        LED_StartFinishEffect();
        UI_SetFinishMode(1);
    }

    if (s_obstacleMode)
    {
        /* Base speed in obstacle mode: only limit while actually in obstacle region. */
        vx = CONTROL_VX_MAX;
        if (isStraight)
        {
            vx = CONTROL_VX_BOOST;
        }
        if (cross)
        {
            vx = CONTROL_VX_MAX - CONTROL_VX_CURVE_DROP;
        }
        if (isSlope)
        {
            vx = CONTROL_VX_MIN + 50;
        }

        if (s_obstacleActive)
        {
            if (vx > CONTROL_OBS_VX_MAX) vx = CONTROL_OBS_VX_MAX;
            if (vx < CONTROL_OBS_VX_MIN) vx = CONTROL_OBS_VX_MIN;
            s_obstacleBoostVx = 0;
        }
        else if (obsPre)
        {
            if (vx > CONTROL_OBS_PRE_VX_MAX) vx = CONTROL_OBS_PRE_VX_MAX;
            if (vx < CONTROL_OBS_VX_MIN) vx = CONTROL_OBS_VX_MIN;
        }
        else
        {
            /* After leaving obstacle region, ramp up speed smoothly. */
            if (s_obstacleBoostVx < CONTROL_VX_BOOST)
            {
                s_obstacleBoostVx += CONTROL_OBS_BOOST_STEP;
                if (s_obstacleBoostVx > CONTROL_VX_BOOST) s_obstacleBoostVx = CONTROL_VX_BOOST;
            }

            if (vx < s_obstacleBoostVx)
            {
                vx = s_obstacleBoostVx;
            }
        }
    }
    else
    {
        vx = CONTROL_VX_MAX;
        if (isStraight)
        {
            vx = CONTROL_VX_BOOST;
        }
        if (cross)
        {
            vx = CONTROL_VX_MAX - CONTROL_VX_CURVE_DROP;
        }
        if (isSlope)
        {
            vx = CONTROL_VX_MIN + 50;
        }

        if (vx < CONTROL_VX_MIN) vx = CONTROL_VX_MIN;
    }

    {
        int8_t ePrev = s_lastError;
        float ePred = (float)error + CONTROL_PREDICT_K * ((float)error - (float)ePrev);
        vz = (int16_t)LinePid_Update(&s_linePid, ePred, dt);
    }

    if (count > 0)
    {
        s_lastError = error;
    }

    if (s_state == CONTROL_STATE_OBSTACLE)
    {
        float gyroAdj = clamp_abs_f(gyroZ, CONTROL_OBS_GYRO_MAX) / CONTROL_OBS_GYRO_MAX;
        int16_t turn = (int16_t)(CONTROL_OBS_TURN * (1.0f - gyroAdj));

        if (distanceCm > 0.0f && distanceCm < CONTROL_OBS_NEAR_CM)
        {
            vx = CONTROL_OBS_VX_MIN;
        }
        else
        {
            vx = (vx > CONTROL_OBS_VX_MAX) ? CONTROL_OBS_VX_MAX : vx;
        }
        vz = (int16_t)(s_lostDirection * turn);
        LinePid_Reset(&s_linePid);
    }
    else if (s_state == CONTROL_STATE_LOST_LINE)
    {
        vx = CONTROL_VX_MIN;
        if (isRepeat)
        {
            vz = (int16_t)(s_lostDirection * 650);
        }
        else
        {
            vz = (int16_t)(s_lostDirection * 500);
        }
        LinePid_Reset(&s_linePid);

        if (s_lostCount > CONTROL_LOST_IDLE_COUNT)
        {
            vx = 0;
            vz = 0;
            s_state = CONTROL_STATE_STOP;
        }
    }
    else if (s_state == CONTROL_STATE_STUCK)
    {
        vx = 0;
        vz = 0;
    }
    else if (s_state == CONTROL_STATE_FINISH)
    {
        vx = 0;
        vz = 0;
    }
    else if (s_state == CONTROL_STATE_IDLE)
    {
        /* Not armed: keep safe */
        vx = 0;
        vz = 0;
        LinePid_Reset(&s_linePid);
    }
    else if (s_state == CONTROL_STATE_LIFTED)
    {
        /* Lifted off ground: slow rotation, no forward */
        vx = 0;
        vz = CONTROL_LIFTED_VZ_SLOW;
        LinePid_Reset(&s_linePid);
    }

    output.vx = clamp_i16(vx, 0, 1000);
    output.vz = clamp_i16(vz, -1000, 1000);

    output.vx = (int16_t)((output.vx * CONTROL_VX_SCALE_PERCENT) / 100);
    output.vz = (int16_t)((output.vz * CONTROL_VZ_SCALE_PERCENT) / 100);

    if (distanceCm >= 0.0f && distanceCm > CONTROL_HANDHELD_DISTANCE_CM)
    {
        if (output.vx > CONTROL_HANDHELD_VX_LIMIT) output.vx = CONTROL_HANDHELD_VX_LIMIT;
    }
    output.trackError = error;
    output.trackCount = count;
    output.trackLost = (count == 0) ? 1 : 0;
    output.isStraight = isStraight;
    output.isSlope = isSlope;
    output.isRepeat = isRepeat;
    output.isLifted = isLifted;
    output.state = s_state;

    return output;
}

static void Control_ObstacleReset(void)
{
    for (uint8_t i = 0; i < CONTROL_OBS_SAMPLE_COUNT; i++)
    {
        s_obsSamples[i] = -1.0f;
    }
    s_obsIndex = 0;
    s_obsFilled = 0;
    s_obsInvalidCount = 0;
    s_obstacleActive = 0;
}

static void Control_ObstaclePush(float distanceCm)
{
    if (distanceCm < 0.0f)
    {
        if (s_obsInvalidCount < 255) s_obsInvalidCount++;
        return;
    }

    s_obsInvalidCount = 0;
    s_obsSamples[s_obsIndex] = distanceCm;
    s_obsIndex++;
    if (s_obsIndex >= CONTROL_OBS_SAMPLE_COUNT)
    {
        s_obsIndex = 0;
        s_obsFilled = 1;
    }
}

static float Control_ObstacleGetTrimmedMean(uint8_t total)
{
    float samples[CONTROL_OBS_SAMPLE_COUNT];
    for (uint8_t i = 0; i < total; i++)
    {
        samples[i] = s_obsSamples[i];
    }

    for (uint8_t i = 0; i < total; i++)
    {
        for (uint8_t j = i + 1; j < total; j++)
        {
            if (samples[j] < samples[i])
            {
                float tmp = samples[i];
                samples[i] = samples[j];
                samples[j] = tmp;
            }
        }
    }

    /* trimmed mean: drop min/max if possible */
    uint8_t start = 0;
    uint8_t end = total;
    if (total >= 5)
    {
        start = 1;
        end = total - 1;
    }

    float sum = 0.0f;
    uint8_t cnt = 0;
    for (uint8_t i = start; i < end; i++)
    {
        sum += samples[i];
        cnt++;
    }

    if (cnt == 0)
    {
        return -1.0f;
    }

    return sum / (float)cnt;
}

static uint8_t Control_ObstacleDetect(void)
{
    uint8_t total = s_obsFilled ? CONTROL_OBS_SAMPLE_COUNT : s_obsIndex;
    if (total < CONTROL_OBS_K)
    {
        return 0;
    }

    float avg = Control_ObstacleGetTrimmedMean(total);
    if (avg > 0.0f && avg < CONTROL_OBS_DISTANCE_CM)
    {
        return 1;
    }
    return 0;
}

static uint8_t Control_ObstacleDetectExit(void)
{
    uint8_t total = s_obsFilled ? CONTROL_OBS_SAMPLE_COUNT : s_obsIndex;
    if (total < CONTROL_OBS_K)
    {
        return 0;
    }

    float avg = Control_ObstacleGetTrimmedMean(total);
    if (avg > CONTROL_OBS_EXIT_DISTANCE_CM)
    {
        return 1;
    }
    return 0;
}
