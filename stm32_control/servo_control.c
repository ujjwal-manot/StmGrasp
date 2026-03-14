/**
 * Servo/Gripper Control Implementation
 * TIM3 4-channel PWM with trapezoidal velocity profiling.
 *
 * Timer config: 275 MHz APB1 / (prescaler+1) = 1 MHz tick.
 * ARR = 19999 => 50 Hz period (20 ms).
 * CCR values in microseconds: 500 (0 deg) to 2400 (180 deg).
 */

#include "servo_control.h"
#include "safety.h"

/* Map channel index to HAL channel constant */
static const uint32_t s_timChannels[SERVO_COUNT] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4,
};

/* Total travel distance recorded at move start, per channel (for trapezoidal ramp-up) */
static float s_totalTravel[SERVO_COUNT];

/* -------------------------------------------------------------------------- */
/*  Init                                                                       */
/* -------------------------------------------------------------------------- */

void initServos(void)
{
    osMutexAcquire(servoMutex, osWaitForever);

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        g_servoState.current_angle[i]   = 90.0f; /* Center position */
        g_servoState.target_angle[i]    = 90.0f;
        g_servoState.speed_ms_per_deg[i] = SERVO_DEFAULT_SPEED;
        g_servoState.moving[i]          = false;
    }

    osMutexRelease(servoMutex);

    /* Start PWM on all 4 channels at center position */
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        uint32_t ccr = angleToCCR(90.0f);
        __HAL_TIM_SET_COMPARE(&htim3, s_timChannels[i], ccr);
        HAL_TIM_PWM_Start(&htim3, s_timChannels[i]);
    }

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        s_totalTravel[i] = 0.0f;
    }
}

/* -------------------------------------------------------------------------- */
/*  Angle to CCR conversion                                                    */
/* -------------------------------------------------------------------------- */

uint32_t angleToCCR(float angle)
{
    if (angle < (float)SERVO_ANGLE_MIN) angle = (float)SERVO_ANGLE_MIN;
    if (angle > (float)SERVO_ANGLE_MAX) angle = (float)SERVO_ANGLE_MAX;

    /* Linear interpolation: 0 deg => 500 us, 180 deg => 2400 us */
    float pulse_us = (float)SERVO_PULSE_MIN_US +
                     (angle * (float)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)) / (float)SERVO_ANGLE_MAX;

    /* Timer tick = 1 MHz, so CCR value = pulse width in microseconds directly */
    return (uint32_t)(pulse_us + 0.5f);
}

/* -------------------------------------------------------------------------- */
/*  Immediate angle set (no interpolation)                                     */
/* -------------------------------------------------------------------------- */

void setServoAngle(uint8_t channel, float angle)
{
    if (channel >= SERVO_COUNT) return;
    if (angle < (float)SERVO_ANGLE_MIN) angle = (float)SERVO_ANGLE_MIN;
    if (angle > (float)SERVO_ANGLE_MAX) angle = (float)SERVO_ANGLE_MAX;

    uint32_t ccr = angleToCCR(angle);
    __HAL_TIM_SET_COMPARE(&htim3, s_timChannels[channel], ccr);

    osMutexAcquire(servoMutex, osWaitForever);
    g_servoState.current_angle[channel] = angle;
    g_servoState.target_angle[channel]  = angle;
    g_servoState.moving[channel]        = false;
    osMutexRelease(servoMutex);
}

/* -------------------------------------------------------------------------- */
/*  Smooth move with trapezoidal velocity profile                              */
/*  Enqueues a request; ServoTask processes it.                                */
/* -------------------------------------------------------------------------- */

void moveServoSmooth(uint8_t channel, float target_angle, uint8_t ms_per_deg)
{
    if (channel >= SERVO_COUNT) return;
    if (target_angle < (float)SERVO_ANGLE_MIN) target_angle = (float)SERVO_ANGLE_MIN;
    if (target_angle > (float)SERVO_ANGLE_MAX) target_angle = (float)SERVO_ANGLE_MAX;
    if (ms_per_deg == 0) ms_per_deg = SERVO_DEFAULT_SPEED;

    /* Set target and flag as moving; ServoTask will interpolate */
    osMutexAcquire(servoMutex, osWaitForever);
    float travel = target_angle - g_servoState.current_angle[channel];
    if (travel < 0.0f) travel = -travel;
    s_totalTravel[channel] = travel;
    g_servoState.target_angle[channel]  = target_angle;
    g_servoState.speed_ms_per_deg[channel] = ms_per_deg;
    g_servoState.moving[channel]        = true;
    osMutexRelease(servoMutex);
}

/* -------------------------------------------------------------------------- */
/*  Trapezoidal profile step (called from ServoTask at 1ms intervals)          */
/*                                                                             */
/*  Profile: ramp up speed over ACCEL_ZONE degrees, hold constant, ramp down.  */
/*  Speed modulation is done by adjusting step size per tick.                   */
/* -------------------------------------------------------------------------- */

void servoTaskProcess(void)
{
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        osMutexAcquire(servoMutex, osWaitForever);
        bool is_moving = g_servoState.moving[ch];
        float current  = g_servoState.current_angle[ch];
        float target   = g_servoState.target_angle[ch];
        uint8_t speed  = g_servoState.speed_ms_per_deg[ch];
        osMutexRelease(servoMutex);

        if (!is_moving) continue;

        float remaining = target - current;
        float abs_remaining = (remaining >= 0.0f) ? remaining : -remaining;
        float total_travel = s_totalTravel[ch];

        if (abs_remaining < 0.2f) {
            /* Arrived at target */
            setServoAngle(ch, target);
            osMutexAcquire(servoMutex, osWaitForever);
            g_servoState.moving[ch] = false;
            osMutexRelease(servoMutex);
            continue;
        }

        /* Trapezoidal velocity factor (0.2 to 1.0)
         * total_travel was captured at move start; dist_from_start grows as we move. */
        float accel_zone = (float)SERVO_ACCEL_ZONE_DEG;
        if (total_travel < 1.0f) total_travel = 1.0f; /* Guard against zero */
        if (accel_zone > total_travel / 2.0f) {
            accel_zone = total_travel / 2.0f;
        }

        float velocity_factor = 1.0f;
        float dist_from_start = total_travel - abs_remaining;

        if (dist_from_start < accel_zone && accel_zone > 0.1f) {
            velocity_factor = 0.2f + 0.8f * (dist_from_start / accel_zone);
        } else if (abs_remaining < accel_zone && accel_zone > 0.1f) {
            velocity_factor = 0.2f + 0.8f * (abs_remaining / accel_zone);
        }

        /* Step size: base is 1 degree per (speed) ms.
         * ServoTask runs every 1ms, so base step = 1.0 / speed degrees per call.
         * Modulated by velocity_factor. */
        float step = velocity_factor / (float)speed;

        if (remaining > 0.0f) {
            current += step;
            if (current > target) current = target;
        } else {
            current -= step;
            if (current < target) current = target;
        }

        uint32_t ccr = angleToCCR(current);
        __HAL_TIM_SET_COMPARE(&htim3, s_timChannels[ch], ccr);

        osMutexAcquire(servoMutex, osWaitForever);
        g_servoState.current_angle[ch] = current;
        osMutexRelease(servoMutex);
    }
}

/* -------------------------------------------------------------------------- */
/*  Gripper convenience functions                                              */
/* -------------------------------------------------------------------------- */

void openGripper(void)
{
    if (isEstopActive()) return;
    moveServoSmooth(GRIPPER_CHANNEL, (float)GRIPPER_OPEN_ANGLE, SERVO_DEFAULT_SPEED);
}

void closeGripper(void)
{
    if (isEstopActive()) return;
    moveServoSmooth(GRIPPER_CHANNEL, (float)GRIPPER_CLOSE_ANGLE, SERVO_DEFAULT_SPEED);
}

/**
 * Close gripper incrementally until target force is reached.
 * Force reading comes from ESP32 (g_systemState.current_force_n).
 * Steps 1 degree at a time, checking force after each step.
 */
void setGripForce(float target_force_n)
{
    if (isEstopActive()) return;
    if (target_force_n > g_systemState.force_limit_n) {
        target_force_n = g_systemState.force_limit_n;
    }

    float current = getServoPosition(GRIPPER_CHANNEL);

    while (current < (float)GRIPPER_CLOSE_ANGLE) {
        if (isEstopActive()) return;

        osMutexAcquire(stateMutex, osWaitForever);
        float force = g_systemState.current_force_n;
        osMutexRelease(stateMutex);

        if (force >= target_force_n) {
            break;
        }

        current += (float)GRIPPER_FORCE_STEP_DEG;
        if (current > (float)GRIPPER_CLOSE_ANGLE) {
            current = (float)GRIPPER_CLOSE_ANGLE;
        }

        setServoAngle(GRIPPER_CHANNEL, current);
        osDelay(30); /* Allow force to settle */
    }
}

/**
 * Quick close-open pulse for acoustic tap test.
 * Moves gripper 5 degrees closed then immediately back.
 */
void doTapMotion(void)
{
    if (isEstopActive()) return;

    float pos = getServoPosition(GRIPPER_CHANNEL);
    float tap_pos = pos + 5.0f;
    if (tap_pos > (float)SERVO_ANGLE_MAX) tap_pos = (float)SERVO_ANGLE_MAX;

    setServoAngle(GRIPPER_CHANNEL, tap_pos);
    osDelay(50);
    setServoAngle(GRIPPER_CHANNEL, pos);
}

/* -------------------------------------------------------------------------- */
/*  Getters                                                                    */
/* -------------------------------------------------------------------------- */

float getServoPosition(uint8_t channel)
{
    if (channel >= SERVO_COUNT) return 0.0f;

    osMutexAcquire(servoMutex, osWaitForever);
    float angle = g_servoState.current_angle[channel];
    osMutexRelease(servoMutex);

    return angle;
}

void setServoSpeed(uint8_t channel, uint8_t ms_per_deg)
{
    if (channel >= SERVO_COUNT) return;
    if (ms_per_deg == 0) ms_per_deg = 1;

    osMutexAcquire(servoMutex, osWaitForever);
    g_servoState.speed_ms_per_deg[channel] = ms_per_deg;
    osMutexRelease(servoMutex);
}

/* -------------------------------------------------------------------------- */
/*  Emergency disable                                                          */
/* -------------------------------------------------------------------------- */

void disableAllServos(void)
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        HAL_TIM_PWM_Stop(&htim3, s_timChannels[i]);
    }

    osMutexAcquire(servoMutex, osWaitForever);
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        g_servoState.moving[i] = false;
    }
    osMutexRelease(servoMutex);
}
