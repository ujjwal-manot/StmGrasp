/**
 * Motor Control Implementation
 * Inter-board I2C master on STM32H725, commanding the STM32G071 motor board.
 *
 * The motor board handles STSPIN240 PWM generation, direction pins, and
 * encoder counting in hardware. This driver sends speed commands and
 * reads encoder data via the shared I2C1 bus (also used by VL53L8CX).
 *
 * I2C bus arbitration: the depthMutex is reused as the I2C1 bus mutex
 * (see depth_sensor.c which also uses I2C1). All I2C1 accesses in this
 * module acquire depthMutex first. This prevents bus contention between
 * ToF reads and motor commands.
 *
 * Speed encoding: int16_t in range -1000 to +1000 (0.1% resolution).
 * This module accepts -100 to +100 percent and scales to the wire format.
 */

#include "motor_control.h"
#include "depth_sensor.h"
#include "safety.h"
#include "uart_protocol.h"

/* ========================================================================== */
/*                       STATIC STATE                                          */
/* ========================================================================== */

/* Previous encoder values for speed computation */
static int32_t  s_prevEncoderLeft;
static int32_t  s_prevEncoderRight;
static uint32_t s_prevEncoderTick;

/* Motor board presence flag */
static bool s_motorBoardPresent;

/* ========================================================================== */
/*  Low-level I2C register access (8-bit addresses, shared I2C1 bus)           */
/* ========================================================================== */

HAL_StatusTypeDef motor_write_byte(uint8_t reg, uint8_t value)
{
    osMutexAcquire(depthMutex, osWaitForever);
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, MOTOR_BOARD_I2C_ADDR,
                                               reg, I2C_MEMADD_SIZE_8BIT,
                                               &value, 1, MOTOR_I2C_TIMEOUT_MS);
    osMutexRelease(depthMutex);
    return ret;
}

HAL_StatusTypeDef motor_write_multi(uint8_t reg, const uint8_t *data, uint8_t len)
{
    osMutexAcquire(depthMutex, osWaitForever);
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, MOTOR_BOARD_I2C_ADDR,
                                               reg, I2C_MEMADD_SIZE_8BIT,
                                               (uint8_t *)data, len,
                                               MOTOR_I2C_TIMEOUT_MS);
    osMutexRelease(depthMutex);
    return ret;
}

HAL_StatusTypeDef motor_read_byte(uint8_t reg, uint8_t *value)
{
    osMutexAcquire(depthMutex, osWaitForever);
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MOTOR_BOARD_I2C_ADDR,
                                              reg, I2C_MEMADD_SIZE_8BIT,
                                              value, 1, MOTOR_I2C_TIMEOUT_MS);
    osMutexRelease(depthMutex);
    return ret;
}

HAL_StatusTypeDef motor_read_multi(uint8_t reg, uint8_t *data, uint8_t len)
{
    osMutexAcquire(depthMutex, osWaitForever);
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MOTOR_BOARD_I2C_ADDR,
                                              reg, I2C_MEMADD_SIZE_8BIT,
                                              data, len, MOTOR_I2C_TIMEOUT_MS);
    osMutexRelease(depthMutex);
    return ret;
}

/* ========================================================================== */
/*  Initialization                                                             */
/* ========================================================================== */

/**
 * Scan for motor board on I2C1 and initialize motor state.
 * Returns HAL_OK if motor board responds at the expected address.
 */
HAL_StatusTypeDef initMotors(void)
{
    HAL_StatusTypeDef ret;

    s_motorBoardPresent = false;
    s_prevEncoderLeft   = 0;
    s_prevEncoderRight  = 0;
    s_prevEncoderTick   = HAL_GetTick();

    /* Initialize global motor state */
    osMutexAcquire(stateMutex, osWaitForever);
    g_motorState.speed_left    = 0;
    g_motorState.speed_right   = 0;
    g_motorState.encoder_left  = 0;
    g_motorState.encoder_right = 0;
    g_motorState.distance_mm   = 0.0f;
    g_motorState.enabled       = false;
    osMutexRelease(stateMutex);

    /* Probe the motor board: read WHO_AM_I register */
    uint8_t who_am_i = 0x00;
    ret = motor_read_byte(MOTOR_REG_WHO_AM_I, &who_am_i);
    if (ret != HAL_OK) {
        return ret;
    }

    if (who_am_i != MOTOR_BOARD_WHO_AM_I_VAL) {
        return HAL_ERROR;
    }

    s_motorBoardPresent = true;

    /* Ensure motors are stopped and disabled on init */
    ret = motor_write_byte(MOTOR_REG_ENABLE, 0x00);
    if (ret != HAL_OK) {
        return ret;
    }

    uint8_t zero_speed[4] = {0x00, 0x00, 0x00, 0x00};
    ret = motor_write_multi(MOTOR_REG_LEFT_SPEED_HI, zero_speed, 4);
    if (ret != HAL_OK) {
        return ret;
    }

    return HAL_OK;
}

/* ========================================================================== */
/*  Speed control                                                              */
/* ========================================================================== */

/**
 * Send speed command to both motors.
 * Speed is clamped to -100..+100 percent, then scaled to -1000..+1000
 * wire format for the motor board.
 */
HAL_StatusTypeDef sendMotorCommand(int16_t left_speed, int16_t right_speed)
{
    if (!s_motorBoardPresent) {
        return HAL_ERROR;
    }

    if (isEstopActive()) {
        return HAL_ERROR;
    }

    /* Clamp to valid range */
    if (left_speed > MOTOR_SPEED_MAX)  left_speed  = MOTOR_SPEED_MAX;
    if (left_speed < MOTOR_SPEED_MIN)  left_speed  = MOTOR_SPEED_MIN;
    if (right_speed > MOTOR_SPEED_MAX) right_speed = MOTOR_SPEED_MAX;
    if (right_speed < MOTOR_SPEED_MIN) right_speed = MOTOR_SPEED_MIN;

    /* Scale to wire format: -100..+100 => -1000..+1000 */
    int16_t left_wire  = left_speed * 10;
    int16_t right_wire = right_speed * 10;

    /* Pack as big-endian signed int16 */
    uint8_t cmd_data[4];
    cmd_data[0] = (uint8_t)((uint16_t)left_wire >> 8);
    cmd_data[1] = (uint8_t)((uint16_t)left_wire & 0xFF);
    cmd_data[2] = (uint8_t)((uint16_t)right_wire >> 8);
    cmd_data[3] = (uint8_t)((uint16_t)right_wire & 0xFF);

    HAL_StatusTypeDef ret = motor_write_multi(MOTOR_REG_LEFT_SPEED_HI, cmd_data, 4);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Update global state */
    osMutexAcquire(stateMutex, osWaitForever);
    g_motorState.speed_left  = left_speed;
    g_motorState.speed_right = right_speed;
    osMutexRelease(stateMutex);

    return HAL_OK;
}

/**
 * Set speed for a single motor. The other motor retains its current speed.
 */
HAL_StatusTypeDef setMotorSpeed(uint8_t motor_id, int16_t speed_percent)
{
    if (motor_id >= MOTOR_COUNT) {
        return HAL_ERROR;
    }

    osMutexAcquire(stateMutex, osWaitForever);
    int16_t left  = g_motorState.speed_left;
    int16_t right = g_motorState.speed_right;
    osMutexRelease(stateMutex);

    if (motor_id == MOTOR_LEFT) {
        left = speed_percent;
    } else {
        right = speed_percent;
    }

    return sendMotorCommand(left, right);
}

/**
 * Stop both motors immediately (zero speed, keep enabled).
 */
HAL_StatusTypeDef stopMotors(void)
{
    return sendMotorCommand(0, 0);
}

/**
 * Enable the STSPIN240 driver on the motor board.
 */
HAL_StatusTypeDef enableMotors(void)
{
    if (!s_motorBoardPresent) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret = motor_write_byte(MOTOR_REG_ENABLE, 0x01);
    if (ret != HAL_OK) {
        return ret;
    }

    osMutexAcquire(stateMutex, osWaitForever);
    g_motorState.enabled = true;
    osMutexRelease(stateMutex);

    return HAL_OK;
}

/**
 * Disable the STSPIN240 driver and stop all motors.
 */
HAL_StatusTypeDef disableMotors(void)
{
    if (!s_motorBoardPresent) {
        return HAL_ERROR;
    }

    /* Stop motors first, then disable driver */
    HAL_StatusTypeDef ret = sendMotorCommand(0, 0);
    if (ret != HAL_OK) {
        return ret;
    }

    ret = motor_write_byte(MOTOR_REG_ENABLE, 0x00);
    if (ret != HAL_OK) {
        return ret;
    }

    osMutexAcquire(stateMutex, osWaitForever);
    g_motorState.enabled = false;
    osMutexRelease(stateMutex);

    return HAL_OK;
}

/* ========================================================================== */
/*  Encoder readback                                                           */
/* ========================================================================== */

/**
 * Read both encoder counts from the motor board and update global state.
 * Each encoder is a 32-bit signed value (4 bytes, little-endian on the board).
 */
HAL_StatusTypeDef readEncoders(void)
{
    if (!s_motorBoardPresent) {
        return HAL_ERROR;
    }

    uint8_t enc_data[8];
    HAL_StatusTypeDef ret = motor_read_multi(MOTOR_REG_ENC_LEFT_0, enc_data, 8);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Parse little-endian int32 for each encoder */
    int32_t enc_left = (int32_t)((uint32_t)enc_data[0] |
                                 ((uint32_t)enc_data[1] << 8) |
                                 ((uint32_t)enc_data[2] << 16) |
                                 ((uint32_t)enc_data[3] << 24));

    int32_t enc_right = (int32_t)((uint32_t)enc_data[4] |
                                  ((uint32_t)enc_data[5] << 8) |
                                  ((uint32_t)enc_data[6] << 16) |
                                  ((uint32_t)enc_data[7] << 24));

    /* Update global state and compute distance */
    osMutexAcquire(stateMutex, osWaitForever);
    int32_t prev_left  = g_motorState.encoder_left;
    int32_t prev_right = g_motorState.encoder_right;
    g_motorState.encoder_left  = enc_left;
    g_motorState.encoder_right = enc_right;

    /* Accumulate distance: average of both wheels' travel */
    int32_t delta_left  = enc_left - prev_left;
    int32_t delta_right = enc_right - prev_right;
    float delta_mm = ((float)(delta_left + delta_right) / 2.0f) * MM_PER_TICK;
    if (delta_mm < 0.0f) delta_mm = -delta_mm;
    g_motorState.distance_mm += delta_mm;
    osMutexRelease(stateMutex);

    return HAL_OK;
}

/**
 * Get the current encoder count for a motor.
 */
int32_t getEncoderCount(uint8_t motor_id)
{
    int32_t count = 0;

    osMutexAcquire(stateMutex, osWaitForever);
    if (motor_id == MOTOR_LEFT) {
        count = g_motorState.encoder_left;
    } else if (motor_id == MOTOR_RIGHT) {
        count = g_motorState.encoder_right;
    }
    osMutexRelease(stateMutex);

    return count;
}

/**
 * Compute motor speed in ticks per second from encoder deltas.
 * Uses the time elapsed since the last call to readEncoders().
 * This function should be called periodically (e.g., every ENCODER_SAMPLE_PERIOD_MS).
 */
float getMotorSpeed(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) {
        return 0.0f;
    }

    osMutexAcquire(stateMutex, osWaitForever);
    int32_t current_left  = g_motorState.encoder_left;
    int32_t current_right = g_motorState.encoder_right;
    osMutexRelease(stateMutex);

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_prevEncoderTick;

    if (elapsed == 0) {
        return 0.0f;
    }

    int32_t delta;
    if (motor_id == MOTOR_LEFT) {
        delta = current_left - s_prevEncoderLeft;
    } else {
        delta = current_right - s_prevEncoderRight;
    }

    /* Speed in ticks per second */
    float speed = ((float)delta * 1000.0f) / (float)elapsed;

    return speed;
}

/* ========================================================================== */
/*  High-level motion: move forward a specified distance                       */
/* ========================================================================== */

/**
 * Move forward (or backward if speed is negative) by a specified distance
 * in millimeters. Uses encoder feedback with proportional control to maintain
 * straight-line travel. Blocks until distance is reached or timeout.
 */
HAL_StatusTypeDef moveForward(int16_t speed_percent, float distance_mm)
{
    if (!s_motorBoardPresent || isEstopActive()) {
        return HAL_ERROR;
    }

    if (distance_mm <= 0.0f) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret;

    /* Enable motors */
    ret = enableMotors();
    if (ret != HAL_OK) {
        return ret;
    }

    /* Record starting encoder values */
    ret = readEncoders();
    if (ret != HAL_OK) {
        disableMotors();
        return ret;
    }

    int32_t start_left  = getEncoderCount(MOTOR_LEFT);
    int32_t start_right = getEncoderCount(MOTOR_RIGHT);

    /* Compute target tick count (same for both wheels for straight travel) */
    float target_ticks_f = distance_mm / MM_PER_TICK;
    int32_t target_ticks = (int32_t)(target_ticks_f + 0.5f);
    if (target_ticks < 0) target_ticks = -target_ticks;

    uint32_t start_time = HAL_GetTick();
    int16_t base_speed  = speed_percent;

    while ((HAL_GetTick() - start_time) < MOVE_TIMEOUT_MS) {
        if (isEstopActive()) {
            stopMotors();
            disableMotors();
            return HAL_ERROR;
        }

        /* Read current encoder positions */
        ret = readEncoders();
        if (ret != HAL_OK) {
            stopMotors();
            disableMotors();
            return ret;
        }

        int32_t delta_left  = getEncoderCount(MOTOR_LEFT) - start_left;
        int32_t delta_right = getEncoderCount(MOTOR_RIGHT) - start_right;

        /* Absolute travel */
        int32_t abs_left  = (delta_left >= 0)  ? delta_left  : -delta_left;
        int32_t abs_right = (delta_right >= 0) ? delta_right : -delta_right;
        int32_t avg_travel = (abs_left + abs_right) / 2;

        /* Check if distance reached */
        if (avg_travel >= target_ticks) {
            break;
        }

        /* Proportional correction to keep both wheels in sync.
         * If left has traveled further, slow it down (and vice versa). */
        int32_t drift = abs_left - abs_right;
        float correction = MOVE_KP * (float)drift;

        int16_t left_cmd  = base_speed - (int16_t)correction;
        int16_t right_cmd = base_speed + (int16_t)correction;

        /* Slow down as we approach the target */
        int32_t remaining = target_ticks - avg_travel;
        if (remaining < (target_ticks / 4) && remaining > 0) {
            float scale = (float)remaining / (float)(target_ticks / 4);
            if (scale < 0.3f) scale = 0.3f;
            left_cmd  = (int16_t)((float)left_cmd * scale);
            right_cmd = (int16_t)((float)right_cmd * scale);
        }

        ret = sendMotorCommand(left_cmd, right_cmd);
        if (ret != HAL_OK) {
            stopMotors();
            disableMotors();
            return ret;
        }

        osDelay(ENCODER_SAMPLE_PERIOD_MS);
    }

    /* Stop and disable */
    stopMotors();
    disableMotors();

    /* Check if we timed out */
    if ((HAL_GetTick() - start_time) >= MOVE_TIMEOUT_MS) {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

/* ========================================================================== */
/*  High-level motion: differential turn to a specified angle                  */
/* ========================================================================== */

/**
 * Rotate in place by a specified angle in degrees.
 * Positive = clockwise, negative = counter-clockwise.
 * Blocks until angle is reached or timeout.
 *
 * Differential drive geometry: to rotate by theta degrees,
 * each wheel must travel an arc = (theta/360) * PI * wheelbase.
 * Left and right wheels move in opposite directions.
 */
HAL_StatusTypeDef turnToAngle(float angle_degrees)
{
    if (!s_motorBoardPresent || isEstopActive()) {
        return HAL_ERROR;
    }

    if (angle_degrees == 0.0f) {
        return HAL_OK;
    }

    HAL_StatusTypeDef ret;

    /* Compute arc distance each wheel must travel */
    float abs_angle = (angle_degrees >= 0.0f) ? angle_degrees : -angle_degrees;
    float arc_mm = (abs_angle / 360.0f) * 3.14159265f * WHEEL_BASE_MM;
    int32_t target_ticks = (int32_t)(arc_mm / MM_PER_TICK + 0.5f);

    if (target_ticks <= 0) {
        return HAL_OK;
    }

    /* Enable motors */
    ret = enableMotors();
    if (ret != HAL_OK) {
        return ret;
    }

    /* Record starting encoder values */
    ret = readEncoders();
    if (ret != HAL_OK) {
        disableMotors();
        return ret;
    }

    int32_t start_left  = getEncoderCount(MOTOR_LEFT);
    int32_t start_right = getEncoderCount(MOTOR_RIGHT);

    /* Determine direction: CW = left forward, right backward */
    int16_t left_speed, right_speed;
    if (angle_degrees > 0.0f) {
        /* Clockwise */
        left_speed  =  TURN_SPEED_PERCENT;
        right_speed = -TURN_SPEED_PERCENT;
    } else {
        /* Counter-clockwise */
        left_speed  = -TURN_SPEED_PERCENT;
        right_speed =  TURN_SPEED_PERCENT;
    }

    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < TURN_TIMEOUT_MS) {
        if (isEstopActive()) {
            stopMotors();
            disableMotors();
            return HAL_ERROR;
        }

        ret = readEncoders();
        if (ret != HAL_OK) {
            stopMotors();
            disableMotors();
            return ret;
        }

        int32_t delta_left  = getEncoderCount(MOTOR_LEFT) - start_left;
        int32_t delta_right = getEncoderCount(MOTOR_RIGHT) - start_right;

        /* Absolute travel for each wheel */
        int32_t abs_left  = (delta_left >= 0)  ? delta_left  : -delta_left;
        int32_t abs_right = (delta_right >= 0) ? delta_right : -delta_right;
        int32_t avg_travel = (abs_left + abs_right) / 2;

        /* Check if angle reached */
        if (avg_travel >= target_ticks) {
            break;
        }

        /* Slow down near the target */
        int32_t remaining = target_ticks - avg_travel;
        int16_t cmd_left  = left_speed;
        int16_t cmd_right = right_speed;

        if (remaining < (target_ticks / 3) && remaining > 0) {
            float scale = (float)remaining / (float)(target_ticks / 3);
            if (scale < 0.3f) scale = 0.3f;
            cmd_left  = (int16_t)((float)cmd_left * scale);
            cmd_right = (int16_t)((float)cmd_right * scale);
        }

        ret = sendMotorCommand(cmd_left, cmd_right);
        if (ret != HAL_OK) {
            stopMotors();
            disableMotors();
            return ret;
        }

        osDelay(ENCODER_SAMPLE_PERIOD_MS);
    }

    stopMotors();
    disableMotors();

    if ((HAL_GetTick() - start_time) >= TURN_TIMEOUT_MS) {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

/* ========================================================================== */
/*  Autonomous approach using VL53L8CX depth feedback                          */
/* ========================================================================== */

/**
 * Drive toward the closest object detected by the VL53L8CX until the
 * specified target distance is reached. Uses proportional speed control:
 * fast when far, slow when close, stop when within margin.
 *
 * The approach scans the center 4x4 region of the 8x8 depth grid to
 * find the nearest object, then drives forward proportionally.
 */
HAL_StatusTypeDef approachObject(uint16_t target_distance_mm)
{
    if (!s_motorBoardPresent || isEstopActive()) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret;

    ret = enableMotors();
    if (ret != HAL_OK) {
        return ret;
    }

    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < APPROACH_TIMEOUT_MS) {
        if (isEstopActive()) {
            stopMotors();
            disableMotors();
            return HAL_ERROR;
        }

        /* Read the current depth grid */
        DepthGrid_t local_grid;
        osMutexAcquire(depthMutex, osWaitForever);
        memcpy(&local_grid, (const void *)&g_depthGrid, sizeof(DepthGrid_t));
        osMutexRelease(depthMutex);

        if (!local_grid.valid) {
            /* No valid depth data, wait for next frame */
            osDelay(DEPTH_READ_PERIOD_MS);
            continue;
        }

        /* Find the closest object in the center 4x4 region (rows 2-5, cols 2-5) */
        uint16_t min_dist = 0xFFFF;
        for (uint8_t row = 2; row < 6; row++) {
            for (uint8_t col = 2; col < 6; col++) {
                uint16_t d = local_grid.distance_mm[row][col];
                if (d > 0 && d < min_dist) {
                    min_dist = d;
                }
            }
        }

        if (min_dist == 0xFFFF) {
            /* No object detected, stop and wait */
            stopMotors();
            osDelay(DEPTH_READ_PERIOD_MS);
            continue;
        }

        /* Check if we've reached the target distance */
        int32_t error_mm = (int32_t)min_dist - (int32_t)target_distance_mm;

        if (error_mm <= (int32_t)APPROACH_STOP_MARGIN_MM &&
            error_mm >= -(int32_t)APPROACH_STOP_MARGIN_MM) {
            /* Within margin, stop */
            break;
        }

        if (error_mm < -(int32_t)APPROACH_STOP_MARGIN_MM) {
            /* Too close, back up slightly */
            int16_t backup_speed = -(int16_t)APPROACH_MIN_SPEED;
            ret = sendMotorCommand(backup_speed, backup_speed);
            if (ret != HAL_OK) {
                stopMotors();
                disableMotors();
                return ret;
            }
            osDelay(ENCODER_SAMPLE_PERIOD_MS);
            continue;
        }

        /* Proportional speed control: faster when far, slower when close */
        float speed_f = APPROACH_KP * (float)error_mm;

        /* Clamp to approach speed range */
        int16_t speed_cmd = (int16_t)speed_f;
        if (speed_cmd > APPROACH_MAX_SPEED)  speed_cmd = APPROACH_MAX_SPEED;
        if (speed_cmd < APPROACH_MIN_SPEED)  speed_cmd = APPROACH_MIN_SPEED;

        ret = sendMotorCommand(speed_cmd, speed_cmd);
        if (ret != HAL_OK) {
            stopMotors();
            disableMotors();
            return ret;
        }

        /* Read encoders for distance tracking */
        readEncoders();

        osDelay(ENCODER_SAMPLE_PERIOD_MS);
    }

    stopMotors();
    disableMotors();

    if ((HAL_GetTick() - start_time) >= APPROACH_TIMEOUT_MS) {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

/* ========================================================================== */
/*  Motor task periodic processing                                             */
/* ========================================================================== */

/**
 * Called from MotorTask loop at ENCODER_SAMPLE_PERIOD_MS intervals.
 * Reads encoder data and updates speed estimates. Also handles
 * sending motor status responses to ESP32.
 */
void motorTaskProcess(void)
{
    if (!s_motorBoardPresent) {
        return;
    }

    /* Read encoder data from motor board */
    HAL_StatusTypeDef ret = readEncoders();
    if (ret != HAL_OK) {
        return;
    }

    /* Update speed estimates */
    osMutexAcquire(stateMutex, osWaitForever);
    int32_t current_left  = g_motorState.encoder_left;
    int32_t current_right = g_motorState.encoder_right;
    osMutexRelease(stateMutex);

    /* Store previous values for next speed calculation */
    s_prevEncoderLeft  = current_left;
    s_prevEncoderRight = current_right;
    s_prevEncoderTick  = HAL_GetTick();
}
