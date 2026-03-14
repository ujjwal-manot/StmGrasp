/**
 * LSM6DSV16BX 6-Axis IMU Driver Implementation
 *
 * Accelerometer + gyroscope on the STEVAL-ROBKIT1 main board.
 * Connected via I2C1 (shared bus with VL53L8CX ToF sensor).
 * SA0 pulled to VDD => 7-bit address 0x6B.
 *
 * Configuration:
 *   Accelerometer: ODR=104Hz, FS=+/-4g  (sensitivity 0.122 mg/LSB)
 *   Gyroscope:     ODR=104Hz, FS=+/-500dps (sensitivity 17.5 mdps/LSB)
 *
 * The I2C bus is shared with the VL53L8CX. All I2C accesses from this
 * driver acquire depthMutex to prevent bus collisions.
 *
 * Reference: ST LSM6DSV16BX datasheet (DocID034703).
 */

#include "imu_sensor.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static bool s_imuInitialized = false;

/* ========================================================================== */
/*  Low-level I2C register access (8-bit addresses, shared bus)                */
/* ========================================================================== */

HAL_StatusTypeDef imu_write_byte(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef ret;

    osMutexAcquire(depthMutex, osWaitForever);
    ret = HAL_I2C_Mem_Write(&hi2c1, LSM6DSV16BX_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    osMutexRelease(depthMutex);

    return ret;
}

HAL_StatusTypeDef imu_read_byte(uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef ret;

    osMutexAcquire(depthMutex, osWaitForever);
    ret = HAL_I2C_Mem_Read(&hi2c1, LSM6DSV16BX_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, value, 1, 100);
    osMutexRelease(depthMutex);

    return ret;
}

HAL_StatusTypeDef imu_read_multi(uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;

    osMutexAcquire(depthMutex, osWaitForever);
    ret = HAL_I2C_Mem_Read(&hi2c1, LSM6DSV16BX_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, data, len, 100);
    osMutexRelease(depthMutex);

    return ret;
}

/* ========================================================================== */
/*  Sensor initialization                                                      */
/* ========================================================================== */

/**
 * Initialize the LSM6DSV16BX IMU sensor.
 * Verifies WHO_AM_I, configures accelerometer and gyroscope, enables BDU
 * and auto-increment for multi-byte reads.
 */
HAL_StatusTypeDef initIMU(void)
{
    HAL_StatusTypeDef ret;
    uint8_t tmp;

    s_imuInitialized = false;

    /* Verify the sensor is present by reading WHO_AM_I */
    ret = imu_read_byte(LSM6DSV16BX_WHO_AM_I, &tmp);
    if (ret != HAL_OK) {
        return ret;
    }

    if (tmp != LSM6DSV16BX_WHO_AM_I_VAL) {
        return HAL_ERROR;
    }

    /* Allow sensor to settle after power-on */
    osDelay(10);

    /* Configure CTRL3_C: enable Block Data Update (BDU) and auto-increment (IF_INC).
     * Read-modify-write to preserve other bits. */
    ret = imu_read_byte(LSM6DSV16BX_CTRL3_C, &tmp);
    if (ret != HAL_OK) {
        return ret;
    }

    tmp |= LSM6DSV16BX_CTRL3_BDU | LSM6DSV16BX_CTRL3_IF_INC;
    ret = imu_write_byte(LSM6DSV16BX_CTRL3_C, tmp);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Configure accelerometer: ODR=104Hz, FS=+/-4g */
    ret = imu_write_byte(LSM6DSV16BX_CTRL1_XL, LSM6DSV16BX_CTRL1_XL_VAL);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Configure gyroscope: ODR=104Hz, FS=+/-500dps */
    ret = imu_write_byte(LSM6DSV16BX_CTRL2_G, LSM6DSV16BX_CTRL2_G_VAL);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Wait for first data to be available */
    osDelay(20);

    s_imuInitialized = true;
    return HAL_OK;
}

/* ========================================================================== */
/*  Read 6-axis data                                                           */
/* ========================================================================== */

/**
 * Read accelerometer and gyroscope data from the LSM6DSV16BX.
 * Checks STATUS_REG for data ready, then reads 12 bytes (gyro + accel)
 * starting at OUTX_L_G (0x22) through OUTZ_H_XL (0x2D) in one burst
 * thanks to auto-increment.
 *
 * Converts raw int16_t to physical units:
 *   - Acceleration in m/s^2
 *   - Angular rate in deg/s
 */
HAL_StatusTypeDef readIMU(IMUData_t *data)
{
    if (!s_imuInitialized) {
        data->valid = false;
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret;
    uint8_t status;

    /* Check STATUS_REG for data ready bits */
    ret = imu_read_byte(LSM6DSV16BX_STATUS_REG, &status);
    if (ret != HAL_OK) {
        data->valid = false;
        return ret;
    }

    /* Both accelerometer and gyroscope data must be ready */
    if ((status & (LSM6DSV16BX_STATUS_XLDA | LSM6DSV16BX_STATUS_GDA)) !=
        (LSM6DSV16BX_STATUS_XLDA | LSM6DSV16BX_STATUS_GDA)) {
        /* No new data yet; keep previous valid state */
        return HAL_OK;
    }

    /* Read 12 bytes: gyro (6 bytes at 0x22) + accel (6 bytes at 0x28)
     * With auto-increment enabled, read from 0x22 through 0x2D = 12 bytes */
    uint8_t raw[12];
    ret = imu_read_multi(LSM6DSV16BX_OUTX_L_G, raw, 12);
    if (ret != HAL_OK) {
        data->valid = false;
        return ret;
    }

    /* Parse gyroscope: raw[0..5] = OUTX_L_G, OUTX_H_G, OUTY_L_G, OUTY_H_G, OUTZ_L_G, OUTZ_H_G */
    int16_t gyro_x_raw = (int16_t)((uint16_t)raw[0]  | ((uint16_t)raw[1]  << 8));
    int16_t gyro_y_raw = (int16_t)((uint16_t)raw[2]  | ((uint16_t)raw[3]  << 8));
    int16_t gyro_z_raw = (int16_t)((uint16_t)raw[4]  | ((uint16_t)raw[5]  << 8));

    /* Parse accelerometer: raw[6..11] = OUTX_L_A, OUTX_H_A, OUTY_L_A, OUTY_H_A, OUTZ_L_A, OUTZ_H_A */
    int16_t accel_x_raw = (int16_t)((uint16_t)raw[6]  | ((uint16_t)raw[7]  << 8));
    int16_t accel_y_raw = (int16_t)((uint16_t)raw[8]  | ((uint16_t)raw[9]  << 8));
    int16_t accel_z_raw = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));

    /* Convert gyroscope: sensitivity = 17.5 mdps/LSB => deg/s = raw * 17.5 / 1000 */
    data->gyro_x = (float)gyro_x_raw * LSM6DSV16BX_GYRO_SENS / 1000.0f;
    data->gyro_y = (float)gyro_y_raw * LSM6DSV16BX_GYRO_SENS / 1000.0f;
    data->gyro_z = (float)gyro_z_raw * LSM6DSV16BX_GYRO_SENS / 1000.0f;

    /* Convert accelerometer: sensitivity = 0.122 mg/LSB => m/s^2 = raw * 0.122 / 1000 * 9.80665 */
    data->accel_x = (float)accel_x_raw * LSM6DSV16BX_ACCEL_SENS / 1000.0f * IMU_GRAVITY_MS2;
    data->accel_y = (float)accel_y_raw * LSM6DSV16BX_ACCEL_SENS / 1000.0f * IMU_GRAVITY_MS2;
    data->accel_z = (float)accel_z_raw * LSM6DSV16BX_ACCEL_SENS / 1000.0f * IMU_GRAVITY_MS2;

    /* Compute orientation from accelerometer */
    getOrientation(data);

    /* Compute arm tilt */
    data->arm_tilt = getArmTilt(data);

    data->timestamp = HAL_GetTick();
    data->valid = true;

    return HAL_OK;
}

/* ========================================================================== */
/*  Orientation computation                                                    */
/* ========================================================================== */

/**
 * Compute pitch and roll from accelerometer data using atan2.
 * These tell us if the gripper is tilted, which affects grasp approach.
 *
 * pitch = atan2(-ax, sqrt(ay^2 + az^2)) * 180/PI
 * roll  = atan2(ay, az) * 180/PI
 */
void getOrientation(IMUData_t *data)
{
    float ax = data->accel_x;
    float ay = data->accel_y;
    float az = data->accel_z;

    data->pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);
    data->roll  = atan2f(ay, az) * (180.0f / M_PI);
}

/* ========================================================================== */
/*  Arm tilt computation                                                       */
/* ========================================================================== */

/**
 * Returns tilt angle of the arm relative to horizontal (in degrees).
 * Used to compensate for arm position in grasp planning.
 * If arm is tilted >30 degrees, grip force should be adjusted upward
 * because gravity pulls the object out of the grip.
 *
 * Tilt is computed as the angle between the accelerometer vector and
 * the gravity vector (assumed to be along Z when arm is horizontal).
 */
float getArmTilt(const IMUData_t *data)
{
    float ax = data->accel_x;
    float ay = data->accel_y;
    float az = data->accel_z;

    /* Total acceleration magnitude */
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (mag < 0.01f) {
        return 0.0f; /* Guard against division by zero */
    }

    /* Angle between accel vector and Z axis (gravity direction when horizontal).
     * cos(tilt) = az / |a|
     * tilt = acos(az / |a|) converted to degrees */
    float cos_tilt = az / mag;

    /* Clamp to [-1, 1] to prevent NaN from floating point rounding */
    if (cos_tilt > 1.0f) cos_tilt = 1.0f;
    if (cos_tilt < -1.0f) cos_tilt = -1.0f;

    float tilt_rad = acosf(fabsf(cos_tilt));
    return tilt_rad * (180.0f / M_PI);
}
