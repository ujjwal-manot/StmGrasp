/**
 * LSM6DSV16BX 6-Axis IMU Driver (Accelerometer + Gyroscope)
 * On-board STEVAL-ROBKIT1 sensor, connected via I2C1 (shared bus with VL53L8CX).
 * SA0 pulled to VDD => I2C address 0x6B.
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========================================================================== */
/*                       LSM6DSV16BX I2C ADDRESS                               */
/* ========================================================================== */

#define LSM6DSV16BX_I2C_ADDR      (0x6BU << 1)  /* 7-bit addr shifted for HAL */

/* ========================================================================== */
/*                       LSM6DSV16BX REGISTER MAP                              */
/* ========================================================================== */

#define LSM6DSV16BX_WHO_AM_I      0x0FU
#define LSM6DSV16BX_CTRL1_XL      0x10U  /* Accelerometer config */
#define LSM6DSV16BX_CTRL2_G       0x11U  /* Gyroscope config */
#define LSM6DSV16BX_CTRL3_C       0x12U  /* Control: BDU, IF_INC */
#define LSM6DSV16BX_STATUS_REG    0x1EU
#define LSM6DSV16BX_OUTX_L_G      0x22U  /* Gyro data start (6 bytes) */
#define LSM6DSV16BX_OUTX_L_A      0x28U  /* Accel data start (6 bytes) */

/* Expected WHO_AM_I value */
#define LSM6DSV16BX_WHO_AM_I_VAL  0x70U

/* CTRL1_XL: ODR=104Hz, FS=+/-4g => 0x48 */
#define LSM6DSV16BX_CTRL1_XL_VAL  0x48U

/* CTRL2_G: ODR=104Hz, FS=+/-500dps => 0x44 */
#define LSM6DSV16BX_CTRL2_G_VAL   0x44U

/* CTRL3_C bits */
#define LSM6DSV16BX_CTRL3_BDU     (1U << 6)  /* Block Data Update */
#define LSM6DSV16BX_CTRL3_IF_INC  (1U << 2)  /* Auto-increment */

/* STATUS_REG bits */
#define LSM6DSV16BX_STATUS_XLDA   (1U << 0)  /* Accel data available */
#define LSM6DSV16BX_STATUS_GDA    (1U << 1)  /* Gyro data available */

/* Sensitivity values */
#define LSM6DSV16BX_ACCEL_SENS    0.122f   /* mg/LSB at +/-4g */
#define LSM6DSV16BX_GYRO_SENS     17.5f    /* mdps/LSB at +/-500dps */

/* Gravity constant for m/s^2 conversion */
#define IMU_GRAVITY_MS2           9.80665f

/* IMU read period (50Hz = 20ms) */
#define IMU_READ_PERIOD_MS        20U

/* Arm tilt threshold for grip force compensation (degrees) */
#define IMU_ARM_TILT_THRESHOLD    30.0f

/* ========================================================================== */
/*                       DATA STRUCTURES                                       */
/* ========================================================================== */

typedef struct {
    float    accel_x, accel_y, accel_z;  /* m/s^2 */
    float    gyro_x, gyro_y, gyro_z;     /* deg/s */
    float    pitch, roll;                /* degrees */
    float    arm_tilt;                   /* degrees from horizontal */
    bool     valid;
    uint32_t timestamp;
} IMUData_t;

/* ========================================================================== */
/*                       FUNCTION PROTOTYPES                                   */
/* ========================================================================== */

HAL_StatusTypeDef initIMU(void);
HAL_StatusTypeDef readIMU(IMUData_t *data);
void              getOrientation(IMUData_t *data);
float             getArmTilt(const IMUData_t *data);

/* Low-level I2C helpers */
HAL_StatusTypeDef imu_write_byte(uint8_t reg, uint8_t value);
HAL_StatusTypeDef imu_read_byte(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef imu_read_multi(uint8_t reg, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SENSOR_H */
