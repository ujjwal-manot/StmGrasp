/**
 * Motor Control - STSPIN240 Dual Brush DC Motor Driver
 * Inter-board I2C communication with motor control board (STM32G071).
 *
 * The STEVAL-ROBKIT1 uses a two-board architecture: the main board
 * (STM32H725) sends commands over I2C to the motor board (STM32G071)
 * which drives the STSPIN240 directly with PWM and direction signals.
 * Encoder data is read back over the same I2C bus.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========================================================================== */
/*                       MOTOR BOARD I2C DEFINITIONS                           */
/* ========================================================================== */

/* Motor board I2C address (7-bit shifted for HAL) */
#define MOTOR_BOARD_I2C_ADDR      (0x20U << 1)

/* I2C timeout for motor board communication (ms) */
#define MOTOR_I2C_TIMEOUT_MS      50U

/* Motor board register / command map */
#define MOTOR_REG_WHO_AM_I        0x00U   /* Returns 0x4D 'M' if board present */
#define MOTOR_REG_STATUS          0x01U   /* Board status flags */
#define MOTOR_REG_LEFT_SPEED_HI   0x10U   /* Left motor speed high byte */
#define MOTOR_REG_LEFT_SPEED_LO   0x11U   /* Left motor speed low byte */
#define MOTOR_REG_RIGHT_SPEED_HI  0x12U   /* Right motor speed high byte */
#define MOTOR_REG_RIGHT_SPEED_LO  0x13U   /* Right motor speed low byte */
#define MOTOR_REG_ENABLE          0x14U   /* 0x00=disable, 0x01=enable */
#define MOTOR_REG_ENC_LEFT_0      0x20U   /* Encoder left byte 0 (LSB) */
#define MOTOR_REG_ENC_LEFT_1      0x21U   /* Encoder left byte 1 */
#define MOTOR_REG_ENC_LEFT_2      0x22U   /* Encoder left byte 2 */
#define MOTOR_REG_ENC_LEFT_3      0x23U   /* Encoder left byte 3 (MSB) */
#define MOTOR_REG_ENC_RIGHT_0     0x24U   /* Encoder right byte 0 (LSB) */
#define MOTOR_REG_ENC_RIGHT_1     0x25U   /* Encoder right byte 1 */
#define MOTOR_REG_ENC_RIGHT_2     0x26U   /* Encoder right byte 2 */
#define MOTOR_REG_ENC_RIGHT_3     0x27U   /* Encoder right byte 3 (MSB) */

/* Motor board identification */
#define MOTOR_BOARD_WHO_AM_I_VAL  0x4DU   /* 'M' for motor board */

/* Motor board command byte format for bulk write:
 * [CMD_MOTOR_SET] [LEFT_HI] [LEFT_LO] [RIGHT_HI] [RIGHT_LO]
 * Speed is signed int16_t: -1000 to +1000 (0.1% resolution) */
#define MOTOR_CMD_SET_SPEED       0x30U
#define MOTOR_CMD_STOP            0x31U
#define MOTOR_CMD_ENABLE          0x32U
#define MOTOR_CMD_DISABLE         0x33U

/* ========================================================================== */
/*                       MOTOR CONFIGURATION                                   */
/* ========================================================================== */

#define MOTOR_LEFT                0U
#define MOTOR_RIGHT               1U
#define MOTOR_COUNT               2U

#define MOTOR_SPEED_MIN           (-100)
#define MOTOR_SPEED_MAX           100

/* Encoder ticks per wheel revolution (depends on motor + gearbox + encoder) */
#define ENCODER_TICKS_PER_REV     1440U

/* Wheel diameter in mm (STEVAL-ROBKIT1 wheels) */
#define WHEEL_DIAMETER_MM         65.0f

/* Wheel circumference in mm */
#define WHEEL_CIRCUMFERENCE_MM    (3.14159265f * WHEEL_DIAMETER_MM)

/* Wheelbase (distance between wheels) in mm */
#define WHEEL_BASE_MM             150.0f

/* Distance per encoder tick in mm */
#define MM_PER_TICK               (WHEEL_CIRCUMFERENCE_MM / (float)ENCODER_TICKS_PER_REV)

/* Encoder speed sampling period (ms) */
#define ENCODER_SAMPLE_PERIOD_MS  50U

/* Approach control parameters */
#define APPROACH_MIN_SPEED        15      /* Minimum approach speed (percent) */
#define APPROACH_MAX_SPEED        40      /* Maximum approach speed (percent) */
#define APPROACH_STOP_MARGIN_MM   10U     /* Stop within this margin of target */
#define APPROACH_TIMEOUT_MS       15000U  /* Maximum approach time */
#define APPROACH_KP               0.3f    /* Proportional gain for approach control */

/* Move distance parameters */
#define MOVE_TIMEOUT_MS           10000U  /* Maximum move time */
#define MOVE_KP                   0.5f    /* Proportional gain for distance control */

/* Turn parameters */
#define TURN_SPEED_PERCENT        30      /* Turn speed */
#define TURN_TIMEOUT_MS           10000U  /* Maximum turn time */

/* ========================================================================== */
/*                       FUNCTION PROTOTYPES                                   */
/* ========================================================================== */

/* Initialization and board detection */
HAL_StatusTypeDef initMotors(void);

/* Speed control via inter-board I2C */
HAL_StatusTypeDef sendMotorCommand(int16_t left_speed, int16_t right_speed);
HAL_StatusTypeDef setMotorSpeed(uint8_t motor_id, int16_t speed_percent);
HAL_StatusTypeDef stopMotors(void);
HAL_StatusTypeDef enableMotors(void);
HAL_StatusTypeDef disableMotors(void);

/* Encoder readback */
HAL_StatusTypeDef readEncoders(void);
int32_t           getEncoderCount(uint8_t motor_id);
float             getMotorSpeed(uint8_t motor_id);

/* High-level motion commands */
HAL_StatusTypeDef moveForward(int16_t speed_percent, float distance_mm);
HAL_StatusTypeDef turnToAngle(float angle_degrees);
HAL_StatusTypeDef approachObject(uint16_t target_distance_mm);

/* Motor task processing (called from MotorTask loop) */
void motorTaskProcess(void);

/* Low-level I2C helpers */
HAL_StatusTypeDef motor_write_byte(uint8_t reg, uint8_t value);
HAL_StatusTypeDef motor_write_multi(uint8_t reg, const uint8_t *data, uint8_t len);
HAL_StatusTypeDef motor_read_byte(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef motor_read_multi(uint8_t reg, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
