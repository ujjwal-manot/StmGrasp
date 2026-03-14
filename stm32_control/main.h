/**
 * STM32H725 Gripper Controller - Main Header
 * Target: STEVAL-ROBKIT1 (STM32H725IGT6)
 * Firmware for servo/gripper control, VL53L8CX depth sensing,
 * LSM6DSV16BX IMU, microphone tap analysis,
 * PID force control, and UART communication with ESP32.
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ========================================================================== */
/*                          PIN DEFINITIONS                                    */
/* ========================================================================== */

/* USART2 - ESP32 Communication (on RPi-compatible header) */
#define ESP32_UART_INSTANCE       USART2
#define ESP32_UART_TX_PIN         GPIO_PIN_5
#define ESP32_UART_TX_PORT        GPIOD
#define ESP32_UART_RX_PIN         GPIO_PIN_6
#define ESP32_UART_RX_PORT        GPIOD
#define ESP32_UART_AF             GPIO_AF7_USART2
#define ESP32_UART_BAUDRATE       115200U
#define ESP32_UART_IRQn           USART2_IRQn
#define ESP32_UART_IRQ_PRIORITY   6U

/* I2C1 - VL53L8CX ToF Sensor (via FPC connector on imaging board) */
#define TOF_I2C_INSTANCE          I2C1
#define TOF_I2C_SCL_PIN           GPIO_PIN_8
#define TOF_I2C_SCL_PORT          GPIOB
#define TOF_I2C_SDA_PIN           GPIO_PIN_9
#define TOF_I2C_SDA_PORT          GPIOB
#define TOF_I2C_AF                GPIO_AF4_I2C1
#define TOF_I2C_TIMING            0x10C0ECFF  /* 400kHz Fast mode for H7 @ 275MHz APB */

/* TIM3 - Servo PWM (4 channels, on GPIO header) */
#define SERVO_TIM_INSTANCE        TIM3
#define SERVO_TIM_CHANNEL_1       TIM_CHANNEL_1   /* Gripper open/close */
#define SERVO_TIM_CHANNEL_2       TIM_CHANNEL_2   /* Wrist rotate */
#define SERVO_TIM_CHANNEL_3       TIM_CHANNEL_3   /* Aux 1 */
#define SERVO_TIM_CHANNEL_4       TIM_CHANNEL_4   /* Aux 2 */

#define SERVO_CH1_PIN             GPIO_PIN_6
#define SERVO_CH1_PORT            GPIOC
#define SERVO_CH2_PIN             GPIO_PIN_7
#define SERVO_CH2_PORT            GPIOC
#define SERVO_CH3_PIN             GPIO_PIN_8
#define SERVO_CH3_PORT            GPIOC
#define SERVO_CH4_PIN             GPIO_PIN_9
#define SERVO_CH4_PORT            GPIOC
#define SERVO_TIM_AF              GPIO_AF2_TIM3

/* Emergency stop button (directly on GPIO header, active low) */
#define ESTOP_PIN                 GPIO_PIN_0
#define ESTOP_PORT                GPIOA

/* Status LED (directly on ROBKIT1 board) */
#define STATUS_LED_PIN            GPIO_PIN_13
#define STATUS_LED_PORT           GPIOH

/* ========================================================================== */
/*                       SERVO CONFIGURATION                                   */
/* ========================================================================== */

#define SERVO_COUNT               4U
#define SERVO_PWM_FREQ_HZ         50U       /* Standard servo 50Hz */
#define SERVO_TIM_PRESCALER       274U      /* 275MHz APB1 / (274+1) = 1MHz tick */
#define SERVO_TIM_PERIOD          19999U    /* 1MHz / 20000 = 50Hz */

#define SERVO_PULSE_MIN_US        500U      /* 0 degrees */
#define SERVO_PULSE_MAX_US        2400U     /* 180 degrees */
#define SERVO_ANGLE_MIN           0U
#define SERVO_ANGLE_MAX           180U

#define SERVO_DEFAULT_SPEED       2U        /* ms per degree for smooth moves */
#define SERVO_ACCEL_ZONE_DEG      15U       /* degrees for acceleration ramp */

/* Gripper-specific */
#define GRIPPER_CHANNEL           0U        /* Index into servo array */
#define GRIPPER_OPEN_ANGLE        10U
#define GRIPPER_CLOSE_ANGLE       160U
#define GRIPPER_FORCE_STEP_DEG    1U        /* Degrees per force-control step */

/* ========================================================================== */
/*                       SAFETY LIMITS                                         */
/* ========================================================================== */

#define MAX_FORCE_DEFAULT_N       10.0f     /* Default max grip force (Newtons) */
#define MAX_FORCE_ABSOLUTE_N      25.0f     /* Absolute max, never exceed */
#define MAX_CURRENT_MA            2000U     /* Motor current limit */
#define SERVO_TRAVEL_MIN_DEG      0U
#define SERVO_TRAVEL_MAX_DEG      180U
#define HEARTBEAT_TIMEOUT_MS      3000U     /* ESP32 comms timeout */
#define SAFETY_CHECK_PERIOD_MS    50U
#define WATCHDOG_TIMEOUT_MS       2000U

/* ========================================================================== */
/*                    UART PROTOCOL DEFINITIONS                                */
/* ========================================================================== */

#define UART_RX_BUF_SIZE          256U
#define UART_TX_BUF_SIZE          256U
#define UART_PACKET_HEADER        0xAAU
#define UART_MAX_PAYLOAD          200U

/* Command IDs (ESP32 -> STM32) */
#define CMD_SET_SERVO_ANGLE       0x01U
#define CMD_MOVE_SERVO_SMOOTH     0x02U
#define CMD_OPEN_GRIPPER          0x03U
#define CMD_CLOSE_GRIPPER         0x04U
#define CMD_SET_GRIP_FORCE        0x05U
#define CMD_TAP_MOTION            0x06U
#define CMD_REQUEST_DEPTH         0x07U
#define CMD_REQUEST_STATUS        0x08U
#define CMD_SET_FORCE_LIMIT       0x09U
#define CMD_HEARTBEAT             0x0AU
#define CMD_EMERGENCY_STOP        0x0BU
#define CMD_SET_SERVO_SPEED       0x0CU
#define CMD_FORCE_UPDATE          0x0DU  /* ESP32 sending force sensor reading */
#define CMD_REQUEST_IMU           0x0EU  /* Request IMU orientation data */
#define CMD_TRIGGER_MIC_CAPTURE   0x0FU  /* Trigger microphone tap capture */
#define CMD_MOTOR_MOVE            0x10U  /* Motor move: [left_hi][left_lo][right_hi][right_lo] */
#define CMD_MOTOR_STOP            0x11U  /* Stop both motors immediately */
#define CMD_APPROACH_OBJECT       0x12U  /* Approach nearest object: [target_dist_hi][target_dist_lo] */
#define CMD_START_FULL_SCAN       0x20U  /* Begin 360-degree 3D scan */
#define CMD_START_QUICK_SCAN      0x21U  /* Single-position quick scan (no rotation) */
#define CMD_GET_SCAN_STATUS       0x22U  /* Query scanner state and progress */
#define CMD_GET_SCAN_RESULT       0x23U  /* Request completed scan result */

/* Response IDs (STM32 -> ESP32) */
#define RSP_ACK                   0x80U
#define RSP_POSITION              0x81U
#define RSP_DEPTH_GRID            0x82U
#define RSP_STATUS                0x83U
#define RSP_ERROR                 0x84U
#define RSP_DEPTH_OBJECT          0x85U
#define RSP_IMU_DATA              0x86U
#define RSP_TAP_RESULT            0x87U
#define RSP_MOTOR_STATUS          0x88U  /* Motor status: [left_speed][right_speed][enabled] */
#define RSP_ENCODER_DATA          0x89U  /* Encoder data: [left_enc(4)][right_enc(4)][distance(4)] */
#define RSP_SCAN_STATUS           0x90U  /* Scanner state + progress */
#define RSP_SCAN_RESULT           0x91U  /* 3D scan result (points, bbox, edges) */

/* Error codes */
#define ERR_NONE                  0x00U
#define ERR_INVALID_CMD           0x01U
#define ERR_CHECKSUM_FAIL         0x02U
#define ERR_SERVO_OUT_OF_RANGE    0x03U
#define ERR_FORCE_EXCEEDED        0x04U
#define ERR_SENSOR_FAULT          0x05U
#define ERR_COMMS_TIMEOUT         0x06U
#define ERR_ESTOP_ACTIVE          0x07U
#define ERR_I2C_FAILURE           0x08U

/* ========================================================================== */
/*                       VL53L8CX DEFINITIONS                                  */
/* ========================================================================== */

#define VL53L8CX_DEFAULT_ADDR     (0x29U << 1)  /* 7-bit addr shifted for HAL */
#define VL53L8CX_GRID_SIZE        8U
#define VL53L8CX_ZONE_COUNT       64U
#define DEPTH_READ_PERIOD_MS      200U  /* 5Hz */
#define DEPTH_DISTANCE_THRESHOLD  300U  /* mm, for object detection */

/* ========================================================================== */
/*                         DATA STRUCTURES                                     */
/* ========================================================================== */

/* UART packet structure:
 * [0xAA] [LEN] [CMD/RSP] [PAYLOAD...] [XOR_CHECKSUM]
 * LEN = number of bytes after LEN (cmd + payload + checksum)
 */
typedef struct {
    uint8_t header;
    uint8_t length;
    uint8_t cmd;
    uint8_t payload[UART_MAX_PAYLOAD];
    uint8_t payload_len;
    uint8_t checksum;
} UartPacket_t;

typedef struct {
    uint16_t distance_mm[VL53L8CX_GRID_SIZE][VL53L8CX_GRID_SIZE];
    uint32_t timestamp;
    bool     valid;
} DepthGrid_t;

typedef struct {
    float center_x;    /* 0.0 - 7.0, column index */
    float center_y;    /* 0.0 - 7.0, row index */
    float width;       /* Estimated width in grid cells */
    float height;      /* Estimated height in grid cells */
    uint16_t min_dist; /* Closest point in mm */
    uint16_t zone_count; /* Number of zones with object */
} ObjectProfile_t;

typedef struct {
    float    current_angle[SERVO_COUNT];
    float    target_angle[SERVO_COUNT];
    uint8_t  speed_ms_per_deg[SERVO_COUNT]; /* ms per degree */
    bool     moving[SERVO_COUNT];
} ServoState_t;

typedef struct {
    bool     estop_active;
    bool     esp32_connected;
    float    force_limit_n;
    float    current_force_n;
    uint32_t last_heartbeat_tick;
    uint8_t  system_status;    /* Bitfield: bit0=servos_ok, bit1=tof_ok, bit2=comms_ok, bit3=safety_ok */
} SystemState_t;

typedef struct {
    int16_t  speed_left;      /* -100 to 100 percent */
    int16_t  speed_right;     /* -100 to 100 percent */
    int32_t  encoder_left;    /* cumulative ticks */
    int32_t  encoder_right;   /* cumulative ticks */
    float    distance_mm;     /* estimated distance traveled */
    bool     enabled;
} MotorState_t;

typedef struct {
    bool    initialized;
    bool    connected;
    char    device_name[20];
    uint8_t rx_buffer[64];
    uint8_t rx_len;
} BLEState_t;

/* System status bitfield */
#define STATUS_SERVOS_OK          (1U << 0)
#define STATUS_TOF_OK             (1U << 1)
#define STATUS_COMMS_OK           (1U << 2)
#define STATUS_SAFETY_OK          (1U << 3)
#define STATUS_IMU_OK             (1U << 4)
#define STATUS_MIC_OK             (1U << 5)
#define STATUS_MOTORS_OK          (1U << 6)
#define STATUS_ESTOP              (1U << 7)

/* ========================================================================== */
/*                       GLOBAL EXTERNS                                         */
/* ========================================================================== */

extern UART_HandleTypeDef  huart2;
extern UART_HandleTypeDef  huart3;   /* BLE module USART3 */
extern I2C_HandleTypeDef   hi2c1;
extern TIM_HandleTypeDef   htim3;
extern IWDG_HandleTypeDef  hiwdg;

extern osMutexId_t         servoMutex;
extern osMutexId_t         depthMutex;   /* Also used as I2C1 bus mutex */
extern osMutexId_t         stateMutex;

extern volatile SystemState_t  g_systemState;
extern volatile ServoState_t   g_servoState;
extern volatile DepthGrid_t    g_depthGrid;
extern volatile MotorState_t   g_motorState;
extern volatile BLEState_t     g_bleState;

extern osMutexId_t             imuMutex;

/* ========================================================================== */
/*                       FUNCTION PROTOTYPES                                   */
/* ========================================================================== */

/* System initialization (main.c) */
extern void SystemClock_Config(void);   /* Generated by CubeMX */
void MX_GPIO_Init(void);
void MX_TIM3_Init(void);
void MX_USART2_Init(void);
void MX_I2C1_Init(void);
void MX_IWDG_Init(void);
void Error_Handler(void);

/* FreeRTOS task entry points (main.c) */
void CommandTask(void *argument);
void ServoTask(void *argument);
void DepthTask(void *argument);
void SafetyTask(void *argument);
void IMUTask(void *argument);
void MotorTask(void *argument);
void BLETask(void *argument);
void ScannerTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
