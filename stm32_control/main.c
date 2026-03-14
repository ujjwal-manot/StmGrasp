/**
 * STM32H725 Gripper Controller - Main Application
 * Target: STEVAL-ROBKIT1 (STM32H725IGT6)
 *
 * FreeRTOS tasks:
 *   SafetyTask   (priority 7 - highest) - Watchdog, force limits, E-STOP
 *   CommandTask  (priority 6)           - Parse UART commands from ESP32
 *   ServoTask    (priority 5)           - Smooth servo interpolation
 *   IMUTask      (priority 4)           - LSM6DSV16BX 50Hz IMU reads
 *   MotorTask    (priority 3)           - Motor control and encoder readback
 *   BLETask      (priority 2)           - BLE communication with mobile app
 *   DepthTask    (priority 1)           - VL53L8CX 5Hz depth reads
 *   ScannerTask  (priority 0 - lowest)  - 3D scanning (photometric stereo + ToF fusion)
 *
 * Peripheral mapping:
 *   USART2 (PD5/PD6)  - ESP32 communication, 115200 baud
 *   USART3 (PD8/PD9)  - BLE module communication, 115200 baud
 *   TIM3 (PC6-PC9)    - 4-channel servo PWM, 50Hz
 *   I2C1 (PB8/PB9)    - VL53L8CX ToF + LSM6DSV16BX IMU + motor board (shared bus)
 *   ADC3 (PF3)        - Analog microphone input (8kHz via TIM6 trigger)
 *   IWDG              - 2-second independent watchdog
 */

#include "main.h"
#include "servo_control.h"
#include "depth_sensor.h"
#include "imu_sensor.h"
#include "mic_sensor.h"
#include "motor_control.h"
#include "ble_comm.h"
#include "uart_protocol.h"
#include "scanner_3d.h"
#include "safety.h"

/* ========================================================================== */
/*                       GLOBAL HANDLES                                        */
/* ========================================================================== */

UART_HandleTypeDef  huart2;
I2C_HandleTypeDef   hi2c1;
TIM_HandleTypeDef   htim3;
IWDG_HandleTypeDef  hiwdg;

osMutexId_t         servoMutex;
osMutexId_t         depthMutex;
osMutexId_t         stateMutex;
osMutexId_t         imuMutex;
volatile SystemState_t  g_systemState;
volatile ServoState_t   g_servoState;
volatile DepthGrid_t    g_depthGrid;
volatile IMUData_t      g_imuData;
volatile MicCapture_t   g_micCapture;
volatile MotorState_t   g_motorState;
volatile BLEState_t     g_bleState;

/* Task handles */
static osThreadId_t s_commandTaskHandle;
static osThreadId_t s_servoTaskHandle;
static osThreadId_t s_depthTaskHandle;
static osThreadId_t s_safetyTaskHandle;
static osThreadId_t s_imuTaskHandle;
static osThreadId_t s_motorTaskHandle;
static osThreadId_t s_bleTaskHandle;
static osThreadId_t s_scannerTaskHandle;

/* ========================================================================== */
/*                       PERIPHERAL INITIALIZATION                             */
/* ========================================================================== */

/**
 * GPIO initialization for all used pins.
 * Alternate-function pins (UART, I2C, PWM) are configured here.
 */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Enable all required GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* E-STOP button: PA0, input with pull-up (active low) */
    gpio.Pin   = ESTOP_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ESTOP_PORT, &gpio);

    /* Status LED: PH13, push-pull output */
    gpio.Pin   = STATUS_LED_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STATUS_LED_PORT, &gpio);
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);

    /* USART2 TX: PD5 */
    gpio.Pin       = ESP32_UART_TX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = ESP32_UART_AF;
    HAL_GPIO_Init(ESP32_UART_TX_PORT, &gpio);

    /* USART2 RX: PD6 */
    gpio.Pin       = ESP32_UART_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = ESP32_UART_AF;
    HAL_GPIO_Init(ESP32_UART_RX_PORT, &gpio);

    /* I2C1 SCL: PB8 */
    gpio.Pin       = TOF_I2C_SCL_PIN;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;  /* External pull-ups on board */
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = TOF_I2C_AF;
    HAL_GPIO_Init(TOF_I2C_SCL_PORT, &gpio);

    /* I2C1 SDA: PB9 */
    gpio.Pin       = TOF_I2C_SDA_PIN;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = TOF_I2C_AF;
    HAL_GPIO_Init(TOF_I2C_SDA_PORT, &gpio);

    /* TIM3 CH1-CH4: PC6, PC7, PC8, PC9 (servo PWM) */
    gpio.Pin       = SERVO_CH1_PIN | SERVO_CH2_PIN | SERVO_CH3_PIN | SERVO_CH4_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = SERVO_TIM_AF;
    HAL_GPIO_Init(SERVO_CH1_PORT, &gpio); /* All on GPIOC */
}

/**
 * TIM3 configuration: 4-channel PWM for servos.
 * APB1 timer clock = 275 MHz (HCLK/2 with AHB prescaler on H7).
 * Prescaler = 274 => 1 MHz tick.
 * ARR = 19999 => 50 Hz (20 ms period).
 */
void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance               = SERVO_TIM_INSTANCE;
    htim3.Init.Prescaler         = SERVO_TIM_PRESCALER;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = SERVO_TIM_PERIOD;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    /* Configure all 4 channels in PWM Mode 1 */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = 1500; /* Center position = 1500 us */
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * USART2 configuration: 115200, 8N1, interrupt RX.
 */
void MX_USART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance          = ESP32_UART_INSTANCE;
    huart2.Init.BaudRate     = ESP32_UART_BAUDRATE;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }

    /* Enable USART2 interrupt */
    HAL_NVIC_SetPriority(ESP32_UART_IRQn, ESP32_UART_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(ESP32_UART_IRQn);
}

/**
 * I2C1 configuration: Fast mode (400 kHz) for VL53L8CX.
 */
void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance              = TOF_I2C_INSTANCE;
    hi2c1.Init.Timing           = TOF_I2C_TIMING;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    /* Enable analog filter, disable digital filter */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * Independent Watchdog: ~2 second timeout.
 * LSI = 32 kHz. Prescaler = 64 => 500 Hz. Reload = 1000 => 2.0 s.
 */
void MX_IWDG_Init(void)
{
    hiwdg.Instance       = IWDG1;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    hiwdg.Init.Reload    = 1000U;

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}

/* ========================================================================== */
/*                       USART2 IRQ HANDLER                                    */
/* ========================================================================== */

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* ========================================================================== */
/*                       FREERTOS TASKS                                        */
/* ========================================================================== */

/**
 * CommandTask: Parse and dispatch UART commands from ESP32.
 * Priority 3. Runs whenever data is available in ring buffer.
 */
void CommandTask(void *argument)
{
    (void)argument;
    UartPacket_t pkt;

    for (;;) {
        if (parseCommand(&pkt)) {
            switch (pkt.cmd) {

            case CMD_SET_SERVO_ANGLE: {
                /* Payload: [channel(1)] [angle_x10(2, big-endian)] */
                if (pkt.payload_len >= 3) {
                    uint8_t ch = pkt.payload[0];
                    uint16_t angle_x10 = ((uint16_t)pkt.payload[1] << 8) | pkt.payload[2];
                    float angle = (float)angle_x10 / 10.0f;
                    if (ch < SERVO_COUNT && angle >= 0.0f && angle <= 180.0f) {
                        setServoAngle(ch, angle);
                        sendAck(CMD_SET_SERVO_ANGLE);
                    } else {
                        sendError(ERR_SERVO_OUT_OF_RANGE);
                    }
                }
                break;
            }

            case CMD_MOVE_SERVO_SMOOTH: {
                /* Payload: [channel(1)] [angle_x10(2)] [speed_ms_per_deg(1)] */
                if (pkt.payload_len >= 4) {
                    uint8_t ch = pkt.payload[0];
                    uint16_t angle_x10 = ((uint16_t)pkt.payload[1] << 8) | pkt.payload[2];
                    uint8_t speed = pkt.payload[3];
                    float angle = (float)angle_x10 / 10.0f;
                    if (ch < SERVO_COUNT && angle >= 0.0f && angle <= 180.0f) {
                        moveServoSmooth(ch, angle, speed);
                        sendAck(CMD_MOVE_SERVO_SMOOTH);
                    } else {
                        sendError(ERR_SERVO_OUT_OF_RANGE);
                    }
                }
                break;
            }

            case CMD_OPEN_GRIPPER:
                openGripper();
                sendAck(CMD_OPEN_GRIPPER);
                break;

            case CMD_CLOSE_GRIPPER:
                closeGripper();
                sendAck(CMD_CLOSE_GRIPPER);
                break;

            case CMD_SET_GRIP_FORCE: {
                /* Payload: [force_x10(2, big-endian)] */
                if (pkt.payload_len >= 2) {
                    uint16_t force_x10 = ((uint16_t)pkt.payload[0] << 8) | pkt.payload[1];
                    float force = (float)force_x10 / 10.0f;
                    setGripForce(force);
                    sendAck(CMD_SET_GRIP_FORCE);
                }
                break;
            }

            case CMD_TAP_MOTION:
                doTapMotion();
                sendAck(CMD_TAP_MOTION);
                break;

            case CMD_REQUEST_DEPTH: {
                osMutexAcquire(depthMutex, osWaitForever);
                DepthGrid_t local_grid;
                memcpy(&local_grid, (const void *)&g_depthGrid, sizeof(DepthGrid_t));
                osMutexRelease(depthMutex);
                sendDepthGrid(&local_grid);
                break;
            }

            case CMD_REQUEST_STATUS:
                sendStatus();
                break;

            case CMD_SET_FORCE_LIMIT: {
                /* Payload: [force_x10(2, big-endian)] */
                if (pkt.payload_len >= 2) {
                    uint16_t limit_x10 = ((uint16_t)pkt.payload[0] << 8) | pkt.payload[1];
                    float limit = (float)limit_x10 / 10.0f;
                    setForceLimit(limit);
                    sendAck(CMD_SET_FORCE_LIMIT);
                }
                break;
            }

            case CMD_HEARTBEAT:
                recordHeartbeat();
                sendAck(CMD_HEARTBEAT);
                break;

            case CMD_EMERGENCY_STOP:
                emergencyStop();
                break;

            case CMD_SET_SERVO_SPEED: {
                /* Payload: [channel(1)] [speed_ms_per_deg(1)] */
                if (pkt.payload_len >= 2) {
                    setServoSpeed(pkt.payload[0], pkt.payload[1]);
                    sendAck(CMD_SET_SERVO_SPEED);
                }
                break;
            }

            case CMD_FORCE_UPDATE: {
                /* Payload: [force_x10(2, big-endian)] - force reading from ESP32 sensor */
                if (pkt.payload_len >= 2) {
                    uint16_t force_x10 = ((uint16_t)pkt.payload[0] << 8) | pkt.payload[1];
                    float force = (float)force_x10 / 10.0f;
                    updateForceReading(force);
                }
                break;
            }

            case CMD_REQUEST_IMU: {
                /* Read latest IMU data and send to ESP32 */
                IMUData_t local_imu;
                osMutexAcquire(imuMutex, osWaitForever);
                memcpy(&local_imu, (const void *)&g_imuData, sizeof(IMUData_t));
                osMutexRelease(imuMutex);

                if (local_imu.valid) {
                    /* Pack IMU data: accel(3x int16 x10), gyro(3x int16 x10),
                     * pitch(int16 x10), roll(int16 x10), arm_tilt(int16 x10) = 18 bytes */
                    uint8_t payload[18];
                    int16_t ax = (int16_t)(local_imu.accel_x * 10.0f);
                    int16_t ay = (int16_t)(local_imu.accel_y * 10.0f);
                    int16_t az = (int16_t)(local_imu.accel_z * 10.0f);
                    int16_t gx = (int16_t)(local_imu.gyro_x * 10.0f);
                    int16_t gy = (int16_t)(local_imu.gyro_y * 10.0f);
                    int16_t gz = (int16_t)(local_imu.gyro_z * 10.0f);
                    int16_t pitch = (int16_t)(local_imu.pitch * 10.0f);
                    int16_t roll  = (int16_t)(local_imu.roll * 10.0f);
                    int16_t tilt  = (int16_t)(local_imu.arm_tilt * 10.0f);

                    payload[0]  = (uint8_t)(ax >> 8);
                    payload[1]  = (uint8_t)(ax & 0xFF);
                    payload[2]  = (uint8_t)(ay >> 8);
                    payload[3]  = (uint8_t)(ay & 0xFF);
                    payload[4]  = (uint8_t)(az >> 8);
                    payload[5]  = (uint8_t)(az & 0xFF);
                    payload[6]  = (uint8_t)(gx >> 8);
                    payload[7]  = (uint8_t)(gx & 0xFF);
                    payload[8]  = (uint8_t)(gy >> 8);
                    payload[9]  = (uint8_t)(gy & 0xFF);
                    payload[10] = (uint8_t)(gz >> 8);
                    payload[11] = (uint8_t)(gz & 0xFF);
                    payload[12] = (uint8_t)(pitch >> 8);
                    payload[13] = (uint8_t)(pitch & 0xFF);
                    payload[14] = (uint8_t)(roll >> 8);
                    payload[15] = (uint8_t)(roll & 0xFF);
                    payload[16] = (uint8_t)(tilt >> 8);
                    payload[17] = (uint8_t)(tilt & 0xFF);

                    sendResponse(RSP_IMU_DATA, payload, 18);
                } else {
                    sendError(ERR_SENSOR_FAULT);
                }
                break;
            }

            case CMD_TRIGGER_MIC_CAPTURE: {
                /* Trigger microphone capture, then wait for completion and run FFT */
                triggerTapCapture();
                sendAck(CMD_TRIGGER_MIC_CAPTURE);

                /* Wait up to 500ms for capture to complete */
                uint32_t wait_start = HAL_GetTick();
                while (!isCaptureComplete() && (HAL_GetTick() - wait_start) < 500) {
                    osDelay(5);
                }

                if (isCaptureComplete()) {
                    /* Run FFT analysis on captured data */
                    MicCapture_t local_mic;
                    uint16_t count = 0;
                    uint16_t *buf = getTapBuffer(&count);

                    if (buf != NULL && count > 0) {
                        memcpy(local_mic.samples, buf, count * sizeof(uint16_t));
                        local_mic.sample_count = count;
                        local_mic.sample_rate = MIC_SAMPLE_RATE_HZ;
                        local_mic.capture_complete = true;

                        computeBasicFFT(&local_mic);

                        /* Pack result: dominant_freq(uint16), centroid(uint16), decay_ratio(uint16 x1000) = 6 bytes */
                        uint8_t payload[6];
                        uint16_t freq = (uint16_t)local_mic.dominant_freq_hz;
                        uint16_t centroid = (uint16_t)local_mic.spectral_centroid_hz;
                        uint16_t decay = (uint16_t)(local_mic.decay_ratio * 1000.0f);

                        payload[0] = (uint8_t)(freq >> 8);
                        payload[1] = (uint8_t)(freq & 0xFF);
                        payload[2] = (uint8_t)(centroid >> 8);
                        payload[3] = (uint8_t)(centroid & 0xFF);
                        payload[4] = (uint8_t)(decay >> 8);
                        payload[5] = (uint8_t)(decay & 0xFF);

                        sendResponse(RSP_TAP_RESULT, payload, 6);
                    } else {
                        sendError(ERR_SENSOR_FAULT);
                    }
                } else {
                    sendError(ERR_SENSOR_FAULT);
                }
                break;
            }

            case CMD_MOTOR_MOVE: {
                /* Payload: [left_speed_hi(1)] [left_speed_lo(1)] [right_speed_hi(1)] [right_speed_lo(1)] */
                if (pkt.payload_len >= 4) {
                    int16_t left  = (int16_t)(((uint16_t)pkt.payload[0] << 8) | pkt.payload[1]);
                    int16_t right = (int16_t)(((uint16_t)pkt.payload[2] << 8) | pkt.payload[3]);
                    HAL_StatusTypeDef mret = enableMotors();
                    if (mret == HAL_OK) {
                        mret = sendMotorCommand(left, right);
                    }
                    if (mret == HAL_OK) {
                        sendAck(CMD_MOTOR_MOVE);
                    } else {
                        sendError(ERR_I2C_FAILURE);
                    }
                }
                break;
            }

            case CMD_MOTOR_STOP: {
                HAL_StatusTypeDef mret = stopMotors();
                if (mret == HAL_OK) {
                    disableMotors();
                    sendAck(CMD_MOTOR_STOP);
                } else {
                    sendError(ERR_I2C_FAILURE);
                }
                break;
            }

            case CMD_APPROACH_OBJECT: {
                /* Payload: [target_dist_hi(1)] [target_dist_lo(1)] */
                if (pkt.payload_len >= 2) {
                    uint16_t target = ((uint16_t)pkt.payload[0] << 8) | pkt.payload[1];
                    sendAck(CMD_APPROACH_OBJECT);
                    HAL_StatusTypeDef mret = approachObject(target);
                    if (mret == HAL_OK) {
                        /* Send encoder data after approach completes */
                        osMutexAcquire(stateMutex, osWaitForever);
                        int32_t enc_l = g_motorState.encoder_left;
                        int32_t enc_r = g_motorState.encoder_right;
                        float dist = g_motorState.distance_mm;
                        osMutexRelease(stateMutex);

                        uint8_t enc_payload[12];
                        enc_payload[0]  = (uint8_t)((uint32_t)enc_l >> 24);
                        enc_payload[1]  = (uint8_t)((uint32_t)enc_l >> 16);
                        enc_payload[2]  = (uint8_t)((uint32_t)enc_l >> 8);
                        enc_payload[3]  = (uint8_t)((uint32_t)enc_l & 0xFF);
                        enc_payload[4]  = (uint8_t)((uint32_t)enc_r >> 24);
                        enc_payload[5]  = (uint8_t)((uint32_t)enc_r >> 16);
                        enc_payload[6]  = (uint8_t)((uint32_t)enc_r >> 8);
                        enc_payload[7]  = (uint8_t)((uint32_t)enc_r & 0xFF);
                        uint32_t dist_x10 = (uint32_t)(dist * 10.0f);
                        enc_payload[8]  = (uint8_t)(dist_x10 >> 24);
                        enc_payload[9]  = (uint8_t)(dist_x10 >> 16);
                        enc_payload[10] = (uint8_t)(dist_x10 >> 8);
                        enc_payload[11] = (uint8_t)(dist_x10 & 0xFF);
                        sendResponse(RSP_ENCODER_DATA, enc_payload, 12);
                    } else {
                        sendError(ERR_I2C_FAILURE);
                    }
                }
                break;
            }

            case CMD_START_FULL_SCAN: {
                HAL_StatusTypeDef sret = startFullScan();
                if (sret == HAL_OK) {
                    sendAck(CMD_START_FULL_SCAN);
                } else if (sret == HAL_BUSY) {
                    sendError(ERR_SENSOR_FAULT); /* Scan already in progress */
                } else {
                    sendError(ERR_SENSOR_FAULT);
                }
                break;
            }

            case CMD_START_QUICK_SCAN: {
                HAL_StatusTypeDef sret = startQuickScan();
                if (sret == HAL_OK) {
                    sendAck(CMD_START_QUICK_SCAN);
                } else {
                    sendError(ERR_SENSOR_FAULT);
                }
                break;
            }

            case CMD_GET_SCAN_STATUS:
                sendScanStatus();
                break;

            case CMD_GET_SCAN_RESULT: {
                ScanResult_t *scan = getScanResult();
                if (scan->complete) {
                    sendScanResult(scan);
                } else {
                    sendError(ERR_SENSOR_FAULT);
                }
                break;
            }

            default:
                sendError(ERR_INVALID_CMD);
                break;
            }
        }

        osDelay(1); /* Yield when no data */
    }
}

/**
 * ServoTask: Execute smooth servo movements with trapezoidal profiles.
 * Priority 2. Runs at 1 ms intervals for smooth interpolation.
 */
void ServoTask(void *argument)
{
    (void)argument;

    for (;;) {
        servoTaskProcess();
        osDelay(1);
    }
}

/**
 * DepthTask: Read VL53L8CX at ~5Hz and store in global depth grid.
 * Priority 1 (lowest). Automatically forwards data to ESP32 on each read.
 */
void DepthTask(void *argument)
{
    (void)argument;

    /* Initialize the ToF sensor (hold depthMutex to prevent I2C bus collision with IMUTask) */
    osMutexAcquire(depthMutex, osWaitForever);
    HAL_StatusTypeDef ret = initVL53L8CX();
    osMutexRelease(depthMutex);
    if (ret == HAL_OK) {
        osMutexAcquire(stateMutex, osWaitForever);
        g_systemState.system_status |= STATUS_TOF_OK;
        osMutexRelease(stateMutex);
    } else {
        osMutexAcquire(stateMutex, osWaitForever);
        g_systemState.system_status &= ~STATUS_TOF_OK;
        osMutexRelease(stateMutex);
    }

    for (;;) {
        DepthGrid_t local_grid;

        /* Acquire depthMutex for the entire I2C + data copy operation.
         * This mutex also protects I2C1 bus access shared with IMUTask. */
        osMutexAcquire(depthMutex, osWaitForever);
        memcpy(&local_grid, (const void *)&g_depthGrid, sizeof(DepthGrid_t));
        ret = readDepthGrid(&local_grid);

        if (ret == HAL_OK && local_grid.valid) {
            memcpy((void *)&g_depthGrid, &local_grid, sizeof(DepthGrid_t));
        }
        osMutexRelease(depthMutex);

        if (ret == HAL_OK && local_grid.valid) {

            forwardDepthData(&local_grid);

            osMutexAcquire(stateMutex, osWaitForever);
            g_systemState.system_status |= STATUS_TOF_OK;
            osMutexRelease(stateMutex);
        } else if (ret != HAL_OK) {
            osMutexAcquire(stateMutex, osWaitForever);
            g_systemState.system_status &= ~STATUS_TOF_OK;
            osMutexRelease(stateMutex);
        }

        osDelay(DEPTH_READ_PERIOD_MS);
    }
}

/**
 * IMUTask: Read LSM6DSV16BX at 50Hz and store in global IMU data.
 * Priority 2. Reads accelerometer + gyroscope, computes orientation.
 */
void IMUTask(void *argument)
{
    (void)argument;

    /* Initialize the IMU sensor */
    HAL_StatusTypeDef ret = initIMU();
    if (ret == HAL_OK) {
        osMutexAcquire(stateMutex, osWaitForever);
        g_systemState.system_status |= STATUS_IMU_OK;
        osMutexRelease(stateMutex);
    } else {
        osMutexAcquire(stateMutex, osWaitForever);
        g_systemState.system_status &= ~STATUS_IMU_OK;
        osMutexRelease(stateMutex);
    }

    for (;;) {
        IMUData_t local_imu;

        osMutexAcquire(imuMutex, osWaitForever);
        memcpy(&local_imu, (const void *)&g_imuData, sizeof(IMUData_t));
        osMutexRelease(imuMutex);

        ret = readIMU(&local_imu);

        if (ret == HAL_OK && local_imu.valid) {
            osMutexAcquire(imuMutex, osWaitForever);
            memcpy((void *)&g_imuData, &local_imu, sizeof(IMUData_t));
            osMutexRelease(imuMutex);

            osMutexAcquire(stateMutex, osWaitForever);
            g_systemState.system_status |= STATUS_IMU_OK;
            osMutexRelease(stateMutex);
        } else if (ret != HAL_OK) {
            osMutexAcquire(stateMutex, osWaitForever);
            g_systemState.system_status &= ~STATUS_IMU_OK;
            osMutexRelease(stateMutex);
        }

        osDelay(IMU_READ_PERIOD_MS);
    }
}

/**
 * MotorTask: Periodic encoder readback and motor status updates.
 * Priority 3. Reads encoder data from the motor board via I2C at
 * ENCODER_SAMPLE_PERIOD_MS intervals.
 */
void MotorTask(void *argument)
{
    (void)argument;

    /* Initialize motor subsystem (probe motor board on I2C1) */
    HAL_StatusTypeDef ret = initMotors();
    if (ret == HAL_OK) {
        osMutexAcquire(stateMutex, osWaitForever);
        g_systemState.system_status |= STATUS_MOTORS_OK;
        osMutexRelease(stateMutex);
    } else {
        osMutexAcquire(stateMutex, osWaitForever);
        g_systemState.system_status &= ~STATUS_MOTORS_OK;
        osMutexRelease(stateMutex);
    }

    for (;;) {
        motorTaskProcess();

        /* Periodically send motor status to ESP32 */
        osMutexAcquire(stateMutex, osWaitForever);
        bool motor_enabled = g_motorState.enabled;
        int16_t left_spd   = g_motorState.speed_left;
        int16_t right_spd  = g_motorState.speed_right;
        osMutexRelease(stateMutex);

        /* Only send status if motors are active */
        if (motor_enabled && (left_spd != 0 || right_spd != 0)) {
            uint8_t status_payload[5];
            status_payload[0] = (uint8_t)((uint16_t)left_spd >> 8);
            status_payload[1] = (uint8_t)((uint16_t)left_spd & 0xFF);
            status_payload[2] = (uint8_t)((uint16_t)right_spd >> 8);
            status_payload[3] = (uint8_t)((uint16_t)right_spd & 0xFF);
            status_payload[4] = motor_enabled ? 0x01 : 0x00;
            sendResponse(RSP_MOTOR_STATUS, status_payload, 5);
        }

        osDelay(ENCODER_SAMPLE_PERIOD_MS);
    }
}

/**
 * BLETask: Handle BLE communication with mobile app.
 * Priority 2. Processes incoming BLE commands and sends periodic
 * status notifications to the connected phone.
 */
void BLETask(void *argument)
{
    (void)argument;

    /* Initialize BLE module */
    HAL_StatusTypeDef ret = initBLE();
    if (ret != HAL_OK) {
        /* BLE init failed. System continues without BLE.
         * Retry periodically in the task loop. */
    }

    for (;;) {
        bleTaskProcess();
        osDelay(10); /* 100 Hz BLE processing */
    }
}

/**
 * ScannerTask: 3D scanning via photometric stereo + ToF depth fusion.
 * Priority below DepthTask. Runs the scanner state machine.
 * Stack needs 4KB for float-heavy normal map computation and point cloud.
 */
void ScannerTask(void *argument)
{
    scannerTask(argument);
}

/**
 * SafetyTask: Watchdog refresh, safety limit checks, E-STOP monitoring.
 * Priority 7 (highest). Must never be starved.
 */
void SafetyTask(void *argument)
{
    (void)argument;

    for (;;) {
        kickWatchdog();
        checkSafetyLimits();
        osDelay(SAFETY_CHECK_PERIOD_MS);
    }
}

/* ========================================================================== */
/*                       MAIN ENTRY POINT                                      */
/* ========================================================================== */

int main(void)
{
    /* HAL and system initialization */
    HAL_Init();
    SystemClock_Config();  /* CubeMX-generated, declared extern */

    /* Peripheral initialization */
    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_USART2_Init();
    MX_I2C1_Init();

    /* RTOS kernel initialization */
    osKernelInitialize();

    /* Create mutexes */
    const osMutexAttr_t mutexAttr = {
        .name = "GenericMutex",
        .attr_bits = osMutexRecursive | osMutexPrioInherit,
        .cb_mem = NULL,
        .cb_size = 0,
    };

    servoMutex = osMutexNew(&mutexAttr);
    depthMutex = osMutexNew(&mutexAttr);
    stateMutex = osMutexNew(&mutexAttr);
    imuMutex   = osMutexNew(&mutexAttr);

    if (servoMutex == NULL || depthMutex == NULL || stateMutex == NULL || imuMutex == NULL) {
        Error_Handler();
    }

    /* Initialize subsystems (after mutexes are created) */
    initUART();
    initServos();
    initSafety();

    /* Initialize microphone (ADC3 + DMA + TIM6).
     * IMU init happens in IMUTask to share depthMutex safely after RTOS starts. */
    HAL_StatusTypeDef mic_ret = initMicrophone();
    if (mic_ret == HAL_OK) {
        g_systemState.system_status |= STATUS_MIC_OK;
    }

    /* Initialize watchdog last (after all init is done to avoid premature reset) */
    MX_IWDG_Init();

    /* Create FreeRTOS tasks (priority 7 = highest, 1 = lowest) */
    const osThreadAttr_t safetyTaskAttr = {
        .name = "SafetyTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityRealtime,
    };
    s_safetyTaskHandle = osThreadNew(SafetyTask, NULL, &safetyTaskAttr);

    const osThreadAttr_t commandTaskAttr = {
        .name = "CommandTask",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityAboveNormal3,
    };
    s_commandTaskHandle = osThreadNew(CommandTask, NULL, &commandTaskAttr);

    const osThreadAttr_t servoTaskAttr = {
        .name = "ServoTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityAboveNormal2,
    };
    s_servoTaskHandle = osThreadNew(ServoTask, NULL, &servoTaskAttr);

    const osThreadAttr_t imuTaskAttr = {
        .name = "IMUTask",
        .stack_size = 1024,  /* Stack for float orientation math */
        .priority = (osPriority_t) osPriorityAboveNormal1,
    };
    s_imuTaskHandle = osThreadNew(IMUTask, NULL, &imuTaskAttr);

    const osThreadAttr_t motorTaskAttr = {
        .name = "MotorTask",
        .stack_size = 1024,  /* Stack for motor control + approach logic */
        .priority = (osPriority_t) osPriorityAboveNormal,
    };
    s_motorTaskHandle = osThreadNew(MotorTask, NULL, &motorTaskAttr);

    const osThreadAttr_t bleTaskAttr = {
        .name = "BLETask",
        .stack_size = 1024,  /* Stack for BLE AT commands + data processing */
        .priority = (osPriority_t) osPriorityNormal1,
    };
    s_bleTaskHandle = osThreadNew(BLETask, NULL, &bleTaskAttr);

    const osThreadAttr_t depthTaskAttr = {
        .name = "DepthTask",
        .stack_size = 2048,  /* Larger stack for depth buffer processing */
        .priority = (osPriority_t) osPriorityNormal,
    };
    s_depthTaskHandle = osThreadNew(DepthTask, NULL, &depthTaskAttr);

    const osThreadAttr_t scannerTaskAttr = {
        .name = "ScannerTask",
        .stack_size = 4096,  /* Large stack for float-heavy 3D math + point cloud */
        .priority = (osPriority_t) osPriorityBelowNormal,
    };
    s_scannerTaskHandle = osThreadNew(ScannerTask, NULL, &scannerTaskAttr);

    if (s_safetyTaskHandle  == NULL || s_commandTaskHandle == NULL ||
        s_servoTaskHandle   == NULL || s_depthTaskHandle   == NULL ||
        s_imuTaskHandle     == NULL || s_motorTaskHandle   == NULL ||
        s_bleTaskHandle     == NULL || s_scannerTaskHandle == NULL) {
        Error_Handler();
    }

    /* Start the RTOS scheduler - does not return */
    osKernelStart();

    /* Should never reach here */
    for (;;) {}
}

/* ========================================================================== */
/*                       ERROR HANDLER                                         */
/* ========================================================================== */

void Error_Handler(void)
{
    __disable_irq();

    /* Rapid LED blink to indicate fault */
    for (;;) {
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
        /* Crude delay without SysTick (interrupts disabled) */
        for (volatile uint32_t i = 0; i < 2000000; i++) {
            __NOP();
        }
    }
}

/* ========================================================================== */
/*                       HAL REQUIRED CALLBACKS                                */
/* ========================================================================== */

/**
 * SysTick handler for HAL timebase. When using FreeRTOS, this is typically
 * handled by the RTOS, but HAL_IncTick() must still be called.
 * If using FreeRTOS with CMSIS-OS2, the SysTick is usually managed by the
 * RTOS port. Include this for standalone HAL init before RTOS starts.
 */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* System interrupt priority grouping: 4 bits preemption, 0 bits sub-priority.
     * Required for FreeRTOS on Cortex-M7. */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* MemoryManagement_IRQn, BusFault_IRQn, UsageFault_IRQn */
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);

    /* SVC, PendSV, SysTick priorities set by FreeRTOS port */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
