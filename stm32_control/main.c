/**
 * STM32H725 Gripper Controller - Main Application
 * Target: STEVAL-ROBKIT1 (STM32H725IGT6)
 *
 * FreeRTOS tasks:
 *   SafetyTask   (priority 4 - highest) - Watchdog, force limits, E-STOP
 *   CommandTask  (priority 3)           - Parse UART commands from ESP32
 *   ServoTask    (priority 2)           - Smooth servo interpolation
 *   DepthTask    (priority 1 - lowest)  - VL53L8CX 5Hz depth reads
 *
 * Peripheral mapping:
 *   USART2 (PD5/PD6)  - ESP32 communication, 115200 baud
 *   TIM3 (PC6-PC9)    - 4-channel servo PWM, 50Hz
 *   I2C1 (PB8/PB9)    - VL53L8CX ToF sensor
 *   IWDG              - 2-second independent watchdog
 */

#include "main.h"
#include "servo_control.h"
#include "depth_sensor.h"
#include "uart_protocol.h"
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
osMessageQueueId_t  cmdQueue;

volatile SystemState_t  g_systemState;
volatile ServoState_t   g_servoState;
volatile DepthGrid_t    g_depthGrid;

/* Task handles */
static osThreadId_t s_commandTaskHandle;
static osThreadId_t s_servoTaskHandle;
static osThreadId_t s_depthTaskHandle;
static osThreadId_t s_safetyTaskHandle;

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

    /* Initialize the ToF sensor */
    HAL_StatusTypeDef ret = initVL53L8CX();
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

        osMutexAcquire(depthMutex, osWaitForever);
        memcpy(&local_grid, (const void *)&g_depthGrid, sizeof(DepthGrid_t));
        osMutexRelease(depthMutex);

        ret = readDepthGrid(&local_grid);

        if (ret == HAL_OK && local_grid.valid) {
            osMutexAcquire(depthMutex, osWaitForever);
            memcpy((void *)&g_depthGrid, &local_grid, sizeof(DepthGrid_t));
            osMutexRelease(depthMutex);

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
 * SafetyTask: Watchdog refresh, safety limit checks, E-STOP monitoring.
 * Priority 4 (highest). Must never be starved.
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

    if (servoMutex == NULL || depthMutex == NULL || stateMutex == NULL) {
        Error_Handler();
    }

    /* Initialize subsystems (after mutexes are created) */
    initUART();
    initServos();
    initSafety();

    /* Initialize watchdog last (after all init is done to avoid premature reset) */
    MX_IWDG_Init();

    /* Create FreeRTOS tasks */
    const osThreadAttr_t safetyTaskAttr = {
        .name = "SafetyTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityAboveNormal3,
    };
    s_safetyTaskHandle = osThreadNew(SafetyTask, NULL, &safetyTaskAttr);

    const osThreadAttr_t commandTaskAttr = {
        .name = "CommandTask",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityAboveNormal2,
    };
    s_commandTaskHandle = osThreadNew(CommandTask, NULL, &commandTaskAttr);

    const osThreadAttr_t servoTaskAttr = {
        .name = "ServoTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityAboveNormal1,
    };
    s_servoTaskHandle = osThreadNew(ServoTask, NULL, &servoTaskAttr);

    const osThreadAttr_t depthTaskAttr = {
        .name = "DepthTask",
        .stack_size = 2048,  /* Larger stack for depth buffer processing */
        .priority = (osPriority_t) osPriorityAboveNormal,
    };
    s_depthTaskHandle = osThreadNew(DepthTask, NULL, &depthTaskAttr);

    if (s_safetyTaskHandle == NULL || s_commandTaskHandle == NULL ||
        s_servoTaskHandle == NULL  || s_depthTaskHandle == NULL) {
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
