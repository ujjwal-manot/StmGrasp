/**
 * Safety Systems Implementation
 * Independent Watchdog (IWDG), force limits, comms timeout, emergency stop.
 *
 * The IWDG on STM32H7 runs from the 32 kHz LSI oscillator.
 * Prescaler = 64 => 500 Hz tick. Reload = 1000 => 2 second timeout.
 */

#include "safety.h"
#include "servo_control.h"
#include "uart_protocol.h"

/* -------------------------------------------------------------------------- */
/*  Init                                                                       */
/* -------------------------------------------------------------------------- */

void initSafety(void)
{
    osMutexAcquire(stateMutex, osWaitForever);
    g_systemState.estop_active       = false;
    g_systemState.esp32_connected    = false;
    g_systemState.force_limit_n      = MAX_FORCE_DEFAULT_N;
    g_systemState.current_force_n    = 0.0f;
    g_systemState.last_heartbeat_tick = HAL_GetTick();
    g_systemState.system_status      = 0;
    osMutexRelease(stateMutex);

    /* Check hardware E-STOP pin at startup */
    if (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_RESET) {
        emergencyStop();
    }
}

/* -------------------------------------------------------------------------- */
/*  Watchdog (IWDG) initialization                                             */
/*  Called after RTOS is running to avoid false resets during init.             */
/* -------------------------------------------------------------------------- */

/* MX_IWDG_Init is in main.c (uses HAL handle). This function kicks it. */
void kickWatchdog(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

/* -------------------------------------------------------------------------- */
/*  Periodic safety check (called every SAFETY_CHECK_PERIOD_MS from SafetyTask)*/
/* -------------------------------------------------------------------------- */

void checkSafetyLimits(void)
{
    uint8_t status = 0;
    bool trigger_estop = false;

    /* --- 1. Check hardware E-STOP button (active low) --- */
    if (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_RESET) {
        trigger_estop = true;
    }

    /* --- 2. Check servo positions within travel limits --- */
    bool servos_ok = true;
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        float pos = getServoPosition(i);
        if (pos < (float)SERVO_TRAVEL_MIN_DEG - 1.0f ||
            pos > (float)SERVO_TRAVEL_MAX_DEG + 1.0f) {
            servos_ok = false;
        }
    }
    if (servos_ok) {
        status |= STATUS_SERVOS_OK;
    }

    /* --- 3. Check force limits --- */
    osMutexAcquire(stateMutex, osWaitForever);
    float force = g_systemState.current_force_n;
    float limit = g_systemState.force_limit_n;
    osMutexRelease(stateMutex);

    if (force > limit || force > MAX_FORCE_ABSOLUTE_N) {
        /* Over-force: open gripper immediately */
        openGripper();
        sendError(ERR_FORCE_EXCEEDED);

        if (force > MAX_FORCE_ABSOLUTE_N) {
            trigger_estop = true;
        }
    }

    /* --- 4. Communication timeout (ESP32 heartbeat) --- */
    osMutexAcquire(stateMutex, osWaitForever);
    uint32_t last_hb = g_systemState.last_heartbeat_tick;
    osMutexRelease(stateMutex);

    uint32_t now = HAL_GetTick();
    bool comms_ok;

    if ((now - last_hb) < HEARTBEAT_TIMEOUT_MS) {
        comms_ok = true;
        status |= STATUS_COMMS_OK;
    } else {
        comms_ok = false;
        /* Lost communication: safe state = open gripper */
        openGripper();
        sendError(ERR_COMMS_TIMEOUT);
    }

    /* --- 5. Update system state (preserve TOF_OK bit managed by DepthTask) --- */
    osMutexAcquire(stateMutex, osWaitForever);
    g_systemState.esp32_connected = comms_ok;
    if (!trigger_estop) {
        status |= STATUS_SAFETY_OK;
    }
    /* Preserve STATUS_TOF_OK from DepthTask */
    status |= (g_systemState.system_status & STATUS_TOF_OK);
    g_systemState.system_status = status;
    osMutexRelease(stateMutex);

    /* --- 6. Trigger E-STOP if needed --- */
    if (trigger_estop && !isEstopActive()) {
        emergencyStop();
    }

    /* --- 7. Blink status LED: solid if OK, toggle if fault --- */
    if (status == (STATUS_SERVOS_OK | STATUS_COMMS_OK | STATUS_SAFETY_OK | STATUS_TOF_OK)) {
        HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
    }
}

/* -------------------------------------------------------------------------- */
/*  Emergency stop                                                             */
/* -------------------------------------------------------------------------- */

void emergencyStop(void)
{
    /* Disable all servo PWM outputs */
    disableAllServos();

    /* Open gripper via direct register write (bypass smooth move) */
    uint32_t ccr_open = angleToCCR((float)GRIPPER_OPEN_ANGLE);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr_open);

    osMutexAcquire(stateMutex, osWaitForever);
    g_systemState.estop_active = true;
    g_systemState.system_status |= STATUS_ESTOP;
    osMutexRelease(stateMutex);

    osMutexAcquire(servoMutex, osWaitForever);
    g_servoState.current_angle[GRIPPER_CHANNEL] = (float)GRIPPER_OPEN_ANGLE;
    g_servoState.target_angle[GRIPPER_CHANNEL]  = (float)GRIPPER_OPEN_ANGLE;
    osMutexRelease(servoMutex);

    /* Notify ESP32 */
    sendError(ERR_ESTOP_ACTIVE);
}

/* -------------------------------------------------------------------------- */
/*  Force limit management                                                     */
/* -------------------------------------------------------------------------- */

void setForceLimit(float max_force_n)
{
    if (max_force_n <= 0.0f) max_force_n = 1.0f;
    if (max_force_n > MAX_FORCE_ABSOLUTE_N) max_force_n = MAX_FORCE_ABSOLUTE_N;

    osMutexAcquire(stateMutex, osWaitForever);
    g_systemState.force_limit_n = max_force_n;
    osMutexRelease(stateMutex);
}

void updateForceReading(float force_n)
{
    if (force_n < 0.0f) force_n = 0.0f;

    osMutexAcquire(stateMutex, osWaitForever);
    g_systemState.current_force_n = force_n;
    osMutexRelease(stateMutex);
}

void recordHeartbeat(void)
{
    osMutexAcquire(stateMutex, osWaitForever);
    g_systemState.last_heartbeat_tick = HAL_GetTick();
    g_systemState.esp32_connected = true;
    osMutexRelease(stateMutex);
}

bool isEstopActive(void)
{
    osMutexAcquire(stateMutex, osWaitForever);
    bool active = g_systemState.estop_active;
    osMutexRelease(stateMutex);
    return active;
}
