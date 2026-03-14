/**
 * BLE Communication Implementation
 * UART-to-BLE module driver with AT command initialization and
 * binary data protocol for mobile app communication.
 *
 * USART3 on PD8 (TX) / PD9 (RX) connects to the onboard BLE module.
 * Initialization uses AT commands to set the device name and
 * advertising parameters. After init, the module enters transparent
 * data mode where bytes sent over UART are transmitted via BLE
 * notifications, and bytes received from the phone appear on UART RX.
 *
 * Packet format (both directions):
 * [0xBB] [LEN] [CMD/NOTIFY] [PAYLOAD...] [XOR_CHECKSUM]
 * LEN = bytes after LEN (cmd + payload + checksum).
 */

#include "ble_comm.h"
#include "motor_control.h"
#include "servo_control.h"
#include "safety.h"
#include "uart_protocol.h"

/* ========================================================================== */
/*                       STATIC STATE                                          */
/* ========================================================================== */

UART_HandleTypeDef huart3;

static BLERingBuffer_t s_bleRxRing;
static BLEParseState_t s_bleParseState;
static uint8_t         s_bleParseLen;
static uint8_t         s_bleParseBuf[BLE_TX_BUF_SIZE];
static uint8_t         s_bleParseIdx;
static uint8_t         s_bleRxByte;   /* Single-byte IT target */
static uint32_t        s_lastStatusTick;
static uint32_t        s_lastConnCheckTick;

/* ========================================================================== */
/*  Ring buffer helpers                                                        */
/* ========================================================================== */

static void bleRingBufReset(BLERingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

static bool bleRingBufPut(BLERingBuffer_t *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1U) % BLE_RX_BUF_SIZE;
    if (next == rb->tail) {
        return false; /* Full */
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}

static bool bleRingBufGet(BLERingBuffer_t *rb, uint8_t *byte)
{
    if (rb->head == rb->tail) {
        return false; /* Empty */
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1U) % BLE_RX_BUF_SIZE;
    return true;
}

/* ========================================================================== */
/*  Checksum                                                                   */
/* ========================================================================== */

static uint8_t bleComputeChecksum(const uint8_t *data, uint8_t len)
{
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < len; i++) {
        xor_val ^= data[i];
    }
    return xor_val;
}

/* ========================================================================== */
/*  USART3 peripheral initialization                                           */
/* ========================================================================== */

/**
 * USART3 configuration: 115200, 8N1, interrupt RX.
 * GPIO pins PD8 (TX) and PD9 (RX) configured as AF7.
 */
void MX_USART3_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Enable clocks */
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();  /* Already enabled in MX_GPIO_Init, safe to call again */

    /* USART3 TX: PD8 */
    gpio.Pin       = BLE_UART_TX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = BLE_UART_AF;
    HAL_GPIO_Init(BLE_UART_TX_PORT, &gpio);

    /* USART3 RX: PD9 */
    gpio.Pin       = BLE_UART_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = BLE_UART_AF;
    HAL_GPIO_Init(BLE_UART_RX_PORT, &gpio);

    /* USART3 configuration */
    huart3.Instance          = BLE_UART_INSTANCE;
    huart3.Init.BaudRate     = BLE_UART_BAUDRATE;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }

    /* Enable USART3 interrupt */
    HAL_NVIC_SetPriority(BLE_UART_IRQn, BLE_UART_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(BLE_UART_IRQn);
}

/* ========================================================================== */
/*  USART3 IRQ handler                                                         */
/* ========================================================================== */

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

/* ========================================================================== */
/*  ISR callback                                                               */
/* ========================================================================== */

void bleRxIrqHandler(void)
{
    bleRingBufPut(&s_bleRxRing, s_bleRxByte);
    /* Re-arm reception for next byte */
    HAL_UART_Receive_IT(&huart3, &s_bleRxByte, 1);
}

/* ========================================================================== */
/*  AT command interface                                                       */
/* ========================================================================== */

/**
 * Send an AT command string to the BLE module and wait for "OK" response.
 * The AT command should NOT include the trailing \r\n; this function adds it.
 * Returns HAL_OK if "OK" received within timeout, HAL_TIMEOUT otherwise.
 */
HAL_StatusTypeDef bleSendAT(const char *cmd, uint32_t timeout_ms)
{
    /* Build AT command with CRLF */
    uint8_t at_buf[64];
    uint8_t cmd_len = 0;

    while (cmd[cmd_len] != '\0' && cmd_len < 60) {
        at_buf[cmd_len] = (uint8_t)cmd[cmd_len];
        cmd_len++;
    }
    at_buf[cmd_len++] = '\r';
    at_buf[cmd_len++] = '\n';

    /* Flush any pending RX data before sending */
    uint8_t discard;
    while (bleRingBufGet(&s_bleRxRing, &discard)) {
        /* Drain the buffer */
    }

    /* Transmit the AT command */
    HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart3, at_buf, cmd_len, 100);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Wait for response: look for 'O' followed by 'K' in the RX ring */
    uint32_t start = HAL_GetTick();
    uint8_t resp_buf[32];
    uint8_t resp_idx = 0;

    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t byte;
        if (bleRingBufGet(&s_bleRxRing, &byte)) {
            if (resp_idx < sizeof(resp_buf) - 1) {
                resp_buf[resp_idx++] = byte;
            }

            /* Check for "OK" in response */
            if (resp_idx >= 2 &&
                resp_buf[resp_idx - 2] == 'O' &&
                resp_buf[resp_idx - 1] == 'K') {
                return HAL_OK;
            }

            /* Check for "ERROR" in response */
            if (resp_idx >= 5 &&
                resp_buf[resp_idx - 5] == 'E' &&
                resp_buf[resp_idx - 4] == 'R' &&
                resp_buf[resp_idx - 3] == 'R' &&
                resp_buf[resp_idx - 2] == 'O' &&
                resp_buf[resp_idx - 1] == 'R') {
                return HAL_ERROR;
            }
        } else {
            osDelay(5);
        }
    }

    return HAL_TIMEOUT;
}

/* ========================================================================== */
/*  Initialization                                                             */
/* ========================================================================== */

/**
 * Initialize BLE module: configure USART3, send AT commands to set
 * device name and advertising mode, then enter data mode.
 */
HAL_StatusTypeDef initBLE(void)
{
    /* Initialize USART3 peripheral and GPIO */
    MX_USART3_Init();

    /* Reset ring buffer and parser state */
    bleRingBufReset(&s_bleRxRing);
    s_bleParseState = BLE_PARSE_WAIT_HEADER;
    s_bleParseIdx   = 0;
    s_lastStatusTick    = HAL_GetTick();
    s_lastConnCheckTick = HAL_GetTick();

    /* Start interrupt-driven reception */
    HAL_UART_Receive_IT(&huart3, &s_bleRxByte, 1);

    /* Initialize global BLE state */
    osMutexAcquire(stateMutex, osWaitForever);
    g_bleState.initialized = false;
    g_bleState.connected   = false;
    g_bleState.rx_len      = 0;
    memset(g_bleState.rx_buffer, 0, sizeof(g_bleState.rx_buffer));
    osMutexRelease(stateMutex);

    /* Allow BLE module to boot */
    osDelay(500);

    /* Send AT initialization commands */
    HAL_StatusTypeDef ret;

    /* Test basic AT communication */
    ret = bleSendAT("AT", BLE_AT_TIMEOUT_MS);
    if (ret != HAL_OK) {
        /* Module may not respond to AT if already in data mode.
         * Try sending +++ to exit data mode first, then retry. */
        uint8_t exit_cmd[] = "+++";
        HAL_UART_Transmit(&huart3, exit_cmd, 3, 100);
        osDelay(1000);

        ret = bleSendAT("AT", BLE_AT_TIMEOUT_MS);
        if (ret != HAL_OK) {
            /* Module not responding. Mark as not initialized but don't fail
             * hard - the system can still operate without BLE. */
            return ret;
        }
    }

    /* Set device name */
    ret = bleSendAT("AT+NAME=" BLE_DEVICE_NAME, BLE_AT_TIMEOUT_MS);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Set role to peripheral (slave) */
    ret = bleSendAT("AT+ROLE=0", BLE_AT_TIMEOUT_MS);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Set advertising interval (100ms units, 1 = 100ms) */
    ret = bleSendAT("AT+ADVI=1", BLE_AT_TIMEOUT_MS);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Set TX power to maximum for range */
    ret = bleSendAT("AT+POWE=3", BLE_AT_TIMEOUT_MS);
    if (ret != HAL_OK) {
        /* Non-critical, continue */
    }

    /* Enter data/transparent mode */
    ret = bleSendAT("AT+MODE=0", BLE_AT_TIMEOUT_MS);
    if (ret != HAL_OK) {
        /* Non-critical for some modules, continue */
    }

    /* Copy device name into state */
    osMutexAcquire(stateMutex, osWaitForever);
    strncpy(g_bleState.device_name, BLE_DEVICE_NAME, sizeof(g_bleState.device_name) - 1);
    g_bleState.device_name[sizeof(g_bleState.device_name) - 1] = '\0';
    g_bleState.initialized = true;
    osMutexRelease(stateMutex);

    return HAL_OK;
}

/* ========================================================================== */
/*  Connection management                                                      */
/* ========================================================================== */

/**
 * Start or stop BLE advertising.
 */
HAL_StatusTypeDef bleAdvertise(bool enable)
{
    if (enable) {
        return bleSendAT("AT+ADVEN=1", BLE_AT_TIMEOUT_MS);
    } else {
        return bleSendAT("AT+ADVEN=0", BLE_AT_TIMEOUT_MS);
    }
}

/**
 * Check if a BLE client (phone) is currently connected.
 * Uses the AT+STATE? query. Some modules also indicate via a GPIO pin.
 * For transparent-mode modules, we infer connection from recent data activity.
 */
bool bleIsConnected(void)
{
    osMutexAcquire(stateMutex, osWaitForever);
    bool conn = g_bleState.connected;
    osMutexRelease(stateMutex);
    return conn;
}

/* ========================================================================== */
/*  Data send (notifications to phone)                                         */
/* ========================================================================== */

/**
 * Internal helper: send a BLE data packet over USART3.
 * Format: [0xBB] [LEN] [NOTIFY_ID] [PAYLOAD...] [XOR_CHECKSUM]
 */
static HAL_StatusTypeDef bleSendPacket(uint8_t notify_id, const uint8_t *payload, uint8_t len)
{
    uint8_t txbuf[BLE_TX_BUF_SIZE];
    uint8_t total_len = len + 2; /* NOTIFY_ID + payload + checksum */

    if (total_len > (BLE_TX_BUF_SIZE - 2)) {
        return HAL_ERROR;
    }

    txbuf[0] = BLE_PACKET_HEADER;
    txbuf[1] = total_len;
    txbuf[2] = notify_id;

    if (len > 0 && payload != NULL) {
        memcpy(&txbuf[3], payload, len);
    }

    /* XOR checksum over LEN + NOTIFY_ID + PAYLOAD */
    uint8_t cksum = bleComputeChecksum(&txbuf[1], 1 + 1 + len);
    txbuf[3 + len] = cksum;

    uint8_t frame_len = 4 + len;
    return HAL_UART_Transmit(&huart3, txbuf, frame_len, 50);
}

/**
 * Send current system status over BLE notification.
 * Compact binary format: [STATE][MATERIAL][FORCE_x10_HI][FORCE_x10_LO]
 *                        [QUALITY][MOTOR_LEFT][MOTOR_RIGHT]
 */
HAL_StatusTypeDef bleSendStatus(void)
{
    uint8_t payload[7];

    osMutexAcquire(stateMutex, osWaitForever);
    payload[0] = g_systemState.system_status;
    payload[1] = g_systemState.estop_active ? 0xFF : 0x00;

    uint16_t force_x10 = (uint16_t)(g_systemState.current_force_n * 10.0f);
    payload[2] = (uint8_t)(force_x10 >> 8);
    payload[3] = (uint8_t)(force_x10 & 0xFF);
    payload[4] = g_systemState.esp32_connected ? 0x01 : 0x00;

    payload[5] = (uint8_t)((g_motorState.speed_left + 100) & 0xFF);  /* Offset to unsigned */
    payload[6] = (uint8_t)((g_motorState.speed_right + 100) & 0xFF);
    osMutexRelease(stateMutex);

    return bleSendPacket(BLE_NOTIFY_STATUS, payload, 7);
}

/**
 * Send ACK for a received BLE command.
 */
HAL_StatusTypeDef bleSendAck(uint8_t cmd_id)
{
    return bleSendPacket(BLE_NOTIFY_ACK, &cmd_id, 1);
}

/**
 * Send error notification over BLE.
 */
HAL_StatusTypeDef bleSendError(uint8_t err_code)
{
    return bleSendPacket(BLE_NOTIFY_ERROR, &err_code, 1);
}

/**
 * Send sensor data notification.
 * Format: [STATE] [MATERIAL] [FORCE_HI] [FORCE_LO] [QUALITY]
 */
HAL_StatusTypeDef bleSendSensorData(uint8_t state, uint8_t material,
                                     uint16_t force_x10, uint8_t quality)
{
    uint8_t payload[5];
    payload[0] = state;
    payload[1] = material;
    payload[2] = (uint8_t)(force_x10 >> 8);
    payload[3] = (uint8_t)(force_x10 & 0xFF);
    payload[4] = quality;

    return bleSendPacket(BLE_NOTIFY_SENSOR_DATA, payload, 5);
}

/* ========================================================================== */
/*  Command processing (from phone)                                            */
/* ========================================================================== */

/**
 * Parse incoming BLE data and dispatch commands.
 * Returns true if a command was successfully parsed and dispatched.
 *
 * Packet format: [0xBB] [LEN] [CMD] [PAYLOAD...] [XOR_CHECKSUM]
 */
bool bleProcessCommand(void)
{
    uint8_t byte;

    while (bleRingBufGet(&s_bleRxRing, &byte)) {
        switch (s_bleParseState) {

        case BLE_PARSE_WAIT_HEADER:
            if (byte == BLE_PACKET_HEADER) {
                s_bleParseState = BLE_PARSE_WAIT_LENGTH;
            }
            /* Non-header bytes might indicate connection activity */
            osMutexAcquire(stateMutex, osWaitForever);
            g_bleState.connected = true;
            osMutexRelease(stateMutex);
            break;

        case BLE_PARSE_WAIT_LENGTH:
            if (byte < 2 || byte > (BLE_TX_BUF_SIZE - 2)) {
                s_bleParseState = BLE_PARSE_WAIT_HEADER;
            } else {
                s_bleParseLen       = byte;
                s_bleParseIdx       = 0;
                s_bleParseBuf[0]    = byte; /* Store LEN for checksum calc */
                s_bleParseState     = BLE_PARSE_WAIT_DATA;
            }
            break;

        case BLE_PARSE_WAIT_DATA:
            s_bleParseBuf[1 + s_bleParseIdx] = byte;
            s_bleParseIdx++;

            if (s_bleParseIdx >= s_bleParseLen) {
                /* Complete packet: validate checksum */
                uint8_t received_cksum = s_bleParseBuf[s_bleParseLen];
                uint8_t calc_cksum = bleComputeChecksum(s_bleParseBuf, s_bleParseLen);

                if (calc_cksum == received_cksum) {
                    uint8_t cmd = s_bleParseBuf[1];
                    uint8_t payload_len = s_bleParseLen - 2;
                    uint8_t *payload = &s_bleParseBuf[2];

                    /* Dispatch the BLE command */
                    switch (cmd) {

                    case CMD_BLE_START_GRASP:
                        if (!isEstopActive()) {
                            closeGripper();
                            bleSendAck(CMD_BLE_START_GRASP);
                        } else {
                            bleSendError(ERR_ESTOP_ACTIVE);
                        }
                        break;

                    case CMD_BLE_RELEASE:
                        openGripper();
                        bleSendAck(CMD_BLE_RELEASE);
                        break;

                    case CMD_BLE_MOVE_FORWARD: {
                        if (!isEstopActive()) {
                            /* Payload: [speed(1)] [distance_mm_hi(1)] [distance_mm_lo(1)] */
                            int16_t speed = 30; /* Default speed */
                            float dist = 100.0f; /* Default 100mm */
                            if (payload_len >= 1) {
                                speed = (int16_t)payload[0];
                            }
                            if (payload_len >= 3) {
                                uint16_t d = ((uint16_t)payload[1] << 8) | payload[2];
                                dist = (float)d;
                            }
                            bleSendAck(CMD_BLE_MOVE_FORWARD);
                            moveForward(speed, dist);
                        } else {
                            bleSendError(ERR_ESTOP_ACTIVE);
                        }
                        break;
                    }

                    case CMD_BLE_MOVE_BACKWARD: {
                        if (!isEstopActive()) {
                            int16_t speed = -30;
                            float dist = 100.0f;
                            if (payload_len >= 1) {
                                speed = -(int16_t)payload[0];
                            }
                            if (payload_len >= 3) {
                                uint16_t d = ((uint16_t)payload[1] << 8) | payload[2];
                                dist = (float)d;
                            }
                            bleSendAck(CMD_BLE_MOVE_BACKWARD);
                            moveForward(speed, dist);
                        } else {
                            bleSendError(ERR_ESTOP_ACTIVE);
                        }
                        break;
                    }

                    case CMD_BLE_TURN_LEFT: {
                        if (!isEstopActive()) {
                            /* Payload: [angle_degrees(1)] */
                            float angle = -90.0f; /* Default 90 deg CCW */
                            if (payload_len >= 1) {
                                angle = -(float)payload[0];
                            }
                            bleSendAck(CMD_BLE_TURN_LEFT);
                            turnToAngle(angle);
                        } else {
                            bleSendError(ERR_ESTOP_ACTIVE);
                        }
                        break;
                    }

                    case CMD_BLE_TURN_RIGHT: {
                        if (!isEstopActive()) {
                            float angle = 90.0f;
                            if (payload_len >= 1) {
                                angle = (float)payload[0];
                            }
                            bleSendAck(CMD_BLE_TURN_RIGHT);
                            turnToAngle(angle);
                        } else {
                            bleSendError(ERR_ESTOP_ACTIVE);
                        }
                        break;
                    }

                    case CMD_BLE_ESTOP:
                        emergencyStop();
                        stopMotors();
                        disableMotors();
                        bleSendAck(CMD_BLE_ESTOP);
                        break;

                    case CMD_BLE_SET_SPEED: {
                        /* Payload: [left_speed(1, signed)] [right_speed(1, signed)] */
                        if (!isEstopActive() && payload_len >= 2) {
                            int16_t left  = (int16_t)(int8_t)payload[0];
                            int16_t right = (int16_t)(int8_t)payload[1];
                            enableMotors();
                            sendMotorCommand(left, right);
                            bleSendAck(CMD_BLE_SET_SPEED);
                        } else if (isEstopActive()) {
                            bleSendError(ERR_ESTOP_ACTIVE);
                        }
                        break;
                    }

                    case CMD_BLE_APPROACH: {
                        /* Payload: [target_dist_hi(1)] [target_dist_lo(1)] */
                        if (!isEstopActive()) {
                            uint16_t target = 80; /* Default 80mm */
                            if (payload_len >= 2) {
                                target = ((uint16_t)payload[0] << 8) | payload[1];
                            }
                            bleSendAck(CMD_BLE_APPROACH);
                            approachObject(target);
                        } else {
                            bleSendError(ERR_ESTOP_ACTIVE);
                        }
                        break;
                    }

                    case CMD_BLE_REQUEST_STATUS:
                        bleSendStatus();
                        break;

                    default:
                        bleSendError(ERR_INVALID_CMD);
                        break;
                    }

                    s_bleParseState = BLE_PARSE_WAIT_HEADER;
                    return true;
                } else {
                    /* Checksum mismatch */
                    bleSendError(ERR_CHECKSUM_FAIL);
                    s_bleParseState = BLE_PARSE_WAIT_HEADER;
                }
            }
            break;
        }
    }

    return false;
}

/* ========================================================================== */
/*  BLE task processing                                                        */
/* ========================================================================== */

/**
 * Called from BLETask loop. Handles:
 * 1. Incoming command parsing and dispatch
 * 2. Periodic status notifications to connected phone
 * 3. Connection state monitoring
 */
void bleTaskProcess(void)
{
    osMutexAcquire(stateMutex, osWaitForever);
    bool initialized = g_bleState.initialized;
    osMutexRelease(stateMutex);

    if (!initialized) {
        return;
    }

    /* Process any incoming BLE commands */
    bleProcessCommand();

    /* Periodic status notification to phone */
    uint32_t now = HAL_GetTick();

    if (bleIsConnected() && (now - s_lastStatusTick) >= BLE_STATUS_INTERVAL_MS) {
        bleSendStatus();
        s_lastStatusTick = now;
    }

    /* Connection timeout: if no data received for a while, mark disconnected */
    if ((now - s_lastConnCheckTick) >= BLE_CHECK_INTERVAL_MS) {
        s_lastConnCheckTick = now;

        /* Check if ring buffer has had any activity.
         * If not, and we were connected, mark as disconnected after timeout. */
        if (s_bleRxRing.head == s_bleRxRing.tail) {
            /* No data in buffer. If we haven't received anything for
             * multiple check intervals, mark as disconnected. */
            static uint8_t s_inactiveCount = 0;
            s_inactiveCount++;

            if (s_inactiveCount >= 5) { /* 5 seconds of silence */
                osMutexAcquire(stateMutex, osWaitForever);
                g_bleState.connected = false;
                osMutexRelease(stateMutex);
                s_inactiveCount = 0;
            }
        } else {
            /* Data present, connection is alive */
            osMutexAcquire(stateMutex, osWaitForever);
            g_bleState.connected = true;
            osMutexRelease(stateMutex);
        }
    }
}
