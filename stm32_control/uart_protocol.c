/**
 * UART Protocol Implementation
 * Interrupt-driven UART with ring buffer, packet framing, XOR checksum.
 *
 * Packet format: [0xAA] [CMD] [LEN] [PAYLOAD...] [XOR_CHECKSUM]
 * LEN = number of payload bytes only (excludes CMD and checksum).
 * XOR_CHECKSUM = CMD ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n-1].
 */

#include "uart_protocol.h"
#include "servo_control.h"

static RingBuffer_t  s_rxRing;
static ParseState_t  s_parseState;
static uint8_t       s_parseCmd;
static uint8_t       s_parseLen;
static uint8_t       s_parseBuf[UART_MAX_PAYLOAD];
static uint8_t       s_parseIdx;
static uint8_t       s_parseXor;
static uint8_t       s_rxByte;  /* Single-byte DMA/IT target */

/* -------------------------------------------------------------------------- */
/*  Ring buffer helpers                                                        */
/* -------------------------------------------------------------------------- */

static void ringBufReset(RingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

static bool ringBufPut(RingBuffer_t *rb, uint8_t byte)
{
    uint16_t next = (rb->head + 1U) % UART_RX_BUF_SIZE;
    if (next == rb->tail) {
        return false; /* Full */
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
    return true;
}

static bool ringBufGet(RingBuffer_t *rb, uint8_t *byte)
{
    if (rb->head == rb->tail) {
        return false; /* Empty */
    }
    *byte = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1U) % UART_RX_BUF_SIZE;
    return true;
}

uint16_t ringBufAvailable(void)
{
    uint16_t h = s_rxRing.head;
    uint16_t t = s_rxRing.tail;
    return (h >= t) ? (h - t) : (UART_RX_BUF_SIZE - t + h);
}

/* -------------------------------------------------------------------------- */
/*  Checksum                                                                   */
/* -------------------------------------------------------------------------- */

static uint8_t computeXorChecksum(const uint8_t *data, uint8_t len)
{
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < len; i++) {
        xor_val ^= data[i];
    }
    return xor_val;
}

/* -------------------------------------------------------------------------- */
/*  Init                                                                       */
/* -------------------------------------------------------------------------- */

void initUART(void)
{
    ringBufReset(&s_rxRing);
    s_parseState = PARSE_WAIT_HEADER;
    s_parseCmd   = 0;
    s_parseLen   = 0;
    s_parseIdx   = 0;
    s_parseXor   = 0;

    /* Start interrupt-driven single-byte reception */
    HAL_UART_Receive_IT(&huart2, &s_rxByte, 1);
}

/* -------------------------------------------------------------------------- */
/*  ISR callback - called from HAL UART IRQ handler                            */
/* -------------------------------------------------------------------------- */

void uartRxIrqHandler(void)
{
    ringBufPut(&s_rxRing, s_rxByte);
    /* Re-arm reception for next byte */
    HAL_UART_Receive_IT(&huart2, &s_rxByte, 1);
}

/* HAL callback - wired to our handler.
 * Dispatches to both ESP32 UART and BLE UART handlers. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == ESP32_UART_INSTANCE) {
        uartRxIrqHandler();
    } else if (huart->Instance == USART3) {
        /* BLE module UART - declared in ble_comm.h */
        extern void bleRxIrqHandler(void);
        bleRxIrqHandler();
    }
}

/* -------------------------------------------------------------------------- */
/*  Packet parser (non-blocking, call from CommandTask)                        */
/* -------------------------------------------------------------------------- */

bool parseCommand(UartPacket_t *pkt)
{
    uint8_t byte;

    while (ringBufGet(&s_rxRing, &byte)) {
        switch (s_parseState) {

        case PARSE_WAIT_HEADER:
            if (byte == UART_PACKET_HEADER) {
                s_parseXor = 0;
                s_parseState = PARSE_WAIT_CMD;
            }
            break;

        case PARSE_WAIT_CMD:
            s_parseCmd = byte;
            s_parseXor ^= byte;
            s_parseState = PARSE_WAIT_LENGTH;
            break;

        case PARSE_WAIT_LENGTH:
            s_parseLen = byte;
            s_parseXor ^= byte;
            s_parseIdx = 0;
            if (s_parseLen == 0) {
                s_parseState = PARSE_WAIT_CHECKSUM;
            } else if (s_parseLen > UART_MAX_PAYLOAD) {
                /* Invalid length, reset */
                s_parseState = PARSE_WAIT_HEADER;
            } else {
                s_parseState = PARSE_WAIT_DATA;
            }
            break;

        case PARSE_WAIT_DATA:
            s_parseBuf[s_parseIdx++] = byte;
            s_parseXor ^= byte;
            if (s_parseIdx >= s_parseLen) {
                s_parseState = PARSE_WAIT_CHECKSUM;
            }
            break;

        case PARSE_WAIT_CHECKSUM:
            if (byte == s_parseXor) {
                pkt->header      = UART_PACKET_HEADER;
                pkt->length      = s_parseLen;
                pkt->cmd         = s_parseCmd;
                pkt->payload_len = s_parseLen;
                if (s_parseLen > 0) {
                    memcpy(pkt->payload, s_parseBuf, s_parseLen);
                }
                pkt->checksum = byte;
                s_parseState  = PARSE_WAIT_HEADER;
                return true;
            } else {
                /* Checksum mismatch */
                sendError(ERR_CHECKSUM_FAIL);
                s_parseState = PARSE_WAIT_HEADER;
            }
            break;
        }
    }

    return false;
}

/* -------------------------------------------------------------------------- */
/*  Transmit functions                                                         */
/* -------------------------------------------------------------------------- */

void sendResponse(uint8_t rsp_id, const uint8_t *payload, uint8_t len)
{
    uint8_t txbuf[UART_TX_BUF_SIZE];

    if ((len + 4) > UART_TX_BUF_SIZE) {
        return; /* Packet too large */
    }

    /* Format: [0xAA] [CMD] [LEN] [PAYLOAD...] [XOR_CHECKSUM]
     * LEN = payload byte count only.
     * XOR_CHECKSUM = CMD ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n-1] */
    txbuf[0] = UART_PACKET_HEADER;
    txbuf[1] = rsp_id;
    txbuf[2] = len;

    if (len > 0 && payload != NULL) {
        memcpy(&txbuf[3], payload, len);
    }

    uint8_t cksum = rsp_id ^ len;
    for (uint8_t i = 0; i < len; i++) {
        cksum ^= payload[i];
    }
    txbuf[3 + len] = cksum;

    uint8_t frame_len = 4 + len; /* HEADER + CMD + LEN + PAYLOAD + CHECKSUM */
    HAL_UART_Transmit(&huart2, txbuf, frame_len, 50);
}

void sendAck(uint8_t cmd_id)
{
    sendResponse(RSP_ACK, &cmd_id, 1);
}

void sendError(uint8_t err_code)
{
    sendResponse(RSP_ERROR, &err_code, 1);
}

void sendServoPositions(void)
{
    /* Pack 4 servo positions as uint16_t (angle * 10 for 0.1 degree resolution) */
    uint8_t payload[SERVO_COUNT * 2];

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        float angle = getServoPosition(i);
        uint16_t angle_x10 = (uint16_t)(angle * 10.0f);
        payload[i * 2]     = (uint8_t)(angle_x10 >> 8);
        payload[i * 2 + 1] = (uint8_t)(angle_x10 & 0xFF);
    }

    sendResponse(RSP_POSITION, payload, SERVO_COUNT * 2);
}

void sendDepthGrid(const DepthGrid_t *grid)
{
    /* Pack 64 uint16_t distance values = 128 bytes */
    uint8_t payload[VL53L8CX_ZONE_COUNT * 2];

    for (uint8_t row = 0; row < VL53L8CX_GRID_SIZE; row++) {
        for (uint8_t col = 0; col < VL53L8CX_GRID_SIZE; col++) {
            uint8_t idx = (row * VL53L8CX_GRID_SIZE + col) * 2;
            uint16_t dist = grid->distance_mm[row][col];
            payload[idx]     = (uint8_t)(dist >> 8);
            payload[idx + 1] = (uint8_t)(dist & 0xFF);
        }
    }

    sendResponse(RSP_DEPTH_GRID, payload, VL53L8CX_ZONE_COUNT * 2);
}

void sendStatus(void)
{
    uint8_t payload[7];

    osMutexAcquire(stateMutex, osWaitForever);
    payload[0] = g_systemState.system_status;
    payload[1] = g_systemState.estop_active ? 1 : 0;
    payload[2] = g_systemState.esp32_connected ? 1 : 0;

    uint16_t force_x10 = (uint16_t)(g_systemState.current_force_n * 10.0f);
    uint16_t limit_x10 = (uint16_t)(g_systemState.force_limit_n * 10.0f);
    osMutexRelease(stateMutex);

    payload[3] = (uint8_t)(force_x10 >> 8);
    payload[4] = (uint8_t)(force_x10 & 0xFF);
    payload[5] = (uint8_t)(limit_x10 >> 8);
    payload[6] = (uint8_t)(limit_x10 & 0xFF);

    sendResponse(RSP_STATUS, payload, 7);
}
