/**
 * UART Protocol - ESP32 Communication
 * Ring-buffered interrupt-driven UART with packet framing and XOR checksum.
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Ring buffer */
typedef struct {
    uint8_t  buf[UART_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer_t;

/* Parser state machine (matches ESP32 format: [0xAA][CMD][LEN][DATA][CHECKSUM]) */
typedef enum {
    PARSE_WAIT_HEADER,
    PARSE_WAIT_CMD,
    PARSE_WAIT_LENGTH,
    PARSE_WAIT_DATA,
    PARSE_WAIT_CHECKSUM,
} ParseState_t;

void     initUART(void);
void     uartRxIrqHandler(void);      /* Call from USART2_IRQHandler */
bool     parseCommand(UartPacket_t *pkt);
void     sendResponse(uint8_t rsp_id, const uint8_t *payload, uint8_t len);
void     sendAck(uint8_t cmd_id);
void     sendError(uint8_t err_code);
void     sendServoPositions(void);
void     sendDepthGrid(const DepthGrid_t *grid);
void     sendStatus(void);
uint16_t ringBufAvailable(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_PROTOCOL_H */
