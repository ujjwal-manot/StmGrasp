/**
 * BLE Communication - UART-to-BLE Module Driver
 * AT command interface to the onboard BLE module on STEVAL-ROBKIT1.
 *
 * The BLE module connects via USART3 (PD8 TX, PD9 RX) and provides
 * transparent UART bridge mode for mobile app communication.
 * The STRobotics app sends grasp and motor commands; this module
 * provides AT-command init and a binary data protocol.
 */

#ifndef BLE_COMM_H
#define BLE_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========================================================================== */
/*                       BLE UART PIN DEFINITIONS                              */
/* ========================================================================== */

#define BLE_UART_INSTANCE         USART3
#define BLE_UART_TX_PIN           GPIO_PIN_8
#define BLE_UART_TX_PORT          GPIOD
#define BLE_UART_RX_PIN           GPIO_PIN_9
#define BLE_UART_RX_PORT          GPIOD
#define BLE_UART_AF               GPIO_AF7_USART3
#define BLE_UART_BAUDRATE         115200U
#define BLE_UART_IRQn             USART3_IRQn
#define BLE_UART_IRQ_PRIORITY     7U

/* ========================================================================== */
/*                       BLE PROTOCOL DEFINITIONS                              */
/* ========================================================================== */

/* BLE device name */
#define BLE_DEVICE_NAME           "HydraGrasp"

/* AT command response timeout (ms) */
#define BLE_AT_TIMEOUT_MS         500U

/* BLE data packet header */
#define BLE_PACKET_HEADER         0xBBU

/* BLE RX buffer size */
#define BLE_RX_BUF_SIZE           128U

/* BLE TX buffer size */
#define BLE_TX_BUF_SIZE           128U

/* BLE command IDs (phone -> STM32) */
#define CMD_BLE_START_GRASP       0x01U
#define CMD_BLE_RELEASE           0x02U
#define CMD_BLE_MOVE_FORWARD      0x03U
#define CMD_BLE_MOVE_BACKWARD     0x04U
#define CMD_BLE_TURN_LEFT         0x05U
#define CMD_BLE_TURN_RIGHT        0x06U
#define CMD_BLE_ESTOP             0x07U
#define CMD_BLE_SET_SPEED         0x08U
#define CMD_BLE_APPROACH          0x09U
#define CMD_BLE_REQUEST_STATUS    0x0AU

/* BLE notification IDs (STM32 -> phone) */
#define BLE_NOTIFY_STATUS         0x80U
#define BLE_NOTIFY_ACK            0x81U
#define BLE_NOTIFY_ERROR          0x82U
#define BLE_NOTIFY_SENSOR_DATA    0x83U

/* BLE status packet format:
 * [0xBB] [LEN] [BLE_NOTIFY_STATUS]
 * [STATE] [MATERIAL] [FORCE_x10_HI] [FORCE_x10_LO] [QUALITY]
 * [MOTOR_LEFT] [MOTOR_RIGHT] [CHECKSUM]
 */

/* BLE connection check interval (ms) */
#define BLE_CHECK_INTERVAL_MS     1000U

/* BLE status send interval (ms) */
#define BLE_STATUS_INTERVAL_MS    500U

/* ========================================================================== */
/*                       DATA STRUCTURES                                       */
/* ========================================================================== */

typedef struct {
    bool    initialized;
    bool    connected;
    char    device_name[20];
    uint8_t rx_buffer[BLE_RX_BUF_SIZE];
    uint8_t rx_len;
} BLEState_t;

/* BLE ring buffer for interrupt reception */
typedef struct {
    uint8_t  buf[BLE_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} BLERingBuffer_t;

/* BLE packet parser state */
typedef enum {
    BLE_PARSE_WAIT_HEADER,
    BLE_PARSE_WAIT_LENGTH,
    BLE_PARSE_WAIT_DATA,
} BLEParseState_t;

/* ========================================================================== */
/*                       FUNCTION PROTOTYPES                                   */
/* ========================================================================== */

/* Initialization */
HAL_StatusTypeDef initBLE(void);
void              MX_USART3_Init(void);

/* Connection management */
HAL_StatusTypeDef bleAdvertise(bool enable);
bool              bleIsConnected(void);

/* Data send (notifications to phone) */
HAL_StatusTypeDef bleSendStatus(void);
HAL_StatusTypeDef bleSendAck(uint8_t cmd_id);
HAL_StatusTypeDef bleSendError(uint8_t err_code);
HAL_StatusTypeDef bleSendSensorData(uint8_t state, uint8_t material,
                                     uint16_t force_x10, uint8_t quality);

/* Command processing (from phone) */
bool              bleProcessCommand(void);

/* BLE task processing (called from BLETask loop) */
void              bleTaskProcess(void);

/* ISR callback */
void              bleRxIrqHandler(void);

/* AT command interface */
HAL_StatusTypeDef bleSendAT(const char *cmd, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BLE_COMM_H */
