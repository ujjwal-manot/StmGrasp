#include "comms.h"
#include <HardwareSerial.h>

// STM32 UART on Serial2 (TX=GPIO16, RX=GPIO17)
// LED UART on Serial1 (TX=GPIO5 only, RX unused)

// ─────────────────────────────────────────────────────────────
// Ring buffer for incoming STM32 data
// ─────────────────────────────────────────────────────────────
static uint8_t _rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t _rx_head = 0;
static volatile uint16_t _rx_tail = 0;

// Parsed data storage
static DepthGrid _depth_grid;
static bool      _depth_new = false;
static float     _position[6];    // up to 6 axes
static bool      _position_new = false;
static uint8_t   _stm32_status = 0;
static uint8_t   _stm32_error = 0;

// Packet parser state
enum ParseState : uint8_t {
    PARSE_SYNC = 0,
    PARSE_CMD,
    PARSE_LEN,
    PARSE_DATA,
    PARSE_CHECKSUM
};

static ParseState _parse_state = PARSE_SYNC;
static uint8_t _parse_cmd = 0;
static uint8_t _parse_len = 0;
static uint8_t _parse_buf[UART_MAX_PAYLOAD];
static uint8_t _parse_idx = 0;
static uint8_t _parse_xor = 0;


void initComms() {
    // STM32 UART
    Serial2.begin(UART_STM32_BAUD, SERIAL_8N1, PIN_UART_STM32_RX, PIN_UART_STM32_TX);

    // LED UART (TX only — we assign RX to an unused pin)
    Serial1.begin(UART_LED_BAUD, SERIAL_8N1, -1, PIN_UART_LED_TX);

    _rx_head = 0;
    _rx_tail = 0;
    _depth_new = false;
    _position_new = false;
    _stm32_status = 0;
    _stm32_error = 0;
    _parse_state = PARSE_SYNC;

    memset(&_depth_grid, 0, sizeof(_depth_grid));
    memset(_position, 0, sizeof(_position));

    Serial.println("[COMMS] UART initialized");
}


// ─────────────────────────────────────────────────────────────
// XOR checksum over cmd + len + data
// ─────────────────────────────────────────────────────────────
static uint8_t _computeChecksum(uint8_t cmd, const uint8_t* data, uint8_t len) {
    uint8_t xor_val = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        xor_val ^= data[i];
    }
    return xor_val;
}


// ─────────────────────────────────────────────────────────────
// Send command to STM32
// ─────────────────────────────────────────────────────────────
void sendSTM32Command(uint8_t cmd, const uint8_t* data, uint8_t len) {
    uint8_t checksum = _computeChecksum(cmd, data, len);
    Serial2.write(UART_SYNC_BYTE_STM32);
    Serial2.write(cmd);
    Serial2.write(len);
    if (len > 0 && data != nullptr) {
        Serial2.write(data, len);
    }
    Serial2.write(checksum);
    Serial2.flush();
}


void sendGripOpen() {
    sendSTM32Command(CMD_GRIP_OPEN, nullptr, 0);
}


void sendGripClose() {
    sendSTM32Command(CMD_GRIP_CLOSE, nullptr, 0);
}


void sendSetForce(float force_N) {
    // Encode force as uint16_t in millinewtons
    uint16_t mN = (uint16_t)(force_N * 1000.0f);
    uint8_t data[2] = { (uint8_t)(mN >> 8), (uint8_t)(mN & 0xFF) };
    sendSTM32Command(CMD_SET_FORCE, data, 2);
}


void sendTapCommand() {
    sendSTM32Command(CMD_TAP, nullptr, 0);
}


void sendServoMove(uint8_t servo_id, uint16_t position_us) {
    uint8_t data[3] = {
        servo_id,
        (uint8_t)(position_us >> 8),
        (uint8_t)(position_us & 0xFF)
    };
    sendSTM32Command(CMD_SERVO_MOVE, data, 3);
}


void sendEstop() {
    sendSTM32Command(CMD_ESTOP, nullptr, 0);
}


void sendRequestDepth() {
    sendSTM32Command(CMD_REQUEST_DEPTH, nullptr, 0);
}


// ─────────────────────────────────────────────────────────────
// Parse a completed packet from STM32
// ─────────────────────────────────────────────────────────────
static void _handlePacket(uint8_t cmd, const uint8_t* data, uint8_t len) {
    switch (cmd) {
        case RSP_DEPTH_GRID:
            // Expected: 128 bytes (64 zones x 2 bytes each, big-endian mm)
            if (len >= 128) {
                uint16_t min_mm = DEPTH_MAX_MM;
                uint16_t max_mm = 0;
                float cx_sum = 0.0f, cy_sum = 0.0f, weight_sum = 0.0f;

                for (int r = 0; r < DEPTH_GRID_ROWS; r++) {
                    for (int c = 0; c < DEPTH_GRID_COLS; c++) {
                        int idx = (r * DEPTH_GRID_COLS + c) * 2;
                        uint16_t val = ((uint16_t)data[idx] << 8) | data[idx + 1];
                        if (val > DEPTH_MAX_MM) val = DEPTH_MAX_MM;
                        _depth_grid.mm[r][c] = val;

                        if (val > 0 && val < min_mm) min_mm = val;
                        if (val > max_mm) max_mm = val;

                        // Inverse-distance weighting for centroid
                        if (val > 0 && val < DEPTH_MAX_MM) {
                            float w = 1.0f / (float)val;
                            cx_sum += w * (float)c;
                            cy_sum += w * (float)r;
                            weight_sum += w;
                        }
                    }
                }

                _depth_grid.min_mm = min_mm;
                _depth_grid.max_mm = max_mm;
                if (weight_sum > 1e-6f) {
                    _depth_grid.centroid_x = cx_sum / weight_sum;
                    _depth_grid.centroid_y = cy_sum / weight_sum;
                } else {
                    _depth_grid.centroid_x = 3.5f;
                    _depth_grid.centroid_y = 3.5f;
                }
                _depth_grid.valid = true;
                _depth_new = true;
            }
            break;

        case RSP_POSITION:
            // Up to 6 axis positions as int16_t (0.1° or 0.1mm units)
            {
                int num_axes = len / 2;
                if (num_axes > 6) num_axes = 6;
                for (int i = 0; i < num_axes; i++) {
                    int16_t raw = ((int16_t)data[i * 2] << 8) | data[i * 2 + 1];
                    _position[i] = (float)raw * 0.1f;
                }
                _position_new = true;
            }
            break;

        case RSP_STATUS:
            if (len >= 1) {
                _stm32_status = data[0];
            }
            break;

        case RSP_ERROR:
            if (len >= 1) {
                _stm32_error = data[0];
                Serial.printf("[COMMS] STM32 error: 0x%02X\n", data[0]);
            }
            break;
    }
}


// ─────────────────────────────────────────────────────────────
// Non-blocking UART parser
// ─────────────────────────────────────────────────────────────
bool processUARTData() {
    bool got_packet = false;

    while (Serial2.available()) {
        uint8_t byte = Serial2.read();

        switch (_parse_state) {
            case PARSE_SYNC:
                if (byte == UART_SYNC_BYTE_STM32) {
                    _parse_xor = 0;
                    _parse_state = PARSE_CMD;
                }
                break;

            case PARSE_CMD:
                _parse_cmd = byte;
                _parse_xor ^= byte;
                _parse_state = PARSE_LEN;
                break;

            case PARSE_LEN:
                _parse_len = byte;
                _parse_xor ^= byte;
                _parse_idx = 0;
                if (_parse_len == 0) {
                    _parse_state = PARSE_CHECKSUM;
                } else if (_parse_len > UART_MAX_PAYLOAD) {
                    // Invalid length — resync
                    _parse_state = PARSE_SYNC;
                } else {
                    _parse_state = PARSE_DATA;
                }
                break;

            case PARSE_DATA:
                _parse_buf[_parse_idx++] = byte;
                _parse_xor ^= byte;
                if (_parse_idx >= _parse_len) {
                    _parse_state = PARSE_CHECKSUM;
                }
                break;

            case PARSE_CHECKSUM:
                if (byte == _parse_xor) {
                    _handlePacket(_parse_cmd, _parse_buf, _parse_len);
                    got_packet = true;
                } else {
                    Serial.printf("[COMMS] Checksum mismatch: got 0x%02X, expected 0x%02X\n",
                                  byte, _parse_xor);
                }
                _parse_state = PARSE_SYNC;
                break;
        }
    }

    return got_packet;
}


bool hasNewDepthGrid() {
    if (_depth_new) {
        _depth_new = false;
        return true;
    }
    return false;
}


bool hasNewPosition() {
    if (_position_new) {
        _position_new = false;
        return true;
    }
    return false;
}


DepthGrid getLatestDepthGrid() {
    return _depth_grid;
}


uint8_t getSTM32Status() {
    return _stm32_status;
}


uint8_t getSTM32Error() {
    return _stm32_error;
}


// ─────────────────────────────────────────────────────────────
// NodeMCU LED commands
// ─────────────────────────────────────────────────────────────
static void _sendLEDPacket(uint8_t cmd, uint8_t p1, uint8_t p2, uint8_t p3) {
    Serial1.write(UART_SYNC_BYTE_LED);
    Serial1.write(cmd);
    Serial1.write(p1);
    Serial1.write(p2);
    Serial1.write(p3);
    Serial1.flush();
}


void sendLEDSolid(uint8_t r, uint8_t g, uint8_t b) {
    _sendLEDPacket(LED_SOLID, r, g, b);
}


void sendLEDAnimate(uint8_t pattern, uint8_t speed) {
    _sendLEDPacket(LED_ANIMATE, pattern, speed, 0);
}


void sendLEDPixel(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
    // Two packets: first sets index, second sets color
    _sendLEDPacket(LED_PIXEL, idx, r, g);
    // Third byte (b) sent as separate pixel continuation
    delayMicroseconds(500);
    _sendLEDPacket(LED_PIXEL, idx | 0x80, b, 0); // MSB flag = continuation
}


void sendLEDMaterial(uint8_t material_idx) {
    uint8_t r = 0, g = 0, b = 0;
    if (material_idx < MATERIAL_COUNT) {
        uint32_t col = MATERIAL_DB[material_idx].led_color;
        r = (col >> 16) & 0xFF;
        g = (col >> 8) & 0xFF;
        b = col & 0xFF;
    }
    _sendLEDPacket(LED_MATERIAL, r, g, b);
}


void sendLEDHeartbeat(uint8_t bpm) {
    _sendLEDPacket(LED_HEARTBEAT, bpm, 0, 0);
}


void sendLEDForceMap(uint8_t f1, uint8_t f2, uint8_t f3) {
    _sendLEDPacket(LED_FORCE_MAP, f1, f2, f3);
}
