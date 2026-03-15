#include "comms.h"
#include <HardwareSerial.h>

static uint8_t _rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t _rx_head = 0;
static volatile uint16_t _rx_tail = 0;

static DepthGrid _depth_grid;
static bool      _depth_new = false;
static float     _position[6];
static bool      _position_new = false;
static uint8_t   _stm32_status = 0;
static uint8_t   _stm32_error = 0;

static IMUTapResult _imu_tap;
static bool         _imu_tap_new = false;
static MicTapResult _mic_tap;
static bool         _mic_tap_new = false;

enum ParseState : uint8_t { PARSE_SYNC=0, PARSE_CMD, PARSE_LEN, PARSE_DATA, PARSE_CHECKSUM };
static ParseState _parse_state = PARSE_SYNC;
static uint8_t _parse_cmd = 0;
static uint8_t _parse_len = 0;
static uint8_t _parse_buf[UART_MAX_PAYLOAD];
static uint8_t _parse_idx = 0;
static uint8_t _parse_xor = 0;

void initComms() {
    Serial2.begin(UART_STM32_BAUD, SERIAL_8N1, PIN_UART_STM32_RX, PIN_UART_STM32_TX);
    _rx_head = 0; _rx_tail = 0;
    _depth_new = false; _position_new = false;
    _imu_tap_new = false; _mic_tap_new = false;
    _stm32_status = 0; _stm32_error = 0;
    _parse_state = PARSE_SYNC;
    memset(&_depth_grid, 0, sizeof(_depth_grid));
    memset(&_imu_tap, 0, sizeof(_imu_tap));
    memset(&_mic_tap, 0, sizeof(_mic_tap));
}

static uint8_t _computeChecksum(uint8_t cmd, const uint8_t* data, uint8_t len) {
    uint8_t xor_val = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) xor_val ^= data[i];
    return xor_val;
}

void sendSTM32Command(uint8_t cmd, const uint8_t* data, uint8_t len) {
    uint8_t checksum = _computeChecksum(cmd, data, len);
    Serial2.write(UART_SYNC_BYTE_STM32);
    Serial2.write(cmd);
    Serial2.write(len);
    if (len > 0 && data) Serial2.write(data, len);
    Serial2.write(checksum);
    Serial2.flush();
}

void sendGripOpen()    { sendSTM32Command(CMD_GRIP_OPEN, nullptr, 0); }
void sendGripClose()   { sendSTM32Command(CMD_GRIP_CLOSE, nullptr, 0); }
void sendEstop()       { sendSTM32Command(CMD_ESTOP, nullptr, 0); }
void sendRequestDepth(){ sendSTM32Command(CMD_REQUEST_DEPTH, nullptr, 0); }
void sendTapCommand()  { sendSTM32Command(CMD_TAP, nullptr, 0); }

void sendRequestIMUTap() { sendSTM32Command(CMD_REQUEST_IMU_TAP, nullptr, 0); }
void sendRequestMicTap() { sendSTM32Command(CMD_REQUEST_MIC_TAP, nullptr, 0); }

void sendSetForce(float force_N) {
    uint16_t mN = (uint16_t)(force_N * 1000.0f);
    uint8_t data[2] = { (uint8_t)(mN >> 8), (uint8_t)(mN & 0xFF) };
    sendSTM32Command(CMD_SET_FORCE, data, 2);
}

void sendServoMove(uint8_t servo_id, uint16_t position_us) {
    uint8_t data[3] = { servo_id, (uint8_t)(position_us >> 8), (uint8_t)(position_us & 0xFF) };
    sendSTM32Command(CMD_SERVO_MOVE, data, 3);
}

static void _handlePacket(uint8_t cmd, const uint8_t* data, uint8_t len) {
    switch (cmd) {
        case RSP_DEPTH_GRID:
            if (len >= 128) {
                uint16_t min_mm = DEPTH_MAX_MM, max_mm = 0;
                float cx_sum = 0, cy_sum = 0, w_sum = 0;
                for (int r = 0; r < DEPTH_GRID_ROWS; r++) {
                    for (int c = 0; c < DEPTH_GRID_COLS; c++) {
                        int idx = (r * DEPTH_GRID_COLS + c) * 2;
                        uint16_t val = ((uint16_t)data[idx] << 8) | data[idx+1];
                        if (val > DEPTH_MAX_MM) val = DEPTH_MAX_MM;
                        _depth_grid.mm[r][c] = val;
                        if (val > 0 && val < min_mm) min_mm = val;
                        if (val > max_mm) max_mm = val;
                        if (val > 0 && val < DEPTH_MAX_MM) {
                            float w = 1.0f / (float)val;
                            cx_sum += w * (float)c;
                            cy_sum += w * (float)r;
                            w_sum += w;
                        }
                    }
                }
                _depth_grid.min_mm = min_mm;
                _depth_grid.max_mm = max_mm;
                _depth_grid.centroid_x = (w_sum > 1e-6f) ? cx_sum/w_sum : 3.5f;
                _depth_grid.centroid_y = (w_sum > 1e-6f) ? cy_sum/w_sum : 3.5f;
                _depth_grid.valid = true;
                _depth_new = true;
            }
            break;

        case RSP_POSITION: {
            int n = len / 2; if (n > 6) n = 6;
            for (int i = 0; i < n; i++) {
                int16_t raw = ((int16_t)data[i*2] << 8) | data[i*2+1];
                _position[i] = (float)raw * 0.1f;
            }
            _position_new = true;
            break;
        }

        case RSP_IMU_TAP_DATA:
            if (len >= 6) {
                _imu_tap.peak_accel = (float)(((int16_t)data[0]<<8)|data[1]) / 100.0f;
                _imu_tap.vibration_rms = (float)(((uint16_t)data[2]<<8)|data[3]) / 1000.0f;
                _imu_tap.dominant_freq = (float)(((uint16_t)data[4]<<8)|data[5]);
                _imu_tap.valid = true;
                _imu_tap_new = true;
            }
            break;

        case RSP_MIC_TAP_DATA:
            if (len >= 6) {
                _mic_tap.dominant_freq = (float)(((uint16_t)data[0]<<8)|data[1]);
                _mic_tap.spectral_centroid = (float)(((uint16_t)data[2]<<8)|data[3]);
                _mic_tap.decay_ratio = (float)(((uint16_t)data[4]<<8)|data[5]) / 1000.0f;
                _mic_tap.valid = true;
                _mic_tap_new = true;
            }
            break;

        case RSP_STATUS:
            if (len >= 1) _stm32_status = data[0];
            break;

        case RSP_ERROR:
            if (len >= 1) _stm32_error = data[0];
            break;
    }
}

bool processUARTData() {
    bool got = false;
    while (Serial2.available()) {
        uint8_t byte = Serial2.read();
        switch (_parse_state) {
            case PARSE_SYNC:
                if (byte == UART_SYNC_BYTE_STM32) { _parse_xor = 0; _parse_state = PARSE_CMD; }
                break;
            case PARSE_CMD:
                _parse_cmd = byte; _parse_xor ^= byte; _parse_state = PARSE_LEN;
                break;
            case PARSE_LEN:
                _parse_len = byte; _parse_xor ^= byte; _parse_idx = 0;
                if (_parse_len == 0) _parse_state = PARSE_CHECKSUM;
                else if (_parse_len > UART_MAX_PAYLOAD) _parse_state = PARSE_SYNC;
                else _parse_state = PARSE_DATA;
                break;
            case PARSE_DATA:
                _parse_buf[_parse_idx++] = byte; _parse_xor ^= byte;
                if (_parse_idx >= _parse_len) _parse_state = PARSE_CHECKSUM;
                break;
            case PARSE_CHECKSUM:
                if (byte == _parse_xor) { _handlePacket(_parse_cmd, _parse_buf, _parse_len); got = true; }
                _parse_state = PARSE_SYNC;
                break;
        }
    }
    return got;
}

bool hasNewDepthGrid() { if (_depth_new) { _depth_new = false; return true; } return false; }
bool hasNewPosition()  { if (_position_new) { _position_new = false; return true; } return false; }
bool hasNewIMUTap()    { if (_imu_tap_new) { _imu_tap_new = false; return true; } return false; }
bool hasNewMicTap()    { if (_mic_tap_new) { _mic_tap_new = false; return true; } return false; }

DepthGrid getLatestDepthGrid()  { return _depth_grid; }
IMUTapResult getLatestIMUTap()  { return _imu_tap; }
MicTapResult getLatestMicTap()  { return _mic_tap; }
uint8_t getSTM32Status()        { return _stm32_status; }
uint8_t getSTM32Error()         { return _stm32_error; }
