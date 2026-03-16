# HYDRA GRASP v2.0 - Project Guide

## Project Overview

HYDRA GRASP (Haptic-Yielding Dexterous Robotic Autonomous Grasping System Platform) is an intelligent robotic gripper that uses multi-modal sensor fusion to identify object materials and autonomously select optimal grasping strategies. It combines impedance spectroscopy, acoustic tap analysis, depth sensing, force sensing, and curvature detection with Dempster-Shafer evidence theory for material classification and hyperdimensional computing for slip detection.

## Architecture

```
ESP32 (Sensor Fusion Brain)                STM32H725 (Motor/Sensor I/O)
 - Impedance spectroscopy                   - STEVAL-ROBKIT1 board
 - Acoustic tap via piezo                    - 4x servo PWM (TIM3)
 - FSR force sensing (3x FSR402)             - VL53L8CX 8x8 ToF depth
 - Dempster-Shafer material fusion           - LSM6DSV16BX IMU
 - HD computing slip detection               - Microphone tap capture
 - Grasp state machine (12 states)           - PID force control
 - Success predictor (logistic reg.)         - E-stop monitoring
 - WiFi AP + WebSocket dashboard             - DC motor control (2x)
 - Grasp strategy planner                    - 3D scanning subsystem
      |                                           |
      +------------ UART (115200 baud) -----------+
      |
   Flutter App (via WebSocket over WiFi)
```

The ESP32 is the decision-making brain. The STM32 handles real-time motor control, depth sensing via VL53L8CX (I2C1), IMU orientation, and microphone-based tap analysis. They communicate over UART2 at 115200 baud.

## Build Instructions

### ESP32 (Arduino IDE)
1. Open `esp32_brain/esp32_brain.ino` in Arduino IDE.
2. Board: ESP32 Dev Module (or your specific board).
3. Install required libraries (see Dependencies below).
4. Upload at 115200 baud.

### STM32 (STM32CubeIDE)
1. Import `stm32_control/` as a CubeIDE project.
2. Target: STM32H725IGT6 (STEVAL-ROBKIT1).
3. FreeRTOS (CMSIS-RTOS v2) is used for multitasking.
4. Build and flash via ST-Link.

### Flutter App
1. `cd hydra_app && flutter pub get && flutter run`
2. Connects to ESP32 WiFi AP, then WebSocket at `ws://192.168.4.1/ws`.

## Pin Mapping

### ESP32
| Function           | Pin | Notes                    |
|--------------------|-----|--------------------------|
| DAC Sine (imp.)    | 25  | DAC1, AC excitation      |
| ADC Vref (imp.)    | 34  | ADC1_CH6                 |
| ADC Vmut (imp.)    | 35  | ADC1_CH7                 |
| FSR1               | 36  | ADC1_CH0 / VP            |
| FSR2               | 39  | ADC1_CH3 / VN            |
| FSR3               | 32  | ADC1_CH4                 |
| I2C SDA            | 21  | Reserved                 |
| I2C SCL            | 22  | Reserved                 |
| UART TX to STM32   | 16  | TX2                      |
| UART RX from STM32 | 17  | RX2                      |
| Piezo ADC          | 33  | ADC1_CH5, contact sensor |
| Start button       | 5   |                          |
| Spare button       | 4   |                          |

### STM32 (STEVAL-ROBKIT1)
| Function        | Pin/Port  | Notes                          |
|-----------------|-----------|--------------------------------|
| UART TX (ESP32) | PD5       | USART2, AF7                    |
| UART RX (ESP32) | PD6       | USART2, AF7                    |
| I2C1 SCL (ToF)  | PB8       | VL53L8CX, AF4, 400kHz         |
| I2C1 SDA (ToF)  | PB9       | VL53L8CX, AF4                  |
| Servo CH1       | PC6       | TIM3_CH1, gripper open/close   |
| Servo CH2       | PC7       | TIM3_CH2, wrist rotate         |
| Servo CH3       | PC8       | TIM3_CH3, aux 1                |
| Servo CH4       | PC9       | TIM3_CH4, aux 2                |
| E-Stop          | PA0       | Active low, GPIO header        |
| Status LED      | PH13      | On-board LED                   |
| BLE UART        | USART3    | BLE module                     |

## UART Protocol Specification

Sync byte: `0xAA`. Packet format:

```
[0xAA] [CMD] [LEN] [PAYLOAD...] [XOR_CHECKSUM]
```

- **CMD**: Command or response ID (1 byte)
- **LEN**: Number of payload bytes (1 byte, 0 if no payload)
- **PAYLOAD**: `LEN` bytes of data
- **XOR_CHECKSUM**: `CMD ^ LEN ^ PAYLOAD[0] ^ ... ^ PAYLOAD[n-1]`

### Command IDs (ESP32 -> STM32)
| ID   | Name                | Payload                                      |
|------|---------------------|----------------------------------------------|
| 0x01 | CMD_SET_SERVO_ANGLE | [servo_id][angle_hi][angle_lo]               |
| 0x02 | CMD_MOVE_SERVO_SMOOTH | [servo_id][angle_hi][angle_lo][speed]      |
| 0x03 | CMD_OPEN_GRIPPER    | (none)                                       |
| 0x04 | CMD_CLOSE_GRIPPER   | (none)                                       |
| 0x05 | CMD_SET_GRIP_FORCE  | [force_mN_hi][force_mN_lo]                   |
| 0x06 | CMD_TAP_MOTION      | (none)                                       |
| 0x07 | CMD_REQUEST_DEPTH   | (none)                                       |
| 0x08 | CMD_REQUEST_STATUS  | (none)                                       |
| 0x09 | CMD_SET_FORCE_LIMIT | [limit_hi][limit_lo]                         |
| 0x0A | CMD_HEARTBEAT       | (none)                                       |
| 0x0B | CMD_EMERGENCY_STOP  | (none)                                       |
| 0x0C | CMD_SET_SERVO_SPEED | [servo_id][speed_ms_per_deg]                 |
| 0x0D | CMD_FORCE_UPDATE    | [force_hi][force_lo]                         |
| 0x0E | CMD_REQUEST_IMU     | (none)                                       |
| 0x0F | CMD_TRIGGER_MIC_CAP | (none)                                       |
| 0x10 | CMD_MOTOR_MOVE      | [left_hi][left_lo][right_hi][right_lo]       |
| 0x11 | CMD_MOTOR_STOP      | (none)                                       |
| 0x12 | CMD_APPROACH_OBJECT | [target_dist_hi][target_dist_lo]             |
| 0x20 | CMD_START_FULL_SCAN | (none)                                       |
| 0x21 | CMD_START_QUICK_SCAN| (none)                                       |
| 0x22 | CMD_GET_SCAN_STATUS | (none)                                       |
| 0x23 | CMD_GET_SCAN_RESULT | (none)                                       |
| 0x24 | CMD_REQUEST_IMU_TAP | (none)                                       |
| 0x25 | CMD_REQUEST_MIC_TAP | (none)                                       |

### Response IDs (STM32 -> ESP32)
| ID   | Name              | Payload                                        |
|------|-------------------|------------------------------------------------|
| 0x80 | RSP_ACK           | [acked_cmd_id]                                 |
| 0x81 | RSP_POSITION      | 4x [angle_x10_hi][angle_x10_lo] (8 bytes)     |
| 0x82 | RSP_DEPTH_GRID    | 64x [dist_hi][dist_lo] (128 bytes)             |
| 0x83 | RSP_STATUS        | [status][estop][connected][force(2)][limit(2)] |
| 0x84 | RSP_ERROR         | [error_code]                                   |
| 0x90 | RSP_SCAN_STATUS   | Scanner state + progress                       |
| 0x91 | RSP_SCAN_RESULT   | 3D scan result                                 |
| 0x92 | RSP_IMU_TAP_DATA  | [peak_accel(2)][vib_rms(2)][dom_freq(2)]       |
| 0x93 | RSP_MIC_TAP_DATA  | [dom_freq(2)][spectral_cent(2)][decay(2)]      |

## Key Algorithms

### Dempster-Shafer Fusion (`ds_fusion.cpp`)
- Frame of discernment: {Metal, Skin, Plastic, Wood, Glass, Cardboard}
- Each sensor modality (impedance, acoustic, curvature) produces a Basic Probability Assignment (BPA/mass function)
- Gaussian likelihood models convert raw readings to mass over material singletons + Theta (ignorance)
- Dempster's rule of combination fuses independent sources, with conflict K quantifying disagreement
- Output: belief (lower bound) and plausibility (upper bound) per material, plus best material pick

### HD Computing Slip Detection (`hd_slip.cpp`)
- Binary hypervectors (D=2048 bits = 256 bytes)
- 7 features: 3x FSR AC-RMS, 3x FSR DC, dF/dt
- Level encoding -> XOR binding -> Hamming distance classification
- Two classes: STABLE vs SLIP
- Online/incremental training with auto-bootstrap from RMS-based ground truth
- Total model size: ~1KB, inference in microseconds

### 4-Strategy Grasp Repertoire (`grasp_planner.cpp`)
- **POWER**: High force, full close, robust objects (metal, wood)
- **PRECISION**: Low force, fingertip, fragile/small objects (glass, skin)
- **WRAP**: Progressive close, curved objects (cylinders, convex)
- **EDGE**: Angled approach, thin/flat objects (cardboard, edges)
- Strategy selected based on material classification + geometry + flatness
- 12-state FSM: IDLE -> DETECTED -> SCANNING -> ANALYZING -> TAP_TESTING -> CLASSIFYING -> PLANNING -> APPROACHING -> GRIPPING -> HOLDING -> RELEASING -> ERROR

### Success Predictor (`success_predictor.cpp`)
- Logistic regression on 6 features (material confidence, flatness, depth quality, plan quality, object size, geometry compatibility)
- Online SGD updates after each grasp attempt
- Weights persisted to SPIFFS

## WiFi / WebSocket

- WiFi AP SSID: `HydraGrasp`
- WiFi AP Password: `hydra2026`
- Dashboard: `http://192.168.4.1/` (served from PROGMEM)
- WebSocket: `ws://192.168.4.1/ws`
- JSON messages pushed at 10 Hz with full sensor state, DS fusion results, decision log
- Commands from app: `{"cmd":"start"}`, `{"cmd":"release"}`, `{"cmd":"estop"}`, `{"cmd":"calibrate"}`
- Max 4 concurrent WebSocket clients

## Dependencies

### ESP32 (Arduino libraries)
- **ArduinoJson** (v6.x) - JSON serialization for WebSocket messages
- **ESPAsyncWebServer** - Async HTTP + WebSocket server
- **AsyncTCP** - TCP backend for ESPAsyncWebServer
- **arduinoFFT** - FFT for acoustic tap spectral analysis

### STM32
- STM32 HAL (via CubeIDE)
- FreeRTOS (CMSIS-RTOS v2)
- VL53L8CX ULD driver (ST)

### Flutter
- See `hydra_app/pubspec.yaml` for package list

## Known Limitations and TODOs

- No automated test suite yet (unit tests, hardware-in-the-loop)
- HD slip model requires manual calibration phase before first use
- Success predictor initial weights are hand-tuned, needs real grasp data
- Impedance spectroscopy uses single-frequency (1 kHz); multi-frequency sweep would improve classification
- No persistent storage of grasp history across ESP32 reboots (SPIFFS planned)
- VL53L8CX depth grid refresh rate limited to 5 Hz
- BLE communication module is stubbed but not fully integrated
- 3D scanning subsystem is experimental
- Force control PID tuning values are placeholder
- No OTA firmware update mechanism

## Test Instructions

No automated tests exist yet. Manual testing procedure:

1. **UART loopback**: Connect ESP32 TX2 to RX2, send a command, verify echo parse.
2. **Impedance**: Touch known materials (metal coin, plastic card, skin) and verify DS fusion output on dashboard.
3. **Force**: Press FSR pads, verify force bar chart on dashboard.
4. **Depth**: Place object in front of VL53L8CX, verify 8x8 heatmap on dashboard.
5. **Grasp cycle**: Press START on dashboard, verify full FSM state progression.
6. **E-Stop**: Press E-Stop button, verify immediate motor halt and ERROR state.
7. **Slip detection**: During HOLDING state, slowly pull object to trigger slip detection.
