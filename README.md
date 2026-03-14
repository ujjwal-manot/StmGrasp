# HYDRA GRASP

**Haptic Yielding Detection with Real-time Adaptation**

Multi-modal robotic grasp planning system built on the STEVAL-ROBKIT1. Fuses 10 sensing modalities across three networked microcontrollers to identify object material, geometry, and fragility before and during grasping. Classifies materials in real time using electrical impedance spectroscopy, acoustic tap analysis, and photometric stereo 3D scanning, then generates force-controlled grasp plans with continuous slip monitoring. Includes a live WiFi dashboard, BLE phone control, and an LED nervous system for transparent operation.

---

## System Architecture

```
  +------------------------------------------------------------------+
  |                        WiFi Dashboard                             |
  |                    (any device, port 80)                          |
  +------------------------------+-----------------------------------+
                                 | WebSocket (10 Hz)
                                 |
  +------------------------------+-----------------------------------+
  |                       ESP32 DevKit v1                             |
  |                     (Sensor Fusion Brain)                         |
  |                                                                   |
  |  Impedance    FSR x3     Piezo      ADS1115       Grasp          |
  |  Spectroscopy (force)   (acoustic)  (IR x4)      Planner        |
  |  [GPIO25/34/35] [36/39/32] [GPIO33] [I2C 21/22]  [12-state FSM] |
  +--------+-------------------------------------------+-------------+
           | UART 115200                                | UART 9600
           | (commands + telemetry)                     | (LED commands)
  +--------+-----------------------------+    +---------+-------------+
  |  STEVAL-ROBKIT1 (STM32H725IGT6)     |    |  NodeMCU ESP8266      |
  |  (Real-time Controller)             |    |  (LED Nervous System) |
  |                                     |    |                       |
  |  VL53L8CX 8x8 ToF  (depth grid)    |    |  WS2812B x20 strip   |
  |  LSM6DSV16BX IMU    (orientation)   |    |  11 animation modes   |
  |  Onboard microphone (tap FFT)       |    |  Material colors      |
  |  STSPIN240 motors   (navigation)    |    |  Force heatmap        |
  |  BLE module         (phone app)     |    |  State feedback       |
  |  4ch servo PWM      (gripper)       |    |                       |
  |  3D scanner         (photometric)   |    +-----------------------+
  |  Safety watchdog    (E-stop)        |
  |  8 FreeRTOS tasks   (CMSIS-OS2)     |
  +-------------------------------------+
           |               |
  +--------+------+  +-----+--------+
  | Servos (x3-4) |  | Motor Board  |
  | Gripper + wrist|  | STM32G071   |
  +---------------+  | (I2C 0x20)  |
                     +--------------+
```

---

## Sensing Modalities

| # | Modality | Hardware | Description |
|---|----------|----------|-------------|
| 1 | **Impedance Spectroscopy** | ESP32 DAC + electrode pair | Software lock-in amplifier measures complex impedance |Z| + phase. Classifies metal, plastic, wood, glass, skin, cardboard |
| 2 | **Acoustic Tap Analysis** | Piezo disc + CMSIS-DSP FFT | 1024-point FFT on STM32 Cortex-M7 FPU + ESP32 cross-validation. Dominant frequency, spectral centroid, decay ratio |
| 3 | **Tri-point Force Sensing** | FSR402 x3 + ESP32 ADC1 | Center-of-pressure tracking with vibration-based slip detection (AC RMS on high-rate samples) |
| 4 | **8x8 Depth Grid** | VL53L8CX ToF (on kit) | 64-zone time-of-flight grid for object centroid, size, and approach distance |
| 5 | **Surface Curvature** | TCRT5000 x4 via ADS1115 | IR reflectance cross-array classifies flat, convex, concave, edge, cylinder geometries |
| 6 | **6-axis IMU** | LSM6DSV16BX (on kit) | Accelerometer + gyroscope for pitch/roll/arm-tilt-compensated grasp orientation |
| 7 | **Onboard Microphone** | ADC3 + BDMA + TIM6 (on kit) | 8kHz capture with threshold-triggered recording and 1024-pt arm_rfft_fast_f32 FFT |
| 8 | **Motor Navigation** | STSPIN240 + encoders (on kit) | Inter-board I2C to motor board. Proportional drift correction, ToF-guided approach |
| 9 | **BLE Control** | BLE module (on kit) | AT command interface, 10 remote commands, 500ms status push to phone |
| 10 | **3D Photometric Stereo** | 3 IR LEDs + VL53L8CX signal_per_spad | Woodham's method surface normal reconstruction, 7 edge types, 12-position rotational sweep |

---

## Hardware Requirements

| Component | Qty | Purpose |
|-----------|-----|---------|
| STEVAL-ROBKIT1 (STM32H725IGT6) | 1 | Main controller with VL53L8CX, IMU, mic, motors, BLE onboard |
| ESP32 DevKit v1 | 1 | Sensor fusion brain, impedance engine, WiFi dashboard |
| NodeMCU ESP8266 | 1 | WS2812B LED strip controller |
| FSR402 force-sensitive resistor | 3 | Finger force + slip detection |
| Piezo disc (27mm) | 1 | Acoustic tap sensor |
| TCRT5000 IR reflectance sensor | 4 | Surface curvature array |
| ADS1115 16-bit ADC module | 1 | 4-channel IR sensor readout |
| WS2812B LED strip | 20 LEDs | Status and material visualization |
| SG90 / MG996R servo | 3-4 | Gripper fingers + wrist rotation |
| IR LEDs (940nm) | 3 | Photometric stereo illumination |
| Electrode pair (copper tape) | 1 pair | Impedance contact electrodes |
| Passive components | assorted | Resistors, caps, diodes (see WIRING_GUIDE.md) |
| 5V 3A power supply | 1 | ESP32 + NodeMCU + LEDs + sensors |
| ROBKIT1 battery pack | 1 | STM32 + servos + motors |

---

## Documentation

| Document | Description |
|----------|-------------|
| [BUILD_GUIDE.md](docs/BUILD_GUIDE.md) | Step-by-step prototype construction: board setup, circuit building, 3D printing, assembly, flashing, testing, troubleshooting |
| [WIRING_GUIDE.md](docs/WIRING_GUIDE.md) | Complete pin assignments, circuit schematics, power architecture |
| [HYDRA_GRASP_BLUEPRINT.md](docs/HYDRA_GRASP_BLUEPRINT.md) | Full technical blueprint: sensing theory, algorithms, calibration, demo script, judge Q&A |

---

## Directory Structure

```
StmGrasp/
├── README.md
├── docs/
│   ├── BUILD_GUIDE.md            # Step-by-step prototype construction guide
│   ├── WIRING_GUIDE.md           # Complete pin assignments and circuit diagrams
│   └── HYDRA_GRASP_BLUEPRINT.md  # Technical blueprint, demo script, judge Q&A
│
├── esp32_brain/                  # ESP32 firmware (Arduino/PlatformIO)
│   ├── config.h                  # Pin defs, material database, shared structs
│   ├── impedance.h/.cpp          # Lock-in amplifier, material classifier
│   ├── acoustic.h/.cpp           # Piezo FFT, cross-validation with impedance
│   ├── sensors.h/.cpp            # ADS1115, TCRT5000 curvature, FSR, slip detect
│   ├── grasp_planner.h/.cpp      # 12-state grasp FSM, quality scoring
│   ├── comms.h/.cpp              # Binary UART protocol, depth grid unpacking
│   ├── web_server.h/.cpp         # WiFi AP dashboard, WebSocket 10Hz push
│   └── esp32_brain.ino           # Main loop, millis() scheduler
│
├── stm32_control/                # STM32H725 firmware (STM32CubeIDE / FreeRTOS)
│   ├── main.h                    # All definitions, commands, structs, externs
│   ├── main.c                    # HAL init, 8 FreeRTOS tasks, command dispatch
│   ├── servo_control.h/.c        # TIM3 4ch PWM, trapezoidal profiles, PID force
│   ├── depth_sensor.h/.c         # VL53L8CX 8x8 driver, DCI protocol, object profile
│   ├── imu_sensor.h/.c           # LSM6DSV16BX 6-axis, pitch/roll/arm_tilt
│   ├── mic_sensor.h/.c           # ADC3+BDMA+TIM6 8kHz capture, CMSIS-DSP FFT
│   ├── motor_control.h/.c        # I2C to motor board, encoders, approach control
│   ├── ble_comm.h/.c             # USART3 BLE, AT commands, phone notifications
│   ├── scanner_3d.h/.c           # Photometric stereo, Woodham's method, edge detect
│   ├── uart_protocol.h/.c        # Ring buffer RX, packet framing, telemetry TX
│   └── safety.h/.c               # IWDG, E-STOP, force limits, heartbeat timeout
│
├── nodemcu_leds/                 # NodeMCU ESP8266 firmware (Arduino)
│   └── nodemcu_leds.ino          # FastLED 20-LED, 11 animations, UART packet parser
│
└── 3d_parts/                     # 3D printable parts (OpenSCAD)
    ├── gripper_finger.scad       # Parametric finger with sensor mount pockets
    ├── gripper_base.scad         # U-frame with servo pockets and LED channel
    ├── ir_led_ring.scad          # 40mm ring for 3 IR LEDs at 120 deg spacing
    ├── sensor_backpack.scad      # ESP32 + NodeMCU + breadboard tray
    ├── spring_contact_pad.scad   # Compliant fingertip with FSR pocket
    └── assembly_preview.scad     # Full assembly visualization (color-coded)
```

**44 source files, 11,300+ lines of firmware and design code.**

---

## Setup Instructions

### 1. Flash the ESP32 (Sensor Fusion Brain)

```bash
# PlatformIO (recommended)
cd esp32_brain
pio run --target upload --upload-port COMx

# Arduino IDE: Board = ESP32 Dev Module, Flash = 4MB, Partition = Default
# Required libraries: WiFi, AsyncTCP, ESPAsyncWebServer, ArduinoJson, Wire, arduinoFFT
```

### 2. Flash the STM32 (STEVAL-ROBKIT1)

Open `stm32_control/` as an STM32CubeIDE project. Build and flash via the onboard ST-LINK. Ensure the FPC cable to the imaging board (VL53L8CX) is properly seated. Required: CMSIS-DSP library (arm_math.h) for microphone FFT.

### 3. Flash the NodeMCU (LED Controller)

```bash
# Arduino IDE: Board = NodeMCU 1.0 (ESP-12E), Flash = 4MB
# Required library: FastLED
```

### 4. 3D Print Parts

Open each `.scad` file in OpenSCAD. Render (F6) and export as STL. Print with PLA at 0.2mm layer height, 20% infill. `assembly_preview.scad` shows the full assembly for reference. Print `gripper_finger.scad` twice (left and right, use `mirror_finger = true`).

### 5. Wiring

Connect all three boards per `docs/WIRING_GUIDE.md`. Key points:

- **Common GND** between all boards and all sensors
- **UART**: ESP32 TX(GPIO16) -> STM32 RX(PD6), ESP32 RX(GPIO17) <- STM32 TX(PD5)
- **LED UART**: ESP32 TX(GPIO5) -> NodeMCU RX
- **I2C shared bus**: VL53L8CX + LSM6DSV16BX + motor board all on I2C1 (PB8/PB9), mutex-protected in firmware
- All MCU pairs are 3.3V logic, no level shifters needed
- Power servos from ROBKIT1 battery, never from USB
- See WIRING_GUIDE.md for full schematic including impedance circuit, FSR dividers, and piezo input

### 6. Power Up Sequence

1. Connect 5V USB supply (ESP32, NodeMCU, sensors, LEDs)
2. Connect ROBKIT1 battery (STM32, servos, motors)
3. Verify: STM32 status LED steady, ESP32 serial output, NodeMCU blue breathing animation

---

## WiFi Dashboard

1. Power on ESP32. It creates AP: **SSID** `HydraGrasp`, **Password** `hydra2026`
2. Connect and open `http://192.168.4.1`
3. Dashboard streams at 10 Hz via WebSocket with 8 panels:
   - Live 8x8 depth heatmap
   - Impedance magnitude/phase with material classification
   - Tri-point force gauge with slip indicator
   - Acoustic FFT spectrum
   - IR curvature geometry display
   - Grasp state machine status
   - Motor/encoder status
   - LED color preview

---

## BLE Phone Control

When BLE is connected, the following commands are available from any BLE terminal app:

| Command | Function |
|---------|----------|
| `SCAN` | Start 3D scan |
| `GRIP` | Close gripper with auto force |
| `RELEASE` | Open gripper |
| `APPROACH <mm>` | Drive toward object to specified distance |
| `STOP` | Emergency stop all motors |
| `STATUS` | Request full system status |
| `IMPEDANCE` | Trigger impedance measurement |
| `TAP` | Trigger acoustic tap capture |
| `FORCE <N>` | Set target grip force |
| `SPEED <val>` | Set servo speed |

---

## Demo Procedure

### Pre-Demo Checklist

1. Charge ROBKIT1 battery. Verify 5V USB supply rated for 3A+.
2. Flash latest firmware on all three boards.
3. Run impedance baseline calibration (electrodes open air).
4. Verify WiFi dashboard accessible. Verify BLE pairing works.
5. Prepare sample objects: metal can, plastic bottle, wooden block, glass jar, cardboard box.

### Live Demo Steps

1. **Power on.** Wait for NodeMCU blue breathing animation.
2. **Connect** demo device to HydraGrasp WiFi. Open dashboard.
3. **Present object** in front of gripper. 8x8 ToF detects and transitions IDLE -> DETECTED.
4. **3D scan.** Robot rotates around object using motors. IR LEDs illuminate from 3 angles. Photometric stereo reconstructs surface normals. Dashboard shows edge classification (smooth, ridge, corner, etc.).
5. **Touch electrodes** to object. Impedance runs automatically. Dashboard shows |Z|, phase, material classification with confidence. LEDs change to material color.
6. **Tap object** with piezo finger. STM32 captures audio, runs CMSIS-DSP FFT, sends spectral features to ESP32. Cross-validated with impedance (0.7/0.3 weighting).
7. **IMU compensation.** Tilt the gripper arm. Dashboard shows real-time pitch/roll/arm_tilt. Grasp planner adjusts force targets for gravity.
8. **Grasp planning.** System selects force, speed, slip threshold based on material. Quality score shown on dashboard.
9. **Motor approach.** Robot drives toward object using VL53L8CX center zones. Proportional drift correction keeps it centered. Stops at target distance.
10. **Grip.** Gripper closes with force-ramped PID control. Real-time force bars show FSR readings. Slip detection active with auto force increase.
11. **Hold.** Tilt or tug object. Slip response visible on dashboard. IR curvature shows surface geometry under fingers.
12. **Release.** Command from dashboard, BLE, or timeout. Gripper opens smoothly. LEDs return to idle.

### Quick Reset

Press E-stop on ROBKIT1 header. All servos release immediately. Power cycle to restart.

---

## Key Technical Details

- **FreeRTOS**: 8 tasks with priority inversion protection (recursive mutexes with priority inheritance)
- **I2C bus sharing**: VL53L8CX, LSM6DSV16BX IMU, and motor board (STM32G071 @ 0x20) share I2C1, protected by depthMutex
- **UART protocol**: Binary packets [0xAA][CMD][LEN][DATA...][XOR_CHECKSUM] with 256-byte ring buffer
- **LED protocol**: [0xBB][CMD][P1][P2][P3] at 9600 baud to NodeMCU
- **Safety**: Independent watchdog (2s timeout), E-stop GPIO monitoring, force limits, heartbeat timeout (3s -> open gripper)
- **ADC2/WiFi conflict**: All ESP32 ADC readings on ADC1 pins (GPIO32-39) to avoid ADC2+WiFi conflict
- **DMA buffer placement**: Microphone DMA buffer in `.sram4` section for STM32H725 BDMA/ADC3 domain compatibility
- **Photometric stereo**: Uses VL53L8CX `signal_per_spad` as brightness proxy (avoids complex camera driver)

---

## License

MIT License

Copyright (c) 2026

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
