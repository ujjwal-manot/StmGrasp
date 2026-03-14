# HYDRA GRASP

**Haptic Yielding Detection with Real-time Adaptation**

HYDRA GRASP is a multi-modal robotic grasp planning system that fuses seven sensing modalities across three networked microcontrollers to identify object material, geometry, and fragility before and during grasping. The system classifies materials in real time using impedance spectroscopy and acoustic tap analysis, then generates force-controlled grasp plans with continuous slip monitoring. A live WiFi dashboard streams all sensor data, grasp state, and 3D depth maps for transparent decision-making.

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
  |                    (Sensor Fusion Brain)                          |
  |                                                                   |
  |  Impedance    FSR x3     Piezo      ADS1115       Grasp          |
  |  Spectroscopy (force)   (acoustic)  (IR x4)      Planner        |
  |  [GPIO25/34/35] [36/39/32] [GPIO33] [I2C 21/22]  [state machine]|
  +--------+-------------------------------------------+-------------+
           | UART 115200                                | UART 9600
           | (commands + depth)                         | (LED commands)
  +--------+-----------------------------+    +---------+-------------+
  |  STEVAL-ROBKIT1 (STM32H725)          |    |  NodeMCU ESP8266      |
  |  (Gripper Controller)                |    |  (LED Nervous System) |
  |                                      |    |                       |
  |  VL53L8CX 8x8 ToF  (depth/geometry) |    |  WS2812B strip        |
  |  4ch Servo PWM      (gripper/wrist)  |    |  Material colors      |
  |  PID force control  (closed-loop)    |    |  Force heatmap        |
  |  FreeRTOS tasks     (real-time)      |    |  State animations     |
  |  Safety watchdog    (E-stop)         |    |                       |
  +--------------------------------------+    +-----------------------+
                   |
          +--------+--------+
          | Servos (x3-4)   |
          | Gripper fingers  |
          | Wrist rotation   |
          +-----------------+
```

---

## Sensing Modalities

| # | Modality | Sensor | Description |
|---|----------|--------|-------------|
| 1 | **Impedance Spectroscopy** | ESP32 DAC/ADC + electrode pair | Lock-in amplifier measures complex impedance (magnitude + phase) to classify material type |
| 2 | **Acoustic Tap Analysis** | Piezo disc + ESP32 ADC | FFT-based spectral analysis of tap response identifies material from resonance signature |
| 3 | **Force Sensing** | FSR402 x3 | Tri-point force measurement with center-of-pressure tracking and real-time slip detection |
| 4 | **3D Depth Mapping** | VL53L8CX 8x8 ToF (on STM32) | 64-zone time-of-flight grid maps object geometry, size, and centroid before approach |
| 5 | **Surface Curvature** | TCRT5000 x4 via ADS1115 | IR reflectance array classifies surface geometry: flat, convex, concave, edge, cylinder |
| 6 | **Visual** | Camera module (on ROBKIT1) | Object detection and visual context for grasp planning |
| 7 | **Inertial** | IMU (on ROBKIT1) | Orientation tracking for gravity-aware grasp adjustment |

---

## Hardware Requirements

| Component | Quantity | Purpose |
|-----------|----------|---------|
| STEVAL-ROBKIT1 (STM32H725) | 1 | Gripper controller, depth sensing, servo PWM |
| ESP32 DevKit v1 | 1 | Sensor fusion brain, impedance engine, WiFi dashboard |
| NodeMCU ESP8266 | 1 | WS2812B LED strip controller |
| FSR402 force-sensitive resistor | 3 | Finger force + slip detection |
| Piezo disc (27mm) | 1 | Acoustic tap sensor |
| TCRT5000 IR reflectance sensor | 4 | Surface curvature array |
| ADS1115 16-bit ADC module | 1 | 4-channel IR sensor readout |
| WS2812B LED strip (30 LED/m) | ~0.5m | Status and material visualization |
| SG90 / MG996R servo | 3-4 | Gripper fingers + wrist rotation |
| 10k resistor (1/4W) | 6 | FSR pull-downs, impedance reference |
| 1.5k resistor | 1 | Impedance DAC series |
| 1k resistor | 2 | ADC input protection |
| 100 ohm resistor | 4 | TCRT5000 IR LED current limit |
| 330 ohm resistor | 1 | WS2812B data line |
| 1M resistor | 1 | Piezo bias |
| 100nF ceramic capacitor | 1 | Impedance DC blocking |
| 100uF electrolytic capacitor | 1 | ESP32 VIN decoupling |
| 1000uF electrolytic capacitor | 1 | WS2812B power decoupling |
| 1N4148 signal diode | 4 | ADC clamping protection |
| Electrode pair (copper tape / PCB pads) | 1 pair | Impedance contact electrodes |
| 5V 3A USB power supply | 1 | ESP32 + NodeMCU + LEDs + sensors |
| Battery pack (ROBKIT1 compatible) | 1 | STM32 + servos |
| Jumper wires, breadboard | assorted | Prototyping |

---

## Directory Structure

```
StmGrasp/
├── README.md
├── docs/
│   └── WIRING_GUIDE.md
├── esp32_brain/             # ESP32 firmware (Arduino/PlatformIO)
│   ├── config.h             # Pin definitions, constants, data structures
│   ├── impedance.h          # Impedance spectroscopy interface
│   ├── impedance.cpp        # Lock-in amplifier, material classification
│   └── acoustic.h           # Acoustic tap analysis + cross-validation
├── stm32_control/           # STM32H725 firmware (STM32CubeIDE)
│   ├── main.h               # Pin mapping, servo config, safety limits
│   └── uart_protocol.h      # Packet framing, ring buffer, command API
└── nodemcu_leds/            # NodeMCU ESP8266 firmware (Arduino)
    └── (LED animation controller)
```

---

## Setup Instructions

### 1. Flash the ESP32 (Sensor Fusion Brain)

```bash
# Using PlatformIO (recommended)
cd esp32_brain
pio run --target upload --upload-port COMx

# Or using Arduino IDE:
# Board: ESP32 Dev Module
# Flash Size: 4MB
# Partition: Default 4MB with spiffs
# Upload Speed: 921600
```

### 2. Flash the STM32 (STEVAL-ROBKIT1)

Open `stm32_control/` as an STM32CubeIDE project. Build and flash via the on-board ST-LINK debugger. Ensure the FPC cable to the imaging board (VL53L8CX) is seated properly.

### 3. Flash the NodeMCU (LED Controller)

```bash
# Using Arduino IDE:
# Board: NodeMCU 1.0 (ESP-12E Module)
# Flash Size: 4MB (FS:2MB OTA:~1019KB)
# Upload Speed: 115200
```

### 4. Wiring

Connect all three boards according to `docs/WIRING_GUIDE.md`. Critical points:

- Common GND between all three boards and all sensors
- UART cross-wiring: ESP32 TX16 to STM32 RX, ESP32 RX17 from STM32 TX
- ESP32 GPIO5 TX to NodeMCU RX
- Both MCU pairs are 3.3V logic -- no level shifters required
- Power servos from the ROBKIT1 battery, never from USB

### 5. Power Up Sequence

1. Connect the 5V USB supply (powers ESP32, NodeMCU, sensors, LEDs)
2. Connect / turn on the ROBKIT1 battery (powers STM32 + servos)
3. Verify all boards boot (STM32 status LED, ESP32 serial output, NodeMCU LED test pattern)

---

## WiFi Dashboard

1. Power on the ESP32. It creates a WiFi access point:
   - **SSID:** `HydraGrasp`
   - **Password:** `hydra2026`
2. Connect your phone or laptop to the `HydraGrasp` network.
3. Open a browser and navigate to `http://192.168.4.1`
4. The dashboard streams at 10 Hz via WebSocket:
   - Live 8x8 depth heatmap from VL53L8CX
   - Impedance magnitude/phase plot with material classification
   - Tri-point force gauge with slip indicator
   - Acoustic FFT spectrum (after tap)
   - IR curvature geometry display
   - Grasp state machine status with timing
   - WS2812B color preview matching material

---

## Demo Procedure

### Pre-Demo Checklist

1. Charge ROBKIT1 battery. Verify 5V USB supply is rated for at least 3A.
2. Flash latest firmware on all three boards.
3. Run impedance baseline calibration with electrodes in open air.
4. Verify WiFi dashboard is accessible on your demo device.
5. Prepare sample objects: metal can, plastic bottle, wooden block, glass jar, cardboard box.

### Live Demo Steps

1. **Power on** all boards. Wait for NodeMCU to show idle breathing animation (blue pulse).
2. **Connect** the demo device to the `HydraGrasp` WiFi and open the dashboard.
3. **Present an object** in front of the gripper. The 8x8 ToF grid detects the object and transitions from `IDLE` to `DETECTED`.
4. **Scanning phase.** The depth grid maps the object profile. Dashboard shows the live heatmap and estimated object dimensions.
5. **Touch the electrodes** to the object surface. Impedance measurement runs automatically. The dashboard displays |Z| and phase, and the material classification appears with confidence percentage. LEDs change color to match the material.
6. **Tap the object** with the piezo-equipped finger. Acoustic FFT runs and cross-validates the impedance result. Dashboard shows the frequency spectrum and combined classification.
7. **Grasp planning.** The system selects target force, approach speed, and slip threshold based on the identified material. The grasp plan card appears on the dashboard with quality score.
8. **Approach and grip.** The gripper closes with force-ramped control. Real-time force bars show the three FSR readings. Slip detection is active -- if micro-slip is detected, force automatically increases within safety limits.
9. **Hold and demonstrate.** Tilt or gently tug the object. The slip response is visible on the dashboard. IR curvature data shows the surface geometry under the fingers.
10. **Release.** Command release from the dashboard or let the timeout expire. Gripper opens smoothly. LEDs return to idle animation.

### Quick Reset

If anything goes wrong, press the E-stop button on the ROBKIT1 header. All servos will release immediately. Power cycle to restart.

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
