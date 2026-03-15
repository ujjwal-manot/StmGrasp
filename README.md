# HYDRA GRASP v2.0

**Haptic Yielding Detection with Real-time Adaptation**

Multi-physics pre-grasp material interrogation system built on the STEVAL-ROBKIT1. Classifies object material using impedance spectroscopy + acoustic tap analysis before grasping, then selects an optimal grasp strategy from a 4-strategy repertoire. Uses Dempster-Shafer evidence fusion, hyperdimensional computing for slip detection, and online learning for grasp success prediction.

## What Makes This Different

Most robotic grasping is blind — see object, grab, hope. HYDRA GRASP interrogates objects across multiple physics domains before committing to a grasp, like a human picking up something unfamiliar.

| Feature | HYDRA GRASP | Typical Grasp Planner |
|---------|-------------|----------------------|
| Material identification | Impedance + acoustic + depth (pre-contact) | Vision only |
| Classification method | Dempster-Shafer evidence fusion | Nearest-neighbor or CNN |
| Slip detection | Hyperdimensional computing (0.375KB model) | Simple threshold |
| Grasp strategy | 4-strategy repertoire selection | Single strategy |
| Success prediction | Online logistic regression | None |
| Hardware cost | ~INR 4,000 (excl. ROBKIT1) | INR 50,000+ (GPU + sensors) |
| Compute | ESP32 + STM32 (no GPU) | GPU required |

## System Architecture

```
  ┌───────────────────────────────────────────────────────┐
  │                   Flutter App (Phone)                  │
  │  DS Posterior · Force Bars · Depth Grid · Decision Log │
  └──────────────────────┬────────────────────────────────┘
                         │ WebSocket (10 Hz)
  ┌──────────────────────┴────────────────────────────────┐
  │                  ESP32 DevKit v1                        │
  │                (Sensor Fusion Brain)                    │
  │                                                        │
  │  Impedance         FSR x3         Grasp Planner        │
  │  Spectroscopy     (force/slip)    (4-strategy FSM)     │
  │  [GPIO25/34/35]   [36/39/32]                           │
  │                                                        │
  │  DS Fusion    HD Slip Detection   Success Predictor    │
  │  (Dempster-   (2048-bit binary    (online logistic     │
  │   Shafer)      hypervectors)       regression + SPIFFS) │
  └────────────┬───────────────────────────────────────────┘
               │ UART 115200
  ┌────────────┴───────────────────────────────────────────┐
  │           STEVAL-ROBKIT1 (STM32H725IGT6)               │
  │                                                        │
  │  VL53L8CX 8x8 ToF    (depth + geometry estimation)    │
  │  LSM6DSV16BX IMU      (orientation + tap vibration)    │
  │  Onboard microphone   (acoustic tap FFT via CMSIS-DSP) │
  │  STSPIN240 motors     (approach navigation)            │
  │  BLE module           (phone app)                      │
  │  4ch servo PWM        (gripper: 2 fingers + wrist)     │
  │  Safety watchdog      (E-stop, force limits)           │
  │  8 FreeRTOS tasks     (CMSIS-OS2)                      │
  └────────────────────────────────────────────────────────┘
```

## Novel Algorithms

### 1. Dempster-Shafer Evidence Fusion
Each sensor produces a mass function over {Metal, Skin, Plastic, Wood, Glass, Cardboard}. Dempster's rule combines evidence from independent sources, handling inter-sensor conflict gracefully. Outputs belief (lower bound), plausibility (upper bound), and conflict factor per material.

### 2. Hyperdimensional Computing Slip Detection
Binary hypervectors (D=2048 bits = 256 bytes) encode FSR force features. XOR binding + Hamming distance classification. Model size: 0.375KB. Auto-trains from RMS-based bootstrap during initial grasps.

### 3. Grasp Strategy Repertoire
Four strategies with distinct servo profiles and force curves:
- **POWER**: High force, full close, robust objects (metal, wood)
- **PRECISION**: Low force, fingertip only, fragile (glass, skin)
- **WRAP**: Progressive close, curved objects (cylinders, bottles)
- **EDGE**: Angled approach, thin/flat objects (cardboard, edges)

### 4. Online Grasp Success Predictor
Logistic regression model predicting P(success) from 6 features. Updated via SGD after each grasp. Weights persist in SPIFFS. Gates grasp execution: low P(success) triggers repositioning.

## Hardware BOM

| # | Component | Qty | Purpose |
|---|-----------|-----|---------|
| 1 | STEVAL-ROBKIT1 | 1 | Main controller (STM32H725 + ToF + IMU + mic + BLE + motors) |
| 2 | ESP32 DevKit v1 | 1 | Sensor fusion brain |
| 3 | ST-LINK V2 | 1 | Flashing STM32 via SWD |
| 4 | FSR402 | 3 | Finger force + slip detection |
| 5 | SG90 micro servo | 3 | Gripper (2) + wrist (1) |
| 6 | Copper tape 5mm | 1 roll | Impedance electrodes |
| 7 | Breadboard + jumper wires | 1 set | Sensor circuits |
| 8 | 5V 3A USB supply | 1 | ESP32 power |
| 9 | 7.4V LiPo / 4xAA | 1 | ROBKIT1 + servo power |
| 10 | PLA filament | ~100g | 3D printed gripper |
| 11 | E-stop button (NC) | 1 | Safety |
| 12 | Passive components | -- | Resistors, caps, diodes for impedance circuit |

## Directory Structure

```
StmGrasp/
├── esp32_brain/           # ESP32 sensor fusion firmware (Arduino)
│   ├── esp32_brain.ino    # Main loop + integration
│   ├── config.h           # Pin assignments, structs, material DB
│   ├── ds_fusion.*        # Dempster-Shafer evidence fusion
│   ├── hd_slip.*          # Hyperdimensional slip detection
│   ├── success_predictor.*# Online grasp success predictor
│   ├── impedance.*        # Software lock-in amplifier
│   ├── sensors.*          # FSR force + slip detection
│   ├── grasp_planner.*    # 4-strategy FSM + repertoire
│   ├── comms.*            # UART protocol to STM32
│   └── web_server.*       # WiFi dashboard + WebSocket API
├── stm32_control/         # STM32H725 FreeRTOS firmware
│   ├── main.c/h           # 8 RTOS tasks, peripheral init
│   ├── depth_sensor.*     # VL53L8CX 8x8 ToF
│   ├── imu_sensor.*       # LSM6DSV16BX 6-axis IMU
│   ├── mic_sensor.*       # Onboard mic + CMSIS-DSP FFT
│   ├── servo_control.*    # 4-channel PWM servo driver
│   ├── motor_control.*    # STSPIN240 motor + encoders
│   ├── scanner_3d.*       # Photometric stereo 3D scanner
│   ├── ble_comm.*         # BLE mobile app interface
│   ├── uart_protocol.*    # UART protocol to ESP32
│   └── safety.*           # Watchdog, E-stop, force limits
├── hydra_app/             # Flutter companion app
│   ├── pubspec.yaml
│   └── lib/
│       ├── main.dart
│       ├── models/        # sensor_data.dart
│       ├── services/      # websocket_service.dart
│       └── widgets/       # dashboard, posterior, force, depth, log
├── 3d_parts/              # OpenSCAD gripper parts (5 files)
├── tools/                 # Training scripts
│   └── train_classifier.py
└── docs/                  # Build guide, wiring guide, blueprint
```

## Quick Start

### Flash STM32
1. Connect ST-LINK V2 flat ribbon to J1 header on ROBKIT1 main board
2. Power ROBKIT1 with battery
3. Open `stm32_control/` in STM32CubeIDE, target STM32H725IGT6
4. Build (Ctrl+B) and flash (Run > Run)

### Flash ESP32
1. Open `esp32_brain/esp32_brain.ino` in Arduino IDE
2. Board: ESP32 Dev Module, 240MHz, 4MB Flash
3. Install libraries: ArduinoJson, ESPAsyncWebServer, AsyncTCP
4. Upload

### Run Flutter App
```bash
cd hydra_app
flutter create .
flutter run
```
Connect phone to "HydraGrasp" WiFi (password: hydra2026), app auto-connects.

### Train ML Model (Optional)
```bash
cd tools
pip install scikit-learn numpy
python train_classifier.py
```

## Wiring

```
ROBKIT1 PD5 (TX) ──→ ESP32 GPIO17 (RX)
ROBKIT1 PD6 (RX) ←── ESP32 GPIO16 (TX)
ROBKIT1 GND ──────── ESP32 GND
ROBKIT1 PC6-PC8 ───→ Servo 1-3 signal
ROBKIT1 PA0 ←─────── E-stop button
Battery 5V ─────────→ Servo VCC rail
Battery GND ────────→ Common GND bus
ESP32 GPIO25 ───────→ Impedance DAC (via 1.5k + 100nF + 10k)
ESP32 GPIO34/35 ←───── Impedance ADC (with 1N4148 clamps)
ESP32 GPIO36/39/32 ←── FSR402 voltage dividers (10k pull-down)
```

## References

- Dempster-Shafer: Shafer, G. (1976). A Mathematical Theory of Evidence
- HD Computing: Neubert et al., "HD Computing for Tactile Sensing" (2024)
- Pre-Grasp Sensing: Hanson et al., arxiv:2207.00942 (2022)
- SLURP: Ramkumar et al., RIVeR Lab, Northeastern
- Knocker: Laput et al., ACM UIST (2019)
- emlearn: Nordby, J., github.com/emlearn/emlearn

## License

MIT
