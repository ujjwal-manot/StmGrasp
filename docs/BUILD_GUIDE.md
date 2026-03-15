# HYDRA GRASP v2.0 -- Build Guide

## Day 1: Board Setup

### Software to Install
1. STM32CubeIDE -- st.com/stm32cubeide
2. STM32CubeProgrammer -- st.com/stm32cubeprog
3. Arduino IDE 2.x -- arduino.cc/en/software
4. Flutter SDK -- flutter.dev
5. OpenSCAD -- openscad.org (for 3D parts)

### Arduino IDE Setup
Boards (File > Preferences > Additional Board URLs):
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
Install "esp32" by Espressif in Board Manager.

Libraries (Tools > Manage Libraries):
- ArduinoJson by Benoit Blanchon
- arduinoFFT by Enrique Condes

Manual install (download from GitHub, extract to Arduino/libraries/):
- ESPAsyncWebServer
- AsyncTCP

### Verify ROBKIT1
1. Connect ST-LINK V2 flat ribbon cable to J1 header on main board (pin 1 = red stripe)
2. Plug ST-LINK USB into laptop
3. Power ROBKIT1 with battery
4. Open STM32CubeProgrammer > ST-LINK > Connect
5. Should see: Device: STM32H725xx, Flash: 1024 KB

### Verify ESP32
1. Plug ESP32 into laptop via USB
2. Arduino IDE > Tools > Board: ESP32 Dev Module, Port: your COM
3. Upload File > Examples > WiFi > WiFiScan
4. Serial Monitor 115200 should show WiFi networks

## Day 2: Sensor Circuits

Build on breadboard:

### Impedance Spectroscopy Circuit
```
ESP32 GPIO25 → 1.5k → 100nF → 10k → Electrode A
                                 ├── 1k → GPIO34 (Vref)
                                 └── [material] → Electrode B
                                                    ├── 1k → GPIO35 (Vmut)
                                                    └── 10k → GND
Add 1N4148 clamp diodes on GPIO34 and GPIO35.
```
Test: electrodes touching = low |Z|. Separated = high |Z|.

### FSR Voltage Dividers (x3)
```
3.3V → [FSR402] → junction → [10k] → GND
                     └→ ESP32 ADC pin (GPIO36, 39, 32)
```
Test: press FSR, Serial Monitor should show voltage increase.

## Day 3: 3D Print + Gripper Assembly

### Print parts from 3d_parts/
| Part | Time | Supports |
|------|------|----------|
| gripper_base.scad | ~2.5h | Maybe (servo pockets) |
| gripper_finger.scad x2 | ~1h each | No |
| spring_contact_pad.scad x2 | ~15min each | No |
| sensor_backpack.scad | ~1.5h | No |

PLA, 0.2mm layers, 20% infill.

### Assembly
1. Press servos into gripper base pockets
2. Attach fingers to servo horns
3. Insert FSR402 into finger pockets
4. Thread copper tape through electrode channels
5. Mount gripper to ROBKIT1 with M3 screws
6. Connect servo signals to PC6, PC7, PC8

## Day 4: Flash Firmware + Test

### Flash STM32
1. Open stm32_control/ in STM32CubeIDE
2. New STM32 Project > Target: STM32H725IGT6
3. Copy source files to Core/Src/ and headers to Core/Inc/
4. Add linker section for SRAM4 (mic DMA buffer)
5. Add preprocessor define: ARM_MATH_CM7
6. Build (Ctrl+B) and flash (Run > Run)

### Flash ESP32
1. Open esp32_brain/esp32_brain.ino
2. Tools: ESP32 Dev Module, 240MHz, QIO, 4MB, 921600 upload
3. Upload (Ctrl+U)
4. Serial Monitor should show:
```
== HYDRA Grasp v2.0 ==
[INIT] Ready. Dashboard: http://192.168.4.1
```

### Setup Flutter App
```bash
cd hydra_app
flutter create .
# Copy lib/ files back if overwritten
flutter run
```

### Test Sequence
1. Power up ESP32 (USB) then ROBKIT1 (battery)
2. Connect phone to "HydraGrasp" WiFi (password: hydra2026)
3. Open Flutter app or browser at http://192.168.4.1
4. Wave hand in front of ToF -- depth grid should respond
5. Touch impedance electrodes to metal -- impedance panel updates
6. Press FSRs -- force bars respond
7. Tap START -- state machine runs through detect→scan→analyze→plan

## Day 5: Calibration + Demo Rehearsal

### Impedance Calibration
1. Press CALIBRATE button in app/dashboard with electrodes in open air
2. Touch each demo material 5-10 times, observe DS posterior updating
3. Verify correct material identification on dashboard

### Servo Calibration
1. Send grip commands, verify fingers open/close without binding
2. Adjust servo angle limits in config.h if needed

### Force Calibration
1. Place known weights on FSRs, verify force readings
2. Observe HD slip detection training progress (shown on dashboard)

### Demo Rehearsal
Run full sequence 3+ times:
1. Power on, IDLE state (blue heartbeat in app)
2. Present object → DETECTED → SCANNING → ANALYZING → TAP_TESTING
3. DS posterior updates live showing material probabilities
4. CLASSIFYING → PLANNING (strategy selected, P(success) shown)
5. APPROACHING → GRIPPING → HOLDING
6. RELEASE → back to IDLE

Target: under 60 seconds per cycle.

## Troubleshooting

| Problem | Fix |
|---------|-----|
| STM32 won't connect | Check ST-LINK ribbon orientation (pin 1), ensure battery connected |
| ESP32 won't upload | Hold BOOT button during upload, check COM port |
| No UART data | TX/RX swapped (most common), check GND connection |
| Servos jitter | Power from battery not USB, add 100uF cap |
| Impedance reads infinity | Check electrode wiring, DAC output ~1.65V on GPIO25 |
| FSR always 0 | Check pull-down resistor, FSR orientation |
| Dashboard blank | Connect to HydraGrasp WiFi first, check 192.168.4.1 |
| App can't connect | Ensure phone is on HydraGrasp WiFi, not mobile data |
