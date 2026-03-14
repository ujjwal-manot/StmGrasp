# HYDRA GRASP -- Complete Build Guide

Step-by-step instructions to go from unboxed components to a working prototype.

---

## Table of Contents

1. [Tools and Supplies Needed](#1-tools-and-supplies-needed)
2. [Day 1: Board Setup and Verification](#2-day-1-board-setup-and-verification)
3. [Day 2: Sensor Circuits on Breadboard](#3-day-2-sensor-circuits-on-breadboard)
4. [Day 3: 3D Print, Assemble Gripper, Wire Motors](#4-day-3-3d-print-assemble-gripper-wire-motors)
5. [Day 4: Flash Firmware and Integration Test](#5-day-4-flash-firmware-and-integration-test)
6. [Day 5: Calibration, Demo Rehearsal, Polish](#6-day-5-calibration-demo-rehearsal-polish)
7. [Connecting the STEVAL-ROBKIT1 to Your Laptop](#7-connecting-the-steval-robkit1-to-your-laptop)
8. [Flashing STM32 Firmware](#8-flashing-stm32-firmware)
9. [Flashing ESP32 Firmware](#9-flashing-esp32-firmware)
10. [Flashing NodeMCU Firmware](#10-flashing-nodemcu-firmware)
11. [Building the Sensor Circuits](#11-building-the-sensor-circuits)
12. [3D Printing the Parts](#12-3d-printing-the-parts)
13. [Assembling the Gripper](#13-assembling-the-gripper)
14. [Inter-Board Wiring](#14-inter-board-wiring)
15. [Power Setup](#15-power-setup)
16. [Testing Each Subsystem](#16-testing-each-subsystem)
17. [Full System Integration Test](#17-full-system-integration-test)
18. [Troubleshooting](#18-troubleshooting)

---

## 1. Tools and Supplies Needed

### Tools
- Soldering iron + solder (for FSR/piezo wires, optional for breadboard)
- Wire strippers
- Small Phillips screwdriver (for SG90 servo horn screws)
- Multimeter (for verifying connections and resistor values)
- USB cables: Micro-B for ST-LINK, Micro-B for ESP32, Micro-B for NodeMCU
- Laptop with Windows 10/11
- 3D printer (PLA, 0.2mm layer height)

### Software to Install (Do This First)
1. **STM32CubeIDE** -- https://www.st.com/en/development-tools/stm32cubeide.html
2. **STM32CubeProgrammer** -- https://www.st.com/en/development-tools/stm32cubeprog.html
3. **Arduino IDE 2.x** -- https://www.arduino.cc/en/software
4. **OpenSCAD** -- https://openscad.org/downloads.html (for previewing/exporting STLs)
5. **ST-LINK USB drivers** -- included with CubeIDE, or standalone from st.com

### Arduino IDE Board & Library Setup

**Boards:**
1. File > Preferences > Additional Board URLs, add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   http://arduino.esp8266.com/stable/package_esp8266com_index.json
   ```
2. Tools > Board Manager > Install "esp32" by Espressif and "esp8266" by ESP8266 Community

**Libraries (Tools > Manage Libraries):**
- `FastLED` by Daniel Garcia (for NodeMCU LEDs)
- `ArduinoJson` by Benoit Blanchon (for ESP32 dashboard)
- `arduinoFFT` by Enrique Condes (for ESP32 acoustic analysis)
- `ESPAsyncWebServer` -- install manually: download from GitHub, extract to Arduino/libraries/
- `AsyncTCP` -- install manually: download from GitHub, extract to Arduino/libraries/

---

## 2. Day 1: Board Setup and Verification

**Goal:** Get all 3 boards talking to your laptop. Verify each one works independently.

### Step 1: STEVAL-ROBKIT1

1. Unbox the kit. You'll have 3 PCBs:
   - **Main board** (large, STM32H725) -- has the 40-pin GPIO header and 14-pin SWD header (J1)
   - **Motor board** (medium, STM32G071) -- has its own 14-pin SWD header (J2) and motor connectors
   - **Imaging board** (small) -- has VL53L8CX ToF sensor and VD56G3 camera

2. Connect the imaging board to the main board via the 26-pin FPC cable. Lift the FPC connector latch, slide the cable in (contacts facing down), press latch closed. **Be gentle -- FPC connectors are fragile.**

3. Connect the motor board to the main board via the board-to-board connector (it plugs in mechanically).

4. Connect your **ST-LINK/V2** to the **J1** header (14-pin) on the main board using the flat cable.

5. Plug the ST-LINK into your laptop via USB.

6. Power the ROBKIT1 with its battery pack (or a bench 5V supply to the power input).

7. Open **STM32CubeProgrammer**:
   - Connection method: **ST-LINK**
   - Click **Connect**
   - You should see: `Device: STM32H7xx, Flash: 1024 KB`
   - If you see this, the board is working and your ST-LINK connection is good.

8. In STM32CubeIDE, go to Help > ST-LINK Upgrade to ensure your ST-LINK firmware is up to date.

### Step 2: ESP32 DevKit

1. Plug ESP32 DevKit into laptop via Micro-USB.
2. In Device Manager, note the COM port (e.g., COM3).
3. Open Arduino IDE > Tools:
   - Board: `ESP32 Dev Module`
   - Port: COM3 (your port)
   - Flash Size: `4MB`
   - Upload Speed: `921600`
4. Load File > Examples > WiFi > WiFiScan, click Upload.
5. Open Serial Monitor (115200 baud). You should see WiFi networks listed.

### Step 3: NodeMCU ESP8266

1. Plug NodeMCU into laptop via Micro-USB.
2. Note the COM port (different from ESP32).
3. Arduino IDE > Tools:
   - Board: `NodeMCU 1.0 (ESP-12E Module)`
   - Port: your COM port
   - Flash Size: `4MB`
4. Load File > Examples > Basics > Blink, upload. The onboard LED should blink.

### Step 4: Download STSW-ROBKIT1 (Optional but Recommended)

Download ST's official firmware package from https://www.st.com/en/embedded-software/stsw-robkit1.html. Flash it to verify the kit works with ST's own code first (motors, sensors, BLE all tested). Use the STRobotics mobile app to drive the robot. This confirms all onboard hardware is functional before you flash custom firmware.

---

## 3. Day 2: Sensor Circuits on Breadboard

**Goal:** Build and test each sensor circuit independently.

### Circuit 1: Impedance Spectroscopy

Build on breadboard per the schematic in WIRING_GUIDE.md section "Impedance Spectroscopy Circuit":

```
ESP32 GPIO25 (DAC) --> 1.5k resistor --> 100nF cap --> 10k resistor --> Electrode A
                                                        |
                                                        +-- 1k --> GPIO34 (Vref ADC)
                                                        |
                                                   [material]
                                                        |
                                                   Electrode B --+-- 1k --> GPIO35 (Vmut ADC)
                                                                 |
                                                                10k
                                                                 |
                                                                GND
```

Add 1N4148 clamp diodes on GPIO34 and GPIO35 (anode to GND, cathode to pin; anode to pin, cathode to 3.3V).

**Test:** Upload just the impedance module. Touch electrodes together (should read near 0 ohms, ~0 phase). Separate them (should read very high impedance, ~-89 phase). Touch a metal object (low impedance). Touch plastic (high impedance).

### Circuit 2: FSR Voltage Dividers (x3)

For each FSR402:
```
3.3V --- [FSR402] ---+--- [10k] --- GND
                     |
                     +--> ESP32 ADC pin (GPIO36, GPIO39, GPIO32)
```

**Test:** Press each FSR. Serial monitor should show voltage increase from 0 toward 3.3V.

### Circuit 3: Piezo Acoustic Input

```
Piezo (+) --- 10k --- +---> GPIO33
                      |
                      +--- 1M --- GND
                      |
                      +---|>|--- GND    (1N4148)
                      +---|>|--- 3.3V   (1N4148, reversed)
Piezo (-) --- GND
```

**Test:** Tap the piezo disc. Serial monitor should show voltage spikes.

### Circuit 4: TCRT5000 IR Array + ADS1115

For each TCRT5000:
- IR LED: 5V --- 100 ohm --- LED anode, LED cathode to GND
- Phototransistor: 3.3V --- 10k --- collector, emitter to GND. Tap collector to ADS1115 channel (A0-A3).

ADS1115: SDA to ESP32 GPIO21, SCL to GPIO22, VDD to 3.3V, GND to GND, ADDR to GND (address 0x48).

**Test:** Wave hand over each TCRT5000. ADS1115 readings should change with distance.

---

## 4. Day 3: 3D Print, Assemble Gripper, Wire Motors

### 3D Printing

1. Open each `.scad` file in OpenSCAD
2. Press F6 (Render) -- wait for it to finish
3. File > Export as STL
4. Print settings:
   - Material: PLA
   - Layer height: 0.2mm
   - Infill: 20%
   - Supports: Only for `gripper_base.scad` (servo pockets may need support)
   - Print `gripper_finger.scad` TWICE (once normal, once with `mirror_finger = true` for left/right pair)

Print order (by priority):
1. `gripper_base.scad` -- longest print (~2-3 hours)
2. `gripper_finger.scad` x2 -- ~1 hour each
3. `spring_contact_pad.scad` x2 -- ~15 min each
4. `sensor_backpack.scad` -- ~1.5 hours
5. `ir_led_ring.scad` -- ~30 min

### Gripper Assembly

1. Press-fit SG90 servos into the gripper base pockets
2. Attach servo horns to fingers with the small screws
3. Insert FSR402 sensors into the FSR pockets on each finger
4. Thread copper tape electrodes through the electrode channels
5. Mount TCRT5000 sensors into the TCRT slots (angled 10 deg inward)
6. If using piezo: glue piezo disc into the piezo mount recess with cyanoacrylate
7. Insert compression springs into spring sockets, press-fit contact pads on top
8. Run all sensor wires through the cable channel on the back of each finger
9. Route LED strip through the WS2812B channel on the gripper base
10. Mount the gripper base to the ROBKIT1 using M3 screws through the mounting plate

### Motor Board Wiring

The motor board connects to the main board via the board-to-board connector. The firmware controls it via I2C1 (address 0x20). No extra wiring needed -- it's already connected mechanically.

---

## 5. Day 4: Flash Firmware and Integration Test

### Flash STM32 (see Section 8 below for details)

1. Open `stm32_control/` as a project in STM32CubeIDE
2. Build (Ctrl+B)
3. Flash via ST-LINK (Run > Debug or Run > Run)

### Flash ESP32 (see Section 9 below for details)

1. Open `esp32_brain/esp32_brain.ino` in Arduino IDE
2. Select Board: ESP32 Dev Module, Port: your COM port
3. Click Upload

### Flash NodeMCU (see Section 10 below for details)

1. Open `nodemcu_leds/nodemcu_leds.ino` in Arduino IDE
2. Select Board: NodeMCU 1.0, Port: your COM port
3. Click Upload

### Integration Test Sequence

1. Power up all 3 boards
2. Connect to "HydraGrasp" WiFi on your phone
3. Open http://192.168.4.1 -- dashboard should load
4. Place an object 20-30cm in front of the ToF sensor -- depth heatmap should respond
5. Touch electrodes to different materials -- impedance panel should classify
6. Tap piezo -- acoustic spectrum should appear
7. Press FSRs -- force bars should respond
8. Check NodeMCU LEDs are animating based on state

---

## 6. Day 5: Calibration, Demo Rehearsal, Polish

### Impedance Calibration

1. Electrodes open air → record baseline (should be very high |Z|, near -89 phase)
2. Electrodes touched together → verify near-zero impedance
3. Touch each demo material 5-10 times → record average values
4. Update material signatures in `config.h` if measured values differ from defaults

### Acoustic Calibration

1. Tap each material with consistent force
2. Observe dominant frequency and spectral centroid on dashboard
3. Adjust thresholds in `acoustic.cpp` if needed

### Force Calibration

1. Place known weights on FSRs (100g, 200g, 500g)
2. Verify force readings on dashboard match expected values
3. Adjust FSR conversion constants in `sensors.cpp` if needed

### Servo Calibration

1. Command gripper open → verify fingers fully open without binding
2. Command gripper close → verify fingers close evenly
3. Adjust servo angle limits in `config.h` (`SERVO_MIN_ANGLE`, `SERVO_MAX_ANGLE`)

### Demo Rehearsal

Run through the full demo sequence 3+ times:
1. Power on, wait for idle animation
2. Present object → detect → scan → classify → plan → approach → grip → hold → release
3. Time the full sequence (target: under 60 seconds)
4. Practice explaining each step to an imaginary judge

---

## 7. Connecting the STEVAL-ROBKIT1 to Your Laptop

### What You Need

| Item | Purpose |
|------|---------|
| ST-LINK/V2 or STLINK-V3 debugger | Programming and debugging (NOT built into the kit) |
| 14-pin flat cable | Connects ST-LINK to J1 header on main board |
| USB cable (for ST-LINK) | Micro-B to laptop |
| Battery pack or 5V bench supply | Powers the kit (it won't run from ST-LINK power alone) |

### Connection Procedure

```
Laptop USB  →  ST-LINK/V2  →  14-pin cable  →  J1 header (main board)
                                                         |
                                              Battery → Power connector
```

1. **Plug ST-LINK** into laptop USB. Windows should auto-install drivers.
   - If not: install from STM32CubeIDE installation folder, or download "STSW-LINK009" from st.com
   - Check Device Manager: look for "STMicroelectronics STLink Virtual COM Port" and "ST-LINK Debug"

2. **Connect 14-pin cable** from ST-LINK to **J1** on the ROBKIT1 main board.
   - J1 is labeled "MCU_PROG" on the silkscreen
   - Pin 1 is marked with a small triangle/dot -- align the cable accordingly
   - **Red stripe on cable = pin 1**

3. **Power the board** with battery or bench supply.
   - The ST-LINK provides 3.3V on the SWD header but this is NOT enough to run the full board
   - Motors, sensors, and peripherals need the main power supply

4. **Verify in STM32CubeProgrammer:**
   - Open STM32CubeProgrammer
   - Select "ST-LINK" as the interface (top-right dropdown)
   - Click the blue **Connect** button
   - Success: you'll see device info:
     ```
     Device name: STM32H725xx
     Device ID: 0x480
     Flash size: 1024 KB
     ```
   - If connection fails: check cable orientation, ensure board is powered, try clicking "Reset" first

5. **Verify in STM32CubeIDE:**
   - Open STM32CubeIDE
   - If you have a project loaded: Run > Debug Configurations > STM32 C/C++ Application
   - Under "Debugger" tab: ST-LINK should be auto-detected
   - Click "Scan" to verify it finds the target

### Exploring the Board

Once connected, use STM32CubeProgrammer to:
- **Read flash:** See if ST's factory firmware is loaded
- **Read option bytes:** Check write protection, boot config
- **Read memory:** Browse the register map at runtime

### J1 Header Pinout (14-pin SWD)

```
   J1 (MCU_PROG) - Main Board
   ┌─────────────────────┐
   │  1   2   3   4   5  │  Pin 1: VDD_TARGET (3.3V from board)
   │  6   7   8   9  10  │  Pin 7: SWDIO
   │ 11  12  13  14      │  Pin 9: SWCLK
   └─────────────────────┘  Pin 3: GND
                            Pin 4: GND
```

### Programming the Motor Board (STM32G071)

To flash the motor board separately:
1. Move the 14-pin cable from J1 to **J2** on the motor board
2. In STM32CubeProgrammer, click "Connect" again
3. You should see: `Device: STM32G0xx`

The motor board runs its own firmware that accepts I2C commands from the main board. For HYDRA GRASP, we communicate with the motor board via I2C at address 0x20 -- you only need to program the main board in most cases.

---

## 8. Flashing STM32 Firmware

### Method 1: STM32CubeIDE (Recommended for Development)

1. Open STM32CubeIDE
2. File > Import > Existing Projects into Workspace
3. Browse to `StmGrasp/stm32_control/`
4. If there's no `.project` file, create a new STM32 project:
   - File > New > STM32 Project
   - Target: STM32H725IGT6
   - Project name: `stm32_control`
   - Copy all source files from the repo into `Core/Src/` and headers into `Core/Inc/`

5. **CRITICAL: Configure the linker for SRAM4 section.**
   Open the linker script (`.ld` file) and add this section for the microphone DMA buffer:
   ```
   .sram4 (NOLOAD) :
   {
     . = ALIGN(4);
     *(.sram4)
     *(.sram4*)
     . = ALIGN(4);
   } >RAM_D3
   ```
   The `RAM_D3` memory region should already be defined for STM32H725 (0x38000000, 64KB).

6. **Configure CMSIS-DSP:**
   - Project Properties > C/C++ Build > Settings > MCU GCC Compiler > Preprocessor > Add `ARM_MATH_CM7`
   - Link against `libarm_cortexM7lfsp_math.a` (or add CMSIS-DSP source files directly)

7. Build: Ctrl+B (or Project > Build All)
8. Flash: Run > Run As > STM32 C/C++ Application
9. The ST-LINK will flash and start the firmware. Status LED on ROBKIT1 should go steady.

### Method 2: STM32CubeProgrammer (Quick Flash)

1. Build in CubeIDE first to get the `.bin` or `.hex` file
2. Open STM32CubeProgrammer
3. Connect to ST-LINK
4. Open the `.bin` file (typically in `Debug/` or `Release/` folder)
5. Set start address to `0x08000000` (flash start)
6. Click "Start Programming"
7. Click "Disconnect", then reset the board

### Verifying the Flash

After flashing:
- Status LED (PH13) should turn on steady (not blinking fast -- that's Error_Handler)
- If LED blinks rapidly: a peripheral init failed. Check the FPC cable, battery, and I2C connections
- Connect a serial terminal to the ST-LINK Virtual COM Port at 115200 baud to see any debug output

---

## 9. Flashing ESP32 Firmware

1. Open Arduino IDE
2. Open `esp32_brain/esp32_brain.ino`
3. Verify all 7 tabs are present (config.h, impedance.h/cpp, acoustic.h/cpp, sensors.h/cpp, grasp_planner.h/cpp, comms.h/cpp, web_server.h/cpp)
4. Tools settings:
   ```
   Board:          ESP32 Dev Module
   Upload Speed:   921600
   CPU Frequency:  240MHz (WiFi/BT)
   Flash Frequency: 80MHz
   Flash Mode:     QIO
   Flash Size:     4MB (32Mb)
   Partition:      Default 4MB with spiffs
   Port:           (your COM port)
   ```
5. Click Upload (Ctrl+U)
6. Wait for "Connecting..." -- you may need to hold the BOOT button on the ESP32 while it connects
7. After upload, open Serial Monitor at 115200 baud
8. You should see:
   ```
   HYDRA GRASP - Sensor Fusion Brain
   WiFi AP started: HydraGrasp
   IP: 192.168.4.1
   WebSocket server started on /ws
   ```

### Troubleshooting ESP32 Upload

- **"Failed to connect"**: Hold BOOT button on ESP32 while uploading, release after "Connecting..." appears
- **"A fatal error occurred: Could not open COM port"**: Close Serial Monitor first, or check if another program is using the port
- **Compile errors about ESPAsyncWebServer**: Make sure you installed the library manually (not available via Library Manager). Download from https://github.com/me-no-dev/ESPAsyncWebServer and extract to `Arduino/libraries/`

---

## 10. Flashing NodeMCU Firmware

1. Open Arduino IDE
2. Open `nodemcu_leds/nodemcu_leds.ino`
3. Tools settings:
   ```
   Board:          NodeMCU 1.0 (ESP-12E Module)
   Upload Speed:   115200
   CPU Frequency:  80MHz
   Flash Size:     4MB (FS:2MB OTA:~1019KB)
   Port:           (your COM port)
   ```
4. Click Upload
5. After upload, the LED strip should show a brief test pattern (all white flash), then settle into blue breathing animation (IDLE mode)

---

## 11. Building the Sensor Circuits

### Breadboard Layout Suggestion

Use a single full-size breadboard (830 tie-points). Layout from left to right:

```
[Impedance Circuit] | [FSR Dividers x3] | [Piezo Circuit] | [ADS1115 + TCRT5000 x4]
      Column A-F         Column G-J          Column K-N          Column O-T
```

### Build Order

1. **Power rails first:** 3.3V and GND from ESP32 to breadboard power rails
2. **ADS1115 module:** Place near ESP32. Wire SDA (GPIO21), SCL (GPIO22), VDD (3.3V), GND, ADDR to GND
3. **FSR dividers:** 3x identical circuits. 10k pull-down to GND, FSR to 3.3V, junction to ADC pin
4. **Piezo circuit:** 10k series, 1M bias, diode clamps
5. **Impedance circuit:** DAC output through 1.5k + cap + 10k, electrode connections, ADC inputs with clamp diodes
6. **TCRT5000 connections:** IR LED power (5V through 100 ohm), phototransistor collectors to ADS1115 channels

### Wire Color Convention (Recommended)

| Color | Signal |
|-------|--------|
| Red | 5V power |
| Orange | 3.3V power |
| Black | GND |
| Yellow | I2C SCL |
| Blue | I2C SDA |
| Green | UART TX |
| White | UART RX |
| Purple | Analog signals |
| Grey | Digital I/O |

---

## 12. 3D Printing the Parts

### Export from OpenSCAD

For each `.scad` file:
1. Open in OpenSCAD
2. Press F5 for quick preview (check it looks right)
3. Press F6 for full render (takes 10-60 seconds)
4. File > Export as STL
5. Save to a `stl_exports/` folder

### Print Settings

| Part | Time | Supports | Notes |
|------|------|----------|-------|
| `gripper_finger.scad` | ~60 min | No | Print flat side down. Print TWICE (set `mirror_finger=true` for second) |
| `gripper_base.scad` | ~150 min | Maybe | Print open-side up. May need supports for servo pocket overhangs |
| `ir_led_ring.scad` | ~30 min | No | Print flat side down |
| `sensor_backpack.scad` | ~90 min | No | Print tray-side up |
| `spring_contact_pad.scad` | ~15 min | No | Print flat side down. Print x2 |

### Post-Processing

- Remove any stringing between parts
- Test-fit servos in pockets (may need light sanding)
- Test-fit FSR sensors in pockets (should be snug, not forced)
- Verify sensor backpack standoffs align with ESP32 and NodeMCU mounting holes

---

## 13. Assembling the Gripper

### Step-by-step with Diagrams

```
Step 1: Mount servos into gripper base
┌────────────────────┐
│  ┌─────┐  ┌─────┐ │
│  │SG90 │  │SG90 │ │   Two finger servos side by side
│  │  1  │  │  2  │ │
│  └─────┘  └─────┘ │
│       ┌─────┐     │
│       │SG90 │     │   Wrist servo (optional) at rear
│       │  3  │     │
│       └─────┘     │
│  [LED strip runs   │
│   along top edge]  │
└────────────────────┘

Step 2: Attach fingers to servo horns
         Servo horn screw
              │
    ┌─────────┤
    │ Finger  │──── Cable channel (rear)
    │         │
    │ [FSR]   │──── FSR pocket (front face)
    │ [TCRT]  │──── TCRT slot (side, angled)
    │ [Elec]  │──── Electrode channel (tip)
    │ [Spring]│──── Spring socket (contact face)
    └─────────┘

Step 3: Install sensors into fingers
- Press FSR402 into pocket (should friction-fit)
- Thread copper tape through electrode channels
- Push TCRT5000 into slots
- (Optional) Glue piezo disc into mount

Step 4: Mount to ROBKIT1
- Use M3 screws through the gripper base mounting plate
- Align with the 40-pin header area on the ROBKIT1
- Connect servo signal wires to the GPIO header (PC6, PC7, PC8, PC9)
- Connect servo power wires to battery power rail

Step 5: Attach sensor backpack
- Mount sensor backpack tray to rear of ROBKIT1
- Secure ESP32 and NodeMCU with standoffs
- Place breadboard with sensor circuits in the breadboard recess
```

---

## 14. Inter-Board Wiring

### Critical Connections Checklist

| Connection | Wire From | Wire To | Notes |
|------------|-----------|---------|-------|
| UART (STM32 → ESP32) | ROBKIT1 header PD5 (TX) | ESP32 GPIO17 (RX) | Cross TX/RX! |
| UART (ESP32 → STM32) | ESP32 GPIO16 (TX) | ROBKIT1 header PD6 (RX) | Cross TX/RX! |
| UART (ESP32 → NodeMCU) | ESP32 GPIO5 (TX) | NodeMCU RX pin | One-way only |
| GND (all boards) | ROBKIT1 GND | ESP32 GND | **MANDATORY** |
| GND (all boards) | ESP32 GND | NodeMCU GND | **MANDATORY** |
| Servo 1 signal | ROBKIT1 PC6 | Servo 1 signal wire | Orange wire on SG90 |
| Servo 2 signal | ROBKIT1 PC7 | Servo 2 signal wire | |
| Servo 3 signal | ROBKIT1 PC8 | Servo 3 signal wire | |
| Servo power | Battery 5V rail | Servo red wires | **NOT from USB!** |
| Servo GND | Battery GND | Servo brown wires | |

### Wiring Verification Procedure

Before powering on, use a multimeter in continuity mode:

1. **GND bus:** Touch probes between ROBKIT1 GND, ESP32 GND, NodeMCU GND, breadboard GND rail. All should beep (continuity).
2. **No shorts:** Check there is NO continuity between 3.3V and GND, or between 5V and GND.
3. **UART TX/RX:** Verify with multimeter that ESP32 GPIO16 connects to ROBKIT1 PD6 (not PD5!). Getting TX/RX swapped is the #1 wiring mistake.
4. **Servo signals:** Verify each servo signal wire goes to the correct timer channel pin.

---

## 15. Power Setup

### Two-Domain Power Architecture

```
Domain 1: ROBKIT1 + Servos + Motors
├── Source: Battery pack (7.4V LiPo or 4xAA holder)
├── ROBKIT1 has onboard regulator → 3.3V for MCU
├── Servo power rail → 5V-6V from battery
└── Motor board → powered through board connector

Domain 2: ESP32 + NodeMCU + Sensors + LEDs
├── Source: 5V USB power supply (≥ 3A)
├── ESP32 VIN pin + 100uF decoupling cap
├── NodeMCU VIN pin
├── WS2812B strip VCC + 1000uF decoupling cap
├── TCRT5000 IR LEDs (5V through 100 ohm)
└── ADS1115 VDD from ESP32 3.3V pin

COMMON GND BUS connects both domains
```

### Power-Up Sequence

1. Connect Domain 2 (USB 5V) first -- ESP32 and NodeMCU boot
2. Connect Domain 1 (battery) second -- ROBKIT1 and servos activate
3. This order prevents servos from twitching before the STM32 initializes their PWM

### Power-Down Sequence

1. Disconnect Domain 1 (battery) first -- servos de-energize safely
2. Disconnect Domain 2 (USB 5V) -- ESP32 and NodeMCU shut down

---

## 16. Testing Each Subsystem

### Test 1: UART Communication (STM32 ↔ ESP32)

1. Flash both boards with their respective firmware
2. Open ESP32 Serial Monitor (115200 baud)
3. Look for messages like "STM32 heartbeat OK" or depth grid data
4. If nothing: check TX/RX wiring, check baud rate, check GND connection

### Test 2: Depth Sensor (VL53L8CX)

1. With STM32 running, the depth grid should auto-forward to ESP32 at 5Hz
2. On the WiFi dashboard, the 8x8 depth heatmap should show live data
3. Wave hand at different distances -- colors should change
4. If blank: check FPC cable on imaging board, ensure it's fully seated

### Test 3: IMU

1. On dashboard or Serial Monitor, look for IMU data (pitch/roll/tilt)
2. Tilt the ROBKIT1 -- values should change
3. If zeros: IMU is on the main board, should work if board is powered

### Test 4: Servos

1. Through the dashboard or BLE, send a grip command
2. Servos should move smoothly
3. If jittering: power supply issue (servos need battery, not USB)
4. If not moving: check signal wire connections to PC6-PC9

### Test 5: Motors

1. Send motor command through dashboard
2. Wheels should turn
3. If not: motor board may need separate firmware, or check board-to-board connector

### Test 6: LED Strip

1. NodeMCU should show blue breathing animation immediately after boot
2. When ESP32 sends state changes, LED animation should change
3. If no LEDs: check data pin (GPIO2/D4), check 330 ohm resistor, check 5V power to strip

### Test 7: Impedance

1. Touch electrodes to metal → dashboard shows "metal", low |Z|
2. Touch plastic → shows "plastic", high |Z|
3. Open air → very high |Z|, near -89 phase
4. If unstable: shorten electrode wires, add shielding, check decoupling caps

### Test 8: WiFi Dashboard

1. Connect phone to "HydraGrasp" WiFi
2. Open http://192.168.4.1
3. All 8 panels should show live data
4. If page doesn't load: ESP32 may not have started WiFi AP. Check serial output.

---

## 17. Full System Integration Test

### Checklist

Run through this sequence end-to-end:

- [ ] Power on all boards (Domain 2 first, then Domain 1)
- [ ] NodeMCU shows blue breathing animation
- [ ] Connect to HydraGrasp WiFi, dashboard loads
- [ ] Depth heatmap is live (wave hand)
- [ ] Place object at ~30cm → state transitions to DETECTED
- [ ] Impedance classifies material correctly
- [ ] Acoustic tap produces FFT spectrum
- [ ] Cross-validation shows combined result with confidence
- [ ] Grasp plan appears with force target
- [ ] Motor approach drives toward object (if using motors)
- [ ] Gripper closes with force-ramped control
- [ ] Force bars show FSR readings in real time
- [ ] Slip detection responds to micro-slip (tug gently)
- [ ] LED animation changes with each state
- [ ] Release command opens gripper smoothly
- [ ] E-stop button opens gripper immediately
- [ ] BLE commands work from phone app (if using BLE)
- [ ] System recovers after E-stop (power cycle)

---

## 18. Troubleshooting

### STM32 Won't Connect

| Symptom | Fix |
|---------|-----|
| STM32CubeProgrammer shows "No ST-LINK detected" | Check USB cable to ST-LINK. Try different USB port. Reinstall drivers. |
| "Cannot connect to target" | Ensure board is powered. Check 14-pin cable orientation (pin 1 alignment). Try "Connect Under Reset" mode in CubeProgrammer. |
| "Flash memory write error" | Board may have read protection enabled. Go to Option Bytes > Read Out Protection > Set to Level 0, then try again. |

### ESP32 Won't Upload

| Symptom | Fix |
|---------|-----|
| "Failed to connect to ESP32" | Hold BOOT button during upload. Check USB cable (some are charge-only). |
| "Compilation error" | Check all libraries are installed (especially ESPAsyncWebServer, AsyncTCP manually). |
| "WiFi AP not starting" | ADC2 pins conflict with WiFi. Our code already uses ADC1 only. Check for other libraries using ADC2. |

### No UART Communication

| Symptom | Fix |
|---------|-----|
| ESP32 not receiving from STM32 | TX/RX swapped (most common). Verify with multimeter. |
| Garbled data | Baud rate mismatch. Both must be 115200. |
| Intermittent data | Missing GND connection between boards. |

### Servo Issues

| Symptom | Fix |
|---------|-----|
| Servo jitters/twitches | Power supply insufficient. Use battery, not USB. Add 100uF cap near servo power. |
| Servo doesn't move | Check signal wire to correct TIM3 channel pin. Check PWM output with oscilloscope if available. |
| Servo buzzes but doesn't rotate | Mechanical binding in 3D printed parts. Sand pocket for clearance. |

### Sensor Issues

| Symptom | Fix |
|---------|-----|
| Impedance reads infinity for everything | Check electrode connections. Check 100nF cap orientation. Check DAC output with multimeter (should see ~1.65V average on GPIO25). |
| FSR always reads 0 | Check pull-down resistor. Check FSR wiring (not reversed). |
| ADS1115 not responding | Check I2C address (should be 0x48 with ADDR to GND). Check SDA/SCL connections. Check 3.3V power. |
| Depth sensor shows no data | FPC cable on imaging board not seated. Gently re-seat it. |
| IMU shows all zeros | IMU is onboard, should work if main board is powered. Check I2C bus (shared with ToF). |

### LED Strip Issues

| Symptom | Fix |
|---------|-----|
| No LEDs light up | Check 5V power to strip. Check GND. Check data pin (GPIO2/D4). |
| First LED works, rest don't | Data pin works but strip power is insufficient. Check 5V supply current rating. |
| Colors are wrong (GRB vs RGB) | FastLED is configured for `GRB` order (standard for WS2812B). If using WS2811, change to `RGB`. |
| LEDs flicker randomly | Add 1000uF cap across strip power. Shorten data wire. Add 330 ohm series resistor. |
