# HYDRA GRASP -- Wiring Guide

Comprehensive wiring reference for all three MCUs, sensors, actuators, and power distribution.

---

## Table of Contents

1. [ESP32 Pin Assignments](#esp32-pin-assignments)
2. [STM32 STEVAL-ROBKIT1 Connections](#stm32-steval-robkit1-connections)
3. [NodeMCU ESP8266 Connections](#nodemcu-esp8266-connections)
4. [Impedance Spectroscopy Circuit](#impedance-spectroscopy-circuit)
5. [FSR Voltage Divider](#fsr-voltage-divider)
6. [Piezo Acoustic Circuit](#piezo-acoustic-circuit)
7. [TCRT5000 IR Array to ADS1115](#tcrt5000-ir-array-to-ads1115)
8. [WS2812B LED Strip](#ws2812b-led-strip)
9. [Power Architecture](#power-architecture)
10. [Critical Notes](#critical-notes)

---

## ESP32 Pin Assignments

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 25 | Impedance DAC output | OUT | DAC1 -- 1 kHz sine excitation |
| 34 | Impedance Vref ADC | IN | ADC1_CH6 -- reference voltage sense |
| 35 | Impedance Vmut ADC | IN | ADC1_CH7 -- material-under-test voltage |
| 36 (VP) | FSR 1 (left finger) | IN | ADC1_CH0 -- force-sensitive resistor |
| 39 (VN) | FSR 2 (right finger) | IN | ADC1_CH3 -- force-sensitive resistor |
| 32 | FSR 3 (slip detection) | IN | ADC1_CH4 -- high-rate slip sampling |
| 33 | Piezo ADC (acoustic tap) | IN | ADC1_CH5 -- piezo disc signal |
| 21 | I2C SDA (ADS1115) | I/O | Data line for IR curvature array |
| 22 | I2C SCL (ADS1115) | OUT | Clock line for IR curvature array |
| 16 | UART TX to STM32 | OUT | TX2 -- commands to gripper controller |
| 17 | UART RX from STM32 | IN | RX2 -- depth data and status from STM32 |
| 5 | UART TX to NodeMCU | OUT | Software serial -- LED commands |
| 4 | Spare / user button | IN | Pull-up, active low |

```
                         ESP32 DevKit v1
                     +---------------------+
                     |                     |
              3V3 ---|  3V3           VIN  |--- 5V in
              GND ---|  GND           GND  |--- GND
    Piezo ADC  <-----|  GPIO33      GPIO23 |
   FSR3 (slip) <-----|  GPIO32      GPIO22 |-----> I2C SCL (ADS1115)
   Imp. Vmut   <-----|  GPIO35      GPIO21 |-----> I2C SDA (ADS1115)
   Imp. Vref   <-----|  GPIO34       GPIO5 |-----> UART TX to NodeMCU
   FSR2 right  <-----|  GPIO39(VN)  GPIO17 |<---- UART RX from STM32
   FSR1 left   <-----|  GPIO36(VP)  GPIO16 |-----> UART TX to STM32
                     |  EN          GPIO4  |-----> Spare button
                     |  3V3         GPIO2  |
   Imp. DAC    <-----|  GPIO25      GPIO15 |
                     |  GPIO26      GPIO13 |
                     |  GPIO27      GPIO12 |
                     |  GPIO14      GPIO14 |
                     +---------------------+
```

---

## STM32 STEVAL-ROBKIT1 Connections

The STEVAL-ROBKIT1 exposes STM32H725 peripherals through a 40-pin RPi-compatible header and dedicated connectors.

### UART to ESP32 (USART2, on 40-pin header)

| ROBKIT1 Pin | STM32 Pin | Function | Connect to |
|-------------|-----------|----------|------------|
| Header TX | PD5 | USART2 TX | ESP32 GPIO17 (RX) |
| Header RX | PD6 | USART2 RX | ESP32 GPIO16 (TX) |
| Header GND | GND | Ground | ESP32 GND |

Baud rate: 115200, 8N1. Both sides are 3.3V logic -- no level shifter required.

### VL53L8CX ToF Sensor (I2C1, via FPC)

| STM32 Pin | Function | Notes |
|-----------|----------|-------|
| PB8 | I2C1 SCL | Connected via FPC to imaging board |
| PB9 | I2C1 SDA | Connected via FPC to imaging board |

The VL53L8CX is mounted on the ROBKIT1 imaging daughter board and connects through the flat flex cable. I2C address: 0x29. No external wiring needed -- just ensure the FPC cable is properly seated.

### Servo PWM (TIM3, on GPIO header)

| STM32 Pin | TIM3 Channel | Function | Servo |
|-----------|-------------|----------|-------|
| PC6 | CH1 | Gripper open/close | SG90 or MG996R |
| PC7 | CH2 | Wrist rotation | SG90 |
| PC8 | CH3 | Auxiliary 1 | SG90 (optional) |
| PC9 | CH4 | Auxiliary 2 | SG90 (optional) |

PWM: 50 Hz, pulse width 500-2400 us (0-180 degrees).

### Other Connections

| STM32 Pin | Function | Notes |
|-----------|----------|-------|
| PA0 | E-stop button | Active low, internal pull-up |
| PH13 | Status LED | On-board, active high |

```
   STEVAL-ROBKIT1
   +------------------------------------------+
   |                                          |
   |  [FPC] --> VL53L8CX 8x8 ToF             |
   |            (I2C1: PB8 SCL, PB9 SDA)      |
   |                                          |
   |  [40-pin GPIO Header]                    |
   |   PD5 (TX) -------> ESP32 GPIO17 (RX)   |
   |   PD6 (RX) <------- ESP32 GPIO16 (TX)   |
   |   GND -------------- ESP32 GND           |
   |                                          |
   |   PC6 (TIM3_CH1) --> Servo 1 (gripper)   |
   |   PC7 (TIM3_CH2) --> Servo 2 (wrist)     |
   |   PC8 (TIM3_CH3) --> Servo 3 (aux)       |
   |   PC9 (TIM3_CH4) --> Servo 4 (aux)       |
   |                                          |
   |   PA0 <------------- E-stop button       |
   |                                          |
   |  [Battery connector]                     |
   |   VBAT --> internal regulator --> 3.3V   |
   |   VBAT --> servo power rail              |
   +------------------------------------------+
```

---

## NodeMCU ESP8266 Connections

| NodeMCU Pin | GPIO | Function | Connect to |
|-------------|------|----------|------------|
| RX (D9) | GPIO3 | Serial RX | ESP32 GPIO5 (TX) |
| D4 | GPIO2 | WS2812B data | 330 ohm resistor to strip DIN |
| VIN | -- | 5V input | External 5V supply |
| GND | -- | Ground | Common GND bus |

Baud rate from ESP32: 9600, 8N1. The NodeMCU receives LED commands (color, animation mode, material ID, force map) and drives the WS2812B strip.

```
   NodeMCU ESP8266
   +---------------------+
   |                     |
   |  RX  <------------- ESP32 GPIO5 (UART TX, 9600 baud)
   |  GPIO2 (D4) ------> 330 ohm ---> WS2812B DIN
   |  VIN  <------------- 5V external supply
   |  GND  -------------- Common GND
   |                     |
   +---------------------+
```

---

## Impedance Spectroscopy Circuit

The impedance engine uses the ESP32 DAC to generate a 1 kHz sine wave, passes it through a known reference resistor and the material under test, then samples both voltages for lock-in demodulation.

```
                    1.5k         100nF           10k (R_ref)
  ESP32 GPIO25 ---/\/\/\---||-----/\/\/\---+--- Electrode A
  (DAC output)   (current   (DC block)     |    (touch material)
                  limit)                   |
                                           |  1k
                                           +--/\/\/\---> ESP32 GPIO34
                                           |             (Vref ADC)
                                           |
                                      [MATERIAL]
                                           |
                                    Electrode B ---+
                                                   |
                                                   |  1k
                                                   +--/\/\/\---> ESP32 GPIO35
                                                   |             (Vmut ADC)
                                                   |
                                                  10k
                                                   |
                                                  GND


  ADC Input Protection (on both GPIO34 and GPIO35):

                      To ESP32 ADC pin
                            |
               3.3V ---|-<--+-->-|--- GND
                     1N4148     1N4148
                   (clamp to  (clamp to
                    3.3V)      GND)
```

### Component Values

| Component | Value | Purpose |
|-----------|-------|---------|
| R_series | 1.5k ohm | Limits DAC output current |
| C_block | 100 nF | Blocks DC, passes AC excitation |
| R_ref | 10k ohm | Known reference for impedance calculation |
| R_adc (x2) | 1k ohm | Protects ADC inputs from overvoltage |
| R_gnd | 10k ohm | Provides DC return path for electrode B |
| D_clamp (x4) | 1N4148 | Clamps ADC inputs between GND and 3.3V |

### How It Works

1. DAC outputs a 1 kHz sine centered at 1.65V (mid-rail)
2. AC signal passes through DC-blocking cap into the voltage divider formed by R_ref and the unknown impedance Z_material
3. Vref (measured across R_ref) and Vmut (measured across the material) are sampled synchronously
4. Lock-in demodulation extracts magnitude and phase of the complex impedance
5. Material is classified by nearest-neighbor matching in (log|Z|, phase) space

---

## FSR Voltage Divider

Each FSR402 is wired as a simple voltage divider with a 10k pull-down resistor. Under no force, the FSR resistance is very high and the voltage at the ADC pin is near zero. As force increases, FSR resistance drops and the voltage rises toward 3.3V.

```
  One FSR channel (repeat for all 3):

     3.3V
      |
      |
    [FSR402]     (resistance decreases with force)
      |
      +-----------> ESP32 ADC pin (GPIO36, 39, or 32)
      |
    [10k]        (pull-down resistor)
      |
     GND
```

### FSR Pin Mapping

| FSR | Position | ESP32 GPIO | ADC Channel |
|-----|----------|------------|-------------|
| FSR1 | Left finger | GPIO36 (VP) | ADC1_CH0 |
| FSR2 | Right finger | GPIO39 (VN) | ADC1_CH3 |
| FSR3 | Slip detection (fingertip) | GPIO32 | ADC1_CH4 |

FSR3 is sampled at 1 kHz in burst mode for high-frequency slip detection (AC RMS analysis). FSR1 and FSR2 are sampled at 20 Hz for static force measurement and center-of-pressure calculation.

---

## Piezo Acoustic Circuit

The piezo disc generates voltage spikes when tapped. The circuit biases the output to mid-rail and clamps it within ADC-safe range.

```
                                 10k
  Piezo (+) ----/\/\/\----+-----------> ESP32 GPIO33 (ADC)
                          |
                          |   1M
                          +--/\/\/\--- GND     (DC bias to ~0V, high impedance)
                          |
                          +---|--->--- GND     (1N4148 clamp negative)
                          |
                          +---|--->--- 3.3V    (1N4148 clamp positive)
                          |
  Piezo (-) ------------ GND
```

### Component Values

| Component | Value | Purpose |
|-----------|-------|---------|
| R_series | 10k ohm | Limits current into ADC |
| R_bias | 1M ohm | Provides DC bias path, high impedance to preserve signal |
| D_neg | 1N4148 | Clamps negative-going spikes to GND |
| D_pos | 1N4148 | Clamps positive-going spikes to 3.3V |

### How It Works

1. Servo executes a controlled tap motion (commanded by ESP32 via STM32)
2. Piezo generates a damped oscillation signal on impact
3. ESP32 samples at 8 kHz for 200 ms (1600 samples)
4. 1024-point FFT extracts dominant frequency, spectral centroid, and decay ratio
5. Tap material is classified independently, then cross-validated with impedance result

---

## TCRT5000 IR Array to ADS1115

Four TCRT5000 reflective optical sensors are arranged in a 2x2 pattern on the gripper face. Each sensor has an IR LED emitter and a phototransistor receiver. The phototransistor outputs are read by an ADS1115 16-bit ADC over I2C.

```
  One TCRT5000 channel (repeat for all 4):

  IR LED side:                    Phototransistor side:

     5V                              3.3V
      |                               |
    [100 ohm]   (current limit)     [10k]    (pull-up resistor)
      |                               |
    LED anode                    Collector ---+---> ADS1115 channel (A0-A3)
      |                               |      |
    LED cathode                   Emitter     |
      |                               |      |
     GND                             GND     |
                                              |
                                        (voltage drops as
                                         reflection increases)


  ADS1115 (I2C address 0x48):

     +-----------+
     |  ADS1115  |
     |           |
     |  A0 <---- TCRT5000 #1 (top-left)
     |  A1 <---- TCRT5000 #2 (top-right)
     |  A2 <---- TCRT5000 #3 (bottom-left)
     |  A3 <---- TCRT5000 #4 (bottom-right)
     |           |
     |  SDA ---- ESP32 GPIO21
     |  SCL ---- ESP32 GPIO22
     |  VDD ---- 3.3V
     |  GND ---- GND
     |  ADDR --- GND (address 0x48)
     +-----------+
```

### Curvature Detection

- **Flat surface:** all 4 channels read similar values
- **Convex surface:** center channels read higher (closer), edges lower
- **Concave surface:** center channels read lower, edges higher
- **Edge/corner:** sharp gradient between adjacent channels
- **Cylinder:** gradient along one axis, uniform along the other

---

## WS2812B LED Strip

The WS2812B addressable LED strip is driven by the NodeMCU. It provides visual feedback for material classification, force levels, grasp state, and animations.

```
   NodeMCU                    WS2812B Strip
   GPIO2 (D4) ---[330 ohm]---> DIN

                                VCC ---+--- External 5V supply
                                       |
                                    [1000 uF]  (electrolytic, across VCC/GND)
                                       |
                                GND ---+--- Common GND
```

### Wiring Notes

- The 330 ohm resistor on the data line prevents signal reflections and ringing
- The 1000 uF capacitor absorbs current spikes when many LEDs change simultaneously
- Power the strip from an external 5V supply -- never from the NodeMCU 3.3V or USB
- At 30 LEDs, peak current is ~1.8A (60 mA per LED at full white). Size the supply accordingly
- Connect the NodeMCU GND to the strip GND and to the external supply GND

---

## Power Architecture

Two independent power domains with a common ground bus.

```
   +==============================================================+
   |                     POWER DOMAIN 1                            |
   |                  (Kit Battery Supply)                         |
   |                                                               |
   |  [Battery Pack] ----+----> STEVAL-ROBKIT1 (STM32H725)        |
   |                     |      (internal 3.3V regulator)          |
   |                     |                                         |
   |                     +----> Servo power rail                   |
   |                            (PC6-PC9 servo VCC)                |
   |                            Servo 1 (gripper)                  |
   |                            Servo 2 (wrist)                    |
   |                            Servo 3 (aux)                      |
   |                            Servo 4 (aux)                      |
   +==============================================================+
                              |
                              |  COMMON GND BUS
                              |  (thick wire / copper strip)
                              |
   +==============================================================+
   |                     POWER DOMAIN 2                            |
   |                  (USB 5V Supply, >= 3A)                       |
   |                                                               |
   |  [5V USB Supply] ---+----> ESP32 DevKit (VIN pin)            |
   |                     |      +-- [100 uF] decoupling on VIN    |
   |                     |                                         |
   |                     +----> NodeMCU ESP8266 (VIN pin)          |
   |                     |                                         |
   |                     +----> WS2812B LED strip (VCC)            |
   |                     |      +-- [1000 uF] decoupling at strip  |
   |                     |                                         |
   |                     +----> TCRT5000 IR LEDs (5V via 100 ohm)  |
   |                     |                                         |
   |                     +----> ADS1115 VDD (3.3V from ESP32 pin)  |
   |                                                               |
   +==============================================================+
                              |
                              |  COMMON GND BUS
                              |
        All GND pins connected: ESP32 GND, NodeMCU GND,
        STM32 header GND, ADS1115 GND, servo GND,
        WS2812B GND, FSR dividers, piezo, impedance circuit,
        TCRT5000 cathodes/emitters, 5V supply GND, battery GND
```

### Power Budget Estimate

| Component | Voltage | Typical Current | Max Current |
|-----------|---------|-----------------|-------------|
| ESP32 | 5V (VIN) | 80 mA | 250 mA |
| NodeMCU | 5V (VIN) | 50 mA | 170 mA |
| WS2812B (15 LEDs active) | 5V | 300 mA | 900 mA |
| TCRT5000 x4 (IR LEDs) | 5V | 80 mA | 120 mA |
| ADS1115 | 3.3V | 0.2 mA | 0.5 mA |
| FSR dividers x3 | 3.3V | ~0.3 mA | ~0.3 mA |
| Piezo circuit | 3.3V | ~0 mA | ~0 mA |
| **Domain 2 Total** | **5V** | **~510 mA** | **~1440 mA** |
| STM32H725 | Battery | 200 mA | 400 mA |
| Servos x4 (stall) | Battery | 400 mA | 2800 mA |
| **Domain 1 Total** | **Battery** | **~600 mA** | **~3200 mA** |

---

## Critical Notes

### Ground

- **Common GND between ALL boards is mandatory.** Without a shared ground reference, UART communication will fail and ADC readings will be meaningless. Use a thick wire or copper strip as a GND bus connecting every board and every sensor ground.

### Servo Power

- **Never power servos from MCU USB ports.** Servo stall current can exceed 700 mA per servo. This will brown out the MCU, corrupt flash, or damage USB ports. Always use the ROBKIT1 battery rail for servo power.

### Decoupling

- **Add 100 uF electrolytic on ESP32 VIN**, as close to the pin as possible. The ESP32 has high transient current draw during WiFi transmissions that can cause voltage dips and resets.
- **Add 1000 uF electrolytic across WS2812B power** at the strip end. LED switching currents cause large transient spikes.

### Impedance Circuit

- **Keep electrode wires short (< 10 cm).** Long wires act as antennas and pick up noise from servo PWM, WiFi transmissions, and mains interference. This directly corrupts the impedance measurement.
- **Route impedance wires away from servo wires.** Servo PWM signals (50 Hz, high current) will couple into the high-impedance measurement circuit. Maintain at least 5 cm separation or use shielded cable for the electrode leads.
- **Calibrate baseline with open electrodes** before each demo session. Temperature and humidity affect stray capacitance.

### Logic Levels

- ESP32, STM32H725, and NodeMCU ESP8266 are all 3.3V logic. No level shifters are needed for UART or I2C between any pair of boards.
- The WS2812B data line expects 5V logic but reliably triggers at 3.3V when using a short wire (< 30 cm) and a 330 ohm series resistor from GPIO2.
- TCRT5000 IR LEDs run on 5V. Phototransistor pull-ups connect to 3.3V so the output is ADC-safe.

### UART Wiring

- **Cross TX/RX between boards.** ESP32 TX (GPIO16) connects to STM32 RX (PD6). ESP32 RX (GPIO17) connects to STM32 TX (PD5). Getting this backwards is the most common wiring mistake.
- ESP32 to NodeMCU is one-way: ESP32 GPIO5 (TX) to NodeMCU RX. The NodeMCU does not send data back.

### I2C

- The ADS1115 address is set to 0x48 by connecting its ADDR pin to GND. If you have address conflicts, tie ADDR to VDD for 0x49.
- Use 4.7k pull-ups on SDA/SCL if the bus length exceeds 20 cm. The ESP32 has internal pull-ups but they are weak (~45k) and may not be sufficient for long runs.

### Mechanical

- Mount the TCRT5000 sensors flush with the gripper finger surface, facing outward (toward the object).
- Mount the piezo disc on the inside of one gripper finger, secured with cyanoacrylate adhesive for good acoustic coupling.
- Mount the impedance electrodes on the fingertip contact surface with ~5 mm spacing between the two pads.
- Secure all wires along the gripper arm to prevent snagging during servo motion. Use cable ties or adhesive cable clips.
