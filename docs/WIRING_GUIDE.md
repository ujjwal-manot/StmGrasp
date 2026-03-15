# HYDRA GRASP v2.0 -- Wiring Guide

## Overview

Two boards + sensors + servos. No NodeMCU, no TCRT5000, no piezo.

```
  ROBKIT1 (STM32H725)          ESP32 DevKit v1
  ┌──────────────────┐         ┌──────────────┐
  │ 40-pin header:   │         │              │
  │  PD5 (TX) ───────┼────────→│ GPIO17 (RX)  │
  │  PD6 (RX) ←──────┼─────────│ GPIO16 (TX)  │
  │  GND ────────────┼─────────│ GND          │
  │                  │         │              │
  │  PC6 → Servo 1   │         │ GPIO25 → DAC │
  │  PC7 → Servo 2   │         │ GPIO34 ← Vref│
  │  PC8 → Servo 3   │         │ GPIO35 ← Vmut│
  │  PA0 ← E-stop    │         │ GPIO36 ← FSR1│
  │                  │         │ GPIO39 ← FSR2│
  │ [FPC] → ToF+cam  │         │ GPIO32 ← FSR3│
  │ [snap] → motors   │         │              │
  │ [battery] → power │         │ VIN ← 5V USB │
  └──────────────────┘         └──────────────┘
```

## ESP32 Pin Assignments

| GPIO | Function | Direction |
|------|----------|-----------|
| 25 | Impedance DAC output | OUT |
| 34 | Impedance Vref ADC | IN |
| 35 | Impedance Vmut ADC | IN |
| 36 (VP) | FSR 1 (left finger) | IN |
| 39 (VN) | FSR 2 (right finger) | IN |
| 32 | FSR 3 (slip detection) | IN |
| 16 | UART TX to STM32 | OUT |
| 17 | UART RX from STM32 | IN |
| 33 | Calibrate button (freed from piezo) | IN |
| 5 | Start button (freed from NodeMCU) | IN |
| 4 | Spare | IN |

## ROBKIT1 Connections

### UART to ESP32
| ROBKIT1 Pin | STM32 Pin | Connect to |
|-------------|-----------|------------|
| Header TX | PD5 | ESP32 GPIO17 (RX) |
| Header RX | PD6 | ESP32 GPIO16 (TX) |
| Header GND | GND | ESP32 GND |

115200 baud, 8N1. Both 3.3V logic, no level shifter.

### Servo PWM
| STM32 Pin | Channel | Servo |
|-----------|---------|-------|
| PC6 | TIM3_CH1 | Gripper finger L |
| PC7 | TIM3_CH2 | Gripper finger R |
| PC8 | TIM3_CH3 | Wrist rotation |

50 Hz, pulse 500-2400 us. Power servos from battery, NOT USB.

### Onboard (no wiring needed)
- VL53L8CX ToF: I2C1 via FPC cable to imaging board
- LSM6DSV16BX IMU: on main board
- Microphone: ADC3 on main board
- BLE: on main board
- Motors: board-to-board connector to motor board

## Impedance Spectroscopy Circuit

```
                  1.5k       100nF         10k (Rref)
ESP32 GPIO25 ---/\/\/\---||----/\/\/\---+--- Electrode A
                                        |
                                        +--1k--→ ESP32 GPIO34 (Vref)
                                        |
                                   [MATERIAL]
                                        |
                                  Electrode B ---+
                                                 +--1k--→ ESP32 GPIO35 (Vmut)
                                                 |
                                                10k
                                                 |
                                                GND

ADC protection on GPIO34 and GPIO35:
  3.3V ---|<--- pin --->|--- GND   (1N4148 clamp diodes)
```

### Component list
| Component | Value | Qty |
|-----------|-------|-----|
| Resistor | 1.5k | 1 |
| Resistor | 10k | 2 |
| Resistor | 1k | 2 |
| Capacitor | 100nF ceramic | 1 |
| Diode | 1N4148 | 4 |
| Copper tape | 5mm wide | 1 roll |

## FSR Voltage Dividers (x3)

```
  3.3V --- [FSR402] ---+--- [10k] --- GND
                       +--→ ESP32 ADC pin
```

| FSR | GPIO | ADC Channel |
|-----|------|-------------|
| FSR1 (left finger) | 36 (VP) | ADC1_CH0 |
| FSR2 (right finger) | 39 (VN) | ADC1_CH3 |
| FSR3 (slip detect) | 32 | ADC1_CH4 |

3x 10k resistors needed.

## E-Stop Button

```
PA0 ---[button NC]--- GND
```
Active low, STM32 internal pull-up. Mushroom button recommended.

## Power

```
Domain 1: Battery → ROBKIT1 + Servos
  Battery → ROBKIT1 power connector (internal 3.3V regulator)
  Battery → Servo VCC rail (all 3 servo red wires)

Domain 2: USB 5V → ESP32
  5V 3A USB supply → ESP32 VIN pin
  Add 100uF cap on ESP32 VIN

COMMON GND: All boards, all sensors, all servos share one GND bus.
```

### Power-up sequence
1. Plug in ESP32 USB (domain 2 first)
2. Connect battery (domain 1 second)

### Power-down sequence
1. Disconnect battery first
2. Unplug ESP32 USB

## Wire Checklist

| # | From | To | Verify |
|---|------|----|--------|
| 1 | ROBKIT1 PD5 | ESP32 GPIO17 | TX→RX crossed |
| 2 | ESP32 GPIO16 | ROBKIT1 PD6 | TX→RX crossed |
| 3 | ROBKIT1 GND | ESP32 GND | Continuity |
| 4 | ROBKIT1 PC6 | Servo 1 signal | Orange wire |
| 5 | ROBKIT1 PC7 | Servo 2 signal | Orange wire |
| 6 | ROBKIT1 PC8 | Servo 3 signal | Orange wire |
| 7 | Battery 5V | Servo VCC rail | Red wires |
| 8 | Battery GND | Servo GND + common bus | Brown wires |
| 9 | E-stop one leg | ROBKIT1 PA0 | |
| 10 | E-stop other leg | GND | |
| 11 | ESP32 GPIO25 | Impedance circuit DAC in | Via 1.5k |
| 12 | Impedance Vref out | ESP32 GPIO34 | Via 1k |
| 13 | Impedance Vmut out | ESP32 GPIO35 | Via 1k |
| 14 | FSR1 junction | ESP32 GPIO36 | 10k to GND |
| 15 | FSR2 junction | ESP32 GPIO39 | 10k to GND |
| 16 | FSR3 junction | ESP32 GPIO32 | 10k to GND |

Total: 16 connections. Use multimeter continuity check before powering on.
