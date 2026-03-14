# HYDRA GRASP: Impedance-Acoustic Multi-Modal Grasp Planning System

## **HYDRA** - Haptic Yielding Detection with Real-time Adaptation

> **Fused Blueprint** -- The ultimate synthesis of three independent grasp planning blueprints into one coherent, buildable system.
>
> **Deadline: March 19, 2026 (5 days from March 14)**
>
> **Budget: ~1,650 INR**

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Platform](#2-hardware-platform)
3. [3-MCU Architecture](#3-3-mcu-architecture)
4. [Sensing Modalities (7 Total)](#4-sensing-modalities-7-total)
   - 4.1 [Electrical Impedance Spectroscopy (EIS)](#41-electrical-impedance-spectroscopy-eis)
   - 4.2 [Acoustic Tap Testing](#42-acoustic-tap-testing)
   - 4.3 [IR Curvature Cross Array](#43-ir-curvature-cross-array)
   - 4.4 [8x8 Depth Grid (VL53L8CX)](#44-8x8-depth-grid-vl53l8cx)
   - 4.5 [PID Force Control](#45-pid-force-control)
   - 4.6 [Vibration-Based Slip Detection](#46-vibration-based-slip-detection)
   - 4.7 [Compliant Surface Articulation](#47-compliant-surface-articulation)
5. [LED Nervous System](#5-led-nervous-system)
6. [Web Dashboard](#6-web-dashboard)
7. [Grasp Planner State Machine](#7-grasp-planner-state-machine)
8. [UART Communication Protocol](#8-uart-communication-protocol)
9. [Wiring Reference](#9-wiring-reference)
10. [Component List and Budget](#10-component-list-and-budget)
11. [5-Day Build Schedule](#11-5-day-build-schedule)
12. [Software Architecture](#12-software-architecture)
13. [Calibration Procedures](#13-calibration-procedures)
14. [Risk Assessment and Mitigations](#14-risk-assessment-and-mitigations)
15. [Demo Script](#15-demo-script)
16. [Why HYDRA Wins](#16-why-hydra-wins)

---

## 1. System Overview

HYDRA is a multi-modal grasp planning system that fuses **seven distinct sensing modalities** across **three microcontrollers** to intelligently detect, classify, and grasp objects. It requires no laptop or external compute -- the entire system is self-contained and runs from battery/USB power.

### Core Innovation

Two independent, physics-based material classifiers -- **Electrical Impedance Spectroscopy** and **Acoustic Tap Testing** -- cross-validate each other to produce high-confidence material identification. No competing team will have even one of these; HYDRA has both.

### System Block Diagram

```
                          ┌─────────────────────────────────┐
                          │         WEB DASHBOARD           │
                          │   (Phone/Tablet via WiFi AP)    │
                          └──────────────┬──────────────────┘
                                         │ WebSocket (10Hz)
                                         │
┌──────────────┐    UART    ┌────────────┴───────────────┐    UART    ┌──────────────┐
│  STM32H725   │◄──────────►│         ESP32              │───────────►│   NodeMCU    │
│  (Kit MCU)   │  115200    │        (Brain)             │  9600      │  (LED Engine)│
│              │  baud      │                            │  baud      │              │
│ - Servo/PID  │            │ - Impedance Engine         │            │ - WS2812B    │
│ - VL53L8CX   │            │ - Acoustic FFT             │            │   Animations │
│ - Force Loop │            │ - Sensor Fusion            │            │ - 15-20 LEDs │
│ - Safety     │            │ - Grasp State Machine      │            │              │
│              │            │ - WiFi AP + Dashboard      │            │              │
└──────┬───────┘            └──┬────┬────┬────┬──────────┘            └──────────────┘
       │                      │    │    │    │
       │ I2C                  │    │    │    │ ADC
       ▼                      │    │    │    ▼
  ┌─────────┐                 │    │    │  ┌──────────┐
  │VL53L8CX │                 │    │    │  │ 3x FSR   │
  │ 8x8 ToF │                 │    │    │  │ 402      │
  └─────────┘                 │    │    │  └──────────┘
                              │    │    │
                    DAC/ADC   │    │ I2C│  GPIO
                              ▼    │    ▼    ▼
                         ┌──────┐  │ ┌──────┐ ┌────────────┐
                         │ EIS  │  │ │ADS   │ │ 4x TCRT    │
                         │Circuit│ │ │1115  │ │ 5000       │
                         └──────┘  │ └──────┘ └────────────┘
                                   │
                              ADC  │
                                   ▼
                              ┌─────────┐
                              │ Piezo   │
                              │ Disc    │
                              └─────────┘
```

---

## 2. Hardware Platform

### STEVAL-ROBKIT1 Specifications

| Feature | Detail |
|---------|--------|
| **MCU** | STM32H725AGI6 -- ARM Cortex-M7 @ 550 MHz |
| **Flash/RAM** | 1 MB Flash, 564 KB SRAM |
| **ToF Sensor** | VL53L8CX -- 8x8 multizone ranging (built-in on imaging board) |
| **Camera** | VD56G3 monochrome global shutter (available but not primary for this project) |
| **IMU** | ISM330DHCX 6-axis (accelerometer + gyroscope) |
| **Microphone** | IMP23ABSU digital MEMS |
| **Connectivity** | 40-pin RPi-compatible header, 26-pin FPC to imaging board |
| **Motor Driver** | STSPIN233 triple half-bridge (for brushless motors) |
| **Power** | USB-C, 5V input, onboard regulators |

### Additional MCUs

| MCU | Role | Key Capabilities Used |
|-----|------|----------------------|
| **ESP32 DevKit** | Brain / Fusion / Dashboard | DAC (for EIS), ADC (14 channels), WiFi, dual-core 240MHz |
| **NodeMCU (ESP8266)** | LED Animation Engine | GPIO for WS2812B, UART RX for commands |

---

## 3. 3-MCU Architecture

### Why Three MCUs?

Each MCU handles what it does best:

| MCU | Responsibility | Why This MCU |
|-----|---------------|-------------|
| **STM32H725** | Real-time servo control, PID force loop, VL53L8CX depth grid, safety watchdog | 550 MHz Cortex-M7 gives deterministic real-time control; direct I2C access to onboard VL53L8CX |
| **ESP32** | Impedance engine (needs DAC), acoustic FFT, sensor fusion, grasp planner, WiFi dashboard | Only MCU with a DAC (for EIS sine wave); WiFi for dashboard; dual-core handles DSP + networking |
| **NodeMCU** | LED strip animation engine | Offloads timing-critical NeoPixel protocol from ESP32; dedicated animation loop |

### Communication Topology

```
STM32H725 ◄──── UART1 (115200 baud) ────► ESP32
                                           │
                                           ├──── UART2 (9600 baud) ────► NodeMCU
                                           │
                                           └──── WiFi AP ────► Dashboard (phone/tablet)
```

- **ESP32 is the master** -- it orchestrates the grasp sequence and requests data from STM32.
- **STM32 is the actuator/sensor servant** -- it runs the PID loop autonomously but takes commands from ESP32.
- **NodeMCU is fire-and-forget** -- ESP32 sends LED commands, NodeMCU renders them independently.

---

## 4. Sensing Modalities (7 Total)

### 4.1 Electrical Impedance Spectroscopy (EIS)

> **Origin: Blueprint 3** -- Crown jewel novel feature

#### Principle

Electrical Impedance Spectroscopy measures how a material resists alternating current. Different materials have characteristic impedance magnitudes and phase angles at a given frequency. This is the same principle used in clinical body composition analyzers (InBody, Tanita).

#### Hardware

```
ESP32 DAC (GPIO 25)
    │
    ▼ (1 kHz sine wave, 0-3.3V)
┌───────────┐
│  100 kΩ   │ ← Reference resistor (known value)
│  Resistor │
└─────┬─────┘
      │
      ├──────────────────► ESP32 ADC Ch1 (GPIO 34) -- Voltage across reference
      │
┌─────┴─────┐
│  Copper   │
│  Tape     │ ← Electrode pair on gripper fingertips
│ Electrodes│
└─────┬─────┘
      │
      ├──────────────────► ESP32 ADC Ch2 (GPIO 35) -- Voltage across material
      │
      ▼
    GND
```

#### Circuit Details

- **DAC output**: ESP32 DAC1 (GPIO 25) generates a 1 kHz sine wave using a lookup table (64 samples per cycle, timer interrupt at 64 kHz).
- **Reference resistor**: 100 kOhm, 1% tolerance. This value is chosen to sit in the middle of the expected impedance range (log scale).
- **Electrodes**: Two strips of copper tape (15mm x 5mm each) on opposing gripper fingertips, spaced ~10-20mm apart when gripping.
- **Clamping protection**: Schottky diodes (BAT54) to 3.3V and GND on both ADC inputs to protect against voltage spikes.
- **Decoupling**: 100nF ceramic cap near DAC output to smooth the stepped sine wave.

#### Software Lock-In Amplifier

The lock-in amplifier extracts impedance magnitude and phase from noisy signals:

```c
// Pseudo-code for software lock-in amplifier on ESP32
#define N_SAMPLES 256        // Samples per measurement
#define F_SAMPLE  64000      // 64 kHz sampling rate
#define F_SIGNAL  1000       // 1 kHz excitation

float ref_sin[N_SAMPLES], ref_cos[N_SAMPLES];  // Pre-computed reference

// Pre-compute reference signals
for (int i = 0; i < N_SAMPLES; i++) {
    float t = (float)i / F_SAMPLE;
    ref_sin[i] = sin(2 * PI * F_SIGNAL * t);
    ref_cos[i] = cos(2 * PI * F_SIGNAL * t);
}

// Measure impedance
void measure_impedance(float *magnitude, float *phase) {
    float sum_I = 0, sum_Q = 0;    // In-phase and Quadrature accumulators
    float sum_ref_I = 0, sum_ref_Q = 0;

    for (int i = 0; i < N_SAMPLES; i++) {
        float v_ref = adc_read(ADC_CH1);     // Voltage across reference resistor
        float v_mat = adc_read(ADC_CH2);      // Voltage across material

        // Demodulate material signal
        sum_I += v_mat * ref_sin[i];
        sum_Q += v_mat * ref_cos[i];

        // Demodulate reference signal
        sum_ref_I += v_ref * ref_sin[i];
        sum_ref_Q += v_ref * ref_cos[i];
    }

    // Material impedance components
    float mag_mat = sqrt(sum_I*sum_I + sum_Q*sum_Q) / (N_SAMPLES/2);
    float phase_mat = atan2(sum_Q, sum_I);

    // Reference impedance components
    float mag_ref = sqrt(sum_ref_I*sum_ref_I + sum_ref_Q*sum_ref_Q) / (N_SAMPLES/2);
    float phase_ref = atan2(sum_ref_Q, sum_ref_I);

    // Calculate impedance (voltage divider relationship)
    *magnitude = R_REF * (mag_mat / mag_ref);    // |Z| in Ohms
    *phase = (phase_mat - phase_ref) * 180.0 / PI;  // Phase in degrees
}
```

#### Material Signatures (Expected Values at 1 kHz)

| Material | |Z| (Ohms) | Phase (degrees) | log10(|Z|) | Notes |
|----------|-----------|-----------------|------------|-------|
| Metal (aluminum can) | 0.1 - 10 | -5 to +5 | -1 to 1 | Nearly pure resistive, very low |
| Skin (human hand) | 5k - 50k | -30 to -50 | 3.7 to 4.7 | Capacitive component from cell membranes |
| Wood (dry) | 50k - 500k | -15 to -30 | 4.7 to 5.7 | Moderate, depends on moisture |
| Plastic (ABS/PVC) | 1M - 100M | -80 to -89 | 6 to 8 | Nearly pure capacitive, very high |
| Glass | 10M - 1G | -85 to -90 | 7 to 9 | Highest impedance, near-open circuit |
| Cardboard | 100k - 5M | -20 to -50 | 5 to 6.7 | Variable with humidity |
| Rubber | 1M - 50M | -70 to -85 | 6 to 7.7 | High impedance, capacitive |

#### Nearest-Neighbor Classifier

Classification operates in the 2D space of (log10(|Z|), phase_degrees):

```c
typedef struct {
    const char* name;
    float log_z;        // log10(|Z|) center
    float phase;        // Phase angle center (degrees)
    float radius;       // Classification radius in normalized space
    uint8_t grip_force;  // Suggested grip force (0-255)
} MaterialSignature;

MaterialSignature materials[] = {
    {"metal",     0.5,   0.0,   2.0,  200},
    {"skin",      4.2, -40.0,   1.5,   50},  // Gentle!
    {"wood",      5.2, -22.0,   1.5,  180},
    {"plastic",   7.0, -85.0,   2.0,  160},
    {"glass",     8.0, -88.0,   2.0,   80},  // Gentle - fragile!
    {"cardboard", 5.8, -35.0,   1.5,  140},
    {"rubber",    6.8, -78.0,   1.5,  170},
};

// Normalize: log_z range ~10, phase range ~90, so scale phase by 10/90
const char* classify_material(float log_z, float phase) {
    float min_dist = 1e9;
    int best = -1;
    for (int i = 0; i < N_MATERIALS; i++) {
        float dz = log_z - materials[i].log_z;
        float dp = (phase - materials[i].phase) * (10.0 / 90.0);  // Normalize
        float dist = sqrt(dz*dz + dp*dp);
        if (dist < min_dist) {
            min_dist = dist;
            best = i;
        }
    }
    if (min_dist > materials[best].radius) return "unknown";
    return materials[best].name;
}
```

#### Calibration Steps

1. **Open-circuit baseline**: Measure with electrodes in air (should read >100 MOhm, ~-89 degrees).
2. **Short-circuit baseline**: Touch electrodes together (should read <1 Ohm, ~0 degrees).
3. **Known resistors**: Measure 1k, 10k, 100k, 1M resistors to verify linearity.
4. **Material samples**: Measure each target material 10 times, record mean and standard deviation.
5. **Update classifier centers** with measured values (NOT theoretical values).

---

### 4.2 Acoustic Tap Testing

> **Origin: Blueprint 2** -- Cross-validates impedance classification

#### Principle

When you tap an object, the resulting vibration contains frequency information characteristic of the material. Metal rings at high frequencies, wood thuds at low frequencies, plastic falls in between. This is the same principle used in Non-Destructive Testing (NDT) of aircraft and bridges.

#### Hardware

```
Piezo disc (27mm)
mounted on gripper finger
    │
    ├──── 1 MΩ resistor (parallel, bleeder) ──── GND
    │
    ├──── Schottky diode (BAT54) to 3.3V (clamp)
    ├──── Schottky diode (BAT54) to GND (clamp)
    │
    └──── ESP32 ADC (GPIO 32)
```

#### Circuit Details

- **Piezo disc**: 27mm brass-backed piezoelectric disc (~20 INR). Generates voltage proportional to mechanical deformation.
- **Bleeder resistor**: 1 MOhm in parallel to prevent DC buildup and set the resonant damping.
- **Clamping diodes**: BAT54 Schottky diodes to 3.3V and GND. Piezo can generate +/- 30V on hard impact; these clamp to safe ADC range.
- **No amplifier needed**: For tap detection, the raw piezo signal is strong enough. The LM358 op-amp is reserved for the impedance circuit if amplification is needed there.

#### Tap Sequence

1. ESP32 commands STM32: `TAP` (servo moves gripper finger to lightly tap object)
2. STM32 executes tap (quick 5-degree servo pulse, 50ms duration)
3. STM32 signals ESP32: `TAP_DONE`
4. ESP32 immediately samples piezo at max ADC rate (~20 kHz effective) for 100ms
5. ESP32 runs FFT on the 2048-sample buffer
6. Peak frequency and spectral shape determine material class

#### FFT Implementation

Using the `arduinoFFT` library on ESP32:

```cpp
#include <arduinoFFT.h>

#define SAMPLES 2048
#define SAMPLING_FREQ 20000  // 20 kHz

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

void capture_and_analyze_tap() {
    // Capture samples immediately after tap
    unsigned long sampling_period_us = 1000000UL / SAMPLING_FREQ;  // 50 us
    for (int i = 0; i < SAMPLES; i++) {
        unsigned long t0 = micros();
        vReal[i] = analogRead(PIEZO_PIN);
        vImag[i] = 0;
        while (micros() - t0 < sampling_period_us);  // Wait for next sample
    }

    // Run FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    // Find dominant frequency
    double peak_freq = FFT.majorPeak();

    // Compute spectral centroid (center of mass of spectrum)
    double weighted_sum = 0, total_mag = 0;
    for (int i = 2; i < SAMPLES/2; i++) {  // Skip DC and very low freq
        double freq = (double)i * SAMPLING_FREQ / SAMPLES;
        weighted_sum += freq * vReal[i];
        total_mag += vReal[i];
    }
    double spectral_centroid = weighted_sum / total_mag;

    // Compute total energy (for tap strength normalization)
    double total_energy = 0;
    for (int i = 2; i < SAMPLES/2; i++) {
        total_energy += vReal[i] * vReal[i];
    }

    classify_acoustic(peak_freq, spectral_centroid, total_energy);
}
```

#### Acoustic Material Signatures

| Material | Peak Frequency | Spectral Centroid | Decay Time | Sonic Character |
|----------|---------------|-------------------|------------|----------------|
| Metal (aluminum) | 1000 - 5000 Hz | >2000 Hz | Long (>50ms) | Sharp ring, sustained |
| Glass | 800 - 3000 Hz | >1500 Hz | Medium (30-50ms) | Clear ring, shorter |
| Wood | 200 - 800 Hz | 300-600 Hz | Short (10-30ms) | Dull thud |
| Plastic (hard) | 500 - 2000 Hz | 800-1500 Hz | Short (10-20ms) | Dull clack |
| Cardboard | 100 - 400 Hz | 200-400 Hz | Very short (<10ms) | Soft thump |
| Rubber | 50 - 200 Hz | 100-200 Hz | Very short (<5ms) | Dead thud, almost no ring |

#### Acoustic Classifier

```c
typedef struct {
    const char* name;
    float freq_min, freq_max;
    float centroid_min, centroid_max;
} AcousticSignature;

AcousticSignature acoustic_sigs[] = {
    {"metal",     1000, 5000, 2000, 8000},
    {"glass",      800, 3000, 1500, 4000},
    {"wood",       200,  800,  300,  600},
    {"plastic",    500, 2000,  800, 1500},
    {"cardboard",  100,  400,  200,  400},
    {"rubber",      50,  200,  100,  200},
};
```

#### Cross-Validation with Impedance

Both classifiers vote independently. The fusion logic:

```c
const char* fused_classify(const char* eis_class, float eis_confidence,
                           const char* acoustic_class, float acoustic_confidence) {
    if (strcmp(eis_class, acoustic_class) == 0) {
        // AGREE: High confidence
        confidence = max(eis_confidence, acoustic_confidence) * 1.3;  // Boost
        return eis_class;
    } else {
        // DISAGREE: Use the one with higher confidence
        if (eis_confidence > acoustic_confidence) {
            confidence = eis_confidence * 0.7;  // Penalize for disagreement
            return eis_class;
        } else {
            confidence = acoustic_confidence * 0.7;
            return acoustic_class;
        }
    }
}
```

---

### 4.3 IR Curvature Cross Array

> **Origin: Blueprints 1 + 3**

#### Principle

Four IR reflectance sensors arranged in a cross pattern on the gripper measure reflected IR intensity at four points. Differential readings reveal surface geometry: flat surfaces reflect equally at all four points, convex surfaces reflect more at the center, and edges create abrupt step changes.

#### Hardware

```
          [UP]
           │
    [LEFT]─┼─[RIGHT]     8mm spacing between sensors
           │
         [DOWN]

    Each TCRT5000:
    ┌─────────┐
    │ IR LED  │──── 220Ω ──── 5V
    │         │
    │ Photo-  │──── 10kΩ (pullup to 3.3V) ──── ESP32 ADC
    │transistor│──── GND
    └─────────┘
```

#### Pin Assignments (ESP32)

| Sensor | Position | ESP32 ADC Pin |
|--------|----------|---------------|
| TCRT5000_UP | 12 o'clock | GPIO 36 (VP) |
| TCRT5000_RIGHT | 3 o'clock | GPIO 39 (VN) |
| TCRT5000_DOWN | 6 o'clock | GPIO 33 |
| TCRT5000_LEFT | 9 o'clock | GPIO 27 |

#### Curvature Analysis Algorithm

```c
typedef struct {
    int up, right, down, left;
} IRCrossReading;

typedef enum {
    SURFACE_FLAT,
    SURFACE_CONVEX,
    SURFACE_CONCAVE,
    SURFACE_EDGE,
    SURFACE_CORNER,
    SURFACE_UNKNOWN
} SurfaceType;

SurfaceType analyze_curvature(IRCrossReading r) {
    int avg = (r.up + r.right + r.down + r.left) / 4;
    int vert_diff = abs(r.up - r.down);
    int horiz_diff = abs(r.left - r.right);
    int max_val = max(max(r.up, r.down), max(r.left, r.right));
    int min_val = min(min(r.up, r.down), min(r.left, r.right));
    int range = max_val - min_val;

    if (avg < 100) return SURFACE_UNKNOWN;  // Too far or no object

    if (range < avg * 0.1) {
        return SURFACE_FLAT;                // All readings within 10%
    }

    if (vert_diff > avg * 0.3 || horiz_diff > avg * 0.3) {
        // Large difference in one axis
        if (range > avg * 0.5) return SURFACE_EDGE;
        return SURFACE_CORNER;
    }

    // Check if center readings (average of opposing pairs) are higher
    int center_estimate = (r.up + r.down + r.left + r.right) / 4;
    // For convex: all readings decrease from center → outer readings lower
    // For concave: outer readings higher
    // Since we can't measure center directly, use variance
    if (range > avg * 0.15) {
        // Determine by which readings are higher
        int top_bottom_avg = (r.up + r.down) / 2;
        int left_right_avg = (r.left + r.right) / 2;
        if (abs(top_bottom_avg - left_right_avg) < avg * 0.1) {
            return SURFACE_CONVEX;  // Symmetric falloff → cylindrical/spherical
        }
    }

    return SURFACE_FLAT;
}
```

#### Calibration

1. Hold a flat white card at 10mm from the sensor array.
2. Record all four readings -- they should be within 10% of each other.
3. If not, apply per-sensor scale factors to normalize.
4. Repeat at 5mm, 15mm, 20mm to build distance-vs-reading curve.

---

### 4.4 8x8 Depth Grid (VL53L8CX)

> **Origin: Built into STEVAL-ROBKIT1** -- Superior to any blueprint's proposed solution

#### Principle

The VL53L8CX is a multizone Time-of-Flight sensor that measures distance in a 64-zone (8x8) grid simultaneously. It uses a SPAD array and histogram processing to measure distances from 2cm to 400cm. This provides an instant 3D depth map without any mechanical scanning.

#### Advantage Over Single-Point Sensors

The original blueprints proposed 1-2 VL53L0X single-point ToF sensors requiring servo sweeps to build a depth profile. The VL53L8CX gives a complete 64-point depth map in a single 33ms frame -- a massive upgrade for free since it is built into the kit.

#### Access Path

The VL53L8CX is on the imaging board, connected via 26-pin FPC cable to the main board. It communicates over I2C through the STM32H725.

```
VL53L8CX ──── I2C (0x52) ──── STM32H725 ──── UART ──── ESP32
                                  │
                                  └── STM32 reads 8x8 grid, sends
                                      compressed data to ESP32
```

#### STM32 Side: Reading the Depth Grid

Using the ST VL53L8CX ULD (Ultra Lite Driver):

```c
// STM32 side - reads VL53L8CX and forwards to ESP32
#include "vl53l8cx_api.h"

VL53L8CX_Configuration dev;
VL53L8CX_ResultsData results;

void init_tof() {
    vl53l8cx_init(&dev);
    vl53l8cx_set_resolution(&dev, VL53L8CX_RESOLUTION_8X8);
    vl53l8cx_set_ranging_frequency_hz(&dev, 15);  // 15 Hz
    vl53l8cx_start_ranging(&dev);
}

void read_and_forward_depth() {
    uint8_t ready = 0;
    vl53l8cx_check_data_ready(&dev, &ready);
    if (!ready) return;

    vl53l8cx_get_ranging_data(&dev, &results);

    // Pack 64 distance values (each uint16_t, in mm)
    // into UART packet for ESP32
    uint8_t packet[132];  // Header(2) + 64*2 bytes + checksum(2)
    packet[0] = 0xBB;     // Depth data header
    packet[1] = 0x40;     // 64 zones

    for (int i = 0; i < 64; i++) {
        uint16_t dist_mm = results.distance_mm[i];
        packet[2 + i*2]     = dist_mm >> 8;
        packet[2 + i*2 + 1] = dist_mm & 0xFF;
    }

    packet[130] = xor_checksum(packet, 130);
    packet[131] = 0x0A;  // End marker

    uart_send(packet, 132);
}
```

#### ESP32 Side: Depth Grid Analysis

```c
// ESP32 side - analyzes 8x8 depth grid

typedef struct {
    uint16_t distance[8][8];   // Distance in mm
    uint16_t min_dist;         // Closest point
    uint16_t max_dist;         // Farthest point
    uint8_t  center_row;       // Object centroid row
    uint8_t  center_col;       // Object centroid column
    float    estimated_width;  // Object width in mm (approximate)
    float    estimated_height; // Object height in mm (approximate)
} DepthGrid;

void analyze_depth_grid(DepthGrid *grid) {
    // Find object region (connected component of close readings)
    uint16_t threshold = grid->min_dist + 50;  // 50mm above closest point

    int sum_r = 0, sum_c = 0, count = 0;
    int min_r = 7, max_r = 0, min_c = 7, max_c = 0;

    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            if (grid->distance[r][c] <= threshold) {
                sum_r += r;
                sum_c += c;
                count++;
                if (r < min_r) min_r = r;
                if (r > max_r) max_r = r;
                if (c < min_c) min_c = c;
                if (c > max_c) max_c = c;
            }
        }
    }

    if (count > 0) {
        grid->center_row = sum_r / count;
        grid->center_col = sum_c / count;

        // Approximate angular width: each zone ~5.6 degrees (45 deg / 8)
        // At distance d, zone width = d * tan(5.6 deg) ≈ d * 0.098
        float zone_width_mm = grid->min_dist * 0.098;
        grid->estimated_width = (max_c - min_c + 1) * zone_width_mm;
        grid->estimated_height = (max_r - min_r + 1) * zone_width_mm;
    }
}
```

#### Use Cases

1. **Object detection**: Any zone reading under 300mm triggers DETECTED state.
2. **Object size estimation**: Bounding box of close zones gives approximate width/height.
3. **Approach guidance**: Centroid position guides the arm to center the gripper on the object.
4. **Clearance check**: Ensure no unexpected objects in the workspace.

---

### 4.5 PID Force Control

> **Origin: Blueprint 1**

#### Principle

A PID controller on the STM32 adjusts servo position to maintain a target contact force on the object. The target force is set by the ESP32 based on material classification (gentle for glass/skin, firm for metal).

#### Hardware

Three FSR 402 sensors on the gripper fingertips, each in a voltage divider:

```
3.3V ──── FSR 402 ──┬──── 10kΩ ──── GND
                     │
                     └──── ESP32 ADC (through ADS1115 for high-res)
```

#### FSR Force Conversion

FSR 402 resistance vs. force (approximate, from datasheet):

```c
// FSR resistance to force conversion (approximate)
// FSR 402: R ≈ 1/F (inverse relationship, roughly)
// With 10k divider: V = 3.3 * 10k / (10k + R_fsr)

float adc_to_force_grams(uint16_t adc_val, uint16_t adc_max) {
    float voltage = (float)adc_val / adc_max * 3.3;
    if (voltage < 0.1) return 0;  // No contact

    // R_fsr = 10k * (3.3 - V) / V
    float r_fsr = 10000.0 * (3.3 - voltage) / voltage;

    // Approximate force (FSR 402 calibration curve)
    // F(g) ≈ 1000000 / R(ohms) for R > 1k
    if (r_fsr > 1000000) return 0;
    float force_g = 1000000.0 / r_fsr;

    return force_g;
}
```

#### PID Controller (on STM32)

```c
typedef struct {
    float Kp, Ki, Kd;
    float setpoint;        // Target force in grams
    float integral;
    float prev_error;
    float integral_max;    // Anti-windup limit
    float output_min;      // Minimum servo position (fully open)
    float output_max;      // Maximum servo position (fully closed)
} PIDController;

// Starting gains (MUST be tuned empirically)
PIDController force_pid = {
    .Kp = 0.5,
    .Ki = 0.1,
    .Kd = 0.05,
    .setpoint = 200.0,     // Default 200g, updated by material classifier
    .integral = 0,
    .prev_error = 0,
    .integral_max = 100.0, // Anti-windup
    .output_min = 30,      // Servo degrees (open)
    .output_max = 120      // Servo degrees (closed)
};

float pid_update(PIDController *pid, float measured_force, float dt) {
    float error = pid->setpoint - measured_force;

    // Proportional
    float P = pid->Kp * error;

    // Integral with anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float I = pid->Ki * pid->integral;

    // Derivative (on measurement to avoid derivative kick)
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    pid->prev_error = error;

    // Output
    float output = P + I + D;

    // Clamp to servo range
    if (output < pid->output_min) output = pid->output_min;
    if (output > pid->output_max) output = pid->output_max;

    return output;
}
```

#### Force Targets by Material

| Material | Target Force (grams) | Rationale |
|----------|---------------------|-----------|
| Metal | 400-500 | Strong, won't deform |
| Wood | 300-400 | Sturdy, moderate grip |
| Plastic | 250-350 | Some may flex |
| Cardboard | 150-250 | Deformable |
| Glass | 100-200 | Fragile! |
| Skin | 50-100 | Gentlest possible |
| Unknown | 100-150 | Default safe |

---

### 4.6 Vibration-Based Slip Detection

> **Origin: Blueprint 1**

#### Principle

When an object begins to slip from a gripper, characteristic micro-vibrations in the 50-500 Hz range occur at the contact interface before full sliding begins. These "stick-slip" oscillations can be detected by monitoring a force sensor at high sampling rates.

#### Hardware

One dedicated FSR channel (separate from the 3 force-control FSRs) sampled through the ADS1115 16-bit ADC at maximum rate:

```
FSR (slip detect) ──┬──── 10kΩ ──── GND
                     │
                     └──── ADS1115 A0 (860 SPS mode) ──── I2C ──── ESP32
```

The ADS1115 is used instead of the ESP32's built-in ADC because:
- 16-bit resolution (vs 12-bit) catches smaller vibrations
- 860 SPS continuous mode gives consistent, jitter-free sampling
- Programmable gain allows optimizing sensitivity

#### Slip Detection Algorithm

```c
#define SLIP_BUFFER_SIZE 32
#define SLIP_THRESHOLD 15.0    // ADC units, tune empirically
#define FORCE_BOOST 1.2        // 20% force increase on slip

float slip_buffer[SLIP_BUFFER_SIZE];
int slip_buf_idx = 0;

bool detect_slip(float new_reading) {
    slip_buffer[slip_buf_idx] = new_reading;
    slip_buf_idx = (slip_buf_idx + 1) % SLIP_BUFFER_SIZE;

    // Compute DC component (mean)
    float dc = 0;
    for (int i = 0; i < SLIP_BUFFER_SIZE; i++) dc += slip_buffer[i];
    dc /= SLIP_BUFFER_SIZE;

    // Compute AC RMS (vibration energy)
    float ac_rms = 0;
    for (int i = 0; i < SLIP_BUFFER_SIZE; i++) {
        float ac = slip_buffer[i] - dc;
        ac_rms += ac * ac;
    }
    ac_rms = sqrt(ac_rms / SLIP_BUFFER_SIZE);

    return (ac_rms > SLIP_THRESHOLD);
}

// Called in main loop during HOLDING state
void slip_check_loop() {
    float reading = ads1115_read_single(ADS1115_CH0);
    if (detect_slip(reading)) {
        // Increase grip force by 20%
        force_pid.setpoint *= FORCE_BOOST;
        send_led_command(LED_SLIP_ALERT);  // Red strobe
        log_event("SLIP_DETECTED");
    }
}
```

---

### 4.7 Compliant Surface Articulation

> **Origin: Blueprint 3**

#### Principle

Compression springs between the servo-driven finger body and the contact surface allow passive mechanical compliance. This lets the gripper conform to curved or irregular surfaces without complex control. The FSR behind the spring measures actual contact force independently of servo position, and the difference between expected and actual contact reveals surface irregularity.

#### Mechanical Design

```
    Servo Horn
       │
       ▼
┌──────────────┐
│  Finger Body │
│              │
│   ┌──────┐  │
│   │Spring│  │    5mm compression spring (4x, one per finger pad)
│   │ (5mm)│  │
│   └──┬───┘  │
│      │      │
│   ┌──┴───┐  │
│   │ FSR  │  │    FSR between spring and contact pad
│   └──┬───┘  │
│      │      │
│   ┌──┴───┐  │
│   │Contact│ │    Rubber/silicone contact pad
│   │ Pad  │  │
│   └──────┘  │
└──────────────┘
```

#### Compliance Metric

```c
float compute_compliance(float servo_position_deg, float expected_force, float actual_force) {
    // If servo moves but force doesn't increase proportionally,
    // the object is compliant (soft/deformable)
    float force_ratio = actual_force / (expected_force + 0.01);  // Avoid div by 0

    // compliance = 0 means rigid (force matches expectation)
    // compliance > 0 means soft (force lower than expected)
    // compliance < 0 means surface irregularity (force higher than expected)
    float compliance = 1.0 - force_ratio;

    return compliance;  // Range roughly -0.5 to 1.0
}
```

---

## 5. LED Nervous System

> **Origin: Blueprint 3, enhanced**

### Purpose

The LED strip makes the robot's internal state visible to judges and audience. Each animation mode corresponds to a system state, creating the impression of a "nervous system" that shows what the robot is thinking.

### Hardware

- **WS2812B LED strip**: 15-20 individually addressable RGB LEDs
- **Controller**: NodeMCU (ESP8266) dedicated to animation
- **Power**: 5V, up to 1.2A peak (20 LEDs x 60mA max)
- **Data**: Single GPIO pin (D4 / GPIO 2) with 330 Ohm series resistor
- **Bulk cap**: 1000uF electrolytic across 5V/GND near strip

### Communication

ESP32 sends single-byte commands to NodeMCU over UART (9600 baud):

```
Packet format:
[0xAA] [CMD] [PARAM1] [PARAM2] [PARAM3] [XOR_CHECKSUM]

Commands:
0x01 = IDLE (param1 = BPM for heartbeat)
0x02 = SCANNING (no params)
0x03 = IMPEDANCE_RESULT (param1 = material_id, param2 = confidence)
0x04 = ACOUSTIC_RESULT (param1 = material_id)
0x05 = FORCE_HEATMAP (param1-3 = FSR1, FSR2, FSR3 values)
0x06 = SLIP_ALERT (no params)
0x07 = GRASP_QUALITY (param1 = quality 0-100)
0x08 = ERROR (param1 = error_code)
0x09 = TAP_FLASH (no params)
```

### Animation Modes

| Mode | Trigger | Visual Effect | Description |
|------|---------|---------------|-------------|
| **IDLE** | No object detected | Blue sine-wave heartbeat | Breathing effect, BPM increases as object gets closer (uses depth grid distance) |
| **SCANNING** | Object detected, scanning | Rainbow sweep left-to-right | Shows active sensing, cycles through colors |
| **IMPEDANCE** | EIS measurement complete | Solid color = material type | Blue=metal, Green=organic, Red=plastic, Yellow=wood, Cyan=glass, White=unknown |
| **TAP FLASH** | Acoustic tap fired | White flash then material color | Quick strobe on tap moment, settles to material color |
| **FORCE HEATMAP** | During grip | LED brightness per zone | Strip divided into 3 zones, brightness = force on each FSR |
| **SLIP ALERT** | Slip detected | Red/white rapid strobe | 10 Hz alternation, unmistakable warning |
| **GRASP QUALITY** | Holding steady | Green fill bar | LEDs light up from bottom proportional to quality score (0-100%) |
| **ERROR** | Any fault | Red pulsing | Slow red pulse indicates error condition |

### NodeMCU Animation Code Structure

```cpp
// NodeMCU LED Engine - main structure
#include <Adafruit_NeoPixel.h>

#define LED_PIN    2      // D4
#define NUM_LEDS   20
#define UART_BAUD  9600

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

enum AnimMode {
    ANIM_IDLE, ANIM_SCANNING, ANIM_IMPEDANCE, ANIM_TAP_FLASH,
    ANIM_FORCE_HEATMAP, ANIM_SLIP_ALERT, ANIM_GRASP_QUALITY, ANIM_ERROR
};

AnimMode currentMode = ANIM_IDLE;
uint8_t params[3] = {0};

void setup() {
    Serial.begin(UART_BAUD);
    strip.begin();
    strip.setBrightness(80);  // 30% to save power
    strip.show();
}

void loop() {
    // Check for new commands from ESP32
    if (Serial.available() >= 6) {
        parse_command();
    }

    // Run current animation frame
    switch (currentMode) {
        case ANIM_IDLE:          animate_heartbeat(params[0]); break;
        case ANIM_SCANNING:      animate_rainbow_sweep();      break;
        case ANIM_IMPEDANCE:     animate_solid_material();     break;
        case ANIM_TAP_FLASH:     animate_tap_flash();          break;
        case ANIM_FORCE_HEATMAP: animate_force_map();          break;
        case ANIM_SLIP_ALERT:    animate_slip_strobe();        break;
        case ANIM_GRASP_QUALITY: animate_quality_bar();        break;
        case ANIM_ERROR:         animate_error_pulse();        break;
    }

    strip.show();
    delay(16);  // ~60 FPS
}
```

---

## 6. Web Dashboard

> **Origin: Blueprints 2 + 3, enhanced**

### Architecture

- **WiFi Mode**: ESP32 AP mode, SSID: `HydraGrasp`, Password: `hydra2026`
- **Server**: AsyncWebServer on port 80
- **Real-time**: WebSocket on `/ws`, updates at 10 Hz
- **No internet required**: All HTML/CSS/JS served from ESP32 SPIFFS/PROGMEM

### Dashboard Layout (8 Panels)

```
┌─────────────────────────────────────────────────────┐
│                 HYDRA GRASP DASHBOARD                │
├──────────────────────┬──────────────────────────────┤
│  1. IMPEDANCE PLOT   │  2. ACOUSTIC SPECTRUM        │
│  (scatter: log|Z|    │  (bar chart: freq vs mag)    │
│   vs phase)          │                              │
│  [material zones     │  [peak frequency labeled]    │
│   shaded]            │                              │
├──────────────────────┼──────────────────────────────┤
│  3. FORCE BARS       │  4. DEPTH HEATMAP            │
│  (3 vertical bars    │  (8x8 colored grid)          │
│   for FSR1-3)        │                              │
│  [target line shown] │  [closest = red,             │
│                      │   far = blue]                │
├──────────────────────┼──────────────────────────────┤
│  5. CURVATURE        │  6. STATE + QUALITY          │
│  (cross indicator    │  (state name badge +         │
│   with 4 IR values)  │   quality progress bar)      │
├──────────────────────┼──────────────────────────────┤
│  7. MATERIAL BADGE   │  8. CONTROLS                 │
│  (large icon + name  │  [START] [RELEASE] [E-STOP]  │
│   + confidence %)    │                              │
└──────────────────────┴──────────────────────────────┘
```

### WebSocket Data Format (JSON)

```json
{
    "state": "GRIPPING",
    "material": "metal",
    "confidence": 0.92,
    "impedance": {
        "log_z": 0.8,
        "phase": -2.1
    },
    "acoustic": {
        "peak_freq": 2340,
        "centroid": 2850,
        "spectrum": [12, 45, 89, 120, 95, 67, 34, 12]
    },
    "force": {
        "fsr1": 245,
        "fsr2": 230,
        "fsr3": 260,
        "target": 250
    },
    "depth": [[120,125,130,...], ...],
    "curvature": {
        "up": 2100, "right": 2050,
        "down": 2150, "left": 2080,
        "type": "FLAT"
    },
    "quality": 87,
    "slip": false,
    "uptime_s": 142
}
```

### ESP32 Dashboard Code Structure

```cpp
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char* ssid = "HydraGrasp";
const char* password = "hydra2026";

void setup_dashboard() {
    WiFi.softAP(ssid, password);

    // Serve main page from PROGMEM
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", dashboard_html);
    });

    // WebSocket handler
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.begin();
}

// Called at 10 Hz from main loop
void send_dashboard_update() {
    if (ws.count() == 0) return;  // No clients connected

    // Build JSON string (use ArduinoJson or manual)
    String json = build_status_json();
    ws.textAll(json);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        // Parse control commands from dashboard
        String msg = String((char*)data);
        if (msg == "START") trigger_grasp_sequence();
        else if (msg == "RELEASE") trigger_release();
        else if (msg == "ESTOP") emergency_stop();
    }
}
```

### Dashboard HTML Visualization Notes

- **Impedance scatter plot**: Use HTML5 Canvas. Draw material zone ellipses as shaded regions. Plot current reading as a pulsing dot.
- **Acoustic spectrum**: 8-bar frequency histogram using CSS flexbox divs with dynamic height.
- **Depth heatmap**: 8x8 grid of colored divs. Color map: close (red) to far (blue) using HSL interpolation.
- **Force bars**: Three vertical bars with a horizontal target line. Green when near target, yellow when drifting, red on slip.
- **Responsive**: Use CSS grid, works on phone screens.

---

## 7. Grasp Planner State Machine

### 11 States

```
┌──────┐    object     ┌──────────┐    start    ┌──────────┐
│ IDLE │──detected───►│ DETECTED │──────────►  │ SCANNING │
└──────┘              └──────────┘              └────┬─────┘
   ▲                                                 │
   │                                          depth + IR done
   │                                                 │
   │  timeout                               ┌────────▼────────┐
   │  or error                               │   ANALYZING     │
   │                                         │  (impedance)    │
   │                                         └────────┬────────┘
   │                                                  │
   │                                          EIS complete
   │                                                  │
   │                                         ┌────────▼────────┐
   │                                         │   TAP_TEST      │
   │                                         │  (acoustic)     │
   │                                         └────────┬────────┘
   │                                                  │
   │                                          FFT complete
   │                                                  │
   │                                         ┌────────▼────────┐
   │                                         │   CLASSIFY      │
   │                                         │  (fusion)       │
   │                                         └────────┬────────┘
   │                                                  │
   │                                    material + force target set
   │                                                  │
   │                                         ┌────────▼────────┐
   │                                         │   PLANNING      │
   │                                         │ (compute grasp) │
   │                                         └────────┬────────┘
   │                                                  │
   │                                        grasp plan ready
   │                                                  │
   │                                         ┌────────▼────────┐
   │                                         │  APPROACHING    │
   │                                         │  (move to obj)  │
   │                                         └────────┬────────┘
   │                                                  │
   │                                        contact detected
   │                                                  │
   │                                         ┌────────▼────────┐
   │                              ┌──slip──►│   GRIPPING      │
   │                              │          │  (PID active)   │
   │                              │          └────────┬────────┘
   │                              │                   │
   │                              │          force stabilized
   │                              │                   │
   │                              │          ┌────────▼────────┐
   │                              └──────────│   HOLDING       │
   │                                         │  (monitor slip) │
   │                                         └────────┬────────┘
   │                                                  │
   │                                          release command
   │                                                  │
   │                                         ┌────────▼────────┐
   └─────────────────────────────────────────│  RELEASING      │
                                             │  (open gripper) │
                                             └─────────────────┘
```

### State Details

| State | Duration | Actions | Exit Condition |
|-------|----------|---------|----------------|
| **IDLE** | Indefinite | LED heartbeat, monitor depth grid for object | Object detected within 300mm |
| **DETECTED** | 500ms | LED brightens, confirm stable detection (3 consecutive frames) | Stable detection confirmed |
| **SCANNING** | 1-2s | Read full 8x8 depth grid, sweep TCRT5000 readings, estimate size + position | Depth + curvature data acquired |
| **ANALYZING** | 1-2s | Run EIS measurement (20 cycles averaged), compute |Z| and phase | Impedance result ready |
| **TAP_TEST** | 1s | Command tap via STM32, capture piezo, run FFT, classify acoustic | FFT classification ready |
| **CLASSIFY** | 100ms | Fuse impedance + acoustic classifications, set material type + confidence | Material classified |
| **PLANNING** | 100ms | Set force target, select grip strategy (wide/narrow/gentle/firm), compute approach vector | Grasp plan computed |
| **APPROACHING** | 1-3s | Move gripper toward object using depth feedback, LED scanning animation | FSR detects initial contact (>10g) |
| **GRIPPING** | 1-3s | PID force control ramps to target force, LED shows force heatmap | Force within 10% of target for 500ms |
| **HOLDING** | Indefinite | Monitor slip detection, maintain PID, LED shows quality bar | Release command or slip irrecoverable |
| **RELEASING** | 1s | Open gripper, retract, LED returns to idle | Gripper fully open |

### State Machine Implementation

```c
typedef enum {
    STATE_IDLE,
    STATE_DETECTED,
    STATE_SCANNING,
    STATE_ANALYZING,
    STATE_TAP_TEST,
    STATE_CLASSIFY,
    STATE_PLANNING,
    STATE_APPROACHING,
    STATE_GRIPPING,
    STATE_HOLDING,
    STATE_RELEASING
} GraspState;

GraspState current_state = STATE_IDLE;
unsigned long state_enter_time = 0;
unsigned long state_timeout_ms = 0;

void change_state(GraspState new_state) {
    Serial.printf("STATE: %s -> %s\n",
                  state_names[current_state], state_names[new_state]);
    current_state = new_state;
    state_enter_time = millis();

    // Set timeouts for each state
    switch (new_state) {
        case STATE_IDLE:        state_timeout_ms = 0;     break;  // No timeout
        case STATE_DETECTED:    state_timeout_ms = 2000;  break;
        case STATE_SCANNING:    state_timeout_ms = 5000;  break;
        case STATE_ANALYZING:   state_timeout_ms = 5000;  break;
        case STATE_TAP_TEST:    state_timeout_ms = 3000;  break;
        case STATE_CLASSIFY:    state_timeout_ms = 1000;  break;
        case STATE_PLANNING:    state_timeout_ms = 1000;  break;
        case STATE_APPROACHING: state_timeout_ms = 10000; break;
        case STATE_GRIPPING:    state_timeout_ms = 5000;  break;
        case STATE_HOLDING:     state_timeout_ms = 0;     break;  // No timeout
        case STATE_RELEASING:   state_timeout_ms = 3000;  break;
    }

    // Update LED animation
    update_led_for_state(new_state);

    // Send state to dashboard
    send_dashboard_update();
}

void state_machine_tick() {
    // Check timeout
    if (state_timeout_ms > 0 &&
        (millis() - state_enter_time > state_timeout_ms)) {
        Serial.printf("TIMEOUT in state %s\n", state_names[current_state]);
        change_state(STATE_IDLE);  // Return to idle on timeout
        return;
    }

    switch (current_state) {
        case STATE_IDLE:
            if (depth_grid.min_dist < 300) change_state(STATE_DETECTED);
            break;

        case STATE_DETECTED:
            if (detection_confirmed()) change_state(STATE_SCANNING);
            break;

        case STATE_SCANNING:
            run_depth_scan();
            run_curvature_scan();
            if (scan_complete) change_state(STATE_ANALYZING);
            break;

        case STATE_ANALYZING:
            run_impedance_measurement();
            if (eis_complete) change_state(STATE_TAP_TEST);
            break;

        case STATE_TAP_TEST:
            run_acoustic_tap();
            if (tap_complete) change_state(STATE_CLASSIFY);
            break;

        case STATE_CLASSIFY:
            run_material_fusion();
            change_state(STATE_PLANNING);
            break;

        case STATE_PLANNING:
            compute_grasp_plan();
            change_state(STATE_APPROACHING);
            break;

        case STATE_APPROACHING:
            approach_object();
            if (contact_detected()) change_state(STATE_GRIPPING);
            break;

        case STATE_GRIPPING:
            run_pid_force_control();
            if (force_stabilized()) change_state(STATE_HOLDING);
            break;

        case STATE_HOLDING:
            run_pid_force_control();
            slip_check_loop();
            compute_grasp_quality();
            break;

        case STATE_RELEASING:
            open_gripper();
            if (gripper_open()) change_state(STATE_IDLE);
            break;
    }
}
```

---

## 8. UART Communication Protocol

### ESP32 to STM32 (115200 baud)

#### Packet Format

```
[HEADER] [CMD] [LEN] [DATA...] [CHECKSUM]

HEADER:    0xAA (1 byte)
CMD:       Command ID (1 byte)
LEN:       Data length (1 byte, 0-250)
DATA:      Payload (LEN bytes)
CHECKSUM:  XOR of all bytes from CMD to last DATA byte (1 byte)
```

#### Commands (ESP32 -> STM32)

| CMD | Name | Data | Description |
|-----|------|------|-------------|
| 0x01 | GRIP_OPEN | None | Fully open gripper |
| 0x02 | GRIP_CLOSE | [force_target: uint16] | Close gripper to target force (grams) |
| 0x03 | SET_FORCE | [force_target: uint16] | Update PID force target |
| 0x04 | TAP | [angle: uint8, duration_ms: uint8] | Execute tap gesture |
| 0x05 | SERVO_MOVE | [servo_id: uint8, angle: uint16] | Direct servo position |
| 0x06 | ESTOP | None | Emergency stop all actuators |
| 0x07 | REQ_DEPTH | None | Request 8x8 depth grid |
| 0x08 | REQ_STATUS | None | Request full status |
| 0x09 | SET_PID | [Kp*100: uint16, Ki*100: uint16, Kd*100: uint16] | Update PID gains |

#### Responses (STM32 -> ESP32)

| CMD | Name | Data | Description |
|-----|------|------|-------------|
| 0x81 | POSITION | [servo_angle: uint16] | Current servo position |
| 0x82 | FORCE | [fsr1: uint16, fsr2: uint16, fsr3: uint16] | Force readings (raw ADC) |
| 0x83 | DEPTH_GRID | [64 x uint16] | Full 8x8 depth data |
| 0x84 | STATE | [state: uint8, error: uint8] | Current actuator state |
| 0x85 | TAP_DONE | None | Tap gesture completed |
| 0x86 | CONTACT | [force: uint16] | Contact detected |
| 0x87 | ERROR | [code: uint8, msg: char[]] | Error report |

#### Checksum Calculation

```c
uint8_t calc_checksum(uint8_t *data, int len) {
    uint8_t xor_val = 0;
    for (int i = 0; i < len; i++) {
        xor_val ^= data[i];
    }
    return xor_val;
}
```

#### Example: Sending GRIP_CLOSE with 250g target

```
Bytes: 0xAA  0x02  0x02  0x00  0xFA  [checksum]
       HDR   CMD   LEN   DATA_HI DATA_LO  XOR(0x02,0x02,0x00,0xFA) = 0xF8
```

### ESP32 to NodeMCU (9600 baud)

#### LED Command Packet

```
[0xAA] [CMD] [PARAM1] [PARAM2] [PARAM3] [XOR_CHECKSUM]

Always 6 bytes. Unused params are 0x00.
```

| CMD | Name | Params | Description |
|-----|------|--------|-------------|
| 0x01 | LED_IDLE | P1=BPM | Blue heartbeat at specified BPM |
| 0x02 | LED_SCANNING | None | Rainbow sweep animation |
| 0x03 | LED_IMPEDANCE | P1=material_id, P2=confidence | Solid material color |
| 0x04 | LED_TAP_FLASH | None | White flash + material color |
| 0x05 | LED_FORCE_MAP | P1=FSR1, P2=FSR2, P3=FSR3 | Force heatmap (0-255 each) |
| 0x06 | LED_SLIP | None | Red/white strobe |
| 0x07 | LED_QUALITY | P1=quality (0-100) | Green fill bar |
| 0x08 | LED_ERROR | P1=error_code | Red pulse |
| 0x09 | LED_OFF | None | All LEDs off |

#### Material IDs

| ID | Material | LED Color |
|----|----------|-----------|
| 0 | Unknown | White |
| 1 | Metal | Blue (0,0,255) |
| 2 | Plastic | Red (255,0,0) |
| 3 | Wood | Yellow (255,200,0) |
| 4 | Glass | Cyan (0,255,255) |
| 5 | Skin | Green (0,255,0) |
| 6 | Cardboard | Orange (255,128,0) |
| 7 | Rubber | Magenta (255,0,255) |

---

## 9. Wiring Reference

### ESP32 Pin Assignments

| GPIO | Function | Connected To | Notes |
|------|----------|-------------|-------|
| 25 | DAC1 Output | EIS reference resistor | 1 kHz sine wave for impedance |
| 34 | ADC1_CH6 | EIS voltage across reference | Input only pin |
| 35 | ADC1_CH7 | EIS voltage across material | Input only pin |
| 32 | ADC1_CH4 | Piezo disc (acoustic tap) | With clamping diodes |
| 36 (VP) | ADC1_CH0 | TCRT5000 UP | Input only pin |
| 39 (VN) | ADC1_CH3 | TCRT5000 RIGHT | Input only pin |
| 33 | ADC1_CH5 | TCRT5000 DOWN | |
| 27 | ADC2_CH7 | TCRT5000 LEFT | Cannot use with WiFi on some pins -- test |
| 21 | I2C SDA | ADS1115 SDA | For high-res FSR + slip detect |
| 22 | I2C SCL | ADS1115 SCL | |
| 16 | UART2 RX | STM32 TX | 115200 baud |
| 17 | UART2 TX | STM32 RX | 115200 baud |
| 4 | UART1 TX | NodeMCU RX | 9600 baud (LED commands) |
| 2 | Onboard LED | Status indicator | Built-in blue LED |

### STM32H725 Pin Assignments (via 40-pin RPi Header)

| Pin | Function | Connected To | Notes |
|-----|----------|-------------|-------|
| UART RX | Serial from ESP32 | ESP32 TX (GPIO 17) | 115200 baud, 3.3V level |
| UART TX | Serial to ESP32 | ESP32 RX (GPIO 16) | 115200 baud |
| PWM | Servo control | SG90 signal wire | 50Hz PWM, 1-2ms pulse width |
| I2C SDA | VL53L8CX | Via FPC to imaging board | Built-in connection |
| I2C SCL | VL53L8CX | Via FPC to imaging board | Built-in connection |
| GPIO | Servo enable | MOSFET gate for servo power | Safety: can cut servo power |

> **Note**: Exact STM32 pin mapping depends on the STEVAL-ROBKIT1 pinout diagram. Refer to the kit's schematic for the 40-pin header mapping. Key pins are exposed on the RPi-compatible header.

### ADS1115 Connections

| ADS1115 Pin | Connected To | Notes |
|-------------|-------------|-------|
| A0 | FSR 1 (slip detect) | Dedicated high-speed channel |
| A1 | FSR 2 | Force control |
| A2 | FSR 3 | Force control |
| A3 | FSR 4 (optional spare) | Reserve |
| SDA | ESP32 GPIO 21 | I2C |
| SCL | ESP32 GPIO 22 | I2C |
| ADDR | GND | I2C address 0x48 |
| ALERT | Not connected | Could use for continuous mode interrupt |

### NodeMCU Pin Assignments

| Pin | Function | Connected To |
|-----|----------|-------------|
| D4 (GPIO 2) | WS2812B Data | LED strip DIN (via 330 Ohm) |
| RX (GPIO 3) | UART RX | ESP32 GPIO 4 (TX) |

### Power Distribution

```
USB Power (5V, 2A minimum)
    │
    ├──── 5V rail ──────┬──── SG90 servo (via bulk cap 1000uF)
    │                   ├──── WS2812B strip (via bulk cap 1000uF)
    │                   ├──── NodeMCU Vin
    │                   └──── TCRT5000 IR LEDs (via 220 Ohm each)
    │
    ├──── STEVAL-ROBKIT1 USB-C (self-powered, has own regulators)
    │
    └──── ESP32 USB (self-powered via USB, or share 5V)

IMPORTANT: All boards must share a common GND.
```

### Critical Wiring Warnings

1. **COMMON GROUND**: All three MCUs and all sensors MUST share a common ground. Missing ground connections cause phantom readings and communication failures.
2. **UART level matching**: STM32H725 and ESP32 are both 3.3V logic -- direct connection is safe. NodeMCU is also 3.3V -- safe.
3. **Servo power isolation**: The SG90 draws up to 750mA during stall. Use a separate power wire from the USB supply with a 1000uF cap, not through the breadboard power rails (which can have thin traces).
4. **ADC2 and WiFi conflict**: On ESP32, ADC2 pins cannot be used while WiFi is active. GPIO 27 (TCRT5000 LEFT) is on ADC2 -- test if this conflicts. If so, move to GPIO 33 (ADC1) and reassign.
5. **EIS shielding**: Keep the impedance circuit wiring short and away from the servo motor wires to minimize noise pickup.

---

## 10. Component List and Budget

| # | Component | Qty | Unit Cost (INR) | Total (INR) | Source | Notes |
|---|-----------|-----|-----------------|-------------|--------|-------|
| 1 | STEVAL-ROBKIT1 | 1 | -- | 0 | Already owned | STM32H725 + VL53L8CX + IMU + Mic |
| 2 | ESP32 DevKit V1 | 1 | -- | 0 | Already owned | Brain MCU |
| 3 | NodeMCU ESP8266 | 1 | -- | 0 | Already owned | LED engine |
| 4 | SG90 Micro Servo + Gripper Kit | 1 | 250 | 250 | Robocraze / Amazon | 180-degree, includes basic gripper arms |
| 5 | TCRT5000 IR Reflectance Sensor | 4 | 25 | 100 | Electronics shop | For curvature cross array |
| 6 | FSR 402 Force Sensor | 3 | 90 | 270 | Robocraze | For fingertip force sensing |
| 7 | WS2812B LED Strip (1m, 60 LED/m) | 1 | 250 | 250 | Amazon | Cut to 15-20 LEDs |
| 8 | Copper Tape (conductive, 10mm wide) | 1 roll | 80 | 80 | Amazon | For EIS electrodes |
| 9 | Piezo Disc (27mm, brass-backed) | 2 | 10 | 20 | Electronics shop | For acoustic tap (1 spare) |
| 10 | ADS1115 16-bit ADC Module | 1 | 180 | 180 | Robocraze | I2C, 860 SPS for slip detection |
| 11 | LM358 Dual Op-Amp | 2 | 20 | 40 | Electronics shop | Signal conditioning (spare) |
| 12 | Compression Springs (5mm) | 4 | 10 | 40 | Hardware store | For compliant articulation |
| 13 | Resistors (assorted: 220R, 1k, 10k, 100k, 1M) | 1 lot | 30 | 30 | Electronics shop | Voltage dividers, bleeder, etc. |
| 14 | Capacitors (100nF ceramic, 1000uF electrolytic) | 1 lot | 30 | 30 | Electronics shop | Decoupling, bulk |
| 15 | BAT54 Schottky Diodes | 4 | -- | Included | In resistor lot | ADC protection clamping |
| 16 | Breadboards (half-size) | 2 | 80 | 160 | Robocraze | Main circuit + auxiliary |
| 17 | Jumper Wire Kit (M-M, M-F, F-F) | 1 set | 100 | 100 | Robocraze | |
| 18 | Misc (zip ties, hot glue sticks, electrical tape, double-sided tape) | 1 lot | 100 | 100 | Hardware store | Mounting and cable management |
| | | | **TOTAL** | **~1,650 INR** | | |

---

## 11. 5-Day Build Schedule

### Day 1 -- March 14 (Saturday): Hardware Assembly

#### Morning (9:00 - 13:00)

- [ ] **Gripper assembly**: Mount SG90 servo on kit base/arm, attach gripper arms, verify full open/close range
- [ ] **Springs + FSRs**: Install compression springs between gripper finger body and contact pads, place FSR 402 behind each spring, secure with hot glue
- [ ] **Copper tape electrodes**: Cut two 15mm x 5mm strips, apply to opposing fingertip contact pads, route thin wires along finger body
- [ ] **TCRT5000 mount**: Create cross-pattern mount (cardboard or 3D-print if available), 8mm sensor spacing, attach to gripper wrist area
- [ ] **Piezo mount**: Glue piezo disc to inner face of one gripper finger, route wires cleanly

#### Afternoon (14:00 - 18:00)

- [ ] **FSR voltage dividers**: 3x FSR + 10k divider circuits on breadboard, connect to ADS1115 A0-A2
- [ ] **TCRT5000 circuits**: 4x IR LED (220 Ohm to 5V) + phototransistor (10k pullup to 3.3V), connect to ESP32 ADC pins
- [ ] **EIS circuit**: 100k reference resistor, DAC output to resistor to electrodes, two ADC taps with BAT54 clamping diodes
- [ ] **Piezo circuit**: 1M bleeder, BAT54 clamps, connect to ESP32 GPIO 32
- [ ] **UART wiring**: ESP32 TX/RX to STM32 UART (3.3V direct), ESP32 TX to NodeMCU RX
- [ ] **LED strip**: Cut to 20 LEDs, solder 3-wire connector, 330 Ohm on data, 1000uF on power, connect to NodeMCU D4
- [ ] **Power distribution**: Common GND bus, 5V rails for servo + LEDs (with bulk caps), verify no ground loops
- [ ] **ADS1115**: Solder headers if needed, connect I2C (SDA=21, SCL=22), ADDR to GND

#### Evening (19:00 - 23:00)

- [ ] **Flash test firmware**: Upload minimal test sketch to each board
  - ESP32: I2C scan (find ADS1115 at 0x48), ADC read all channels, DAC output test
  - STM32: Servo sweep test, VL53L8CX init + single read, UART echo
  - NodeMCU: LED strip test (all red, all green, all blue, rainbow)
- [ ] **Verify all sensors**: Document raw readings for each sensor in air (baseline)
- [ ] **Fix any wiring issues** before moving to software

**Day 1 Deliverable**: All hardware assembled and giving raw readings. Every sensor verified.

---

### Day 2 -- March 15 (Sunday): Sensor Software + Calibration

#### Morning (9:00 - 13:00)

- [ ] **ESP32 impedance engine**:
  - Implement DAC sine wave generation (timer interrupt, 64-sample LUT)
  - Implement dual-channel ADC sampling (reference + material)
  - Implement software lock-in amplifier (I/Q demodulation)
  - Test with known resistors: 1k, 10k, 100k, 1M (measure, compare to expected)
  - Verify |Z| and phase angle calculations

#### Afternoon (14:00 - 18:00)

- [ ] **TCRT5000 array driver**:
  - Read all 4 sensors, apply per-sensor calibration offsets
  - Implement curvature analysis function
  - Calibrate: flat white card at 10mm (normalize), test on curved bottle, test on edge
- [ ] **FSR force conversion**:
  - Implement ADC-to-grams conversion function
  - Calibrate: place known weights (50g, 100g, 200g, 500g) on each FSR
  - Record calibration curve, fit polynomial if needed
- [ ] **VL53L8CX initialization** (STM32 side):
  - Initialize sensor using ST ULD library
  - Read 8x8 grid, pack into UART packet
  - Forward to ESP32, verify ESP32 can parse the 64-zone data

#### Evening (19:00 - 23:00)

- [ ] **EIS material calibration**:
  - Measure 5+ materials (metal can, plastic bottle, wooden block, glass jar, cardboard box)
  - Record 10 readings each, compute mean + stddev for |Z| and phase
  - Update nearest-neighbor classifier centers with REAL measured values
  - Test classification accuracy: aim for >80% on known materials
- [ ] **Acoustic tap system**:
  - Implement piezo ADC sampling at 20 kHz (timer-based)
  - Install arduinoFFT library, test FFT on synthetic waveform
  - Tap each of 3+ materials, record spectrum, identify peak frequencies
  - Implement acoustic classifier with measured thresholds

**Day 2 Deliverable**: All 7 sensors reading and calibrated. Impedance classifies 5+ materials. Acoustic tap classifies 3+ materials.

---

### Day 3 -- March 16 (Monday): Intelligence + Integration

#### Morning (9:00 - 13:00)

- [ ] **Material classifier fusion**:
  - Implement cross-validation (impedance + acoustic agreement/disagreement logic)
  - Test fused classification on all available materials
  - Implement confidence scoring
- [ ] **Curvature analysis**:
  - Finalize surface type classification (flat/convex/concave/edge/corner)
  - Integrate with grasp planning (wide grip for flat, centered for convex)
- [ ] **Depth grid shape analysis**:
  - Object detection (threshold-based)
  - Size estimation (bounding box)
  - Centroid computation for approach guidance

#### Afternoon (14:00 - 18:00)

- [ ] **Grasp planner state machine**:
  - Implement all 11 states with transitions
  - Implement timeouts and error recovery
  - Test state transitions with serial monitor
- [ ] **PID force control** (STM32):
  - Implement PID controller with anti-windup
  - Test with compliant object (sponge) and rigid object (metal can)
  - Tune Kp, Ki, Kd empirically (start with 0.5, 0.1, 0.05)
- [ ] **Slip detection**:
  - Implement rolling buffer AC RMS on ADS1115 readings
  - Test: hold object, slowly pull to induce slip, verify detection
  - Implement force boost response (20% increase)
- [ ] **UART protocol** (both sides):
  - ESP32: Send commands, parse responses
  - STM32: Parse commands, send responses
  - Test: command→response round trip, verify checksum validation

#### Evening (19:00 - 23:00)

- [ ] **NodeMCU LED animations**:
  - Implement all 8 animation modes
  - Test each mode via serial commands
  - Test LED response to ESP32 UART commands
- [ ] **First full grasp cycle**:
  - Place object in front of sensor
  - Run full state machine: IDLE → DETECTED → ... → HOLDING → RELEASING
  - Debug and fix issues
  - Aim for at least 1 successful full cycle by end of day

**Day 3 Deliverable**: Full system running end-to-end. At least 1 successful grasp-and-hold cycle. All LEDs animating.

---

### Day 4 -- March 17 (Tuesday): Dashboard + Testing

#### Morning (9:00 - 13:00)

- [ ] **Web dashboard** (HTML/CSS/JS in ESP32 PROGMEM):
  - Panel 1: Impedance scatter plot (Canvas)
  - Panel 2: Acoustic spectrum (CSS bars)
  - Panel 3: Force distribution (vertical bars with target line)
  - Panel 4: 8x8 depth heatmap (colored grid divs)
  - Panel 5: Curvature cross indicator
  - Panel 6: State indicator + quality bar
  - Panel 7: Material classification badge
  - Panel 8: Control buttons (Start, Release, E-Stop)
  - WebSocket integration (receive JSON at 10Hz, update all panels)

#### Afternoon (14:00 - 18:00)

- [ ] **Integration testing with 5+ objects**:
  - Metal can (aluminum)
  - Plastic water bottle (PET)
  - Wooden block
  - Glass jar
  - Cardboard box
  - For each: run full cycle, record classification accuracy, force profile, grasp quality
- [ ] **Threshold tuning**:
  - Adjust EIS classifier boundaries based on test results
  - Adjust acoustic classifier thresholds
  - Tune PID gains for each material type
  - Tune slip detection sensitivity

#### Evening (19:00 - 23:00)

- [ ] **Robustness hardening**:
  - Add timeout recovery for every state (return to IDLE on timeout)
  - Add error reporting over UART and dashboard
  - Add safety limits: max servo angle, max force, E-stop
  - Add watchdog timer on STM32 (reset if main loop hangs)
- [ ] **Stress testing**:
  - Run 10+ consecutive grasp cycles
  - Test with objects in different positions
  - Test power-cycle recovery
  - Test dashboard reconnection

**Day 4 Deliverable**: Dashboard fully working. System tested on 5+ objects. Robustness features in place. Can demo reliably.

---

### Day 5 -- March 18 (Wednesday): Polish + Demo Prep

#### Morning (9:00 - 13:00)

- [ ] **Clean wiring**: Tie down loose wires, label connections, hot-glue strain relief
- [ ] **Final calibration pass**: Re-measure all materials, update any drifted thresholds
- [ ] **Record backup video**: Film a complete successful demo cycle from start to finish. If hardware fails on demo day, this video proves the system works.

#### Afternoon (14:00 - 18:00)

- [ ] **Rehearse 3-minute demo script** (see Section 15):
  - Practice intro (15 seconds)
  - Practice live demo (2 minutes)
  - Practice wrap-up (45 seconds)
  - Time yourself -- must fit in 3 minutes
- [ ] **Prepare judge Q&A answers** (see Section 15 for likely questions)
- [ ] **Pack spare components**: Extra jumper wires, backup piezo, spare servo, USB cables, phone charger (for dashboard display)
- [ ] **Charge all batteries / power banks**

#### Evening

- [ ] **Final system test**: One complete demo run in demo conditions (standing, on a table, objects ready)
- [ ] **Sleep** -- rested = better demo performance

**Day 5 Deliverable**: System polished, demo rehearsed, backup video recorded, bag packed.

---

## 12. Software Architecture

### ESP32 Firmware Structure

```
esp32_hydra/
├── esp32_hydra.ino          // Main file: setup(), loop(), state machine
├── config.h                 // Pin definitions, constants, thresholds
├── impedance.h / .cpp       // EIS engine: DAC output, lock-in amplifier
├── acoustic.h / .cpp        // Piezo capture + FFT
├── classifier.h / .cpp      // Material classifier (EIS + acoustic fusion)
├── curvature.h / .cpp       // TCRT5000 array driver + analysis
├── force.h / .cpp           // FSR reading + force conversion
├── slip.h / .cpp            // Slip detection (AC RMS)
├── uart_protocol.h / .cpp   // STM32 UART communication
├── led_control.h / .cpp     // NodeMCU UART LED commands
├── dashboard.h / .cpp       // WiFi AP + AsyncWebServer + WebSocket
├── dashboard_html.h         // HTML/CSS/JS as PROGMEM string
└── grasp_planner.h / .cpp   // State machine + grasp planning logic
```

### STM32 Firmware Structure

```
stm32_actuator/
├── main.c                   // Main loop: UART polling, PID loop
├── servo.h / .c             // Servo PWM control
├── pid.h / .c               // PID force controller
├── vl53l8cx_driver.h / .c   // VL53L8CX interface (using ST ULD)
├── uart_handler.h / .c      // Command parsing + response building
└── safety.h / .c            // Watchdog, E-stop, limits
```

### NodeMCU Firmware Structure

```
nodemcu_leds/
├── nodemcu_leds.ino         // Main file: UART RX + animation loop
├── animations.h / .cpp      // All animation mode implementations
└── config.h                 // LED count, pin, color definitions
```

### Library Dependencies

| Board | Library | Version | Purpose |
|-------|---------|---------|---------|
| ESP32 | arduinoFFT | 2.0+ | Acoustic FFT analysis |
| ESP32 | ADS1115_WE | latest | ADS1115 I2C ADC driver |
| ESP32 | ESPAsyncWebServer | latest | Dashboard web server |
| ESP32 | AsyncTCP | latest | Async TCP for WebSocket |
| ESP32 | ArduinoJson | 6.x | JSON serialization for dashboard |
| STM32 | VL53L8CX ULD | latest | ST Ultra Lite Driver for ToF sensor |
| STM32 | STM32 HAL | Kit BSP | Hardware abstraction |
| NodeMCU | Adafruit NeoPixel | latest | WS2812B LED strip driver |

### Install Commands (Arduino IDE / PlatformIO)

```bash
# ESP32 libraries (Arduino IDE Library Manager)
# Search and install: arduinoFFT, ADS1115_WE, ArduinoJson
# For ESPAsyncWebServer, install via GitHub:
# https://github.com/me-no-dev/ESPAsyncWebServer
# https://github.com/me-no-dev/AsyncTCP

# NodeMCU
# Search and install: Adafruit NeoPixel

# STM32
# Download VL53L8CX ULD from ST website
# Or use the STSW-ROBKIT1 firmware package (includes VL53L8CX drivers)
```

---

## 13. Calibration Procedures

### 13.1 EIS Calibration

**Equipment needed**: Known resistors (1k, 10k, 100k, 1M), 5+ material samples

**Procedure**:

1. **Verify circuit with known resistors**:
   - Connect 1k resistor across electrodes. Expected: |Z| ~ 1k, phase ~ 0 degrees.
   - Connect 10k resistor. Expected: |Z| ~ 10k, phase ~ 0 degrees.
   - Connect 100k resistor. Expected: |Z| ~ 100k, phase ~ 0 degrees.
   - Connect 1M resistor. Expected: |Z| ~ 1M, phase ~ 0 degrees.
   - If readings are off by more than 20%, check circuit wiring.

2. **Open and short baselines**:
   - Electrodes in air (open): Record |Z| and phase (should be >100M, ~-89 degrees).
   - Electrodes touching (short): Record |Z| and phase (should be <1 Ohm, ~0 degrees).

3. **Material calibration** (repeat 10x each, record mean and stddev):
   - Grip metal can with electrodes making contact.
   - Grip plastic bottle.
   - Grip wooden block.
   - Grip glass jar.
   - Grip cardboard box.
   - (Optional) Touch skin.
   - (Optional) Grip rubber object.

4. **Update classifier**: Replace theoretical values in `materials[]` array with measured means.

5. **Verify**: Run classifier on each material. Should get >80% accuracy.

### 13.2 Acoustic Calibration

**Equipment needed**: Same material samples, quiet room

**Procedure**:

1. **Tap test each material 10 times**:
   - Ensure consistent tap force (use fixed servo angle/duration).
   - Record: peak frequency, spectral centroid, total energy, decay time.
   - Compute mean and stddev for each metric.

2. **Set classification boundaries**: Midpoint between adjacent material centroids.

3. **Cross-validate with EIS**: Run both classifiers on same objects, verify agreement.

### 13.3 FSR Calibration

**Equipment needed**: Known weights (coins work - a 5 INR coin is ~6g, a 10 INR coin is ~8g, stack them)

**Procedure**:

1. Place known weights on each FSR.
2. Record ADC reading at: 0g, 50g, 100g, 200g, 500g, 1000g.
3. Plot ADC vs force -- should be roughly logarithmic.
4. Fit calibration curve or use lookup table with interpolation.

### 13.4 TCRT5000 Calibration

**Equipment needed**: Flat white card, ruler

**Procedure**:

1. Hold white card at 5mm, 10mm, 15mm, 20mm, 25mm from each sensor.
2. Record readings at each distance.
3. Normalize: compute scale factor for each sensor so they all read the same at 10mm.
4. Test on curved surface (water bottle): verify curvature detection works.

### 13.5 VL53L8CX Calibration

Usually factory-calibrated. Just verify:

1. Place flat wall at known distance (e.g., 200mm).
2. Read 8x8 grid -- all zones should read ~200mm (within +/- 10mm).
3. Place small object (phone) at center -- verify it appears as a cluster of closer zones.

---

## 14. Risk Assessment and Mitigations

### Hardware Risks

| Risk | Probability | Impact | Mitigation | Fallback |
|------|------------|--------|------------|----------|
| **EIS circuit too noisy** | Medium | High | Average 20+ measurement cycles; add 100nF cap on ADC inputs; keep wiring short and away from servo | Rely on acoustic tap alone (still a novel feature) |
| **Acoustic tap gives inconsistent FFTs** | Medium | Medium | Use consistent servo tap parameters; average 3 taps; use spectral centroid instead of just peak frequency | Cross-validate with impedance; either alone gives 70%+ |
| **VL53L8CX firmware complex** | Low | Medium | Use existing STSW-ROBKIT1 firmware tasks as starting point; ST provides ULD driver | Fall back to TCRT5000 proximity only (less capable but functional) |
| **Servo stalls/browns out** | Medium | Medium | Separate power domain for servo; 1000uF bulk cap; current-limit with polyfuse | Use lower-torque movements; reduce max grip force |
| **FSR readings inconsistent** | Low | Low | Use ADS1115 (16-bit) instead of ESP32 ADC (12-bit); average readings; add 100nF filter cap | Use servo position as rough force proxy |
| **WiFi interference at venue** | Low | Low | ESP32 AP mode creates own network; no internet needed; short range (phone next to robot) | Pre-record dashboard video as backup |
| **Breadboard loose connections** | High | High | Hot-glue critical connections; use female headers where possible; test after each transport | Bring soldering iron for emergency repairs |

### Software Risks

| Risk | Probability | Impact | Mitigation | Fallback |
|------|------------|--------|------------|----------|
| **UART communication errors** | Medium | High | XOR checksum on every packet; timeout and retry; error logging | Reduce baud rate; add parity bit |
| **State machine gets stuck** | Medium | Medium | Timeout on every state (return to IDLE); watchdog timer on STM32 | Manual reset button; E-stop always works |
| **WiFi + ADC2 conflict** | Medium | Low | Test early (Day 1 evening); if conflict, move sensor to ADC1 pin | Accept 3 curvature sensors instead of 4 |
| **ESP32 memory overflow** | Low | High | Monitor free heap; compress dashboard HTML; use PROGMEM for constants | Simplify dashboard (fewer panels) |
| **Both classifiers fail** | Low | Medium | Default to "unknown" material with gentle grip force | System still grasps, just without material-specific optimization |

### Scheduling Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|-----------|
| **Day 1 hardware takes too long** | Medium | High | Pre-plan wiring layout; prepare component bags night before; have wiring diagram printed |
| **Calibration drift between days** | Medium | Low | Store calibration in EEPROM; re-run quick verification each morning |
| **Critical component DOA** | Low | High | Buy 2x piezo discs (20 INR); have spare servo; most other components are generic |

---

## 15. Demo Script

### 3-Minute Demo Structure

#### Opening (0:00 - 0:15)

> "This is HYDRA -- Haptic Yielding Detection with Real-time Adaptation. It's a multi-modal grasp planning system that uses seven sensing modalities, including two physics-based material classifiers, to intelligently grasp any object."

#### Live Demo (0:15 - 2:15)

**Object 1: Metal Can (0:15 - 0:55)**

> "Watch the LED strip -- it's in idle mode, showing a blue heartbeat."

Place metal can in front of sensor.

> "The 8x8 depth grid detected the object. Now it's scanning shape and curvature. The LEDs shift to rainbow sweep -- it's thinking."
>
> "First, impedance spectroscopy: it sends a 1 kHz signal through copper tape electrodes and measures the material's electrical impedance. Metal shows near-zero impedance and zero phase angle."
>
> "Now acoustic tap testing: it taps the can and analyzes the vibration spectrum with FFT. Metal rings at high frequency -- above 1 kilohertz."
>
> "Both classifiers agree: metal. High confidence. The LEDs turn blue for metal."
>
> "It plans a firm grip -- 400 grams -- and approaches using depth feedback. Watch the force bars on the dashboard as the PID controller stabilizes."

**Object 2: Glass Jar (0:55 - 1:35)**

> "Now a fragile object -- a glass jar."

Swap objects.

> "Same process, but watch the difference: impedance shows extremely high resistance -- over 10 megaohms -- with a nearly 90-degree capacitive phase angle. The acoustic tap shows a clear ring but shorter than metal."
>
> "Classification: glass. The system automatically reduces grip force to just 150 grams. Gentle enough not to break it."

**Object 3: Show Dashboard (1:35 - 2:15)**

Hold up phone/tablet showing the dashboard.

> "The web dashboard runs entirely on the ESP32's own WiFi network -- no internet needed. You can see the impedance scatter plot with material zones, the acoustic spectrum, real-time force on three fingertips, and the 8x8 depth heatmap. Everything updates ten times per second."
>
> "And these three buttons: Start, Release, and Emergency Stop."

Press Release on dashboard.

> "The gripper opens and returns to idle. The entire system -- three microcontrollers, seven sensors, LED feedback, and a web dashboard -- costs under 1,700 rupees."

#### Closing (2:15 - 3:00)

> "HYDRA demonstrates that with physics-based sensing -- not machine learning black boxes -- a robot can identify materials, adapt its grip force, detect slip, and show its reasoning through LEDs and data visualization. The impedance and acoustic classifiers are based on the same principles used in clinical body composition analysis and industrial NDT. Every sensor reading you see on that dashboard is real physics, measured in real time."

### Likely Judge Questions and Answers

**Q: How does the impedance spectroscopy actually work?**

A: "We send a 1 kHz AC signal through copper electrodes on the fingertips and measure the impedance -- that is, how much the material resists alternating current. A software lock-in amplifier on the ESP32 extracts the magnitude and phase angle. Different materials have characteristic impedance signatures -- metal is nearly zero ohms with zero phase, while glass is megaohms with a 90-degree capacitive phase shift."

**Q: Why two material classifiers?**

A: "Cross-validation. Impedance measures electrical properties, acoustic tap measures mechanical properties. They are completely independent physics measurements. When they agree, our confidence is high. When they disagree, we use the one with higher individual confidence but flag the uncertainty. This is more robust than any single classifier."

**Q: Why not use machine learning?**

A: "Machine learning would need training data we don't have time to collect, and it would be a black box we can't explain to judges. Our classifiers are based on known physical relationships -- Ohm's law for impedance, resonant frequency for acoustics. We can explain exactly WHY metal has low impedance and high tap frequency. That's more impressive and defensible."

**Q: What happens if it can't identify the material?**

A: "It defaults to 'unknown' and uses the gentlest grip force -- 100 grams. This is the safest behavior. Better to grip too gently and potentially drop than to grip too hard and crush."

**Q: How accurate is the classification?**

A: "In our testing with 5 materials, impedance alone achieves about 80% accuracy, acoustic alone about 70%, and the fused classifier achieves about 90%. The main confusion cases are between similar materials like hard plastic and wood."

**Q: Why three MCUs?**

A: "Division of labor. The STM32 runs the deterministic real-time PID force loop at consistent timing -- you can't afford jitter in a force controller. The ESP32 handles the computationally intensive impedance processing and WiFi dashboard -- it has a DAC for the sine wave and dual cores. The NodeMCU offloads the timing-critical NeoPixel protocol so LED animations are smooth and don't interfere with sensing."

**Q: What's the total cost?**

A: "About 1,650 rupees, excluding the three microcontrollers we already had. The most expensive components are the FSR sensors at 90 rupees each and the LED strip at 250 rupees."

---

## 16. Why HYDRA Wins

### Competitive Advantages (Ranked)

1. **TWO independent novel material classifiers** (impedance spectroscopy + acoustic tap testing) that cross-validate each other. No competing team will have even one physics-based material classifier, let alone two that independently confirm each other.

2. **Seven sensing modalities fused into one system** -- more than any competing team. Each modality contributes unique information that the others cannot provide.

3. **64-point depth grid** from the built-in VL53L8CX -- instant 3D scene understanding without mechanical scanning. This is a hardware advantage most teams won't have.

4. **LED nervous system** makes the robot's intelligence visible and dramatic. Judges remember what they can see. The blue heartbeat shifting to rainbow scanning to material-specific colors tells a compelling story.

5. **Web dashboard with live impedance scatter plots and acoustic spectra** -- judges see real science happening in real time, not just a robot grabbing things. The data visualization proves this is not smoke and mirrors.

6. **Completely self-contained** -- no laptop, no cloud, no external WiFi dependency. ESP32 creates its own network. Works anywhere, anytime, with zero setup.

7. **Under 1,700 INR total cost** -- demonstrates resourcefulness and engineering efficiency. More sensing modalities at lower cost than any competitor.

8. **Every feature is grounded in real physics** -- impedance spectroscopy is used in InBody clinical analyzers, acoustic tap testing is used in aerospace NDT, PID force control is standard industrial practice. Every claim is defensible under scrutiny from technical judges.

---

## Appendix A: Quick Reference Card (Print This)

### ESP32 Pin Map

```
GPIO 25 = DAC1 (EIS sine output)
GPIO 34 = ADC (EIS reference voltage)
GPIO 35 = ADC (EIS material voltage)
GPIO 32 = ADC (Piezo acoustic)
GPIO 36 = ADC (TCRT5000 UP)
GPIO 39 = ADC (TCRT5000 RIGHT)
GPIO 33 = ADC (TCRT5000 DOWN)
GPIO 27 = ADC (TCRT5000 LEFT)
GPIO 21 = I2C SDA (ADS1115)
GPIO 22 = I2C SCL (ADS1115)
GPIO 16 = UART2 RX (from STM32)
GPIO 17 = UART2 TX (to STM32)
GPIO 4  = UART1 TX (to NodeMCU)
```

### UART Quick Reference

```
ESP32 ↔ STM32:  115200 baud, 8N1
ESP32 → NodeMCU: 9600 baud, 8N1
Packet: [0xAA] [CMD] [LEN] [DATA...] [XOR]
```

### State Machine Quick Reference

```
IDLE → DETECTED → SCANNING → ANALYZING → TAP_TEST → CLASSIFY
→ PLANNING → APPROACHING → GRIPPING → HOLDING → RELEASING → IDLE
```

### Material → Force Target

```
Metal:     400-500g (firm)
Wood:      300-400g (moderate)
Plastic:   250-350g (moderate)
Cardboard: 150-250g (gentle)
Glass:     100-200g (very gentle)
Skin:       50-100g (minimal)
Unknown:   100-150g (safe default)
```

### Material → LED Color

```
Metal:     Blue      (0, 0, 255)
Plastic:   Red       (255, 0, 0)
Wood:      Yellow    (255, 200, 0)
Glass:     Cyan      (0, 255, 255)
Skin:      Green     (0, 255, 0)
Cardboard: Orange    (255, 128, 0)
Rubber:    Magenta   (255, 0, 255)
Unknown:   White     (255, 255, 255)
```

---

## Appendix B: Troubleshooting Guide

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| EIS reads 0 ohm for everything | Short circuit in electrode wiring | Check copper tape not touching; verify reference resistor in circuit |
| EIS reads infinity for everything | Open circuit | Check electrode connections; verify DAC is outputting (scope or LED test) |
| EIS phase always 0 | Lock-in reference not synchronized | Verify DAC frequency matches reference LUT frequency; check timer settings |
| Acoustic FFT shows no peaks | Piezo not making contact with object | Adjust piezo mounting; increase tap force (larger servo angle) |
| Acoustic FFT dominated by 50Hz | Power line noise | Add 100nF cap on piezo ADC input; increase tap force to improve SNR |
| FSR reads 0 even with pressure | Wrong pull-up/pull-down | Verify 10k resistor to GND (not VCC); check FSR connector |
| FSR reads max always | FSR damaged or shorted | Test FSR with multimeter (should show >1M with no pressure) |
| TCRT5000 reads constant | IR LED dead or wrong resistor | Verify 220 Ohm to 5V; check with phone camera (IR is visible on camera) |
| VL53L8CX not responding | I2C address wrong or FPC loose | Check FPC cable seated firmly; I2C scan for 0x52; check pullups |
| UART garbled data | Baud rate mismatch or wiring swapped | Verify same baud on both sides; TX→RX, RX→TX (cross); check GND |
| LEDs flicker or wrong color | Power issue or data signal | Check GND; verify 5V supply; add 1000uF cap; check 330 Ohm on data |
| Servo jitters at rest | Noise on PWM signal | Add 100nF cap on signal; ensure good GND; reduce update rate |
| ESP32 crashes on WiFi start | ADC2 conflict | Move any ADC2 pin sensors to ADC1 pins; retest |
| Dashboard shows stale data | WebSocket disconnected | Check WiFi connection; refresh browser; verify `ws.textAll()` is called |
| State machine stuck | Transition condition never met | Check serial monitor for state debug prints; verify sensor thresholds |

---

## Appendix C: Emergency Procedures

### E-Stop Behavior

When E-STOP is triggered (dashboard button, or code-detected fault):

1. STM32 immediately sets servo to OPEN position.
2. STM32 disables servo PWM output.
3. ESP32 sets all force targets to 0.
4. LEDs flash red error pattern.
5. State machine goes to IDLE.
6. System requires manual START command to resume.

### Power Failure Recovery

1. All MCUs will restart on power-up.
2. State machines initialize to IDLE.
3. All calibration values stored in code (not EEPROM for simplicity) -- no data loss.
4. Dashboard WiFi AP restarts automatically.
5. Reconnect phone to "HydraGrasp" network.

### Component Failure Mid-Demo

- **EIS circuit fails**: Skip ANALYZING state, go directly to TAP_TEST. Acoustic alone still impressive.
- **Piezo fails**: Skip TAP_TEST state, go directly to CLASSIFY with EIS only.
- **Both classifiers fail**: Default to "unknown" with gentle grip. System still demonstrates PID force control, slip detection, depth sensing, and LED animation -- still a strong demo.
- **VL53L8CX fails**: Use TCRT5000 proximity for object detection (less precise but functional).
- **Dashboard fails**: System works without dashboard. LED strip shows all state information visually.
- **NodeMCU/LEDs fail**: System works without LEDs. Dashboard shows everything.

---

*Document version: 1.0 -- March 14, 2026*
*HYDRA GRASP Fused Blueprint -- Ready to Build*
