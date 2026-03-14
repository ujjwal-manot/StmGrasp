#include "sensors.h"
#include <Wire.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────
// FSR slip detection rolling buffer
// ─────────────────────────────────────────────────────────────
static float _slip_buf[FSR_COUNT][FSR_SLIP_WINDOW];
static int   _slip_idx = 0;
static bool  _ads1115_ready = false;


// ─────────────────────────────────────────────────────────────
// FSR402 resistance-to-force conversion
// ─────────────────────────────────────────────────────────────
// FSR402 is roughly: F(N) = (Vout / (Vcc - Vout)) * R_pulldown / sensitivity
// The FSR402 characteristic is nonlinear. We use piecewise approximation:
//   R_fsr = R_pulldown * (Vcc/Vout - 1)
//   Conductance G = 1/R_fsr
//   Force ≈ G * scale_factor  (roughly linear above 0.2N)
// Scale factor calibrated so full-scale ADC ≈ 20N.
static float _adcToForce(int raw_adc) {
    if (raw_adc < 10) return 0.0f; // below noise floor

    float v_out = (float)raw_adc * IMP_ADC_VREF_MV / IMP_ADC_RESOLUTION; // mV
    float v_cc = IMP_ADC_VREF_MV; // 3300 mV

    // Avoid division by zero near saturation
    if (v_out >= v_cc - 1.0f) return FSR_MAX_FORCE_N;

    float r_fsr = FSR_PULLDOWN_OHM * (v_cc / v_out - 1.0f);

    // FSR402 empirical: F ≈ 1/(R/1000)^1.1 for R in kΩ, F in Newtons
    // Adjusted to match 0.2-20N range
    if (r_fsr > 1e7f) return 0.0f;         // open circuit
    if (r_fsr < 100.0f) return FSR_MAX_FORCE_N; // saturated

    float r_kohm = r_fsr / 1000.0f;
    float force = 1.0f / powf(r_kohm, 1.1f);

    // Scale so that ~300Ω ≈ 10N (typical FSR402 midpoint)
    force *= 30.0f;

    return fminf(fmaxf(force, 0.0f), FSR_MAX_FORCE_N);
}


void initSensors() {
    // FSR pins
    pinMode(PIN_FSR1, INPUT);
    pinMode(PIN_FSR2, INPUT);
    pinMode(PIN_FSR3, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Clear slip buffer
    memset(_slip_buf, 0, sizeof(_slip_buf));
    _slip_idx = 0;

    // I2C for ADS1115
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000); // 400 kHz

    _ads1115_ready = initADS1115();
}


bool initADS1115() {
    // Verify the ADS1115 is on the bus by attempting a config write
    Wire.beginTransmission(ADS1115_ADDR);
    Wire.write(ADS1115_REG_CONFIG);

    // Default config: AINp=AIN0, ±4.096V, single-shot, 860 SPS
    uint16_t config = ADS1115_OS_START |
                      (0 << ADS1115_MUX_SHIFT) | // AIN0 vs GND
                      ADS1115_GAIN_4V |
                      ADS1115_MODE_SINGLE |
                      ADS1115_RATE_860SPS |
                      0x0003; // comparator disable
    Wire.write((uint8_t)(config >> 8));
    Wire.write((uint8_t)(config & 0xFF));
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
        Serial.println("[SENSORS] ADS1115 not found on I2C bus");
        return false;
    }
    Serial.println("[SENSORS] ADS1115 initialized");
    return true;
}


int16_t readADS1115Channel(uint8_t channel) {
    if (!_ads1115_ready || channel > 3) return 0;

    // Configure for the selected channel (single-ended vs GND)
    // MUX: 100=AIN0, 101=AIN1, 110=AIN2, 111=AIN3
    uint16_t mux = (uint16_t)(0x04 + channel) << ADS1115_MUX_SHIFT;

    uint16_t config = ADS1115_OS_START |
                      mux |
                      ADS1115_GAIN_4V |
                      ADS1115_MODE_SINGLE |
                      ADS1115_RATE_860SPS |
                      0x0003;

    Wire.beginTransmission(ADS1115_ADDR);
    Wire.write(ADS1115_REG_CONFIG);
    Wire.write((uint8_t)(config >> 8));
    Wire.write((uint8_t)(config & 0xFF));
    Wire.endTransmission();

    // Wait for conversion at 860 SPS (~1.2 ms)
    delayMicroseconds(1300);

    // Poll the OS bit until conversion is complete (timeout after 5 ms)
    uint32_t poll_start = millis();
    while (millis() - poll_start < 5) {
        Wire.beginTransmission(ADS1115_ADDR);
        Wire.write(ADS1115_REG_CONFIG);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)ADS1115_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
            uint16_t cfg_read = ((uint16_t)Wire.read() << 8) | Wire.read();
            if (cfg_read & ADS1115_OS_START) break; // conversion done
        }
        delayMicroseconds(200);
    }

    // Read conversion result
    Wire.beginTransmission(ADS1115_ADDR);
    Wire.write(ADS1115_REG_CONVERSION);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADS1115_ADDR, (uint8_t)2);

    if (Wire.available() < 2) return 0;

    int16_t val = ((int16_t)Wire.read() << 8) | Wire.read();
    return val;
}


CurvatureResult readIRCurvatureArray() {
    CurvatureResult result;
    result.curvature_x = 0.0f;
    result.curvature_y = 0.0f;
    result.flatness = 1.0f;
    result.sharp_edge = false;
    result.geometry = 0;
    result.valid = false;
    memset(result.raw, 0, sizeof(result.raw));

    if (!_ads1115_ready) return result;

    // Read all 4 TCRT5000 sensors via ADS1115
    // Layout:  [0]=Left  [1]=Right  [2]=Top  [3]=Bottom
    int16_t readings[ADS1115_CHANNELS];
    for (int ch = 0; ch < ADS1115_CHANNELS; ch++) {
        readings[ch] = readADS1115Channel(ch);
        // Clamp negative values (shouldn't happen with reflective sensors)
        if (readings[ch] < 0) readings[ch] = 0;
        result.raw[ch] = (uint16_t)readings[ch];
    }

    float left   = (float)readings[0];
    float right  = (float)readings[1];
    float top    = (float)readings[2];
    float bottom = (float)readings[3];

    // Curvature: difference between opposing sensors, normalized
    float max_val = fmaxf(fmaxf(left, right), fmaxf(top, bottom));
    if (max_val < 50.0f) {
        // All sensors dark — no object in range
        return result;
    }

    float norm = 1.0f / fmaxf(max_val, 1.0f);

    // Positive curvature_x: object curves toward right sensor
    result.curvature_x = (right - left) * norm;
    result.curvature_y = (top - bottom) * norm;

    // Flatness: how similar are all 4 readings (0 = very different, 1 = identical)
    float mean = (left + right + top + bottom) * 0.25f;
    float variance = 0.0f;
    float vals[4] = { left, right, top, bottom };
    for (int i = 0; i < 4; i++) {
        float d = vals[i] - mean;
        variance += d * d;
    }
    variance *= 0.25f;
    float std_dev = sqrtf(variance);
    result.flatness = fmaxf(0.0f, 1.0f - (std_dev / fmaxf(mean, 1.0f)));

    // Sharp edge detection: any sensor dramatically different from its neighbor
    float max_diff = 0.0f;
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            float diff = fabsf(vals[i] - vals[j]);
            if (diff > max_diff) max_diff = diff;
        }
    }
    result.sharp_edge = (max_diff > (float)IR_EDGE_THRESHOLD);

    // Geometry classification
    if (result.sharp_edge) {
        result.geometry = 3; // edge
    } else if (result.flatness > 0.85f) {
        result.geometry = 0; // flat
    } else if (result.curvature_x > 0.3f || result.curvature_y > 0.3f) {
        result.geometry = 1; // convex
    } else if (result.curvature_x < -0.3f || result.curvature_y < -0.3f) {
        result.geometry = 2; // concave
    } else {
        // Check for cylinder: one axis curved, other flat
        bool x_curved = fabsf(result.curvature_x) > 0.15f;
        bool y_curved = fabsf(result.curvature_y) > 0.15f;
        if (x_curved != y_curved) {
            result.geometry = 4; // cylinder
        } else {
            result.geometry = 0; // approximately flat
        }
    }

    result.valid = true;
    return result;
}


ForceResult readForces() {
    ForceResult result;
    result.total_N = 0.0f;
    result.balance = 0.0f;
    result.cop_x = 0.0f;
    result.cop_y = 0.0f;
    result.slip_detected = false;
    result.slip_rms = 0.0f;
    result.valid = false;

    // Read the three FSRs
    static const uint8_t fsr_pins[FSR_COUNT] = { PIN_FSR1, PIN_FSR2, PIN_FSR3 };
    for (int i = 0; i < FSR_COUNT; i++) {
        int raw = analogRead(fsr_pins[i]);
        result.forces_N[i] = _adcToForce(raw);
    }

    // Total force
    float sum = 0.0f;
    for (int i = 0; i < FSR_COUNT; i++) sum += result.forces_N[i];
    result.total_N = sum;

    if (sum < 0.01f) {
        // No contact
        result.balance = 0.0f;
        result.valid = true;
        return result;
    }

    // Balance: ratio of min to max force (1 = perfect balance)
    float f_min = result.forces_N[0];
    float f_max = result.forces_N[0];
    for (int i = 1; i < FSR_COUNT; i++) {
        if (result.forces_N[i] < f_min) f_min = result.forces_N[i];
        if (result.forces_N[i] > f_max) f_max = result.forces_N[i];
    }
    result.balance = (f_max > 0.01f) ? (f_min / f_max) : 0.0f;

    // Center of pressure (assuming triangular FSR arrangement):
    //   FSR1 at (-1, -0.577)  — bottom-left
    //   FSR2 at ( 1, -0.577)  — bottom-right
    //   FSR3 at ( 0,  1.155)  — top-center
    static const float fsr_x[FSR_COUNT] = { -1.0f,  1.0f,  0.0f };
    static const float fsr_y[FSR_COUNT] = { -0.577f, -0.577f, 1.155f };

    float wx = 0.0f, wy = 0.0f;
    for (int i = 0; i < FSR_COUNT; i++) {
        wx += result.forces_N[i] * fsr_x[i];
        wy += result.forces_N[i] * fsr_y[i];
    }
    result.cop_x = wx / sum;
    result.cop_y = wy / sum;

    // Slip detection using rolling buffer
    result.slip_detected = detectSlip();

    result.valid = true;
    return result;
}


bool detectSlip() {
    // High-frequency FSR burst sampling for vibration/slip detection
    static const uint8_t fsr_pins[FSR_COUNT] = { PIN_FSR1, PIN_FSR2, PIN_FSR3 };
    uint32_t interval_us = 1000000UL / FSR_SLIP_SAMPLE_RATE;

    // Take FSR_SLIP_WINDOW samples at 1 kHz
    float current_forces[FSR_COUNT][FSR_SLIP_WINDOW];
    uint32_t next_us = micros();

    for (int s = 0; s < FSR_SLIP_WINDOW; s++) {
        while ((int32_t)(next_us - micros()) > 0) { /* spin */ }
        for (int f = 0; f < FSR_COUNT; f++) {
            current_forces[f][s] = _adcToForce(analogRead(fsr_pins[f]));
        }
        next_us += interval_us;
    }

    // Compute AC RMS for each FSR (remove DC component)
    float max_rms = 0.0f;
    for (int f = 0; f < FSR_COUNT; f++) {
        // DC mean
        float mean = 0.0f;
        for (int s = 0; s < FSR_SLIP_WINDOW; s++) {
            mean += current_forces[f][s];
        }
        mean /= (float)FSR_SLIP_WINDOW;

        // AC RMS
        float rms_sum = 0.0f;
        for (int s = 0; s < FSR_SLIP_WINDOW; s++) {
            float ac = current_forces[f][s] - mean;
            rms_sum += ac * ac;
        }
        float rms = sqrtf(rms_sum / (float)FSR_SLIP_WINDOW);

        // Store in rolling buffer for trend analysis
        _slip_buf[f][_slip_idx] = rms;

        if (rms > max_rms) max_rms = rms;
    }
    _slip_idx = (_slip_idx + 1) % FSR_SLIP_WINDOW;

    return (max_rms > FSR_SLIP_RMS_THRESHOLD);
}
