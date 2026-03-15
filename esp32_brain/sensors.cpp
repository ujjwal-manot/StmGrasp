#include "sensors.h"
#include <math.h>

static float _slip_buf[FSR_COUNT][FSR_SLIP_WINDOW];
static int   _slip_idx = 0;

static float _adcToForce(int raw_adc) {
    if (raw_adc < 10) return 0.0f;
    float v_out = (float)raw_adc * IMP_ADC_VREF_MV / IMP_ADC_RESOLUTION;
    float v_cc = IMP_ADC_VREF_MV;
    if (v_out >= v_cc - 1.0f) return FSR_MAX_FORCE_N;
    float r_fsr = FSR_PULLDOWN_OHM * (v_cc / v_out - 1.0f);
    if (r_fsr > 1e7f) return 0.0f;
    if (r_fsr < 100.0f) return FSR_MAX_FORCE_N;
    float r_kohm = r_fsr / 1000.0f;
    float force = 1.0f / powf(r_kohm, 1.1f);
    force *= 30.0f;
    return fminf(fmaxf(force, 0.0f), FSR_MAX_FORCE_N);
}

void initSensors() {
    pinMode(PIN_FSR1, INPUT);
    pinMode(PIN_FSR2, INPUT);
    pinMode(PIN_FSR3, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    memset(_slip_buf, 0, sizeof(_slip_buf));
    _slip_idx = 0;
}

bool detectSlip(float* out_max_rms) {
    static const uint8_t fsr_pins[FSR_COUNT] = { PIN_FSR1, PIN_FSR2, PIN_FSR3 };
    uint32_t interval_us = 1000000UL / FSR_SLIP_SAMPLE_RATE;
    float current_forces[FSR_COUNT][FSR_SLIP_WINDOW];
    uint32_t next_us = micros();

    for (int s = 0; s < FSR_SLIP_WINDOW; s++) {
        while ((int32_t)(next_us - micros()) > 0) {}
        for (int f = 0; f < FSR_COUNT; f++)
            current_forces[f][s] = _adcToForce(analogRead(fsr_pins[f]));
        next_us += interval_us;
    }

    float max_rms = 0.0f;
    for (int f = 0; f < FSR_COUNT; f++) {
        float mean = 0.0f;
        for (int s = 0; s < FSR_SLIP_WINDOW; s++) mean += current_forces[f][s];
        mean /= (float)FSR_SLIP_WINDOW;
        float rms_sum = 0.0f;
        for (int s = 0; s < FSR_SLIP_WINDOW; s++) {
            float ac = current_forces[f][s] - mean;
            rms_sum += ac * ac;
        }
        float rms = sqrtf(rms_sum / (float)FSR_SLIP_WINDOW);
        _slip_buf[f][_slip_idx] = rms;
        if (rms > max_rms) max_rms = rms;
    }
    _slip_idx = (_slip_idx + 1) % FSR_SLIP_WINDOW;

    if (out_max_rms) *out_max_rms = max_rms;
    return (max_rms > FSR_SLIP_RMS_THRESHOLD);
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

    static const uint8_t fsr_pins[FSR_COUNT] = { PIN_FSR1, PIN_FSR2, PIN_FSR3 };
    for (int i = 0; i < FSR_COUNT; i++)
        result.forces_N[i] = _adcToForce(analogRead(fsr_pins[i]));

    float sum = 0.0f;
    for (int i = 0; i < FSR_COUNT; i++) sum += result.forces_N[i];
    result.total_N = sum;

    if (sum < 0.01f) { result.balance = 0.0f; result.valid = true; return result; }

    float f_min = result.forces_N[0], f_max = result.forces_N[0];
    for (int i = 1; i < FSR_COUNT; i++) {
        if (result.forces_N[i] < f_min) f_min = result.forces_N[i];
        if (result.forces_N[i] > f_max) f_max = result.forces_N[i];
    }
    result.balance = (f_max > 0.01f) ? (f_min / f_max) : 0.0f;

    static const float fsr_x[FSR_COUNT] = { -1.0f, 1.0f, 0.0f };
    static const float fsr_y[FSR_COUNT] = { -0.577f, -0.577f, 1.155f };
    float wx = 0.0f, wy = 0.0f;
    for (int i = 0; i < FSR_COUNT; i++) {
        wx += result.forces_N[i] * fsr_x[i];
        wy += result.forces_N[i] * fsr_y[i];
    }
    result.cop_x = wx / sum;
    result.cop_y = wy / sum;

    float max_rms = 0.0f;
    result.slip_detected = detectSlip(&max_rms);
    result.slip_rms = max_rms;
    result.valid = true;
    return result;
}
