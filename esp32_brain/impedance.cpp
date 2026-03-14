#include "impedance.h"
#include <math.h>

// Pre-computed sine lookup table (one cycle, 40 samples, normalized to ±1.0)
static float _sin_lut[IMP_SAMPLES_PER_CYCLE];
static float _cos_lut[IMP_SAMPLES_PER_CYCLE];

// DAC waveform table (8-bit: 0-255)
static uint8_t _dac_lut[IMP_SAMPLES_PER_CYCLE];

// Baseline (open-air) complex impedance for subtraction
static float _baseline_vref_real = 0.0f;
static float _baseline_vref_imag = 0.0f;
static float _baseline_vmut_real = 0.0f;
static float _baseline_vmut_imag = 0.0f;
static bool  _baseline_valid = false;

// Sample interval in microseconds for 40 kSPS
static const uint32_t _sample_interval_us = 1000000UL / IMP_SAMPLE_RATE; // 25 µs


void initImpedance() {
    pinMode(PIN_DAC_SINE, OUTPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);  // 0-3.3 V range

    // Build lookup tables
    for (int i = 0; i < IMP_SAMPLES_PER_CYCLE; i++) {
        float theta = 2.0f * M_PI * (float)i / (float)IMP_SAMPLES_PER_CYCLE;
        _sin_lut[i] = sinf(theta);
        _cos_lut[i] = cosf(theta);
        _dac_lut[i] = (uint8_t)(IMP_DAC_OFFSET + (int)(IMP_DAC_AMPLITUDE * sinf(theta)));
    }

    dacWrite(PIN_DAC_SINE, IMP_DAC_OFFSET); // idle at midpoint
    _baseline_valid = false;
}


// Perform lock-in measurement: drives DAC sine while sampling Vref and Vmut,
// demodulates with reference sin/cos to extract in-phase and quadrature components.
static void _lockInMeasure(float* vref_real, float* vref_imag,
                           float* vmut_real, float* vmut_imag,
                           float* signal_power) {
    float vr_i = 0.0f, vr_q = 0.0f; // Vref in-phase, quadrature
    float vm_i = 0.0f, vm_q = 0.0f; // Vmut in-phase, quadrature
    float power_sum = 0.0f;

    // Disable interrupts briefly is not viable on ESP32 with WiFi,
    // so we use tight timing with micros()
    uint32_t next_us = micros();

    for (int cycle = 0; cycle < IMP_INTEGRATION_CYCLES; cycle++) {
        for (int s = 0; s < IMP_SAMPLES_PER_CYCLE; s++) {
            // Wait for precise sample time
            while ((int32_t)(next_us - micros()) > 0) { /* spin */ }

            // Drive DAC
            dacWrite(PIN_DAC_SINE, _dac_lut[s]);

            // Small settling delay (~2 µs)
            delayMicroseconds(2);

            // Sample both ADC channels
            int raw_vref = analogRead(PIN_ADC_VREF);
            int raw_vmut = analogRead(PIN_ADC_VMUT);

            // Convert to voltage
            float v_ref = (float)raw_vref * IMP_ADC_VREF_MV / IMP_ADC_RESOLUTION;
            float v_mut = (float)raw_vmut * IMP_ADC_VREF_MV / IMP_ADC_RESOLUTION;

            // Remove DC offset (approximate midpoint)
            v_ref -= IMP_ADC_VREF_MV * 0.5f;
            v_mut -= IMP_ADC_VREF_MV * 0.5f;

            // Multiply-accumulate with reference sin/cos
            float sin_val = _sin_lut[s];
            float cos_val = _cos_lut[s];

            vr_i += v_ref * cos_val;
            vr_q += v_ref * sin_val;
            vm_i += v_mut * cos_val;
            vm_q += v_mut * sin_val;

            power_sum += v_ref * v_ref;

            next_us += _sample_interval_us;
        }
    }

    // Return DAC to midpoint
    dacWrite(PIN_DAC_SINE, IMP_DAC_OFFSET);

    // Normalize by number of samples
    float n = (float)IMP_TOTAL_SAMPLES;
    *vref_real = vr_i / n;
    *vref_imag = vr_q / n;
    *vmut_real = vm_i / n;
    *vmut_imag = vm_q / n;
    *signal_power = power_sum / n;
}


void calibrateBaseline() {
    // Measure with nothing touching the electrodes
    float power;
    _lockInMeasure(&_baseline_vref_real, &_baseline_vref_imag,
                   &_baseline_vmut_real, &_baseline_vmut_imag, &power);
    _baseline_valid = true;
}


ImpedanceResult measureImpedance() {
    ImpedanceResult result;
    result.magnitude = 0.0f;
    result.phase_deg = 0.0f;
    result.confidence = 0.0f;
    result.material_idx = MAT_UNKNOWN;
    result.valid = false;

    float vr_real, vr_imag, vm_real, vm_imag, power;
    _lockInMeasure(&vr_real, &vr_imag, &vm_real, &vm_imag, &power);

    // Subtract baseline if available
    if (_baseline_valid) {
        vr_real -= _baseline_vref_real;
        vr_imag -= _baseline_vref_imag;
        vm_real -= _baseline_vmut_real;
        vm_imag -= _baseline_vmut_imag;
    }

    // Check for valid signal (Vref must have reasonable amplitude)
    float vref_mag_sq = vr_real * vr_real + vr_imag * vr_imag;
    if (vref_mag_sq < 1.0f) {
        // Signal too weak — no material present or sensor fault
        return result;
    }

    // Complex impedance: Z = Rref * (Vmut / Vref)
    // Complex division: (a+jb)/(c+jd) = ((ac+bd) + j(bc-ad)) / (c²+d²)
    float denom = vref_mag_sq;
    float z_real = IMP_RREF_OHMS * (vm_real * vr_real + vm_imag * vr_imag) / denom;
    float z_imag = IMP_RREF_OHMS * (vm_imag * vr_real - vm_real * vr_imag) / denom;

    result.magnitude = sqrtf(z_real * z_real + z_imag * z_imag);
    result.phase_deg = atan2f(z_imag, z_real) * 180.0f / M_PI;

    // Confidence based on signal-to-noise ratio
    float expected_power = (IMP_ADC_VREF_MV * 0.5f) * (IMP_ADC_VREF_MV * 0.5f) * 0.5f;
    float snr = power / fmaxf(expected_power, 1.0f);
    result.confidence = fminf(snr, 1.0f);

    if (result.confidence < IMP_MIN_CONFIDENCE) {
        return result;
    }

    result.material_idx = classifyMaterial(result.magnitude, result.phase_deg, &result.confidence);
    result.valid = true;
    return result;
}


uint8_t classifyMaterial(float magnitude, float phase_deg, float* out_confidence) {
    // Nearest-neighbor in normalized (log10|Z|, phase/90) space
    float log_mag = log10f(fmaxf(magnitude, 0.01f));
    float norm_phase = phase_deg / 90.0f;

    float best_dist = 1e9f;
    uint8_t best_idx = MAT_UNKNOWN;

    for (int i = 0; i < MATERIAL_COUNT; i++) {
        float ref_log = log10f(fmaxf(MATERIAL_DB[i].impedance_magnitude, 0.01f));
        float ref_phase = MATERIAL_DB[i].impedance_phase_deg / 90.0f;

        float d_log = log_mag - ref_log;
        float d_phase = norm_phase - ref_phase;
        float dist = sqrtf(d_log * d_log + d_phase * d_phase);

        if (dist < best_dist) {
            best_dist = dist;
            best_idx = (uint8_t)i;
        }
    }

    if (best_dist > IMP_CLASSIFY_THRESHOLD) {
        best_idx = MAT_UNKNOWN;
        if (out_confidence) *out_confidence *= 0.3f;
    } else {
        // Confidence scales inversely with distance
        float dist_conf = 1.0f - (best_dist / IMP_CLASSIFY_THRESHOLD);
        if (out_confidence) *out_confidence *= dist_conf;
    }

    return best_idx;
}
