#include "acoustic.h"
#include <arduinoFFT.h>
#include <math.h>

static float _samples[ACOUSTIC_NUM_SAMPLES];
static double _fft_real[ACOUSTIC_FFT_SIZE];
static double _fft_imag[ACOUSTIC_FFT_SIZE];
static int _baseline_adc = 0;
static ArduinoFFT<double> _fft = ArduinoFFT<double>(_fft_real, _fft_imag, ACOUSTIC_FFT_SIZE, (double)ACOUSTIC_SAMPLE_RATE);

void initAcoustic() {
    pinMode(PIN_PIEZO_ADC, INPUT);
    analogReadResolution(12);
    long sum = 0;
    for (int i = 0; i < 64; i++) {
        sum += analogRead(PIN_PIEZO_ADC);
        delayMicroseconds(500);
    }
    _baseline_adc = (int)(sum / 64);
}

bool waitForTap(uint32_t timeout_ms) {
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        int delta = abs(analogRead(PIN_PIEZO_ADC) - _baseline_adc);
        if (delta > ACOUSTIC_TAP_THRESHOLD) return true;
        delayMicroseconds(100);
    }
    return false;
}

static void _recordSamples() {
    uint32_t interval_us = 1000000UL / ACOUSTIC_SAMPLE_RATE;
    uint32_t next_us = micros();
    for (int i = 0; i < ACOUSTIC_NUM_SAMPLES; i++) {
        while ((int32_t)(next_us - micros()) > 0) {}
        _samples[i] = (float)(analogRead(PIN_PIEZO_ADC) - _baseline_adc);
        next_us += interval_us;
    }
}

AcousticResult analyzeTapLocal() {
    AcousticResult result;
    memset(&result, 0, sizeof(result));
    result.valid = false;

    _recordSamples();

    float max_amp = 0.0f;
    for (int i = 0; i < ACOUSTIC_NUM_SAMPLES; i++) {
        float a = fabsf(_samples[i]);
        if (a > max_amp) max_amp = a;
    }
    if (max_amp < (float)ACOUSTIC_TAP_THRESHOLD * 0.5f) return result;

    for (int i = 0; i < ACOUSTIC_FFT_SIZE; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * M_PI * (float)i / (float)(ACOUSTIC_FFT_SIZE - 1)));
        _fft_real[i] = (i < ACOUSTIC_NUM_SAMPLES) ? (double)(_samples[i] * w) : 0.0;
        _fft_imag[i] = 0.0;
    }

    _fft.compute(FFTDirection::Forward);
    _fft.complexToMagnitude();

    float bin_width = (float)ACOUSTIC_SAMPLE_RATE / (float)ACOUSTIC_FFT_SIZE;
    int useful_bins = ACOUSTIC_FFT_SIZE / 2;

    // Dominant frequency
    float max_mag = 0.0f;
    int max_bin = 1;
    for (int i = 1; i < useful_bins; i++) {
        if ((float)_fft_real[i] > max_mag) { max_mag = (float)_fft_real[i]; max_bin = i; }
    }
    result.dominant_freq = (float)max_bin * bin_width;

    // Spectral centroid
    double w_sum = 0.0, t_mag = 0.0;
    for (int i = 1; i < useful_bins; i++) {
        w_sum += _fft_real[i] * (double)i * (double)bin_width;
        t_mag += _fft_real[i];
    }
    result.spectral_centroid = (t_mag > 1e-6) ? (float)(w_sum / t_mag) : 0.0f;

    // Decay ratio
    float peak = 0.0f;
    int peak_idx = 0;
    for (int i = 0; i < ACOUSTIC_NUM_SAMPLES; i++) {
        float a = fabsf(_samples[i]);
        if (a > peak) { peak = a; peak_idx = i; }
    }
    int tail_start = peak_idx + (ACOUSTIC_NUM_SAMPLES - peak_idx) * 3 / 4;
    if (tail_start >= ACOUSTIC_NUM_SAMPLES) tail_start = ACOUSTIC_NUM_SAMPLES - 1;
    float tail_sum = 0.0f;
    int tail_count = 0;
    for (int i = tail_start; i < ACOUSTIC_NUM_SAMPLES; i++) { tail_sum += fabsf(_samples[i]); tail_count++; }
    result.decay_ratio = (peak > 1.0f && tail_count > 0) ? (tail_sum / tail_count) / peak : 0.0f;

    result.confidence = 0.8f;
    result.valid = true;
    return result;
}
