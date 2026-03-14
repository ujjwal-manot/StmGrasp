#include "acoustic.h"
#include <arduinoFFT.h>
#include <math.h>

static float _samples[ACOUSTIC_NUM_SAMPLES];
static double _fft_real[ACOUSTIC_FFT_SIZE];
static double _fft_imag[ACOUSTIC_FFT_SIZE];

static int _baseline_adc = 0;

// ArduinoFFT instance (uses double arrays internally)
static ArduinoFFT<double> _fft = ArduinoFFT<double>(_fft_real, _fft_imag, ACOUSTIC_FFT_SIZE, (double)ACOUSTIC_SAMPLE_RATE);


void initAcoustic() {
    pinMode(PIN_PIEZO_ADC, INPUT);
    analogReadResolution(12);

    // Establish piezo baseline (average of 64 quiet reads)
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
        int val = analogRead(PIN_PIEZO_ADC);
        int delta = abs(val - _baseline_adc);
        if (delta > ACOUSTIC_TAP_THRESHOLD) {
            return true;
        }
        delayMicroseconds(100);
    }
    return false;
}


void recordTapResponse(float* sample_buf, int num_samples) {
    // Sample at ACOUSTIC_SAMPLE_RATE Hz using tight timing
    uint32_t interval_us = 1000000UL / ACOUSTIC_SAMPLE_RATE; // 125 µs
    uint32_t next_us = micros();

    for (int i = 0; i < num_samples; i++) {
        while ((int32_t)(next_us - micros()) > 0) { /* spin */ }
        int raw = analogRead(PIN_PIEZO_ADC);
        // Convert to signed float centered at zero
        sample_buf[i] = (float)(raw - _baseline_adc);
        next_us += interval_us;
    }
}


static float _computeDominantFreq(const double* magnitudes, int num_bins, float bin_width) {
    float max_mag = 0.0f;
    int max_bin = 1; // skip DC at bin 0
    for (int i = 1; i < num_bins; i++) {
        if ((float)magnitudes[i] > max_mag) {
            max_mag = (float)magnitudes[i];
            max_bin = i;
        }
    }
    return (float)max_bin * bin_width;
}


static float _computeSpectralCentroid(const double* magnitudes, int num_bins, float bin_width) {
    double weighted_sum = 0.0;
    double total_mag = 0.0;
    for (int i = 1; i < num_bins; i++) {
        double freq = (double)i * (double)bin_width;
        weighted_sum += magnitudes[i] * freq;
        total_mag += magnitudes[i];
    }
    if (total_mag < 1e-6) return 0.0f;
    return (float)(weighted_sum / total_mag);
}


static float _computeDecayRatio(const float* samples, int num_samples) {
    // Find peak amplitude
    float peak = 0.0f;
    int peak_idx = 0;
    for (int i = 0; i < num_samples; i++) {
        float a = fabsf(samples[i]);
        if (a > peak) {
            peak = a;
            peak_idx = i;
        }
    }
    if (peak < 1.0f) return 0.0f;

    // Measure amplitude at 75% of the buffer past the peak
    int tail_start = peak_idx + (num_samples - peak_idx) * 3 / 4;
    if (tail_start >= num_samples) tail_start = num_samples - 1;

    // Average absolute amplitude in the last quarter
    float tail_sum = 0.0f;
    int tail_count = 0;
    for (int i = tail_start; i < num_samples; i++) {
        tail_sum += fabsf(samples[i]);
        tail_count++;
    }
    if (tail_count == 0) return 0.0f;
    float tail_avg = tail_sum / (float)tail_count;

    return tail_avg / peak;
}


AcousticResult analyzeTap() {
    AcousticResult result;
    result.material_idx = MAT_UNKNOWN;
    result.confidence = 0.0f;
    result.dominant_freq = 0.0f;
    result.spectral_centroid = 0.0f;
    result.decay_ratio = 0.0f;
    result.valid = false;
    memset(result.spectrum, 0, sizeof(result.spectrum));

    // Record the impulse response
    recordTapResponse(_samples, ACOUSTIC_NUM_SAMPLES);

    // Check that we got a meaningful signal
    float max_amp = 0.0f;
    for (int i = 0; i < ACOUSTIC_NUM_SAMPLES; i++) {
        float a = fabsf(_samples[i]);
        if (a > max_amp) max_amp = a;
    }
    if (max_amp < (float)ACOUSTIC_TAP_THRESHOLD * 0.5f) {
        // Signal too weak to analyze
        return result;
    }

    // Copy first ACOUSTIC_FFT_SIZE samples into FFT buffers, apply Hann window
    for (int i = 0; i < ACOUSTIC_FFT_SIZE; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * M_PI * (float)i / (float)(ACOUSTIC_FFT_SIZE - 1)));
        if (i < ACOUSTIC_NUM_SAMPLES) {
            _fft_real[i] = (double)(_samples[i] * w);
        } else {
            _fft_real[i] = 0.0;
        }
        _fft_imag[i] = 0.0;
    }

    // Compute FFT
    _fft.compute(FFTDirection::Forward);
    _fft.complexToMagnitude();

    // Frequency resolution
    float bin_width = (float)ACOUSTIC_SAMPLE_RATE / (float)ACOUSTIC_FFT_SIZE;
    int useful_bins = ACOUSTIC_FFT_SIZE / 2;

    // Copy magnitude spectrum to result
    for (int i = 0; i < useful_bins; i++) {
        result.spectrum[i] = (float)_fft_real[i];
    }

    result.dominant_freq = _computeDominantFreq(_fft_real, useful_bins, bin_width);
    result.spectral_centroid = _computeSpectralCentroid(_fft_real, useful_bins, bin_width);
    result.decay_ratio = _computeDecayRatio(_samples, ACOUSTIC_NUM_SAMPLES);

    float conf;
    result.material_idx = classifyTapMaterial(result.dominant_freq, result.spectral_centroid,
                                              result.decay_ratio, &conf);
    result.confidence = conf;
    result.valid = true;
    return result;
}


uint8_t classifyTapMaterial(float dominant_freq, float spectral_centroid,
                            float decay_ratio, float* out_confidence) {
    /*
     * Heuristic classification based on acoustic properties:
     *   Metal:     high freq (>1 kHz), slow decay (ratio > 0.2), bright ringing
     *   Glass:     very high freq (>2 kHz), very slow decay (ratio > 0.25)
     *   Plastic:   mid freq (500-1500 Hz), moderate decay
     *   Wood:      low freq (<500 Hz), fast decay (ratio < 0.1)
     *   Cardboard: very low freq (<300 Hz), very fast decay
     *   Skin:      extremely damped, almost no ring
     */

    struct AcousticProfile {
        uint8_t mat_idx;
        float freq_center;
        float freq_width;
        float decay_center;
        float decay_width;
    };

    static const AcousticProfile profiles[] = {
        { MAT_METAL,     1800.0f, 1200.0f, 0.35f, 0.20f },
        { MAT_GLASS,     2800.0f, 1000.0f, 0.30f, 0.15f },
        { MAT_PLASTIC,    900.0f,  600.0f, 0.12f, 0.10f },
        { MAT_WOOD,       350.0f,  250.0f, 0.06f, 0.06f },
        { MAT_CARDBOARD,  200.0f,  150.0f, 0.03f, 0.04f },
        { MAT_SKIN,       100.0f,  100.0f, 0.01f, 0.02f },
    };

    float best_score = 1e9f;
    uint8_t best_idx = MAT_UNKNOWN;

    for (int i = 0; i < 6; i++) {
        float d_freq = (dominant_freq - profiles[i].freq_center) / profiles[i].freq_width;
        float d_decay = (decay_ratio - profiles[i].decay_center) / profiles[i].decay_width;
        float score = sqrtf(d_freq * d_freq + d_decay * d_decay);
        if (score < best_score) {
            best_score = score;
            best_idx = profiles[i].mat_idx;
        }
    }

    // Convert distance to confidence (closer = higher confidence)
    float conf = 1.0f / (1.0f + best_score);
    if (out_confidence) *out_confidence = conf;

    return best_idx;
}


uint8_t crossValidateMaterial(const ImpedanceResult& imp, const AcousticResult& aco,
                              float* out_confidence) {
    if (!imp.valid && !aco.valid) {
        if (out_confidence) *out_confidence = 0.0f;
        return MAT_UNKNOWN;
    }

    if (!aco.valid) {
        if (out_confidence) *out_confidence = imp.confidence;
        return imp.material_idx;
    }

    if (!imp.valid) {
        if (out_confidence) *out_confidence = aco.confidence;
        return aco.material_idx;
    }

    // Both valid — fuse
    if (imp.material_idx == aco.material_idx) {
        // Agreement: boost confidence
        float conf = fminf(1.0f, (imp.confidence * 0.7f + aco.confidence * 0.3f) + 0.1f);
        if (out_confidence) *out_confidence = conf;
        return imp.material_idx;
    }

    // Disagreement: trust impedance more heavily (0.7 vs 0.3)
    float imp_weighted = imp.confidence * 0.7f;
    float aco_weighted = aco.confidence * 0.3f;

    uint8_t chosen;
    float conf;
    if (imp_weighted >= aco_weighted) {
        chosen = imp.material_idx;
        conf = imp_weighted / (imp_weighted + aco_weighted + 1e-6f);
    } else {
        chosen = aco.material_idx;
        conf = aco_weighted / (imp_weighted + aco_weighted + 1e-6f);
    }

    // Reduce confidence due to disagreement
    conf *= 0.85f;
    if (out_confidence) *out_confidence = conf;
    return chosen;
}
