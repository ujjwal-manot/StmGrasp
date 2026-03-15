#include "hd_slip.h"
#include <string.h>
#include <math.h>

/*
 * Binary hyperdimensional computing for slip detection.
 *
 * Memory layout:
 *   _seed_vectors[HD_NUM_FEATURES][HD_BYTES]  = 7 * 256 = 1,792 bytes
 *   _class_vectors[HD_NUM_CLASSES][HD_BYTES]   = 2 * 256 =   512 bytes
 *   _class_counts[HD_NUM_CLASSES][HD_DIM]      = 2 * 2048 * 2 = 8,192 bytes (int16 accumulators)
 *   Total: ~10.5 KB — well within ESP32 budget
 */

// Seed vectors for each feature (generated deterministically from PRNG)
static uint8_t _seed_vectors[HD_NUM_FEATURES][HD_BYTES];

// Class prototype vectors (binary, thresholded from accumulators)
static uint8_t _class_vectors[HD_NUM_CLASSES][HD_BYTES];

// Accumulator for online learning: count of 1-bits per position per class
static int16_t _class_accum[HD_NUM_CLASSES][HD_DIM];

// Training example count per class
static uint16_t _train_count[HD_NUM_CLASSES];

// Feature normalization ranges (updated during training)
static float _feat_min[HD_NUM_FEATURES];
static float _feat_max[HD_NUM_FEATURES];
static bool _norm_initialized;

// ─── PRNG for deterministic random vector generation ─────────

static uint32_t _prng_state;

static uint32_t _prng_next(void) {
    // xorshift32
    _prng_state ^= _prng_state << 13;
    _prng_state ^= _prng_state >> 17;
    _prng_state ^= _prng_state << 5;
    return _prng_state;
}

static void _prng_seed(uint32_t seed) {
    _prng_state = seed;
    if (_prng_state == 0) _prng_state = 1;
}

// ─── Hypervector operations ──────────────────────────────────

// Fill vector with random bits from PRNG
static void _randomVector(uint8_t* vec) {
    for (int i = 0; i < HD_BYTES; i += 4) {
        uint32_t r = _prng_next();
        vec[i]     = (uint8_t)(r & 0xFF);
        vec[i + 1] = (uint8_t)((r >> 8) & 0xFF);
        vec[i + 2] = (uint8_t)((r >> 16) & 0xFF);
        if (i + 3 < HD_BYTES) {
            vec[i + 3] = (uint8_t)((r >> 24) & 0xFF);
        }
    }
}

// Circular bit-shift a vector by 'shift' positions (for level encoding)
static void _circularShift(const uint8_t* src, uint8_t* dst, int shift) {
    shift = shift % HD_DIM;
    if (shift < 0) shift += HD_DIM;
    if (shift == 0) {
        memcpy(dst, src, HD_BYTES);
        return;
    }

    int byte_shift = shift / 8;
    int bit_shift = shift % 8;

    for (int i = 0; i < HD_BYTES; i++) {
        int src_byte1 = (i + byte_shift) % HD_BYTES;
        int src_byte2 = (i + byte_shift + 1) % HD_BYTES;
        if (bit_shift == 0) {
            dst[i] = src[src_byte1];
        } else {
            dst[i] = (uint8_t)((src[src_byte1] << bit_shift) |
                               (src[src_byte2] >> (8 - bit_shift)));
        }
    }
}

// XOR bind two vectors: result = a XOR b
static void _xorBind(const uint8_t* a, const uint8_t* b, uint8_t* result) {
    for (int i = 0; i < HD_BYTES; i++) {
        result[i] = a[i] ^ b[i];
    }
}

// Hamming distance between two binary vectors
static uint16_t _hammingDistance(const uint8_t* a, const uint8_t* b) {
    uint16_t dist = 0;
    for (int i = 0; i < HD_BYTES; i++) {
        uint8_t diff = a[i] ^ b[i];
        // Count bits (Brian Kernighan's algorithm)
        while (diff) {
            diff &= diff - 1;
            dist++;
        }
    }
    return dist;
}

// Threshold accumulators to produce binary class vector
static void _thresholdClass(uint8_t class_idx) {
    uint16_t count = _train_count[class_idx];
    if (count == 0) {
        memset(_class_vectors[class_idx], 0, HD_BYTES);
        return;
    }
    int16_t threshold = (int16_t)(count / 2); // majority vote

    for (int i = 0; i < HD_DIM; i++) {
        int byte_idx = i / 8;
        int bit_idx = 7 - (i % 8);
        if (_class_accum[class_idx][i] > threshold) {
            _class_vectors[class_idx][byte_idx] |= (uint8_t)(1 << bit_idx);
        } else {
            _class_vectors[class_idx][byte_idx] &= (uint8_t)~(1 << bit_idx);
        }
    }
}

// ─── Feature encoding ────────────────────────────────────────

// Quantize a float feature to a level index [0, HD_NUM_LEVELS-1]
static uint8_t _quantize(float value, int feat_idx) {
    if (!_norm_initialized) return HD_NUM_LEVELS / 2;

    float range = _feat_max[feat_idx] - _feat_min[feat_idx];
    if (range < 1e-6f) return HD_NUM_LEVELS / 2;

    float normalized = (value - _feat_min[feat_idx]) / range;
    normalized = fminf(fmaxf(normalized, 0.0f), 1.0f);
    int level = (int)(normalized * (HD_NUM_LEVELS - 1) + 0.5f);
    return (uint8_t)level;
}

// Encode features into a single query hypervector
static void _encodeFeatures(const float features[HD_NUM_FEATURES], uint8_t* query) {
    uint8_t temp1[HD_BYTES];
    uint8_t temp2[HD_BYTES];

    // Start with the first feature's level vector
    uint8_t level = _quantize(features[0], 0);
    int shift = (int)level * (HD_DIM / HD_NUM_LEVELS);
    _circularShift(_seed_vectors[0], query, shift);

    // Bind (XOR) with each subsequent feature's level vector
    for (int f = 1; f < HD_NUM_FEATURES; f++) {
        level = _quantize(features[f], f);
        shift = (int)level * (HD_DIM / HD_NUM_LEVELS);
        _circularShift(_seed_vectors[f], temp1, shift);
        _xorBind(query, temp1, temp2);
        memcpy(query, temp2, HD_BYTES);
    }
}

// Update normalization ranges from a feature vector
static void _updateNorm(const float features[HD_NUM_FEATURES]) {
    if (!_norm_initialized) {
        for (int i = 0; i < HD_NUM_FEATURES; i++) {
            _feat_min[i] = features[i];
            _feat_max[i] = features[i];
        }
        _norm_initialized = true;
        return;
    }
    for (int i = 0; i < HD_NUM_FEATURES; i++) {
        if (features[i] < _feat_min[i]) _feat_min[i] = features[i];
        if (features[i] > _feat_max[i]) _feat_max[i] = features[i];
    }
}

// ─── Public API ──────────────────────────────────────────────

void hdInit(void) {
    memset(_class_accum, 0, sizeof(_class_accum));
    memset(_class_vectors, 0, sizeof(_class_vectors));
    memset(_train_count, 0, sizeof(_train_count));
    _norm_initialized = false;

    // Initialize default normalization ranges
    // FSR AC RMS: 0 to 2.0 N
    _feat_min[HD_FEAT_FSR1_AC] = 0.0f; _feat_max[HD_FEAT_FSR1_AC] = 2.0f;
    _feat_min[HD_FEAT_FSR2_AC] = 0.0f; _feat_max[HD_FEAT_FSR2_AC] = 2.0f;
    _feat_min[HD_FEAT_FSR3_AC] = 0.0f; _feat_max[HD_FEAT_FSR3_AC] = 2.0f;
    // FSR DC: 0 to 20 N
    _feat_min[HD_FEAT_FSR1_DC] = 0.0f; _feat_max[HD_FEAT_FSR1_DC] = 20.0f;
    _feat_min[HD_FEAT_FSR2_DC] = 0.0f; _feat_max[HD_FEAT_FSR2_DC] = 20.0f;
    _feat_min[HD_FEAT_FSR3_DC] = 0.0f; _feat_max[HD_FEAT_FSR3_DC] = 20.0f;
    // dF/dt: -10 to 10 N/s
    _feat_min[HD_FEAT_DFDT] = -10.0f; _feat_max[HD_FEAT_DFDT] = 10.0f;
    _norm_initialized = true;

    // Generate deterministic seed vectors
    _prng_seed(HD_SEED);
    for (int f = 0; f < HD_NUM_FEATURES; f++) {
        _randomVector(_seed_vectors[f]);
    }
}


HDSlipResult hdClassify(const float features[HD_NUM_FEATURES]) {
    HDSlipResult result;
    result.predicted_class = HD_CLASS_STABLE;
    result.confidence = 0.0f;
    result.hamming_stable = HD_DIM / 2;
    result.hamming_slip = HD_DIM / 2;
    result.valid = false;

    if (!hdIsReady()) return result;

    // Encode query
    uint8_t query[HD_BYTES];
    _encodeFeatures(features, query);

    // Compute Hamming distance to each class
    result.hamming_stable = _hammingDistance(query, _class_vectors[HD_CLASS_STABLE]);
    result.hamming_slip   = _hammingDistance(query, _class_vectors[HD_CLASS_SLIP]);

    // Classify by minimum distance
    if (result.hamming_slip < result.hamming_stable) {
        result.predicted_class = HD_CLASS_SLIP;
    } else {
        result.predicted_class = HD_CLASS_STABLE;
    }

    // Confidence: margin between distances, normalized
    int16_t margin = (int16_t)result.hamming_stable - (int16_t)result.hamming_slip;
    // margin > 0 means closer to SLIP, margin < 0 means closer to STABLE
    float abs_margin = fabsf((float)margin);
    // Normalize by HD_DIM/2 (maximum possible distance)
    result.confidence = fminf(abs_margin / (float)(HD_DIM / 4), 1.0f);
    result.valid = true;

    return result;
}


void hdTrain(const float features[HD_NUM_FEATURES], uint8_t label) {
    if (label >= HD_NUM_CLASSES) return;

    _updateNorm(features);

    // Encode the example
    uint8_t encoded[HD_BYTES];
    _encodeFeatures(features, encoded);

    // Accumulate into class vector (bundle operation)
    for (int i = 0; i < HD_DIM; i++) {
        int byte_idx = i / 8;
        int bit_idx = 7 - (i % 8);
        if (encoded[byte_idx] & (1 << bit_idx)) {
            _class_accum[label][i]++;
        }
    }

    _train_count[label]++;

    // Re-threshold to update binary class vector
    _thresholdClass(label);
}


void hdResetTraining(void) {
    memset(_class_accum, 0, sizeof(_class_accum));
    memset(_class_vectors, 0, sizeof(_class_vectors));
    memset(_train_count, 0, sizeof(_train_count));
}


uint16_t hdGetTrainCount(uint8_t label) {
    if (label >= HD_NUM_CLASSES) return 0;
    return _train_count[label];
}


bool hdIsReady(void) {
    return (_train_count[HD_CLASS_STABLE] >= HD_MIN_TRAIN &&
            _train_count[HD_CLASS_SLIP]   >= HD_MIN_TRAIN);
}


void hdAutoTrain(const float features[HD_NUM_FEATURES], bool rms_slip_detected) {
    // Bootstrap HD model from existing RMS-based slip detection.
    // Only train if we have fewer than 200 examples per class
    // to prevent the model from drifting too far.
    uint8_t label = rms_slip_detected ? HD_CLASS_SLIP : HD_CLASS_STABLE;
    if (_train_count[label] < 200) {
        hdTrain(features, label);
    }
}
