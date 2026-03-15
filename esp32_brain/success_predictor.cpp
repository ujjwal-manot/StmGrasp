#include "success_predictor.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <SPIFFS.h>

/*
 * Logistic regression grasp success predictor with online SGD learning.
 *
 * Model: P(success) = sigmoid(w0 + w1*x1 + w2*x2 + ... + w6*x6)
 * where sigmoid(z) = 1 / (1 + exp(-z))
 *
 * Initial weights are hand-tuned based on engineering judgment.
 * After each grasp attempt, the model is updated using binary cross-entropy loss.
 */

// Model weights: bias + SP_NUM_FEATURES weights
static float _weights[SP_NUM_FEATURES + 1];

// History ring buffer
static GraspRecord _history[SP_MAX_HISTORY];
static uint16_t _history_count = 0;
static uint16_t _history_write_idx = 0;
static uint16_t _success_count = 0;

// SPIFFS paths
static const char* WEIGHTS_PATH = "/sp_weights.bin";
static const char* HISTORY_PATH = "/sp_history.bin";

// ─── Internal helpers ────────────────────────────────────────

static float _sigmoid(float z) {
    if (z > 10.0f) return 1.0f;
    if (z < -10.0f) return 0.0f;
    return 1.0f / (1.0f + expf(-z));
}

static float _computeLogit(const float features[SP_NUM_FEATURES]) {
    float z = _weights[0]; // bias
    for (int i = 0; i < SP_NUM_FEATURES; i++) {
        z += _weights[i + 1] * features[i];
    }
    return z;
}

static void _setDefaultWeights(void) {
    // Bias: slightly positive (assume grasps are more likely to succeed than fail)
    _weights[0] = 0.5f;
    // Material confidence: strong positive (high confidence = high success)
    _weights[1] = 2.0f;
    // Flatness: moderate positive (flat surfaces are easier to grasp)
    _weights[2] = 1.0f;
    // Depth quality: moderate positive
    _weights[3] = 1.2f;
    // Plan quality: strong positive
    _weights[4] = 2.5f;
    // Object size: moderate positive (bigger objects are easier)
    _weights[5] = 0.8f;
    // Geometry-strategy compatibility: strong positive
    _weights[6] = 1.8f;
}

// ─── Public API ──────────────────────────────────────────────

void spInit(void) {
    _setDefaultWeights();
    _history_count = 0;
    _history_write_idx = 0;
    _success_count = 0;

    // Try to load saved weights from SPIFFS
    if (SPIFFS.begin(true)) {
        spLoadWeights();
    }
}


SuccessPrediction spPredict(float mat_confidence,
                            float flatness,
                            float depth_quality,
                            float plan_quality,
                            float object_size,
                            float geo_compatibility) {
    SuccessPrediction pred;
    memset(&pred, 0, sizeof(pred));

    float features[SP_NUM_FEATURES] = {
        mat_confidence, flatness, depth_quality,
        plan_quality, object_size, geo_compatibility
    };

    float z = _computeLogit(features);
    pred.probability = _sigmoid(z);
    pred.should_proceed = (pred.probability >= SP_PROCEED_THRESHOLD);

    // Compute per-feature contributions
    for (int i = 0; i < SP_NUM_FEATURES; i++) {
        pred.feature_contributions[i] = _weights[i + 1] * features[i];
    }

    // Generate human-readable reason
    if (pred.should_proceed) {
        // Find the strongest contributing feature
        float max_contrib = 0.0f;
        int max_idx = 0;
        for (int i = 0; i < SP_NUM_FEATURES; i++) {
            if (pred.feature_contributions[i] > max_contrib) {
                max_contrib = pred.feature_contributions[i];
                max_idx = i;
            }
        }
        static const char* feat_names[] = {
            "material ID", "flat surface", "good depth",
            "plan quality", "object size", "geo match"
        };
        snprintf(pred.reason, sizeof(pred.reason),
                 "P=%.0f%% proceed (%s strongest)",
                 pred.probability * 100.0f, feat_names[max_idx]);
    } else {
        // Find the weakest feature dragging down the score
        float min_contrib = 1e9f;
        int min_idx = 0;
        for (int i = 0; i < SP_NUM_FEATURES; i++) {
            if (pred.feature_contributions[i] < min_contrib) {
                min_contrib = pred.feature_contributions[i];
                min_idx = i;
            }
        }
        static const char* feat_names[] = {
            "material ID", "flatness", "depth data",
            "plan quality", "object size", "geo match"
        };
        snprintf(pred.reason, sizeof(pred.reason),
                 "P=%.0f%% reposition (weak %s)",
                 pred.probability * 100.0f, feat_names[min_idx]);
    }

    return pred;
}


void spRecordOutcome(const float features[SP_NUM_FEATURES], bool success) {
    GraspRecord* rec = &_history[_history_write_idx];
    memcpy(rec->features, features, sizeof(float) * SP_NUM_FEATURES);
    rec->success = success;

    _history_write_idx = (_history_write_idx + 1) % SP_MAX_HISTORY;
    if (_history_count < SP_MAX_HISTORY) _history_count++;
    if (success) _success_count++;
}


void spUpdateModel(void) {
    if (_history_count < 5) return; // need minimum data

    // Single pass SGD over recent history
    uint16_t start = (_history_count >= SP_MAX_HISTORY) ?
                      _history_write_idx : 0;
    uint16_t count = (_history_count < SP_MAX_HISTORY) ?
                      _history_count : SP_MAX_HISTORY;

    for (uint16_t k = 0; k < count; k++) {
        uint16_t idx = (start + k) % SP_MAX_HISTORY;
        const GraspRecord* rec = &_history[idx];

        float pred = _sigmoid(_computeLogit(rec->features));
        float target = rec->success ? 1.0f : 0.0f;
        float error = pred - target;  // gradient of cross-entropy

        // Update weights
        _weights[0] -= SP_LEARNING_RATE * error;  // bias
        for (int i = 0; i < SP_NUM_FEATURES; i++) {
            _weights[i + 1] -= SP_LEARNING_RATE * error * rec->features[i];
        }
    }
}


void spSaveWeights(void) {
    File f = SPIFFS.open(WEIGHTS_PATH, "w");
    if (!f) return;
    f.write((uint8_t*)_weights, sizeof(_weights));
    f.close();

    // Also save history
    File fh = SPIFFS.open(HISTORY_PATH, "w");
    if (!fh) return;
    fh.write((uint8_t*)&_history_count, sizeof(_history_count));
    fh.write((uint8_t*)&_history_write_idx, sizeof(_history_write_idx));
    fh.write((uint8_t*)&_success_count, sizeof(_success_count));
    uint16_t to_write = (_history_count < SP_MAX_HISTORY) ?
                         _history_count : SP_MAX_HISTORY;
    fh.write((uint8_t*)_history, sizeof(GraspRecord) * to_write);
    fh.close();
}


bool spLoadWeights(void) {
    File f = SPIFFS.open(WEIGHTS_PATH, "r");
    if (!f) return false;
    if (f.size() != sizeof(_weights)) { f.close(); return false; }
    f.read((uint8_t*)_weights, sizeof(_weights));
    f.close();

    // Load history
    File fh = SPIFFS.open(HISTORY_PATH, "r");
    if (fh) {
        fh.read((uint8_t*)&_history_count, sizeof(_history_count));
        fh.read((uint8_t*)&_history_write_idx, sizeof(_history_write_idx));
        fh.read((uint8_t*)&_success_count, sizeof(_success_count));
        uint16_t to_read = (_history_count < SP_MAX_HISTORY) ?
                            _history_count : SP_MAX_HISTORY;
        fh.read((uint8_t*)_history, sizeof(GraspRecord) * to_read);
        fh.close();
    }

    return true;
}


uint16_t spGetHistoryCount(void) {
    return _history_count;
}


float spGetSuccessRate(void) {
    if (_history_count == 0) return 0.0f;
    return (float)_success_count / (float)_history_count;
}
