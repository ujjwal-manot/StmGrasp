#ifndef HD_SLIP_H
#define HD_SLIP_H

#include "config.h"

/*
 * Hyperdimensional Computing for Slip Detection
 *
 * Uses binary hypervectors (D=2048 bits = 256 bytes) for ultra-lightweight
 * classification of slip vs. stable contact. The model is ~1KB total and
 * runs inference in microseconds.
 *
 * Approach:
 *   1. Each sensor feature (FSR AC-RMS, DC force, dF/dt) is quantized
 *      to discrete levels and mapped to a hypervector via level encoding.
 *   2. Feature hypervectors are bound together (XOR) to form a query vector.
 *   3. Query is compared to learned SLIP and STABLE class vectors via
 *      Hamming distance.
 *   4. The class with lower Hamming distance wins.
 *
 * Training:
 *   - Accumulate examples during a calibration phase.
 *   - Class vectors are the majority-vote bundle of all training examples.
 *   - Can be done online (incremental learning) with single-pass updates.
 *
 * Reference: Neubert et al., "Hyperdimensional Computing for Tactile Sensing"
 */

// Hypervector dimensions (bits)
#define HD_DIM          2048
#define HD_BYTES        (HD_DIM / 8)   // 256 bytes per vector

// Number of input features for slip detection
#define HD_NUM_FEATURES 7
// Feature indices
#define HD_FEAT_FSR1_AC   0
#define HD_FEAT_FSR2_AC   1
#define HD_FEAT_FSR3_AC   2
#define HD_FEAT_FSR1_DC   3
#define HD_FEAT_FSR2_DC   4
#define HD_FEAT_FSR3_DC   5
#define HD_FEAT_DFDT      6  // rate of change of total force

// Quantization levels per feature
#define HD_NUM_LEVELS   16

// Number of classes
#define HD_NUM_CLASSES  2
#define HD_CLASS_STABLE 0
#define HD_CLASS_SLIP   1

// Minimum training examples per class before inference is valid
#define HD_MIN_TRAIN    5

// PRNG seed for deterministic random vector generation
#define HD_SEED         0xDEADBEEF

// Result of HD slip inference
typedef struct {
    uint8_t  predicted_class;   // HD_CLASS_STABLE or HD_CLASS_SLIP
    float    confidence;        // 0-1, based on distance margin
    uint16_t hamming_stable;    // Hamming distance to STABLE class
    uint16_t hamming_slip;      // Hamming distance to SLIP class
    bool     valid;             // false if insufficient training data
} HDSlipResult;

// ─── Public API ──────────────────────────────────────────────

// Initialize HD computing module (generates seed vectors)
void hdInit(void);

// Encode a set of sensor features into a query hypervector
// features[HD_NUM_FEATURES]: raw float values
// Returns result of classification
HDSlipResult hdClassify(const float features[HD_NUM_FEATURES]);

// Add a training example (call during calibration)
// label: HD_CLASS_STABLE or HD_CLASS_SLIP
void hdTrain(const float features[HD_NUM_FEATURES], uint8_t label);

// Reset learned class vectors (start fresh training)
void hdResetTraining(void);

// Get training count per class
uint16_t hdGetTrainCount(uint8_t label);

// Check if model has enough training data for valid inference
bool hdIsReady(void);

// Auto-train from threshold-based ground truth
// Call this every slip detection cycle with the old RMS-based result
// to bootstrap the HD model from existing slip detection
void hdAutoTrain(const float features[HD_NUM_FEATURES], bool rms_slip_detected);

#endif // HD_SLIP_H
