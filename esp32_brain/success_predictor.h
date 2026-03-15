#ifndef SUCCESS_PREDICTOR_H
#define SUCCESS_PREDICTOR_H

#include "config.h"

/*
 * Grasp Success Predictor
 *
 * Estimates P(success) before executing a grasp, using a logistic regression
 * model on features extracted from the current sensor state and grasp plan.
 *
 * Features:
 *   1. Material classification confidence (from DS fusion)
 *   2. Surface flatness (from curvature sensor)
 *   3. Depth data quality (valid zones / total zones)
 *   4. Grasp plan quality score
 *   5. Object size estimate (from depth grid)
 *   6. Geometry-strategy compatibility score
 *
 * The model can be updated online using stochastic gradient descent
 * after each grasp attempt (success/failure feedback).
 *
 * Initial weights are hand-tuned based on engineering judgment,
 * then refined through actual grasp experience stored in SPIFFS.
 */

// Number of predictor features
#define SP_NUM_FEATURES  6

// Feature indices
#define SP_FEAT_MAT_CONF     0  // Material classification confidence
#define SP_FEAT_FLATNESS     1  // Surface flatness 0-1
#define SP_FEAT_DEPTH_QUAL   2  // Depth data quality 0-1
#define SP_FEAT_PLAN_QUAL    3  // Grasp plan quality score 0-1
#define SP_FEAT_OBJ_SIZE     4  // Normalized object size 0-1
#define SP_FEAT_GEO_COMPAT   5  // Geometry-strategy compatibility 0-1

// Minimum confidence to proceed with grasp
#define SP_PROCEED_THRESHOLD  0.55f

// Learning rate for online SGD updates
#define SP_LEARNING_RATE     0.05f

// Maximum stored grasp history entries (in SPIFFS)
#define SP_MAX_HISTORY       100

// Prediction result
typedef struct {
    float probability;      // P(success) in [0, 1]
    bool  should_proceed;   // true if probability >= SP_PROCEED_THRESHOLD
    char  reason[64];       // human-readable explanation
    float feature_contributions[SP_NUM_FEATURES]; // per-feature contribution to score
} SuccessPrediction;

// Grasp outcome record for online learning
typedef struct {
    float features[SP_NUM_FEATURES];
    bool  success;
    uint8_t material;
    uint8_t geometry;
    uint8_t strategy;
} GraspRecord;

// ─── Public API ──────────────────────────────────────────────

// Initialize predictor (loads weights from SPIFFS if available)
void spInit(void);

// Predict grasp success given current sensor state
SuccessPrediction spPredict(float mat_confidence,
                            float flatness,
                            float depth_quality,
                            float plan_quality,
                            float object_size,
                            float geo_compatibility);

// Record a grasp outcome for online learning
void spRecordOutcome(const float features[SP_NUM_FEATURES], bool success);

// Update model weights using accumulated outcomes (call periodically)
void spUpdateModel(void);

// Save current weights to SPIFFS
void spSaveWeights(void);

// Load weights from SPIFFS
bool spLoadWeights(void);

// Get number of recorded outcomes
uint16_t spGetHistoryCount(void);

// Get overall success rate from history
float spGetSuccessRate(void);

#endif // SUCCESS_PREDICTOR_H
