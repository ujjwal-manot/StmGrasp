#ifndef DS_FUSION_H
#define DS_FUSION_H

#include "config.h"

/*
 * Dempster-Shafer Evidence Theory for Multi-Modal Material Classification
 *
 * Each sensor modality produces a mass function (basic probability assignment)
 * over the frame of discernment Theta = {metal, skin, plastic, wood, glass, cardboard}.
 * Masses are assigned to singleton sets (individual materials) and to Theta itself
 * (representing ignorance/uncertainty).
 *
 * Dempster's rule of combination fuses evidence from independent sources,
 * handling conflict between sensors gracefully.
 *
 * This replaces the naive weighted-average crossValidateMaterial() approach
 * with principled, theoretically-grounded evidence accumulation.
 */

// Subset representation: bitmask over MATERIAL_COUNT materials
// Bit 0 = MAT_METAL, Bit 1 = MAT_SKIN, ..., Bit 5 = MAT_CARDBOARD
#define DS_THETA    ((uint8_t)((1 << MATERIAL_COUNT) - 1))  // full set = 0b00111111
#define DS_EMPTY    ((uint8_t)0)

// Maximum number of focal elements per mass function
// Singletons (6) + full set (1) + a few composite sets = 10
#define DS_MAX_FOCAL  10

// Minimum mass to keep (prune below this)
#define DS_MASS_EPSILON  0.001f

// Maximum ignorance when sensor has zero confidence
#define DS_MAX_IGNORANCE 0.95f

// Gaussian model parameters for each material per sensor modality
typedef struct {
    // Impedance: log10(|Z|) and phase in degrees
    float imp_log_mag_mean;
    float imp_log_mag_std;
    float imp_phase_mean;
    float imp_phase_std;
    // Acoustic: dominant frequency, decay ratio
    float aco_freq_mean;
    float aco_freq_std;
    float aco_decay_mean;
    float aco_decay_std;
    // Curvature: compatible geometries (bitmask of geometry types 0-4)
    uint8_t compatible_geometries;  // bit 0=flat, 1=convex, 2=concave, 3=edge, 4=cylinder
} MaterialModel;

// A single focal element: a subset of Theta with assigned mass
typedef struct {
    uint8_t subset;   // bitmask identifying the subset
    float   mass;     // mass value in [0, 1]
} DSFocalElement;

// Mass function: basic probability assignment over 2^Theta
typedef struct {
    DSFocalElement elements[DS_MAX_FOCAL];
    uint8_t count;
} DSMassFunction;

// Final fusion result with belief and plausibility for each material
typedef struct {
    float belief[MATERIAL_COUNT];        // Bel({material_i}) - lower bound on probability
    float plausibility[MATERIAL_COUNT];  // Pl({material_i}) - upper bound on probability
    float mass_singleton[MATERIAL_COUNT]; // mass assigned directly to {material_i}
    float mass_theta;                    // residual ignorance
    float conflict;                      // K: degree of inter-source conflict (0=agree, 1=total conflict)
    uint8_t best_material;               // index of material with highest belief
    float best_belief;                   // belief of the best material
    float best_plausibility;             // plausibility of the best material
    bool valid;
} DSResult;

// Decision log entry for explainable dashboard
#define DS_LOG_MAX 8
#define DS_LOG_MSG_LEN 80

typedef struct {
    char messages[DS_LOG_MAX][DS_LOG_MSG_LEN];
    uint8_t count;
} DSDecisionLog;

// ─── Public API ──────────────────────────────────────────────

void dsInit(void);

// Create mass functions from individual sensor readings
DSMassFunction dsFromImpedance(float magnitude, float phase_deg, float confidence);
DSMassFunction dsFromAcoustic(float dominant_freq, float decay_ratio, float confidence);
DSMassFunction dsFromCurvature(uint8_t geometry, float flatness, bool sharp_edge);

// Combine two mass functions using Dempster's rule
DSMassFunction dsCombine(const DSMassFunction* m1, const DSMassFunction* m2);

// Combine an array of mass functions sequentially
DSMassFunction dsCombineAll(const DSMassFunction* masses, uint8_t count);

// Extract belief, plausibility, and best material from a combined mass function
DSResult dsGetResult(const DSMassFunction* combined);

// Get the human-readable decision log from the last fusion
const DSDecisionLog* dsGetLog(void);

// Utility: get material name from index
const char* dsGetMaterialName(uint8_t idx);

#endif // DS_FUSION_H
