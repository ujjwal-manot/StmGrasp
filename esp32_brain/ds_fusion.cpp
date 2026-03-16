#include "ds_fusion.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * Gaussian material models for each sensor modality.
 * Mean values derived from MATERIAL_DB in config.h.
 * Standard deviations estimated from expected measurement variance.
 *
 * Impedance: operates in log10(|Z|) space for better separation.
 * Acoustic: dominant frequency (Hz) and decay ratio.
 * Curvature: compatible geometry bitmask.
 */
static const MaterialModel _models[MATERIAL_COUNT] = {
    // Metal: very low impedance, near-zero phase, high freq tap, slow decay
    {
        .imp_log_mag_mean = 0.0f,  .imp_log_mag_std = 0.6f,
        .imp_phase_mean   = 0.0f,  .imp_phase_std   = 10.0f,
        .aco_freq_mean    = 1800.0f, .aco_freq_std  = 600.0f,
        .aco_decay_mean   = 0.35f, .aco_decay_std   = 0.12f,
        .compatible_geometries = 0b11111  // all geometries possible
    },
    // Skin: moderate-high impedance, moderate capacitive phase
    {
        .imp_log_mag_mean = 4.7f,  .imp_log_mag_std = 0.5f,
        .imp_phase_mean   = -35.0f, .imp_phase_std  = 12.0f,
        .aco_freq_mean    = 100.0f, .aco_freq_std   = 80.0f,
        .aco_decay_mean   = 0.01f, .aco_decay_std   = 0.02f,
        .compatible_geometries = 0b10011  // flat, convex, cylinder
    },
    // Plastic: very high impedance, strongly capacitive
    {
        .imp_log_mag_mean = 6.0f,  .imp_log_mag_std = 0.5f,
        .imp_phase_mean   = -85.0f, .imp_phase_std  = 8.0f,
        .aco_freq_mean    = 900.0f, .aco_freq_std   = 400.0f,
        .aco_decay_mean   = 0.12f, .aco_decay_std   = 0.08f,
        .compatible_geometries = 0b11111  // all
    },
    // Wood: high impedance, slight capacitive
    {
        .imp_log_mag_mean = 5.7f,  .imp_log_mag_std = 0.5f,
        .imp_phase_mean   = -20.0f, .imp_phase_std  = 12.0f,
        .aco_freq_mean    = 350.0f, .aco_freq_std   = 200.0f,
        .aco_decay_mean   = 0.06f, .aco_decay_std   = 0.05f,
        .compatible_geometries = 0b11001  // flat, edge, cylinder (rarely concave/convex)
    },
    // Glass: extremely high impedance, nearly pure capacitive
    {
        .imp_log_mag_mean = 7.0f,  .imp_log_mag_std = 0.4f,
        .imp_phase_mean   = -88.0f, .imp_phase_std  = 5.0f,
        .aco_freq_mean    = 2800.0f, .aco_freq_std  = 800.0f,
        .aco_decay_mean   = 0.30f, .aco_decay_std   = 0.10f,
        .compatible_geometries = 0b10011  // flat, convex, cylinder
    },
    // Cardboard: moderate impedance, slight capacitive
    {
        .imp_log_mag_mean = 5.3f,  .imp_log_mag_std = 0.5f,
        .imp_phase_mean   = -25.0f, .imp_phase_std  = 12.0f,
        .aco_freq_mean    = 200.0f, .aco_freq_std   = 120.0f,
        .aco_decay_mean   = 0.03f, .aco_decay_std   = 0.03f,
        .compatible_geometries = 0b01001  // flat, edge (cardboard is sheet-like)
    },
};

// Decision log
static DSDecisionLog _log;

// ─── Internal helpers ────────────────────────────────────────

static void _logClear(void) {
    _log.count = 0;
}

static void _logAdd(const char* fmt, ...) {
    if (_log.count >= DS_LOG_MAX) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(_log.messages[_log.count], DS_LOG_MSG_LEN, fmt, args);
    va_end(args);
    _log.count++;
}

// Gaussian probability density (unnormalized log-likelihood)
static float _gaussianLogLikelihood(float x, float mean, float std) {
    if (std < 0.001f) std = 0.001f;
    float d = (x - mean) / std;
    return -0.5f * d * d;
}

// Convert log-likelihoods to normalized mass assignments
// likelihoods[MATERIAL_COUNT]: log-likelihoods for each material
// confidence: 0-1, controls how much mass goes to Theta (ignorance)
// out: populated mass function
static void _likelihoodsToMass(const float likelihoods[MATERIAL_COUNT],
                               float confidence,
                               DSMassFunction* out) {
    out->count = 0;

    // Find max log-likelihood for numerical stability
    float max_ll = -1e9f;
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        if (likelihoods[i] > max_ll) max_ll = likelihoods[i];
    }

    // Convert to normalized probabilities
    float probs[MATERIAL_COUNT];
    float sum = 0.0f;
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        probs[i] = expf(likelihoods[i] - max_ll);
        sum += probs[i];
    }
    if (sum < 1e-12f) sum = 1e-12f;
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        probs[i] /= sum;
    }

    // Allocate mass: sensor_mass goes to singletons, rest to Theta
    // Higher confidence = more mass to singletons
    float sensor_mass = fminf(fmaxf(confidence, 0.0f), 1.0f - DS_MASS_EPSILON);
    float ignorance = 1.0f - sensor_mass;

    // Assign mass to each singleton
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        float m = probs[i] * sensor_mass;
        if (m > DS_MASS_EPSILON) {
            out->elements[out->count].subset = (uint8_t)(1 << i);
            out->elements[out->count].mass = m;
            out->count++;
        }
    }

    // Assign remaining mass to Theta (ignorance)
    if (ignorance > DS_MASS_EPSILON) {
        out->elements[out->count].subset = DS_THETA;
        out->elements[out->count].mass = ignorance;
        out->count++;
    }
}

// Count set bits in a byte
static uint8_t _popcount(uint8_t x) {
    uint8_t count = 0;
    while (x) {
        count += x & 1;
        x >>= 1;
    }
    return count;
}

// ─── Public API Implementation ───────────────────────────────

void dsInit(void) {
    _logClear();
}


DSMassFunction dsFromImpedance(float magnitude, float phase_deg, float confidence) {
    DSMassFunction mf;
    memset(&mf, 0, sizeof(mf));

    if (confidence < 0.05f) {
        // No useful data — assign all mass to Theta
        mf.elements[0].subset = DS_THETA;
        mf.elements[0].mass = 1.0f;
        mf.count = 1;
        _logAdd("IMP: no signal (conf=%.2f), full ignorance", confidence);
        return mf;
    }

    float log_mag = log10f(fmaxf(magnitude, 0.01f));

    float ll[MATERIAL_COUNT];
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        float ll_mag = _gaussianLogLikelihood(log_mag,
                            _models[i].imp_log_mag_mean,
                            _models[i].imp_log_mag_std);
        float ll_phase = _gaussianLogLikelihood(phase_deg,
                            _models[i].imp_phase_mean,
                            _models[i].imp_phase_std);
        ll[i] = ll_mag + ll_phase;  // assume independence
    }

    _likelihoodsToMass(ll, confidence * 0.85f, &mf);

    // Find best for logging
    float best_prob = 0.0f;
    uint8_t best_idx = 0;
    for (int i = 0; i < mf.count; i++) {
        if (_popcount(mf.elements[i].subset) == 1 && mf.elements[i].mass > best_prob) {
            best_prob = mf.elements[i].mass;
            for (int b = 0; b < MATERIAL_COUNT; b++) {
                if (mf.elements[i].subset == (1 << b)) { best_idx = b; break; }
            }
        }
    }
    _logAdd("IMP: |Z|=%.0f phase=%.0f -> %s (%.0f%%)",
            magnitude, phase_deg, MATERIAL_DB[best_idx].name,
            best_prob * 100.0f);

    return mf;
}


DSMassFunction dsFromAcoustic(float dominant_freq, float decay_ratio, float confidence) {
    DSMassFunction mf;
    memset(&mf, 0, sizeof(mf));

    if (confidence < 0.05f) {
        mf.elements[0].subset = DS_THETA;
        mf.elements[0].mass = 1.0f;
        mf.count = 1;
        _logAdd("ACO: no signal (conf=%.2f), full ignorance", confidence);
        return mf;
    }

    float ll[MATERIAL_COUNT];
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        float ll_freq = _gaussianLogLikelihood(dominant_freq,
                            _models[i].aco_freq_mean,
                            _models[i].aco_freq_std);
        float ll_decay = _gaussianLogLikelihood(decay_ratio,
                            _models[i].aco_decay_mean,
                            _models[i].aco_decay_std);
        ll[i] = ll_freq + ll_decay;
    }

    _likelihoodsToMass(ll, confidence * 0.70f, &mf);

    float best_prob = 0.0f;
    uint8_t best_idx = 0;
    for (int i = 0; i < mf.count; i++) {
        if (_popcount(mf.elements[i].subset) == 1 && mf.elements[i].mass > best_prob) {
            best_prob = mf.elements[i].mass;
            for (int b = 0; b < MATERIAL_COUNT; b++) {
                if (mf.elements[i].subset == (1 << b)) { best_idx = b; break; }
            }
        }
    }
    _logAdd("ACO: freq=%.0fHz decay=%.2f -> %s (%.0f%%)",
            dominant_freq, decay_ratio, MATERIAL_DB[best_idx].name,
            best_prob * 100.0f);

    return mf;
}


DSMassFunction dsFromCurvature(uint8_t geometry, float flatness, bool sharp_edge) {
    DSMassFunction mf;
    memset(&mf, 0, sizeof(mf));

    // Curvature provides weak evidence: it can rule out materials
    // but rarely identifies one uniquely.

    // Build a composite subset of compatible materials for this geometry
    uint8_t compatible = DS_EMPTY;
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        if (_models[i].compatible_geometries & (1 << geometry)) {
            compatible |= (uint8_t)(1 << i);
        }
    }

    if (compatible == DS_EMPTY) {
        compatible = DS_THETA; // fallback: everything compatible
    }

    // Confidence based on how discriminative this geometry is
    uint8_t num_compatible = _popcount(compatible);
    float discrimination = 1.0f - ((float)num_compatible / (float)MATERIAL_COUNT);
    // flatness gives stronger evidence (flat surfaces are more reliably classified)
    float curv_confidence = discrimination * 0.5f + flatness * 0.15f;
    if (sharp_edge) curv_confidence += 0.1f;
    curv_confidence = fminf(curv_confidence, 0.6f);

    if (num_compatible < MATERIAL_COUNT) {
        // Assign mass to the compatible subset
        mf.elements[0].subset = compatible;
        mf.elements[0].mass = curv_confidence;
        mf.count = 1;
        // Rest to Theta
        mf.elements[1].subset = DS_THETA;
        mf.elements[1].mass = 1.0f - curv_confidence;
        mf.count = 2;
    } else {
        // All materials compatible — curvature tells us nothing
        mf.elements[0].subset = DS_THETA;
        mf.elements[0].mass = 1.0f;
        mf.count = 1;
    }

    const char* geo_names[] = {"flat", "convex", "concave", "edge", "cylinder"};
    const char* gname = (geometry < 5) ? geo_names[geometry] : "?";
    _logAdd("CURV: %s flat=%.0f%% -> %d/%d materials ruled out",
            gname, flatness * 100.0f,
            MATERIAL_COUNT - num_compatible, MATERIAL_COUNT);

    return mf;
}


DSMassFunction dsCombine(const DSMassFunction* m1, const DSMassFunction* m2) {
    DSMassFunction result;
    memset(&result, 0, sizeof(result));

    // Temporary storage for combination (up to DS_MAX_FOCAL^2 terms,
    // but we merge by subset so the output stays bounded)
    float combined_mass[64]; // bitmask 0..63 = 2^6 subsets
    memset(combined_mass, 0, sizeof(combined_mass));

    float conflict = 0.0f;

    // Dempster's rule: for each pair (A, B), add m1(A)*m2(B) to A∩B
    for (int i = 0; i < m1->count; i++) {
        for (int j = 0; j < m2->count; j++) {
            uint8_t intersection = m1->elements[i].subset & m2->elements[j].subset;
            float product = m1->elements[i].mass * m2->elements[j].mass;

            if (intersection == DS_EMPTY) {
                conflict += product;
            } else {
                combined_mass[intersection] += product;
            }
        }
    }

    // Store the raw conflict K before normalization
    result.conflict = conflict;

    // Normalize by (1 - K) where K is the conflict
    float norm = 1.0f - conflict;
    if (norm < 0.001f) {
        // Total conflict — sources completely disagree
        // Return vacuous mass (all to Theta)
        result.elements[0].subset = DS_THETA;
        result.elements[0].mass = 1.0f;
        result.count = 1;
        result.conflict = conflict;
        return result;
    }

    float inv_norm = 1.0f / norm;

    // Collect non-zero focal elements
    for (uint8_t s = 1; s <= DS_THETA; s++) {
        float m = combined_mass[s] * inv_norm;
        if (m > DS_MASS_EPSILON && result.count < DS_MAX_FOCAL) {
            result.elements[result.count].subset = s;
            result.elements[result.count].mass = m;
            result.count++;
        }
    }

    return result;
}


DSMassFunction dsCombineAll(const DSMassFunction* masses, uint8_t count) {
    if (count == 0) {
        DSMassFunction vacuous;
        vacuous.elements[0].subset = DS_THETA;
        vacuous.elements[0].mass = 1.0f;
        vacuous.count = 1;
        return vacuous;
    }

    DSMassFunction accumulated = masses[0];
    for (int i = 1; i < count; i++) {
        accumulated = dsCombine(&accumulated, &masses[i]);
    }
    return accumulated;
}


DSResult dsGetResult(const DSMassFunction* combined) {
    DSResult result;
    memset(&result, 0, sizeof(result));
    result.valid = false;

    // Compute belief and plausibility for each singleton {material_i}
    for (int mat = 0; mat < MATERIAL_COUNT; mat++) {
        uint8_t singleton = (uint8_t)(1 << mat);
        float bel = 0.0f;
        float pl = 0.0f;

        for (int i = 0; i < combined->count; i++) {
            uint8_t A = combined->elements[i].subset;
            float m = combined->elements[i].mass;

            // Belief: sum of masses of all subsets contained in {mat}
            // For a singleton, only m({mat}) itself qualifies
            if (A == singleton) {
                bel += m;
            }
            // Also empty set subsets, but we don't store empty

            // Plausibility: sum of masses of all subsets that intersect {mat}
            if (A & singleton) {
                pl += m;
            }
        }

        result.belief[mat] = bel;
        result.plausibility[mat] = pl;
        result.mass_singleton[mat] = bel; // for singletons, Bel = direct mass
    }

    // Residual ignorance (mass on Theta or multi-element sets)
    result.mass_theta = 0.0f;
    for (int i = 0; i < combined->count; i++) {
        if (_popcount(combined->elements[i].subset) > 1) {
            result.mass_theta += combined->elements[i].mass;
        }
    }

    // Use the actual conflict K that was computed during Dempster's combination
    result.conflict = combined->conflict;

    // Find best material by belief
    float best_bel = 0.0f;
    uint8_t best_idx = MAT_UNKNOWN;
    for (int i = 0; i < MATERIAL_COUNT; i++) {
        if (result.belief[i] > best_bel) {
            best_bel = result.belief[i];
            best_idx = (uint8_t)i;
        }
    }

    result.best_material = best_idx;
    result.best_belief = best_bel;
    result.best_plausibility = (best_idx < MATERIAL_COUNT) ?
                                result.plausibility[best_idx] : 0.0f;

    _logAdd("FUSED: %s Bel=%.0f%% Pl=%.0f%% ign=%.0f%% K=%.0f%%",
            (best_idx < MATERIAL_COUNT) ? MATERIAL_DB[best_idx].name : "Unknown",
            best_bel * 100.0f,
            result.best_plausibility * 100.0f,
            result.mass_theta * 100.0f,
            result.conflict * 100.0f);

    result.valid = (best_idx < MATERIAL_COUNT && best_bel > 0.1f);
    return result;
}


const DSDecisionLog* dsGetLog(void) {
    return &_log;
}


const char* dsGetMaterialName(uint8_t idx) {
    if (idx < MATERIAL_COUNT) {
        return MATERIAL_DB[idx].name;
    }
    return "Unknown";
}
