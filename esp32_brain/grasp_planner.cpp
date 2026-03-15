#include "grasp_planner.h"
#include <math.h>

static GraspState _state = STATE_IDLE;
static uint32_t   _state_enter_ms = 0;
static bool       _estop = false;
static GraspState _requested_state = STATE_IDLE;
static bool       _state_request_pending = false;

void initGraspPlanner() {
    _state = STATE_IDLE;
    _state_enter_ms = millis();
    _estop = false;
    _state_request_pending = false;
}

static void _enterState(GraspState s) {
    _state = s;
    _state_enter_ms = millis();
    Serial.printf("[GRASP] -> %s\n", STATE_NAMES[s]);
}

GraspState getCurrentState() { return _state; }
const char* getStateName(GraspState s) { return (s <= STATE_ERROR) ? STATE_NAMES[s] : "???"; }
uint32_t getStateElapsed() { return millis() - _state_enter_ms; }
const char* getStrategyName(GraspStrategy s) { return (s < STRATEGY_COUNT) ? STRATEGY_NAMES[s] : "???"; }

void requestStateTransition(GraspState target) {
    _requested_state = target;
    _state_request_pending = true;
}

void triggerEstop() { _estop = true; }

GraspStrategy selectStrategy(uint8_t material, uint8_t geometry, float flatness) {
    if (material >= MATERIAL_COUNT) return STRATEGY_POWER;

    float fragility = 1.0f - (MATERIAL_DB[material].max_grip_force_N / 10.0f);
    fragility = fminf(fmaxf(fragility, 0.0f), 1.0f);
    float slip_risk = MATERIAL_DB[material].slip_risk;

    if (geometry == 3) return STRATEGY_EDGE;

    if (fragility > 0.6f) return STRATEGY_PRECISION;

    if (geometry == 1 || geometry == 2 || geometry == 4) {
        return (slip_risk > 0.5f) ? STRATEGY_WRAP : STRATEGY_POWER;
    }

    if (flatness > 0.8f && fragility < 0.3f) return STRATEGY_POWER;

    return STRATEGY_PRECISION;
}

typedef struct {
    float force_mult;
    float speed_mult;
    float ramp_mult;
    float aperture_add;
} StrategyProfile;

static const StrategyProfile _profiles[STRATEGY_COUNT] = {
    { 1.0f, 0.8f, 1.2f, 10.0f },   // POWER
    { 0.5f, 0.4f, 0.5f, 5.0f  },   // PRECISION
    { 0.7f, 0.5f, 0.8f, 15.0f },   // WRAP
    { 0.4f, 0.3f, 0.4f, 8.0f  },   // EDGE
};

GraspPlan computeGraspPlan(const DSResult& ds,
                           const CurvatureResult& curv,
                           const DepthGrid& depth) {
    GraspPlan plan;
    plan.target_force_N = 3.0f;
    plan.approach_speed = 0.5f;
    plan.finger_aperture = 80.0f;
    plan.force_ramp_Nps = GRASP_RAMP_DEFAULT_NPS;
    plan.slip_threshold = FSR_SLIP_RMS_THRESHOLD;
    plan.quality_score = 0.0f;
    plan.strategy = STRATEGY_POWER;
    plan.success_prob = 0.0f;
    plan.valid = false;

    uint8_t mat = ds.valid ? ds.best_material : MAT_UNKNOWN;
    uint8_t geo = curv.valid ? curv.geometry : 0;
    float flat = curv.valid ? curv.flatness : 0.5f;

    plan.strategy = selectStrategy(mat, geo, flat);
    const StrategyProfile& prof = _profiles[plan.strategy];

    if (mat < MATERIAL_COUNT) {
        const MaterialEntry& m = MATERIAL_DB[mat];
        plan.target_force_N = m.max_grip_force_N * GRASP_FORCE_SAFETY_MARGIN * prof.force_mult;
        plan.slip_threshold = FSR_SLIP_RMS_THRESHOLD * (1.0f + m.slip_risk);
        if (m.max_grip_force_N < 3.0f) plan.force_ramp_Nps = 1.0f;
        if (m.slip_risk > 0.5f) plan.target_force_N *= 1.1f;
    }

    plan.approach_speed = 0.7f * prof.speed_mult;
    plan.force_ramp_Nps *= prof.ramp_mult;

    if (curv.valid && curv.sharp_edge) {
        plan.approach_speed *= 0.6f;
        plan.force_ramp_Nps *= 0.5f;
    }

    if (depth.valid) {
        int min_col = DEPTH_GRID_COLS, max_col = -1;
        uint16_t close_thresh = depth.min_mm + (depth.max_mm - depth.min_mm) / 3;
        for (int r = 0; r < DEPTH_GRID_ROWS; r++) {
            for (int c = 0; c < DEPTH_GRID_COLS; c++) {
                if (depth.mm[r][c] <= close_thresh && depth.mm[r][c] > 0) {
                    if (c < min_col) min_col = c;
                    if (c > max_col) max_col = c;
                }
            }
        }
        if (max_col >= min_col) {
            float fov_at_d = (float)depth.min_mm * 0.83f;
            float pw = fov_at_d / (float)DEPTH_GRID_COLS;
            float ow = (float)(max_col - min_col + 1) * pw;
            plan.finger_aperture = fminf(fmaxf(ow + prof.aperture_add, 30.0f), 120.0f);
        }
    }

    if (ds.valid && ds.best_belief < 0.5f) {
        plan.target_force_N = fminf(plan.target_force_N, 3.0f);
        plan.approach_speed *= 0.6f;
        plan.force_ramp_Nps *= 0.5f;
    }

    float q = 0.0f;
    if (ds.valid) q += 0.4f * ds.best_belief;
    if (curv.valid) q += 0.2f * curv.flatness;
    if (depth.valid) q += 0.2f;
    q += 0.2f * (1.0f - ds.mass_theta);
    plan.quality_score = fminf(q, 1.0f);

    float obj_size = plan.finger_aperture / 120.0f;
    float geo_compat = 1.0f;
    if (plan.strategy == STRATEGY_EDGE && geo != 3) geo_compat = 0.5f;
    if (plan.strategy == STRATEGY_WRAP && geo == 0) geo_compat = 0.6f;
    float depth_qual = depth.valid ? 1.0f : 0.2f;

    SuccessPrediction sp = spPredict(
        ds.valid ? ds.best_belief : 0.2f,
        flat, depth_qual, plan.quality_score, obj_size, geo_compat);
    plan.success_prob = sp.probability;

    plan.valid = (plan.quality_score >= GRASP_QUALITY_MIN);
    return plan;
}

float computeGraspQuality(const ForceResult& force, const ImpedanceResult& imp,
                          const GraspPlan& plan) {
    if (!force.valid || !plan.valid) return 0.0f;

    float q = 0.0f;
    float force_ratio = force.total_N / fmaxf(plan.target_force_N, 0.1f);
    float fs;
    if (force_ratio < 0.5f) fs = force_ratio * 2.0f;
    else if (force_ratio <= 1.2f) fs = 1.0f;
    else fs = fmaxf(0.0f, 1.0f - (force_ratio - 1.2f));
    q += 0.35f * fs;
    q += 0.25f * force.balance;

    int engaged = 0;
    for (int i = 0; i < FSR_COUNT; i++)
        if (force.forces_N[i] > 0.1f) engaged++;
    q += 0.20f * ((float)engaged / (float)FSR_COUNT);

    if (imp.valid) q += 0.20f * imp.confidence;
    if (force.slip_detected) q *= 0.5f;

    return fminf(fmaxf(q, 0.0f), 1.0f);
}

GraspState runStateMachine(GraspState current,
                           const ForceResult& force,
                           const ImpedanceResult& imp,
                           const CurvatureResult& curv,
                           const DepthGrid& depth,
                           const GraspPlan& plan,
                           float grasp_quality) {
    if (_estop) {
        _estop = false;
        _enterState(STATE_RELEASING);
        return _state;
    }

    if (_state_request_pending) {
        _state_request_pending = false;
        GraspState req = _requested_state;
        if (req == STATE_IDLE || req == STATE_RELEASING) { _enterState(req); return _state; }
        if (req == STATE_DETECTED && _state == STATE_IDLE) { _enterState(req); return _state; }
    }

    uint32_t elapsed = millis() - _state_enter_ms;

    switch (_state) {
        case STATE_IDLE:
            if (depth.valid && depth.min_mm < 300) _enterState(STATE_DETECTED);
            break;

        case STATE_DETECTED:
            if (depth.valid && depth.min_mm < 200) _enterState(STATE_SCANNING);
            else if (depth.valid && depth.min_mm > 500) _enterState(STATE_IDLE);
            if (elapsed > 5000) _enterState(STATE_IDLE);
            break;

        case STATE_SCANNING:
            if (curv.valid && depth.valid) _enterState(STATE_ANALYZING);
            if (elapsed > STATE_TIMEOUT_SCANNING_MS) _enterState(STATE_ANALYZING);
            break;

        case STATE_ANALYZING:
            if (imp.valid) _enterState(STATE_TAP_TESTING);
            if (elapsed > STATE_TIMEOUT_ANALYZING_MS) _enterState(STATE_TAP_TESTING);
            break;

        case STATE_TAP_TESTING:
            if (elapsed > STATE_TIMEOUT_TAP_MS) _enterState(STATE_CLASSIFYING);
            break;

        case STATE_CLASSIFYING:
            _enterState(STATE_PLANNING);
            break;

        case STATE_PLANNING:
            if (plan.valid) {
                if (plan.success_prob >= SP_PROCEED_THRESHOLD) {
                    _enterState(STATE_APPROACHING);
                } else {
                    Serial.printf("[GRASP] Low success prob %.0f%%, retrying\n", plan.success_prob * 100);
                    _enterState(STATE_ERROR);
                }
            }
            if (elapsed > STATE_TIMEOUT_PLANNING_MS) {
                if (plan.quality_score > 0.2f) _enterState(STATE_APPROACHING);
                else _enterState(STATE_ERROR);
            }
            break;

        case STATE_APPROACHING:
            if (force.valid && force.total_N > 0.2f) _enterState(STATE_GRIPPING);
            if (elapsed > STATE_TIMEOUT_APPROACHING_MS) _enterState(STATE_ERROR);
            break;

        case STATE_GRIPPING:
            if (force.valid && force.total_N >= plan.target_force_N * 0.9f) _enterState(STATE_HOLDING);
            if (elapsed > STATE_TIMEOUT_GRIPPING_MS) {
                if (force.valid && force.total_N > 0.5f) _enterState(STATE_HOLDING);
                else _enterState(STATE_ERROR);
            }
            break;

        case STATE_HOLDING:
            if (force.valid) {
                if (force.total_N < 0.1f) _enterState(STATE_ERROR);
                if (force.slip_detected && grasp_quality < 0.15f) _enterState(STATE_ERROR);
            }
            break;

        case STATE_RELEASING:
            if (force.valid && force.total_N < 0.05f) _enterState(STATE_IDLE);
            if (elapsed > 3000) _enterState(STATE_IDLE);
            break;

        case STATE_ERROR:
            if (elapsed > 2000) _enterState(STATE_IDLE);
            break;
    }

    return _state;
}
