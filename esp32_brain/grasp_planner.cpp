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


GraspState getCurrentState() {
    return _state;
}


const char* getStateName(GraspState s) {
    if (s > STATE_ERROR) return "???";
    return STATE_NAMES[s];
}


uint32_t getStateElapsed() {
    return millis() - _state_enter_ms;
}


void requestStateTransition(GraspState target) {
    _requested_state = target;
    _state_request_pending = true;
}


void triggerEstop() {
    _estop = true;
}


GraspPlan computeGraspPlan(const ImpedanceResult& imp, const AcousticResult& aco,
                           const CurvatureResult& curv, const DepthGrid& depth,
                           uint8_t fused_material, float fused_confidence) {
    GraspPlan plan;
    plan.target_force_N = 3.0f;    // safe default
    plan.approach_speed = 0.5f;
    plan.finger_aperture = 80.0f;  // mm, wide open default
    plan.force_ramp_Nps = GRASP_RAMP_DEFAULT_NPS;
    plan.slip_threshold = FSR_SLIP_RMS_THRESHOLD;
    plan.quality_score = 0.0f;
    plan.valid = false;

    // ── Material-driven force target ──
    if (fused_material < MATERIAL_COUNT) {
        const MaterialEntry& mat = MATERIAL_DB[fused_material];
        plan.target_force_N = mat.max_grip_force_N * GRASP_FORCE_SAFETY_MARGIN;
        plan.slip_threshold = FSR_SLIP_RMS_THRESHOLD * (1.0f + mat.slip_risk);

        // Delicate materials get slower ramp
        if (mat.max_grip_force_N < 3.0f) {
            plan.force_ramp_Nps = 1.0f;
        }
        // Slippery materials get higher force
        if (mat.slip_risk > 0.5f) {
            plan.target_force_N *= 1.1f;
        }
    }

    // ── Geometry-driven approach speed ──
    if (curv.valid) {
        switch (curv.geometry) {
            case 0: // flat — straightforward
                plan.approach_speed = 0.7f;
                break;
            case 1: // convex — careful centering
                plan.approach_speed = 0.5f;
                break;
            case 2: // concave — slow
                plan.approach_speed = 0.4f;
                break;
            case 3: // edge — very careful
                plan.approach_speed = 0.3f;
                // Reduce force for edges to avoid crushing
                plan.target_force_N *= 0.7f;
                break;
            case 4: // cylinder — moderate
                plan.approach_speed = 0.5f;
                break;
        }

        if (curv.sharp_edge) {
            plan.approach_speed *= 0.6f;
            plan.force_ramp_Nps *= 0.5f;
        }
    }

    // ── Depth-driven aperture ──
    if (depth.valid) {
        // Estimate object width from depth grid: find the span of close pixels
        int min_col = DEPTH_GRID_COLS;
        int max_col = -1;
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
            // Convert column span to approximate mm (assuming 45° FoV over 8 zones at closest range)
            float fov_at_distance = (float)depth.min_mm * 0.83f; // tan(45°/2)*2 * distance
            float pixel_width = fov_at_distance / (float)DEPTH_GRID_COLS;
            float object_width = (float)(max_col - min_col + 1) * pixel_width;
            // Set aperture to object width + 20mm margin
            plan.finger_aperture = fminf(fmaxf(object_width + 20.0f, 30.0f), 120.0f);
        }
    }

    // ── Confidence-driven conservatism ──
    if (fused_confidence < 0.5f) {
        // Low confidence: use conservative parameters
        plan.target_force_N = fminf(plan.target_force_N, 3.0f);
        plan.approach_speed *= 0.6f;
        plan.force_ramp_Nps *= 0.5f;
    }

    // ── Quality prediction ──
    float q = 0.0f;
    // Material identification quality (40% weight)
    q += 0.4f * fused_confidence;
    // Geometry quality (20% weight): flat is best
    if (curv.valid) {
        q += 0.2f * curv.flatness;
    }
    // Depth data quality (20% weight)
    if (depth.valid) {
        q += 0.2f;
    }
    // Impedance signal quality (20% weight)
    if (imp.valid) {
        q += 0.2f * imp.confidence;
    }
    plan.quality_score = fminf(q, 1.0f);
    plan.valid = (plan.quality_score >= GRASP_QUALITY_MIN);

    return plan;
}


float computeGraspQuality(const ForceResult& force, const ImpedanceResult& imp,
                          const GraspPlan& plan) {
    if (!force.valid || !plan.valid) return 0.0f;

    float q = 0.0f;

    // Force adequacy (35%): how close is actual force to target?
    float force_ratio = force.total_N / fmaxf(plan.target_force_N, 0.1f);
    float force_score;
    if (force_ratio < 0.5f) {
        force_score = force_ratio * 2.0f; // ramp up
    } else if (force_ratio <= 1.2f) {
        force_score = 1.0f; // sweet spot
    } else {
        force_score = fmaxf(0.0f, 1.0f - (force_ratio - 1.2f)); // over-gripping penalty
    }
    q += 0.35f * force_score;

    // Balance (25%): even force distribution
    q += 0.25f * force.balance;

    // Contact coverage (20%): all FSRs should be engaged
    int engaged = 0;
    for (int i = 0; i < FSR_COUNT; i++) {
        if (force.forces_N[i] > 0.1f) engaged++;
    }
    q += 0.20f * ((float)engaged / (float)FSR_COUNT);

    // Impedance stability (20%): consistent readings indicate stable contact
    if (imp.valid) {
        q += 0.20f * imp.confidence;
    }

    // Penalty for active slip
    if (force.slip_detected) {
        q *= 0.5f;
    }

    return fminf(fmaxf(q, 0.0f), 1.0f);
}


GraspState runStateMachine(GraspState current,
                           const ForceResult& force,
                           const ImpedanceResult& imp,
                           const CurvatureResult& curv,
                           const DepthGrid& depth,
                           const GraspPlan& plan,
                           float grasp_quality) {
    // E-stop overrides everything
    if (_estop) {
        _estop = false;
        _enterState(STATE_RELEASING);
        return _state;
    }

    // Handle external state transition requests (from dashboard)
    if (_state_request_pending) {
        _state_request_pending = false;
        GraspState req = _requested_state;
        // Validate: only allow certain transitions
        if (req == STATE_IDLE || req == STATE_RELEASING) {
            _enterState(req);
            return _state;
        }
        if (req == STATE_DETECTED && _state == STATE_IDLE) {
            _enterState(req);
            return _state;
        }
    }

    uint32_t elapsed = millis() - _state_enter_ms;

    switch (_state) {
        case STATE_IDLE:
            // Transition to DETECTED when depth sensor sees something close
            if (depth.valid && depth.min_mm < 300) {
                _enterState(STATE_DETECTED);
            }
            break;

        case STATE_DETECTED:
            // Object detected — begin scanning
            if (depth.valid && depth.min_mm < 200) {
                _enterState(STATE_SCANNING);
            } else if (depth.valid && depth.min_mm > 500) {
                // Object moved away
                _enterState(STATE_IDLE);
            }
            if (elapsed > 5000) _enterState(STATE_IDLE); // timeout
            break;

        case STATE_SCANNING:
            // Curvature + depth data collected
            if (curv.valid && depth.valid) {
                _enterState(STATE_ANALYZING);
            }
            if (elapsed > STATE_TIMEOUT_SCANNING_MS) {
                // Proceed with whatever data we have
                _enterState(STATE_ANALYZING);
            }
            break;

        case STATE_ANALYZING:
            // Impedance measurement running
            if (imp.valid) {
                _enterState(STATE_TAP_TESTING);
            }
            if (elapsed > STATE_TIMEOUT_ANALYZING_MS) {
                _enterState(STATE_TAP_TESTING);
            }
            break;

        case STATE_TAP_TESTING:
            // Acoustic analysis requested from main loop — transition after timeout
            // The main loop will trigger acoustic measurement in this state
            if (elapsed > STATE_TIMEOUT_TAP_MS) {
                _enterState(STATE_CLASSIFYING);
            }
            break;

        case STATE_CLASSIFYING:
            // Material classification + cross-validation done in main loop
            _enterState(STATE_PLANNING);
            break;

        case STATE_PLANNING:
            if (plan.valid) {
                _enterState(STATE_APPROACHING);
            }
            if (elapsed > STATE_TIMEOUT_PLANNING_MS) {
                if (plan.quality_score > 0.2f) {
                    _enterState(STATE_APPROACHING);
                } else {
                    Serial.println("[GRASP] Plan quality too low, aborting");
                    _enterState(STATE_ERROR);
                }
            }
            break;

        case STATE_APPROACHING:
            // Check if fingers have contacted the object (force sensed)
            if (force.valid && force.total_N > 0.2f) {
                _enterState(STATE_GRIPPING);
            }
            if (elapsed > STATE_TIMEOUT_APPROACHING_MS) {
                Serial.println("[GRASP] Approach timeout");
                _enterState(STATE_ERROR);
            }
            break;

        case STATE_GRIPPING:
            // Force ramping up — check if target reached
            if (force.valid && force.total_N >= plan.target_force_N * 0.9f) {
                _enterState(STATE_HOLDING);
            }
            if (force.valid && force.slip_detected) {
                // Increase grip during ramp (handled by main loop sending force cmd)
            }
            if (elapsed > STATE_TIMEOUT_GRIPPING_MS) {
                // Accept whatever force level we reached
                if (force.valid && force.total_N > 0.5f) {
                    _enterState(STATE_HOLDING);
                } else {
                    _enterState(STATE_ERROR);
                }
            }
            break;

        case STATE_HOLDING:
            // Stable hold — monitor for slip or loss of contact
            if (force.valid) {
                if (force.total_N < 0.1f) {
                    // Object dropped
                    Serial.println("[GRASP] Contact lost during hold");
                    _enterState(STATE_ERROR);
                }
                if (force.slip_detected) {
                    // Auto-tighten is handled by main loop; if quality drops too low, error out
                    if (grasp_quality < 0.15f) {
                        _enterState(STATE_ERROR);
                    }
                }
            }
            break;

        case STATE_RELEASING:
            // Open fingers, wait for force to drop
            if (force.valid && force.total_N < 0.05f) {
                _enterState(STATE_IDLE);
            }
            if (elapsed > 3000) {
                // Force release complete
                _enterState(STATE_IDLE);
            }
            break;

        case STATE_ERROR:
            // Latch in error for 2 seconds, then return to idle
            if (elapsed > 2000) {
                _enterState(STATE_IDLE);
            }
            break;
    }

    return _state;
}
