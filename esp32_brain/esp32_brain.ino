#include "config.h"
#include "impedance.h"
#include "acoustic.h"
#include "sensors.h"
#include "grasp_planner.h"
#include "comms.h"
#include "web_server.h"
#include "ds_fusion.h"
#include "hd_slip.h"
#include "success_predictor.h"

static ImpedanceResult g_impedance;
static AcousticResult  g_acoustic;
static ForceResult     g_force;
static CurvatureResult g_curvature;
static DepthGrid       g_depth;
static GraspPlan       g_plan;
static GraspState      g_state = STATE_IDLE;
static float           g_grasp_quality = 0.0f;
static DSResult        g_ds_result;
static float           g_prev_total_force = 0.0f;

static uint32_t g_last_sensor_ms = 0;
static uint32_t g_last_impedance_ms = 0;
static uint32_t g_last_state_ms = 0;
static uint32_t g_last_web_ms = 0;
static uint32_t g_last_depth_request_ms = 0;

static bool g_tap_requested = false;
static bool g_tap_mic_done = false;
static bool g_tap_imu_done = false;
static float g_current_force_cmd = 0.0f;

static void _handleWebCommand(const char* cmd) {
    if (strcmp(cmd, "start") == 0) {
        if (g_state == STATE_IDLE) requestStateTransition(STATE_DETECTED);
    } else if (strcmp(cmd, "release") == 0) {
        requestStateTransition(STATE_RELEASING);
        sendGripOpen();
        g_current_force_cmd = 0.0f;
    } else if (strcmp(cmd, "estop") == 0) {
        triggerEstop();
        sendEstop();
        g_current_force_cmd = 0.0f;
    } else if (strcmp(cmd, "calibrate") == 0) {
        calibrateBaseline();
        Serial.println("[MAIN] Impedance recalibrated");
    }
}

static void _updateSensors() {
    g_force = readForces();

    // HD slip auto-training: bootstrap from RMS-based detection
    if (g_force.valid && g_force.total_N > 0.5f) {
        float dfdt = (g_force.total_N - g_prev_total_force) / ((float)LOOP_SENSOR_PERIOD_MS / 1000.0f);
        float per_fsr_rms = g_force.slip_rms;  // max RMS across FSRs
        float hd_features[HD_NUM_FEATURES] = {
            per_fsr_rms * g_force.forces_N[0] / fmaxf(g_force.total_N, 0.1f),
            per_fsr_rms * g_force.forces_N[1] / fmaxf(g_force.total_N, 0.1f),
            per_fsr_rms * g_force.forces_N[2] / fmaxf(g_force.total_N, 0.1f),
            g_force.forces_N[0], g_force.forces_N[1], g_force.forces_N[2],
            dfdt
        };
        hdAutoTrain(hd_features, g_force.slip_detected);

        if (hdIsReady()) {
            HDSlipResult hd = hdClassify(hd_features);
            if (hd.valid) {
                g_force.slip_detected = (hd.predicted_class == HD_CLASS_SLIP);
            }
        }
    }
    g_prev_total_force = g_force.valid ? g_force.total_N : 0.0f;

    // Curvature: stub since TCRT5000 removed. Use depth-based geometry instead.
    if (g_depth.valid) {
        g_curvature.valid = true;
        g_curvature.flatness = 0.7f;
        g_curvature.geometry = 0;
        g_curvature.sharp_edge = false;
        g_curvature.curvature_x = 0.0f;
        g_curvature.curvature_y = 0.0f;

        // Estimate geometry from depth grid variance
        float mean_d = 0; int count = 0;
        for (int r = 0; r < DEPTH_GRID_ROWS; r++)
            for (int c = 0; c < DEPTH_GRID_COLS; c++)
                if (g_depth.mm[r][c] > 0 && g_depth.mm[r][c] < DEPTH_MAX_MM)
                    { mean_d += g_depth.mm[r][c]; count++; }
        if (count > 0) {
            mean_d /= count;
            float var = 0;
            for (int r = 0; r < DEPTH_GRID_ROWS; r++)
                for (int c = 0; c < DEPTH_GRID_COLS; c++)
                    if (g_depth.mm[r][c] > 0 && g_depth.mm[r][c] < DEPTH_MAX_MM) {
                        float d = g_depth.mm[r][c] - mean_d;
                        var += d * d;
                    }
            var /= fmaxf(count, 1);
            float std = sqrtf(var);
            g_curvature.flatness = fmaxf(0.0f, 1.0f - std / 100.0f);
            if (std > 80) g_curvature.geometry = 1; // convex
            if (std > 150) { g_curvature.geometry = 3; g_curvature.sharp_edge = true; }
        }
    }

    processUARTData();
    if (hasNewDepthGrid()) g_depth = getLatestDepthGrid();

    uint32_t now = millis();
    if (now - g_last_depth_request_ms >= 500) {
        sendRequestDepth();
        g_last_depth_request_ms = now;
    }
}

static void _updateImpedance() {
    g_impedance = measureImpedance();
}

static void _runDSFusion() {
    DSMassFunction masses[3];
    uint8_t n = 0;

    if (g_impedance.valid)
        masses[n++] = dsFromImpedance(g_impedance.magnitude, g_impedance.phase_deg, g_impedance.confidence);

    if (g_acoustic.valid)
        masses[n++] = dsFromAcoustic(g_acoustic.dominant_freq, g_acoustic.decay_ratio, g_acoustic.confidence);

    if (g_curvature.valid)
        masses[n++] = dsFromCurvature(g_curvature.geometry, g_curvature.flatness, g_curvature.sharp_edge);

    if (n > 0) {
        DSMassFunction combined = dsCombineAll(masses, n);
        g_ds_result = dsGetResult(&combined);
    } else {
        memset(&g_ds_result, 0, sizeof(g_ds_result));
    }
}

static void _updateStateMachine() {
    GraspState prev_state = g_state;

    g_state = runStateMachine(g_state, g_force, g_impedance, g_curvature,
                              g_depth, g_plan, g_grasp_quality);

    if (g_state != prev_state) {
        switch (g_state) {
            case STATE_IDLE:
                g_tap_requested = false;
                g_tap_mic_done = false;
                g_tap_imu_done = false;
                g_current_force_cmd = 0.0f;
                memset(&g_ds_result, 0, sizeof(g_ds_result));
                memset(&g_acoustic, 0, sizeof(g_acoustic));
                break;

            case STATE_SCANNING:
                sendRequestDepth();
                break;

            case STATE_TAP_TESTING:
                sendTapCommand();
                sendRequestMicTap();
                sendRequestIMUTap();
                g_tap_requested = true;
                g_tap_mic_done = false;
                g_tap_imu_done = false;
                break;

            case STATE_CLASSIFYING:
                _runDSFusion();
                break;

            case STATE_PLANNING:
                g_plan = computeGraspPlan(g_ds_result, g_curvature, g_depth);
                break;

            case STATE_APPROACHING:
                sendGripClose();
                if (g_plan.valid) {
                    sendSetForce(g_plan.target_force_N * 0.3f);
                    g_current_force_cmd = g_plan.target_force_N * 0.3f;
                }
                break;

            case STATE_RELEASING:
                sendGripOpen();
                g_current_force_cmd = 0.0f;
                break;

            case STATE_ERROR:
                sendEstop();
                g_current_force_cmd = 0.0f;
                break;

            default: break;
        }
    }

    switch (g_state) {
        case STATE_TAP_TESTING:
            if (g_tap_requested) {
                if (hasNewMicTap()) {
                    MicTapResult mt = getLatestMicTap();
                    if (mt.valid) {
                        g_acoustic.dominant_freq = mt.dominant_freq;
                        g_acoustic.spectral_centroid = mt.spectral_centroid;
                        g_acoustic.decay_ratio = mt.decay_ratio;
                        g_acoustic.confidence = 0.7f;
                        g_acoustic.valid = true;
                    }
                    g_tap_mic_done = true;
                }
                if (hasNewIMUTap()) {
                    IMUTapResult it = getLatestIMUTap();
                    if (it.valid && g_acoustic.valid) {
                        // Fuse IMU vibration with mic data (Knocker approach)
                        g_acoustic.confidence = fminf(g_acoustic.confidence + 0.15f, 1.0f);
                        // Weight dominant freq toward IMU if IMU has strong signal
                        if (it.vibration_rms > 0.5f) {
                            g_acoustic.dominant_freq = g_acoustic.dominant_freq * 0.7f + it.dominant_freq * 0.3f;
                        }
                    }
                    g_tap_imu_done = true;
                }
                if (g_tap_mic_done || getStateElapsed() > 2000) {
                    requestStateTransition(STATE_CLASSIFYING);
                }
            }
            break;

        case STATE_GRIPPING:
            if (g_plan.valid && g_force.valid) {
                float dt = (float)LOOP_STATE_PERIOD_MS / 1000.0f;
                float inc = g_plan.force_ramp_Nps * dt;
                if (g_current_force_cmd < g_plan.target_force_N) {
                    g_current_force_cmd += inc;
                    if (g_current_force_cmd > g_plan.target_force_N)
                        g_current_force_cmd = g_plan.target_force_N;
                    sendSetForce(g_current_force_cmd);
                }
            }
            break;

        case STATE_HOLDING:
            g_grasp_quality = computeGraspQuality(g_force, g_impedance, g_plan);
            if (g_force.valid && g_force.slip_detected && g_plan.valid) {
                float boost = g_current_force_cmd * 1.15f;
                float max_a = g_plan.target_force_N * 1.3f;
                if (boost <= max_a) {
                    g_current_force_cmd = boost;
                    sendSetForce(g_current_force_cmd);
                }
            }
            break;

        default: break;
    }
}

static void _pushWebData() {
    cleanupWebClients();
    pushSensorData(g_impedance, g_acoustic, g_force, g_curvature, g_depth,
                   g_plan, g_state, g_grasp_quality, g_ds_result);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}
    Serial.println("\n== HYDRA Grasp v2.0 ==");

    initImpedance();
    initAcoustic();
    initSensors();
    initGraspPlanner();
    initComms();
    dsInit();
    hdInit();
    spInit();

    setupWiFiAP();
    setupWebServer();
    setWebCommandCallback(_handleWebCommand);

    calibrateBaseline();

    memset(&g_impedance, 0, sizeof(g_impedance));
    memset(&g_acoustic, 0, sizeof(g_acoustic));
    memset(&g_force, 0, sizeof(g_force));
    memset(&g_curvature, 0, sizeof(g_curvature));
    memset(&g_depth, 0, sizeof(g_depth));
    memset(&g_plan, 0, sizeof(g_plan));
    memset(&g_ds_result, 0, sizeof(g_ds_result));

    uint32_t now = millis();
    g_last_sensor_ms = now;
    g_last_impedance_ms = now;
    g_last_state_ms = now;
    g_last_web_ms = now;
    g_last_depth_request_ms = now;

    Serial.println("[INIT] Ready. Dashboard: http://192.168.4.1\n");
}

void loop() {
    uint32_t now = millis();

    if (now - g_last_sensor_ms >= LOOP_SENSOR_PERIOD_MS) {
        g_last_sensor_ms = now;
        _updateSensors();
    }

    if (now - g_last_impedance_ms >= LOOP_IMPEDANCE_PERIOD_MS) {
        g_last_impedance_ms = now;
        _updateImpedance();
    }

    if (now - g_last_state_ms >= LOOP_STATE_PERIOD_MS) {
        g_last_state_ms = now;
        _updateStateMachine();
    }

    if (now - g_last_web_ms >= LOOP_WEB_PUSH_PERIOD_MS) {
        g_last_web_ms = now;
        _pushWebData();
    }

    processUARTData();
    yield();
}
