/*
 * HYDRA Grasp — ESP32 Brain Firmware
 *
 * Sensor fusion controller for the STEVAL-ROBKIT1 robotic grasping system.
 * Receives depth/IMU data from STM32H725 over UART, performs impedance
 * spectroscopy, acoustic tap analysis, force sensing, curvature detection,
 * grasp planning, and serves a real-time WiFi dashboard.
 *
 * Hardware:
 *   ESP32 DevKitC (or equivalent with DAC1 on GPIO25)
 *   ADS1115 on I2C for TCRT5000 IR array
 *   3x FSR402 on ADC
 *   Piezo transducer on ADC
 *   UART to STM32H725 (sensor hub + motor control)
 *   UART to NodeMCU (LED strip)
 */

#include "config.h"
#include "impedance.h"
#include "acoustic.h"
#include "sensors.h"
#include "grasp_planner.h"
#include "comms.h"
#include "web_server.h"

// ─────────────────────────────────────────────────────────────
// Global sensor state
// ─────────────────────────────────────────────────────────────
static ImpedanceResult g_impedance;
static AcousticResult  g_acoustic;
static ForceResult     g_force;
static CurvatureResult g_curvature;
static DepthGrid       g_depth;
static GraspPlan       g_plan;
static GraspState      g_state = STATE_IDLE;
static float           g_grasp_quality = 0.0f;
static uint8_t         g_fused_material = MAT_UNKNOWN;
static float           g_fused_confidence = 0.0f;

// Timing accumulators
static uint32_t g_last_sensor_ms = 0;
static uint32_t g_last_impedance_ms = 0;
static uint32_t g_last_state_ms = 0;
static uint32_t g_last_web_ms = 0;
static uint32_t g_last_depth_request_ms = 0;

// Acoustic measurement state
static bool g_acoustic_pending = false;
static bool g_acoustic_done = false;

// Slip auto-correction
static float g_current_force_cmd = 0.0f;


// ─────────────────────────────────────────────────────────────
// Dashboard command handler
// ─────────────────────────────────────────────────────────────
static void _handleWebCommand(const char* cmd) {
    if (strcmp(cmd, "start") == 0) {
        if (g_state == STATE_IDLE) {
            requestStateTransition(STATE_DETECTED);
        }
    } else if (strcmp(cmd, "release") == 0) {
        requestStateTransition(STATE_RELEASING);
        sendGripOpen();
        g_current_force_cmd = 0.0f;
    } else if (strcmp(cmd, "estop") == 0) {
        triggerEstop();
        sendEstop();
        sendLEDSolid(255, 0, 0);
        g_current_force_cmd = 0.0f;
    }
}


// ─────────────────────────────────────────────────────────────
// Sensor reading task (runs at 20 Hz from loop)
// ─────────────────────────────────────────────────────────────
static void _updateSensors() {
    g_force = readForces();
    g_curvature = readIRCurvatureArray();

    // Process any incoming UART from STM32
    processUARTData();

    // Grab new depth grid if available
    if (hasNewDepthGrid()) {
        g_depth = getLatestDepthGrid();
    }

    // Periodically request depth data
    uint32_t now = millis();
    if (now - g_last_depth_request_ms >= 500) {
        sendRequestDepth();
        g_last_depth_request_ms = now;
    }

    // Update LED force map
    if (g_force.valid) {
        uint8_t f1 = (uint8_t)fminf(g_force.forces_N[0] * 12.75f, 255.0f);
        uint8_t f2 = (uint8_t)fminf(g_force.forces_N[1] * 12.75f, 255.0f);
        uint8_t f3 = (uint8_t)fminf(g_force.forces_N[2] * 12.75f, 255.0f);
        sendLEDForceMap(f1, f2, f3);
    }
}


// ─────────────────────────────────────────────────────────────
// Impedance measurement (runs at 5 Hz from loop)
// ─────────────────────────────────────────────────────────────
static void _updateImpedance() {
    g_impedance = measureImpedance();
}


// ─────────────────────────────────────────────────────────────
// State machine execution (runs at 20 Hz from loop)
// ─────────────────────────────────────────────────────────────
static void _updateStateMachine() {
    GraspState prev_state = g_state;

    g_state = runStateMachine(g_state, g_force, g_impedance, g_curvature,
                              g_depth, g_plan, g_grasp_quality);

    // ── Actions on state entry ──
    if (g_state != prev_state) {
        switch (g_state) {
            case STATE_IDLE:
                sendLEDHeartbeat(60);
                g_acoustic_done = false;
                g_acoustic_pending = false;
                g_fused_material = MAT_UNKNOWN;
                g_fused_confidence = 0.0f;
                g_current_force_cmd = 0.0f;
                break;

            case STATE_DETECTED:
                sendLEDAnimate(1, 200); // pulsing blue
                break;

            case STATE_SCANNING:
                sendLEDAnimate(2, 150); // scanning sweep
                sendRequestDepth();
                break;

            case STATE_ANALYZING:
                sendLEDAnimate(3, 100); // analysis spin
                break;

            case STATE_TAP_TESTING:
                // Trigger tap on the gripper
                sendTapCommand();
                g_acoustic_pending = true;
                g_acoustic_done = false;
                break;

            case STATE_CLASSIFYING:
                // Cross-validate impedance + acoustic
                g_fused_material = crossValidateMaterial(g_impedance, g_acoustic,
                                                        &g_fused_confidence);
                if (g_fused_material < MATERIAL_COUNT) {
                    sendLEDMaterial(g_fused_material);
                }
                break;

            case STATE_PLANNING:
                g_plan = computeGraspPlan(g_impedance, g_acoustic, g_curvature, g_depth,
                                          g_fused_material, g_fused_confidence);
                break;

            case STATE_APPROACHING:
                sendGripClose();
                if (g_plan.valid) {
                    sendSetForce(g_plan.target_force_N * 0.3f); // initial light contact
                    g_current_force_cmd = g_plan.target_force_N * 0.3f;
                }
                sendLEDAnimate(4, 80); // approach animation
                break;

            case STATE_GRIPPING:
                sendLEDAnimate(5, 60); // gripping pulse
                break;

            case STATE_HOLDING:
                sendLEDSolid(0, 255, 0); // green = stable hold
                break;

            case STATE_RELEASING:
                sendGripOpen();
                g_current_force_cmd = 0.0f;
                sendLEDAnimate(6, 120); // release animation
                break;

            case STATE_ERROR:
                sendEstop();
                sendLEDSolid(255, 0, 0); // red = error
                g_current_force_cmd = 0.0f;
                break;
        }
    }

    // ── Continuous state actions ──
    switch (g_state) {
        case STATE_TAP_TESTING:
            // Try to capture acoustic response if tap was issued
            if (g_acoustic_pending && !g_acoustic_done) {
                if (waitForTap(50)) { // non-blocking short check
                    g_acoustic = analyzeTap();
                    g_acoustic_done = true;
                    g_acoustic_pending = false;
                    // Advance state immediately
                    requestStateTransition(STATE_CLASSIFYING);
                }
            }
            break;

        case STATE_GRIPPING:
            // Ramp force toward target
            if (g_plan.valid && g_force.valid) {
                float dt = (float)LOOP_STATE_PERIOD_MS / 1000.0f;
                float increment = g_plan.force_ramp_Nps * dt;
                if (g_current_force_cmd < g_plan.target_force_N) {
                    g_current_force_cmd += increment;
                    if (g_current_force_cmd > g_plan.target_force_N) {
                        g_current_force_cmd = g_plan.target_force_N;
                    }
                    sendSetForce(g_current_force_cmd);
                }
            }
            break;

        case STATE_HOLDING:
            // Compute live grasp quality
            g_grasp_quality = computeGraspQuality(g_force, g_impedance, g_plan);

            // Slip compensation: if slip detected, increase force by 15%
            if (g_force.valid && g_force.slip_detected && g_plan.valid) {
                float boost = g_current_force_cmd * 1.15f;
                float max_allowed = g_plan.target_force_N * 1.3f;
                if (boost <= max_allowed) {
                    g_current_force_cmd = boost;
                    sendSetForce(g_current_force_cmd);
                    Serial.printf("[MAIN] Slip! Force boosted to %.1f N\n", g_current_force_cmd);
                }
            }
            break;

        default:
            break;
    }
}


// ─────────────────────────────────────────────────────────────
// WebSocket data push (runs at 10 Hz from loop)
// ─────────────────────────────────────────────────────────────
static void _pushWebData() {
    cleanupWebClients();
    pushSensorData(g_impedance, g_acoustic, g_force, g_curvature, g_depth,
                   g_plan, g_state, g_grasp_quality,
                   g_fused_material, g_fused_confidence);
}


// ─────────────────────────────────────────────────────────────
// Arduino setup
// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) { /* wait up to 2s for USB serial */ }
    Serial.println("\n══════════════════════════════════");
    Serial.println("  HYDRA Grasp — ESP32 Brain v1.0");
    Serial.println("══════════════════════════════════");

    // Initialize subsystems
    Serial.println("[INIT] Impedance engine...");
    initImpedance();

    Serial.println("[INIT] Acoustic subsystem...");
    initAcoustic();

    Serial.println("[INIT] Sensors (FSR + IR + ADS1115)...");
    initSensors();

    Serial.println("[INIT] Grasp planner...");
    initGraspPlanner();

    Serial.println("[INIT] UART comms...");
    initComms();

    Serial.println("[INIT] WiFi AP...");
    setupWiFiAP();

    Serial.println("[INIT] Web server + WebSocket...");
    setupWebServer();
    setWebCommandCallback(_handleWebCommand);

    // Calibrate impedance baseline with no object
    Serial.println("[INIT] Calibrating impedance baseline...");
    calibrateBaseline();

    // Initialize global state
    memset(&g_impedance, 0, sizeof(g_impedance));
    memset(&g_acoustic, 0, sizeof(g_acoustic));
    memset(&g_force, 0, sizeof(g_force));
    memset(&g_curvature, 0, sizeof(g_curvature));
    memset(&g_depth, 0, sizeof(g_depth));
    memset(&g_plan, 0, sizeof(g_plan));

    // Start heartbeat LED
    sendLEDHeartbeat(60);

    uint32_t now = millis();
    g_last_sensor_ms = now;
    g_last_impedance_ms = now;
    g_last_state_ms = now;
    g_last_web_ms = now;
    g_last_depth_request_ms = now;

    Serial.println("[INIT] Ready. Dashboard: http://192.168.4.1");
    Serial.println("══════════════════════════════════\n");
}


// ─────────────────────────────────────────────────────────────
// Arduino main loop — non-blocking, millis()-based scheduling
// ─────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    // 20 Hz: sensor reads (force, curvature, UART parsing)
    if (now - g_last_sensor_ms >= LOOP_SENSOR_PERIOD_MS) {
        g_last_sensor_ms = now;
        _updateSensors();
    }

    // 5 Hz: impedance measurement (takes ~20ms for lock-in integration)
    if (now - g_last_impedance_ms >= LOOP_IMPEDANCE_PERIOD_MS) {
        g_last_impedance_ms = now;
        _updateImpedance();
    }

    // 20 Hz: state machine
    if (now - g_last_state_ms >= LOOP_STATE_PERIOD_MS) {
        g_last_state_ms = now;
        _updateStateMachine();
    }

    // 10 Hz: dashboard push
    if (now - g_last_web_ms >= LOOP_WEB_PUSH_PERIOD_MS) {
        g_last_web_ms = now;
        _pushWebData();
    }

    // Always process incoming UART (non-blocking)
    processUARTData();

    // Yield to FreeRTOS / WiFi stack
    yield();
}
