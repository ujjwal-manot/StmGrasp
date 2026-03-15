#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
// Pin Assignments
// ─────────────────────────────────────────────────────────────

// Impedance spectroscopy
#define PIN_DAC_SINE        25   // DAC1 — AC excitation output
#define PIN_ADC_VREF        34   // ADC1_CH6 — reference voltage sense
#define PIN_ADC_VMUT        35   // ADC1_CH7 — material-under-test voltage

// Force-sensitive resistors (FSR402)
#define PIN_FSR1            36   // ADC1_CH0 / VP
#define PIN_FSR2            39   // ADC1_CH3 / VN
#define PIN_FSR3            32   // ADC1_CH4

// I2C bus (reserved, TCRT5000/ADS1115 removed from BOM)
#define PIN_SDA             21
#define PIN_SCL             22

// UART to STM32
#define PIN_UART_STM32_TX   16   // TX2
#define PIN_UART_STM32_RX   17   // RX2

// User buttons
#define PIN_BTN_CALIBRATE   33   // Calibrate / start (was piezo, now freed)
#define PIN_BTN_START        5   // Manual start (was NodeMCU TX, now freed)
#define PIN_SPARE_BTN        4

// ─────────────────────────────────────────────────────────────
// System Timing (milliseconds unless noted)
// ─────────────────────────────────────────────────────────────
#define LOOP_SENSOR_PERIOD_MS       50   // 20 Hz sensor reads
#define LOOP_IMPEDANCE_PERIOD_MS   200   // 5 Hz impedance measurement
#define LOOP_STATE_PERIOD_MS        50   // 20 Hz state machine
#define LOOP_WEB_PUSH_PERIOD_MS    100   // 10 Hz WebSocket broadcast

// ─────────────────────────────────────────────────────────────
// Impedance Engine
// ─────────────────────────────────────────────────────────────
#define IMP_FREQ_HZ              1000
#define IMP_SAMPLES_PER_CYCLE      40
#define IMP_SAMPLE_RATE       (IMP_FREQ_HZ * IMP_SAMPLES_PER_CYCLE)  // 40 kSPS
#define IMP_INTEGRATION_CYCLES     20
#define IMP_TOTAL_SAMPLES     (IMP_SAMPLES_PER_CYCLE * IMP_INTEGRATION_CYCLES) // 800
#define IMP_RREF_OHMS           10000.0f  // 10 kΩ reference resistor
#define IMP_DAC_AMPLITUDE         127     // 8-bit half-range (center at 128)
#define IMP_DAC_OFFSET            128
#define IMP_ADC_VREF_MV         3300.0f
#define IMP_ADC_RESOLUTION      4095.0f
#define IMP_CLASSIFY_THRESHOLD     2.0f   // max Euclidean distance for match
#define IMP_MIN_CONFIDENCE         0.15f

// ─────────────────────────────────────────────────────────────
// Acoustic Tap
// ─────────────────────────────────────────────────────────────
#define ACOUSTIC_SAMPLE_RATE      8000
#define ACOUSTIC_DURATION_MS       200
#define ACOUSTIC_NUM_SAMPLES      1600    // 8000 * 0.2
#define ACOUSTIC_FFT_SIZE         1024
#define ACOUSTIC_TAP_THRESHOLD     150    // ADC counts above baseline
#define ACOUSTIC_WAIT_TIMEOUT_MS  3000    // max wait for tap event

// ─────────────────────────────────────────────────────────────
// FSR (FSR402)
// ─────────────────────────────────────────────────────────────
#define FSR_COUNT                    3
#define FSR_PULLDOWN_OHM        10000.0f   // 10 kΩ pull-down
#define FSR_ADC_MAX             4095
#define FSR_MAX_FORCE_N           20.0f
#define FSR_SLIP_WINDOW            32
#define FSR_SLIP_SAMPLE_RATE     1000      // Hz for slip detection burst
#define FSR_SLIP_RMS_THRESHOLD      0.15f  // Newtons AC RMS

// ─────────────────────────────────────────────────────────────
// ADS1115 / IR Curvature (TCRT5000 x4)
// ─────────────────────────────────────────────────────────────
#define ADS1115_ADDR              0x48
#define ADS1115_REG_CONVERSION    0x00
#define ADS1115_REG_CONFIG        0x01
#define ADS1115_CHANNELS             4
#define ADS1115_GAIN_4V           0x0200   // ±4.096 V (gain = 1)
#define ADS1115_RATE_860SPS       0x00E0
#define ADS1115_MODE_SINGLE       0x0100
#define ADS1115_OS_START          0x8000
#define ADS1115_MUX_SHIFT            12
#define IR_FLAT_THRESHOLD           200    // ADC counts — all sensors similar
#define IR_EDGE_THRESHOLD          1500    // sharp curvature jump

// ─────────────────────────────────────────────────────────────
// UART Protocol
// ─────────────────────────────────────────────────────────────
#define UART_STM32_BAUD         115200
#define UART_SYNC_BYTE_STM32      0xAA
#define UART_MAX_PAYLOAD            72    // 64 depth + overhead
#define UART_RX_RING_SIZE          256
#define UART_TIMEOUT_MS             50

// STM32 command IDs
#define CMD_GRIP_OPEN             0x01
#define CMD_GRIP_CLOSE            0x02
#define CMD_SET_FORCE             0x03
#define CMD_TAP                   0x04
#define CMD_SERVO_MOVE            0x05
#define CMD_ESTOP                 0x06
#define CMD_REQUEST_DEPTH         0x07
#define CMD_REQUEST_IMU_TAP       0x24
#define CMD_REQUEST_MIC_TAP       0x25

// STM32 response IDs
#define RSP_POSITION              0x81
#define RSP_DEPTH_GRID            0x82
#define RSP_STATUS                0x83
#define RSP_ERROR                 0x84
#define RSP_IMU_TAP_DATA          0x92
#define RSP_MIC_TAP_DATA          0x93

// ─────────────────────────────────────────────────────────────
// WiFi / Web
// ─────────────────────────────────────────────────────────────
#define WIFI_SSID            "HydraGrasp"
#define WIFI_PASS            "hydra2026"
#define WEB_PORT                   80
#define WS_MAX_CLIENTS              4

// ─────────────────────────────────────────────────────────────
// Grasp Planner
// ─────────────────────────────────────────────────────────────
#define STATE_TIMEOUT_SCANNING_MS    3000
#define STATE_TIMEOUT_ANALYZING_MS   2000
#define STATE_TIMEOUT_TAP_MS         4000
#define STATE_TIMEOUT_CLASSIFYING_MS 1000
#define STATE_TIMEOUT_PLANNING_MS    1000
#define STATE_TIMEOUT_APPROACHING_MS 5000
#define STATE_TIMEOUT_GRIPPING_MS    3000

#define GRASP_QUALITY_MIN          0.3f
#define GRASP_FORCE_SAFETY_MARGIN  0.85f   // fraction of material max
#define GRASP_RAMP_DEFAULT_NPS     2.0f    // Newtons per second
#define GRASP_SLIP_RESPOND_MS       20

// ─────────────────────────────────────────────────────────────
// Depth Grid (VL53L8CX from STM32)
// ─────────────────────────────────────────────────────────────
#define DEPTH_GRID_ROWS  8
#define DEPTH_GRID_COLS  8
#define DEPTH_MAX_MM   4000

// ─────────────────────────────────────────────────────────────
// Material Database
// ─────────────────────────────────────────────────────────────

struct MaterialEntry {
    const char* name;
    float impedance_magnitude;   // Ohms
    float impedance_phase_deg;   // degrees (negative for capacitive)
    float max_grip_force_N;
    float slip_risk;             // 0-1
    uint32_t led_color;          // RGB packed for dashboard / LEDs
};

#define MATERIAL_COUNT 6

// Indices into the database
#define MAT_METAL     0
#define MAT_SKIN      1
#define MAT_PLASTIC   2
#define MAT_WOOD      3
#define MAT_GLASS     4
#define MAT_CARDBOARD 5
#define MAT_UNKNOWN   255

static const MaterialEntry MATERIAL_DB[MATERIAL_COUNT] = {
    // name         |Z| (Ω)      phase(°)    max_grip  slip_risk  color
    { "Metal",          1.0f,        0.0f,       8.0f,    0.2f,   0x3498DB },
    { "Skin",       50000.0f,      -35.0f,       1.5f,    0.3f,   0xF5B041 },
    { "Plastic",  1000000.0f,      -85.0f,       5.0f,    0.6f,   0x9B59B6 },
    { "Wood",      500000.0f,      -20.0f,       6.0f,    0.3f,   0x28B463 },
    { "Glass",   10000000.0f,      -88.0f,       4.0f,    0.8f,   0xEC7063 },
    { "Cardboard", 200000.0f,      -25.0f,       2.0f,    0.4f,   0xD4AC0D },
};

// ─────────────────────────────────────────────────────────────
// Shared Data Structures
// ─────────────────────────────────────────────────────────────

struct ImpedanceResult {
    float magnitude;       // Ohms
    float phase_deg;       // degrees
    float confidence;      // 0-1
    uint8_t material_idx;  // index into MATERIAL_DB or MAT_UNKNOWN
    bool valid;
};

struct AcousticResult {
    uint8_t material_idx;
    float confidence;
    float dominant_freq;
    float spectral_centroid;
    float decay_ratio;
    float spectrum[ACOUSTIC_FFT_SIZE / 2]; // magnitude spectrum
    bool valid;
};

struct CurvatureResult {
    float curvature_x;     // left-right curvature
    float curvature_y;     // top-bottom curvature
    float flatness;        // 0 = curved, 1 = perfectly flat
    bool sharp_edge;
    uint8_t geometry;      // 0=flat, 1=convex, 2=concave, 3=edge, 4=cylinder
    uint16_t raw[ADS1115_CHANNELS];
    bool valid;
};

struct ForceResult {
    float forces_N[FSR_COUNT];
    float total_N;
    float balance;           // 0-1: 1 = perfectly balanced
    float cop_x;             // center of pressure X (-1 to 1)
    float cop_y;             // center of pressure Y (-1 to 1)
    bool slip_detected;
    float slip_rms;
    bool valid;
};

struct DepthGrid {
    uint16_t mm[DEPTH_GRID_ROWS][DEPTH_GRID_COLS];
    uint16_t min_mm;
    uint16_t max_mm;
    float centroid_x;      // column centroid (0-7)
    float centroid_y;      // row centroid (0-7)
    bool valid;
};

// ─────────────────────────────────────────────────────────────
// Grasp Strategy Repertoire
// ─────────────────────────────────────────────────────────────
enum GraspStrategy : uint8_t {
    STRATEGY_POWER     = 0,  // High force, full close, robust objects
    STRATEGY_PRECISION = 1,  // Low force, fingertip only, fragile/small
    STRATEGY_WRAP      = 2,  // Progressive close, curved objects
    STRATEGY_EDGE      = 3,  // Angled approach, thin/flat objects
    STRATEGY_COUNT     = 4
};

static const char* STRATEGY_NAMES[] = {
    "POWER", "PRECISION", "WRAP", "EDGE"
};

struct GraspPlan {
    float target_force_N;
    float approach_speed;    // 0-1 normalized
    float finger_aperture;   // mm
    float force_ramp_Nps;    // N per second
    float slip_threshold;
    float quality_score;     // 0-1
    GraspStrategy strategy;  // selected grasp strategy
    float success_prob;      // predicted P(success) from success predictor
    bool valid;
};

enum GraspState : uint8_t {
    STATE_IDLE = 0,
    STATE_DETECTED,
    STATE_SCANNING,
    STATE_ANALYZING,
    STATE_TAP_TESTING,
    STATE_CLASSIFYING,
    STATE_PLANNING,
    STATE_APPROACHING,
    STATE_GRIPPING,
    STATE_HOLDING,
    STATE_RELEASING,
    STATE_ERROR
};

static const char* STATE_NAMES[] = {
    "IDLE", "DETECTED", "SCANNING", "ANALYZING", "TAP_TESTING",
    "CLASSIFYING", "PLANNING", "APPROACHING", "GRIPPING",
    "HOLDING", "RELEASING", "ERROR"
};

#endif // CONFIG_H
