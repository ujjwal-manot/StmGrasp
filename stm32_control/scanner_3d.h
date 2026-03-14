/**
 * 3D Scanner Module - Photometric Stereo + ToF Depth Fusion
 * Uses VL53L8CX 8x8 multizone ToF for depth + signal_per_spad as
 * brightness proxy under 3 IR LED illuminations for photometric stereo.
 * Surface normals, curvature, and edge classification enable grasp planning
 * on reconstructed 3D point clouds.
 *
 * Hardware:
 *   - VL53L8CX 8x8 ToF (I2C1, already initialized in depth_sensor.c)
 *   - VD56G3 1.53MP monochrome camera (DCMI, optional - ToF proxy used primarily)
 *   - 3x IR LEDs at 120 degree intervals around camera (PE7, PE8, PE9)
 *   - 2x DC motors with encoders for 360 degree rotation scanning
 */

#ifndef SCANNER_3D_H
#define SCANNER_3D_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "depth_sensor.h"
#include "motor_control.h"

/* ========================================================================== */
/*                       SCAN CONFIGURATION                                    */
/* ========================================================================== */

#define SCAN_IMG_WIDTH              320U    /* Downscaled from 1.53MP for speed */
#define SCAN_IMG_HEIGHT             240U
#define SCAN_IR_LED_COUNT           3U      /* 3 IR LEDs at 120 deg around camera */
#define SCAN_POSITIONS              12U     /* 12 rotation positions (30 deg each = 360 deg) */
#define SCAN_MAX_POINTS             (SCAN_POSITIONS * VL53L8CX_ZONE_COUNT)  /* 12*64 = 768 */

/* ToF-based normal map resolution: 8x8 matching VL53L8CX zones */
#define SCAN_NMAP_ROWS              VL53L8CX_GRID_SIZE
#define SCAN_NMAP_COLS              VL53L8CX_GRID_SIZE

/* Rotation angle per scan position (degrees) */
#define SCAN_ROTATION_STEP_DEG      30.0f

/* LED stabilization time after switching (ms) */
#define SCAN_LED_SETTLE_MS          5U

/* Motor settling time after rotation (ms) */
#define SCAN_MOTOR_SETTLE_MS        200U

/* VL53L8CX field of view (degrees, total) */
#define VL53L8CX_FOV_DEG            45.0f

/* Half FoV per axis */
#define VL53L8CX_HALF_FOV_DEG       (VL53L8CX_FOV_DEG / 2.0f)

/* Angular step per zone (degrees) */
#define VL53L8CX_ZONE_ANGLE_DEG     (VL53L8CX_FOV_DEG / (float)VL53L8CX_GRID_SIZE)

/* ========================================================================== */
/*                       IR LED GPIO PINS                                      */
/* ========================================================================== */

/* IR LEDs on GPIOE (PE7, PE8, PE9 on 40-pin header) */
#define IR_LED1_PORT                GPIOE
#define IR_LED1_PIN                 GPIO_PIN_7
#define IR_LED2_PORT                GPIOE
#define IR_LED2_PIN                 GPIO_PIN_8
#define IR_LED3_PORT                GPIOE
#define IR_LED3_PIN                 GPIO_PIN_9

/* ========================================================================== */
/*                       EDGE DETECTION THRESHOLDS                             */
/* ========================================================================== */

/* Normal angle difference thresholds (degrees) for surface classification */
#define EDGE_THRESH_FLAT_DEG        10.0f
#define EDGE_THRESH_SMOOTH_DEG      30.0f
#define EDGE_THRESH_CURVED_DEG      60.0f
#define EDGE_THRESH_SHARP_DEG       90.0f

/* Curvature thresholds */
#define CURVATURE_FLAT_THRESH       0.05f
#define CURVATURE_SMOOTH_THRESH     0.25f
#define CURVATURE_SHARP_THRESH      0.60f

/* ========================================================================== */
/*                       UART PROTOCOL EXTENSIONS                              */
/* ========================================================================== */

/* Scanner command/response IDs are defined in main.h:
 *   CMD_START_FULL_SCAN   0x20  CMD_START_QUICK_SCAN  0x21
 *   CMD_GET_SCAN_STATUS   0x22  CMD_GET_SCAN_RESULT   0x23
 *   RSP_SCAN_STATUS       0x90  RSP_SCAN_RESULT       0x91
 */

/* ========================================================================== */
/*                       DATA STRUCTURES                                       */
/* ========================================================================== */

/* Surface classification */
typedef enum {
    SURFACE_UNKNOWN = 0,
    SURFACE_FLAT,
    SURFACE_SMOOTH_CURVE,
    SURFACE_SHARP_EDGE,
    SURFACE_CORNER,
    SURFACE_CONCAVE,
    SURFACE_CONVEX
} SurfaceType_t;

/* A single 3D point with surface properties */
typedef struct {
    float x, y, z;              /* 3D position in mm (robot-centric frame) */
    float nx, ny, nz;           /* Surface normal vector (unit) */
    float curvature;            /* Local curvature magnitude (0=flat, high=sharp) */
    SurfaceType_t surface;      /* Classification */
    uint8_t confidence;         /* 0-100 */
} ScanPoint_t;

/* Surface normal map from photometric stereo (per-zone, 8x8) */
typedef struct {
    float nx[SCAN_NMAP_ROWS][SCAN_NMAP_COLS];
    float ny[SCAN_NMAP_ROWS][SCAN_NMAP_COLS];
    float nz[SCAN_NMAP_ROWS][SCAN_NMAP_COLS];
    bool valid;
} NormalMap_t;

/* Edge map from surface analysis (per-zone, 8x8) */
typedef struct {
    uint8_t edge_strength[SCAN_NMAP_ROWS][SCAN_NMAP_COLS];  /* 0-255 */
    SurfaceType_t edge_type[SCAN_NMAP_ROWS][SCAN_NMAP_COLS];
    uint16_t sharp_edge_count;
    uint16_t smooth_curve_count;
    bool valid;
} EdgeMap_t;

/* Signal-per-SPAD "brightness image" from VL53L8CX under one IR LED */
typedef struct {
    uint16_t signal[VL53L8CX_GRID_SIZE][VL53L8CX_GRID_SIZE];
    bool valid;
} ToFSignalImage_t;

/* Complete 3D scan result */
typedef struct {
    ScanPoint_t points[SCAN_MAX_POINTS];
    uint16_t point_count;
    EdgeMap_t edges;
    float bounding_box[6];      /* min_x, max_x, min_y, max_y, min_z, max_z */
    float centroid[3];          /* Object center (x, y, z) */
    float max_dimension;        /* Largest dimension in mm */
    SurfaceType_t dominant_surface; /* Most common surface type */
    bool complete;
    uint32_t scan_time_ms;
} ScanResult_t;

/* Scanner state machine */
typedef enum {
    SCANNER_IDLE,
    SCANNER_STARTING,
    SCANNER_ROTATING,
    SCANNER_CAPTURING,
    SCANNER_PROCESSING,
    SCANNER_COMPLETE,
    SCANNER_ERROR
} ScannerState_t;

/* ========================================================================== */
/*                       FUNCTION PROTOTYPES                                   */
/* ========================================================================== */

/* Lifecycle */
HAL_StatusTypeDef initScanner(void);
HAL_StatusTypeDef startFullScan(void);
HAL_StatusTypeDef startQuickScan(void);    /* Single position, no rotation */
ScannerState_t    getScannerState(void);
ScanResult_t*     getScanResult(void);

/* FreeRTOS task entry */
void scannerTask(void *argument);

/* Internal functions (declared in .h for testing) */
HAL_StatusTypeDef captureWithIRLED(uint8_t led_index, ToFSignalImage_t *signal_img);
void computeNormalMap(const ToFSignalImage_t images[SCAN_IR_LED_COUNT], NormalMap_t *normals);
void detectEdgesFromNormals(const NormalMap_t *normals, EdgeMap_t *edges);
void fuseDepthAndNormals(const DepthGrid_t *depth, const NormalMap_t *normals,
                         float rotation_angle, ScanPoint_t *points, uint16_t *count);
float computeLocalCurvature(const NormalMap_t *normals, uint8_t row, uint8_t col);
SurfaceType_t classifySurface(float curvature, float normal_variation);
void computeBoundingBox(ScanResult_t *result);
void sendScanResult(const ScanResult_t *result);
void sendScanStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* SCANNER_3D_H */
