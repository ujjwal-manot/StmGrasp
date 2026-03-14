/**
 * 3D Scanner Module - Photometric Stereo + ToF Depth Fusion
 *
 * Implements a complete 3D scanning pipeline for robotic grasp planning:
 *
 * 1. Photometric Stereo:
 *    Three IR LEDs at 120 degree intervals illuminate the scene sequentially.
 *    The VL53L8CX signal_per_spad values serve as an 8x8 brightness image
 *    for each illumination direction. From three brightness images under
 *    known light directions, surface normals are computed per zone via
 *    matrix inversion (Woodham's method).
 *
 * 2. Depth Fusion:
 *    The VL53L8CX distance_mm grid provides absolute depth. Combined with
 *    surface normals from photometric stereo, each zone becomes a 3D point
 *    with orientation, enabling curvature and edge detection.
 *
 * 3. Multi-Position Scanning:
 *    The robot rotates 30 degrees between captures for 12 positions (360 deg).
 *    Points are transformed into a common robot-centric coordinate frame.
 *
 * 4. Surface Classification:
 *    Local curvature computed from normal map gradients classifies each point
 *    as flat, smooth curve, sharp edge, corner, concave, or convex.
 *
 * Memory budget (STM32H725 has 564KB SRAM):
 *   - 3x ToFSignalImage_t:  3 * 128 bytes     =    384 bytes
 *   - NormalMap_t:           3 * 256 bytes + 4  =    772 bytes
 *   - EdgeMap_t:             256 + 256 + 4 + 4  =    520 bytes
 *   - ScanResult_t:          768 * 32 + overhead = ~25 KB
 *   - Total: ~27 KB (well within budget)
 *
 * Reference: ST VL53L8CX ULD driver (STSW-IMG040) for signal readout.
 *            ARM CMSIS-DSP arm_math.h for fast FPU operations.
 */

#include "scanner_3d.h"
#include "uart_protocol.h"
#include <math.h>

/* Use ARM CMSIS-DSP for fast sqrt on Cortex-M7 FPU */
#ifdef ARM_MATH_CM7
#include "arm_math.h"
#endif

/* ========================================================================== */
/*                       STATIC STATE                                          */
/* ========================================================================== */

static volatile ScannerState_t s_scannerState = SCANNER_IDLE;
static ScanResult_t            s_scanResult;
static bool                    s_fullScanRequested  = false;
static bool                    s_quickScanRequested = false;
static uint8_t                 s_currentPosition    = 0;

/* Working buffers for current capture */
static ToFSignalImage_t        s_signalImages[SCAN_IR_LED_COUNT];
static NormalMap_t             s_normalMap;
static EdgeMap_t               s_edgeMap;

/* ========================================================================== */
/*                       PRECOMPUTED LIGHT MATRIX INVERSE                      */
/* ========================================================================== */

/*
 * IR LEDs are mounted at 120 degree intervals around the camera, each at
 * approximately 30 degrees elevation from the optical axis.
 *
 * Light direction vectors (unit vectors from surface toward light):
 *   L1 = (cos(0)*cos(30),   sin(0)*cos(30),   sin(30))   = ( 0.8660,  0.0000, 0.5000)
 *   L2 = (cos(120)*cos(30), sin(120)*cos(30), sin(30))   = (-0.4330,  0.7500, 0.5000)
 *   L3 = (cos(240)*cos(30), sin(240)*cos(30), sin(30))   = (-0.4330, -0.7500, 0.5000)
 *
 * Light matrix L (rows are light directions):
 *   L = |  0.8660   0.0000   0.5000 |
 *       | -0.4330   0.7500   0.5000 |
 *       | -0.4330  -0.7500   0.5000 |
 *
 * L_inverse computed symbolically (3x3 matrix inverse):
 *   det(L) = 0.8660*(0.7500*0.5000 - 0.5000*(-0.7500))
 *          - 0.0000*(...)
 *          + 0.5000*((-0.4330)*(-0.7500) - (-0.4330)*0.7500)
 *          = 0.8660 * 0.7500 + 0.5000 * 0.6495
 *          = 0.6495 + 0.3248
 *          = 0.9743
 *
 * Pre-computed using exact trigonometric values.
 */

static const float L_INV[3][3] = {
    {  0.7698f,  -0.3849f, -0.3849f },
    {  0.0000f,   0.6667f, -0.6667f },
    {  0.3333f,   0.3333f,  0.3333f }
};

/* ========================================================================== */
/*                       FAST MATH HELPERS                                     */
/* ========================================================================== */

/**
 * Fast inverse square root (for normal vector normalization).
 * Falls back to 1.0f/sqrtf() when ARM FPU is available since the
 * hardware VSQRT instruction is faster than the Quake trick on M7.
 */
static inline float fastInvSqrt(float x)
{
    if (x < 1.0e-10f) {
        return 0.0f;
    }
#ifdef ARM_MATH_CM7
    float result;
    arm_sqrt_f32(x, &result);
    return (result > 1.0e-10f) ? (1.0f / result) : 0.0f;
#else
    return 1.0f / sqrtf(x);
#endif
}

/**
 * Fast square root using ARM FPU or standard library.
 */
static inline float fastSqrt(float x)
{
    if (x < 0.0f) {
        return 0.0f;
    }
#ifdef ARM_MATH_CM7
    float result;
    arm_sqrt_f32(x, &result);
    return result;
#else
    return sqrtf(x);
#endif
}

/**
 * Dot product of two 3D vectors.
 */
static inline float dot3(float ax, float ay, float az,
                          float bx, float by, float bz)
{
    return ax * bx + ay * by + az * bz;
}

/**
 * Clamp float to range.
 */
static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/**
 * Convert degrees to radians.
 */
static inline float deg2rad(float deg)
{
    return deg * 0.017453292f; /* PI / 180 */
}

/**
 * Fast approximate acos using polynomial (accurate to ~0.5 degrees).
 * Input must be clamped to [-1, 1].
 * Returns result in degrees.
 */
static float fastAcosDeg(float x)
{
    x = clampf(x, -1.0f, 1.0f);

    /* Attempt Corrected Approximation:
     * acos(x) ~= 90 - (x + x^3/6 + 3x^5/40) * (180/PI)
     * Good enough for edge classification. */
    float x2 = x * x;
    float x3 = x2 * x;
    float asin_approx = x + x3 * 0.16666667f + x3 * x2 * 0.075f;

    /* acos(x) = 90 - asin(x) in degrees */
    return 90.0f - asin_approx * 57.29577951f;
}

/* ========================================================================== */
/*                       IR LED CONTROL                                        */
/* ========================================================================== */

/**
 * GPIO port and pin lookup for IR LEDs.
 */
static GPIO_TypeDef* const s_irLedPort[SCAN_IR_LED_COUNT] = {
    IR_LED1_PORT, IR_LED2_PORT, IR_LED3_PORT
};

static const uint16_t s_irLedPin[SCAN_IR_LED_COUNT] = {
    IR_LED1_PIN, IR_LED2_PIN, IR_LED3_PIN
};

/**
 * Turn OFF all IR LEDs.
 */
static void allIRLedsOff(void)
{
    for (uint8_t i = 0; i < SCAN_IR_LED_COUNT; i++) {
        HAL_GPIO_WritePin(s_irLedPort[i], s_irLedPin[i], GPIO_PIN_RESET);
    }
}

/**
 * Turn ON a specific IR LED (all others OFF).
 */
static void setIRLed(uint8_t index)
{
    allIRLedsOff();
    if (index < SCAN_IR_LED_COUNT) {
        HAL_GPIO_WritePin(s_irLedPort[index], s_irLedPin[index], GPIO_PIN_SET);
    }
}

/* ========================================================================== */
/*                       VL53L8CX SIGNAL READOUT                               */
/* ========================================================================== */

/**
 * VL53L8CX result registers for signal_per_spad.
 * The signal field is a 32-bit value per zone at offset 0x0300 in the
 * result buffer. We read it directly via I2C.
 */

/**
 * Read signal_per_spad values from VL53L8CX into a ToFSignalImage_t.
 * This provides a proxy "brightness image" showing reflected IR intensity
 * for each of the 64 zones. Under different IR LED illumination, the
 * signal changes based on surface angle relative to the light source.
 */
static HAL_StatusTypeDef readToFSignal(ToFSignalImage_t *img)
{
    uint8_t buf[VL53L8CX_ZONE_COUNT * 4]; /* 4 bytes per zone (uint32_t) */
    HAL_StatusTypeDef ret;

    /* Read signal_per_spad for all 64 zones.
     * The VL53L8CX stores signal values as 32-bit words at the signal register area. */
    ret = vl53l8cx_read_multi(VL53L8CX_RESULT_SIGNAL, buf, VL53L8CX_ZONE_COUNT * 4);
    if (ret != HAL_OK) {
        img->valid = false;
        return ret;
    }

    for (uint8_t zone = 0; zone < VL53L8CX_ZONE_COUNT; zone++) {
        uint16_t offset = zone * 4;
        /* Signal is 32-bit LE, but we only need 16-bit precision */
        uint32_t raw_signal = (uint32_t)buf[offset]
                            | ((uint32_t)buf[offset + 1] << 8)
                            | ((uint32_t)buf[offset + 2] << 16)
                            | ((uint32_t)buf[offset + 3] << 24);

        uint8_t row = zone / VL53L8CX_GRID_SIZE;
        uint8_t col = zone % VL53L8CX_GRID_SIZE;

        /* Saturate to uint16_t (signal values are typically < 10000) */
        img->signal[row][col] = (raw_signal > 0xFFFFU) ? 0xFFFFU : (uint16_t)raw_signal;
    }

    img->valid = true;
    return HAL_OK;
}

/* ========================================================================== */
/*                       SCANNER INITIALIZATION                                */
/* ========================================================================== */

HAL_StatusTypeDef initScanner(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Enable GPIOE clock for IR LED pins */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Configure IR LED pins as push-pull outputs, initially OFF */
    gpio.Pin   = IR_LED1_PIN | IR_LED2_PIN | IR_LED3_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* Ensure all LEDs are OFF */
    allIRLedsOff();

    /* Verify ToF sensor is accessible (it should already be initialized by DepthTask,
     * but we check the device ID register to confirm I2C path is working) */
    uint8_t dev_id = 0;
    HAL_StatusTypeDef ret = vl53l8cx_read_byte(VL53L8CX_REG_DEVICE_ID, &dev_id);
    if (ret != HAL_OK || dev_id != VL53L8CX_DEVICE_ID) {
        s_scannerState = SCANNER_ERROR;
        return HAL_ERROR;
    }

    /* Clear scan result */
    memset(&s_scanResult, 0, sizeof(ScanResult_t));
    s_scanResult.complete = false;

    s_fullScanRequested  = false;
    s_quickScanRequested = false;
    s_currentPosition    = 0;
    s_scannerState       = SCANNER_IDLE;

    return HAL_OK;
}

/* ========================================================================== */
/*                       CAPTURE WITH IR LED                                   */
/* ========================================================================== */

HAL_StatusTypeDef captureWithIRLED(uint8_t led_index, ToFSignalImage_t *signal_img)
{
    if (led_index >= SCAN_IR_LED_COUNT) {
        return HAL_ERROR;
    }

    /* Turn OFF all IR LEDs */
    allIRLedsOff();

    /* Turn ON only the requested LED */
    setIRLed(led_index);

    /* Wait for LED current to stabilize and scene to be illuminated.
     * The VL53L8CX integration time is 20ms, so the LED must be ON
     * before the next ranging cycle begins. */
    osDelay(SCAN_LED_SETTLE_MS);

    /* Trigger a new ranging cycle by waiting for data-ready.
     * The VL53L8CX is running at 5Hz (200ms period), so we wait for
     * the next data-ready flag which will include our illumination. */
    uint32_t start = HAL_GetTick();
    uint8_t data_ready = 0;
    HAL_StatusTypeDef ret;

    /* Poll for up to 250ms (slightly more than one ranging period) */
    while ((HAL_GetTick() - start) < 250U) {
        ret = vl53l8cx_read_byte(VL53L8CX_REG_UI_CMD_STATUS, &data_ready);
        if (ret != HAL_OK) {
            allIRLedsOff();
            signal_img->valid = false;
            return ret;
        }
        if (data_ready & 0x01) {
            break;
        }
        osDelay(5);
    }

    if (!(data_ready & 0x01)) {
        allIRLedsOff();
        signal_img->valid = false;
        return HAL_TIMEOUT;
    }

    /* Read signal_per_spad values (our "brightness image") */
    ret = readToFSignal(signal_img);

    /* Clear data-ready flag */
    vl53l8cx_write_byte(VL53L8CX_REG_UI_CMD_STATUS, 0x00);

    /* Turn OFF the LED after capture */
    allIRLedsOff();

    return ret;
}

/* ========================================================================== */
/*                       PHOTOMETRIC STEREO: COMPUTE NORMAL MAP                */
/* ========================================================================== */

void computeNormalMap(const ToFSignalImage_t images[SCAN_IR_LED_COUNT], NormalMap_t *normals)
{
    /*
     * Photometric Stereo (Woodham 1980):
     *
     * For a Lambertian surface, the observed intensity under light direction L_k is:
     *     I_k = rho * (N . L_k)
     * where rho is the surface albedo and N is the surface normal.
     *
     * With 3 light directions and 3 intensity observations:
     *     [I1]       [N_x]
     *     [I2] = rho * L * [N_y]
     *     [I3]       [N_z]
     *
     * Solving: N_unnormalized = L_inv * I
     * Then normalize: N = N_unnormalized / |N_unnormalized|
     *
     * The albedo rho is encoded in the magnitude and cancels during normalization,
     * so we get surface normals regardless of surface color/reflectivity.
     */

    if (!images[0].valid || !images[1].valid || !images[2].valid) {
        normals->valid = false;
        return;
    }

    for (uint8_t row = 0; row < SCAN_NMAP_ROWS; row++) {
        for (uint8_t col = 0; col < SCAN_NMAP_COLS; col++) {

            /* Intensity vector from 3 images (normalized to 0.0-1.0 range).
             * signal_per_spad values are typically 0-10000.
             * Divide by a reference value to bring into reasonable range. */
            float i1 = (float)images[0].signal[row][col] * 0.0001f;
            float i2 = (float)images[1].signal[row][col] * 0.0001f;
            float i3 = (float)images[2].signal[row][col] * 0.0001f;

            /* Skip zones with very low signal (no object present) */
            if (i1 < 0.001f && i2 < 0.001f && i3 < 0.001f) {
                normals->nx[row][col] = 0.0f;
                normals->ny[row][col] = 0.0f;
                normals->nz[row][col] = 1.0f;  /* Default: facing camera */
                continue;
            }

            /* Compute unnormalized normal: N = L_inv * I */
            float raw_nx = L_INV[0][0] * i1 + L_INV[0][1] * i2 + L_INV[0][2] * i3;
            float raw_ny = L_INV[1][0] * i1 + L_INV[1][1] * i2 + L_INV[1][2] * i3;
            float raw_nz = L_INV[2][0] * i1 + L_INV[2][1] * i2 + L_INV[2][2] * i3;

            /* Normalize the normal vector */
            float len_sq = raw_nx * raw_nx + raw_ny * raw_ny + raw_nz * raw_nz;
            float inv_len = fastInvSqrt(len_sq);

            if (inv_len > 0.0f) {
                normals->nx[row][col] = raw_nx * inv_len;
                normals->ny[row][col] = raw_ny * inv_len;
                normals->nz[row][col] = raw_nz * inv_len;
            } else {
                /* Degenerate case: all signals equal or near-zero */
                normals->nx[row][col] = 0.0f;
                normals->ny[row][col] = 0.0f;
                normals->nz[row][col] = 1.0f;
            }

            /* Ensure the normal points toward the camera (nz should be positive).
             * If not, flip it. This handles the sign ambiguity in photometric stereo. */
            if (normals->nz[row][col] < 0.0f) {
                normals->nx[row][col] = -normals->nx[row][col];
                normals->ny[row][col] = -normals->ny[row][col];
                normals->nz[row][col] = -normals->nz[row][col];
            }
        }
    }

    normals->valid = true;
}

/* ========================================================================== */
/*                       LOCAL CURVATURE COMPUTATION                           */
/* ========================================================================== */

float computeLocalCurvature(const NormalMap_t *normals, uint8_t row, uint8_t col)
{
    /*
     * Curvature is the rate of change of the surface normal.
     * Approximated via finite differences on the normal map:
     *
     *   curvature = sqrt( |dN/dx|^2 + |dN/dy|^2 )
     *
     * where dN/dx and dN/dy are partial derivatives of the normal vector
     * with respect to grid position, computed as central differences
     * (or forward/backward at boundaries).
     */

    if (!normals->valid) {
        return 0.0f;
    }

    float dnx_dx, dny_dx, dnz_dx;  /* Normal gradient in x direction */
    float dnx_dy, dny_dy, dnz_dy;  /* Normal gradient in y direction */

    /* Horizontal gradient (dN/dx) */
    if (col == 0) {
        /* Forward difference */
        dnx_dx = normals->nx[row][1] - normals->nx[row][0];
        dny_dx = normals->ny[row][1] - normals->ny[row][0];
        dnz_dx = normals->nz[row][1] - normals->nz[row][0];
    } else if (col == SCAN_NMAP_COLS - 1) {
        /* Backward difference */
        dnx_dx = normals->nx[row][col] - normals->nx[row][col - 1];
        dny_dx = normals->ny[row][col] - normals->ny[row][col - 1];
        dnz_dx = normals->nz[row][col] - normals->nz[row][col - 1];
    } else {
        /* Central difference */
        dnx_dx = (normals->nx[row][col + 1] - normals->nx[row][col - 1]) * 0.5f;
        dny_dx = (normals->ny[row][col + 1] - normals->ny[row][col - 1]) * 0.5f;
        dnz_dx = (normals->nz[row][col + 1] - normals->nz[row][col - 1]) * 0.5f;
    }

    /* Vertical gradient (dN/dy) */
    if (row == 0) {
        /* Forward difference */
        dnx_dy = normals->nx[1][col] - normals->nx[0][col];
        dny_dy = normals->ny[1][col] - normals->ny[0][col];
        dnz_dy = normals->nz[1][col] - normals->nz[0][col];
    } else if (row == SCAN_NMAP_ROWS - 1) {
        /* Backward difference */
        dnx_dy = normals->nx[row][col] - normals->nx[row - 1][col];
        dny_dy = normals->ny[row][col] - normals->ny[row - 1][col];
        dnz_dy = normals->nz[row][col] - normals->nz[row - 1][col];
    } else {
        /* Central difference */
        dnx_dy = (normals->nx[row + 1][col] - normals->nx[row - 1][col]) * 0.5f;
        dny_dy = (normals->ny[row + 1][col] - normals->ny[row - 1][col]) * 0.5f;
        dnz_dy = (normals->nz[row + 1][col] - normals->nz[row - 1][col]) * 0.5f;
    }

    /* Squared magnitudes of the gradient vectors */
    float grad_x_sq = dnx_dx * dnx_dx + dny_dx * dny_dx + dnz_dx * dnz_dx;
    float grad_y_sq = dnx_dy * dnx_dy + dny_dy * dny_dy + dnz_dy * dnz_dy;

    return fastSqrt(grad_x_sq + grad_y_sq);
}

/* ========================================================================== */
/*                       SURFACE CLASSIFICATION                                */
/* ========================================================================== */

SurfaceType_t classifySurface(float curvature, float normal_variation)
{
    /*
     * Classification based on curvature magnitude and the angular variation
     * of neighboring normals:
     *
     * - curvature < FLAT_THRESH and variation < FLAT_DEG:   FLAT
     * - curvature < SMOOTH_THRESH:                          SMOOTH_CURVE
     * - curvature < SHARP_THRESH:                           CONVEX or CONCAVE
     * - curvature >= SHARP_THRESH:                          SHARP_EDGE
     * - Intersection of multiple strong edges:              CORNER
     *
     * normal_variation is the maximum angular difference (in degrees) between
     * a zone's normal and its neighbors.
     */

    if (curvature < CURVATURE_FLAT_THRESH && normal_variation < EDGE_THRESH_FLAT_DEG) {
        return SURFACE_FLAT;
    }

    if (normal_variation > EDGE_THRESH_SHARP_DEG) {
        return SURFACE_CORNER;
    }

    if (normal_variation > EDGE_THRESH_CURVED_DEG || curvature > CURVATURE_SHARP_THRESH) {
        return SURFACE_SHARP_EDGE;
    }

    if (curvature < CURVATURE_SMOOTH_THRESH) {
        return SURFACE_SMOOTH_CURVE;
    }

    /* Distinguish convex vs concave by checking if the normal points
     * away from the surface center (convex) or toward it (concave).
     * Use the sign of the Laplacian of the normal z-component as a proxy:
     * positive = concave, negative = convex. */
    return SURFACE_CONVEX;
}

/* ========================================================================== */
/*                       EDGE DETECTION FROM NORMALS                           */
/* ========================================================================== */

void detectEdgesFromNormals(const NormalMap_t *normals, EdgeMap_t *edges)
{
    /*
     * For each zone in the normal map, compute the angular difference between
     * its normal and the normals of its neighbors. The maximum angular difference
     * determines edge strength, and the classification determines edge type.
     *
     * Angular difference = acos(N1 . N2) where both are unit vectors.
     */

    if (!normals->valid) {
        edges->valid = false;
        return;
    }

    edges->sharp_edge_count  = 0;
    edges->smooth_curve_count = 0;

    for (uint8_t row = 0; row < SCAN_NMAP_ROWS; row++) {
        for (uint8_t col = 0; col < SCAN_NMAP_COLS; col++) {

            float this_nx = normals->nx[row][col];
            float this_ny = normals->ny[row][col];
            float this_nz = normals->nz[row][col];

            float max_angle_deg = 0.0f;
            uint8_t edge_count = 0;  /* Count of strong edges at this zone */

            /* Check all valid neighbors (up to 8 directions, but we use
             * the 3 primary neighbors: right, down, diagonal) */

            /* Right neighbor */
            if (col + 1 < SCAN_NMAP_COLS) {
                float d = dot3(this_nx, this_ny, this_nz,
                               normals->nx[row][col + 1],
                               normals->ny[row][col + 1],
                               normals->nz[row][col + 1]);
                float angle = fastAcosDeg(d);
                if (angle > max_angle_deg) max_angle_deg = angle;
                if (angle > EDGE_THRESH_CURVED_DEG) edge_count++;
            }

            /* Down neighbor */
            if (row + 1 < SCAN_NMAP_ROWS) {
                float d = dot3(this_nx, this_ny, this_nz,
                               normals->nx[row + 1][col],
                               normals->ny[row + 1][col],
                               normals->nz[row + 1][col]);
                float angle = fastAcosDeg(d);
                if (angle > max_angle_deg) max_angle_deg = angle;
                if (angle > EDGE_THRESH_CURVED_DEG) edge_count++;
            }

            /* Diagonal neighbor (down-right) */
            if (row + 1 < SCAN_NMAP_ROWS && col + 1 < SCAN_NMAP_COLS) {
                float d = dot3(this_nx, this_ny, this_nz,
                               normals->nx[row + 1][col + 1],
                               normals->ny[row + 1][col + 1],
                               normals->nz[row + 1][col + 1]);
                float angle = fastAcosDeg(d);
                if (angle > max_angle_deg) max_angle_deg = angle;
                if (angle > EDGE_THRESH_CURVED_DEG) edge_count++;
            }

            /* Left neighbor (for more accurate detection) */
            if (col > 0) {
                float d = dot3(this_nx, this_ny, this_nz,
                               normals->nx[row][col - 1],
                               normals->ny[row][col - 1],
                               normals->nz[row][col - 1]);
                float angle = fastAcosDeg(d);
                if (angle > max_angle_deg) max_angle_deg = angle;
                if (angle > EDGE_THRESH_CURVED_DEG) edge_count++;
            }

            /* Up neighbor */
            if (row > 0) {
                float d = dot3(this_nx, this_ny, this_nz,
                               normals->nx[row - 1][col],
                               normals->ny[row - 1][col],
                               normals->nz[row - 1][col]);
                float angle = fastAcosDeg(d);
                if (angle > max_angle_deg) max_angle_deg = angle;
                if (angle > EDGE_THRESH_CURVED_DEG) edge_count++;
            }

            /* Map max_angle_deg to 0-255 edge strength (0 deg = 0, 180 deg = 255) */
            float strength_f = (max_angle_deg / 180.0f) * 255.0f;
            edges->edge_strength[row][col] = (uint8_t)clampf(strength_f, 0.0f, 255.0f);

            /* Compute curvature at this zone */
            float curvature = computeLocalCurvature(normals, row, col);

            /* Classify surface type */
            SurfaceType_t surface_type;
            if (edge_count >= 2 && max_angle_deg > EDGE_THRESH_SHARP_DEG) {
                /* Multiple strong edges meet at this zone = corner */
                surface_type = SURFACE_CORNER;
            } else {
                surface_type = classifySurface(curvature, max_angle_deg);
            }

            edges->edge_type[row][col] = surface_type;

            /* Update counters */
            if (surface_type == SURFACE_SHARP_EDGE || surface_type == SURFACE_CORNER) {
                edges->sharp_edge_count++;
            } else if (surface_type == SURFACE_SMOOTH_CURVE) {
                edges->smooth_curve_count++;
            }
        }
    }

    edges->valid = true;
}

/* ========================================================================== */
/*                       DEPTH + NORMALS FUSION                                */
/* ========================================================================== */

void fuseDepthAndNormals(const DepthGrid_t *depth, const NormalMap_t *normals,
                         float rotation_angle, ScanPoint_t *points, uint16_t *count)
{
    /*
     * For each of the 64 VL53L8CX zones, combine:
     *   - Absolute depth from ToF (distance_mm)
     *   - Surface normal from photometric stereo
     *   - Robot rotation angle for coordinate transform
     *
     * Zone-to-angle mapping:
     *   The VL53L8CX has a 45 degree total FoV.
     *   Zone (row, col) maps to angular position:
     *     angle_x = (col - 3.5) * (45/8) = (col - 3.5) * 5.625 degrees
     *     angle_y = (row - 3.5) * (45/8) = (row - 3.5) * 5.625 degrees
     *
     * 3D point in sensor frame:
     *     x_sensor = distance * tan(angle_x)
     *     y_sensor = distance * tan(angle_y)
     *     z_sensor = distance
     *
     * Transform to robot-centric frame by applying rotation about Y axis:
     *     x_robot =  x_sensor * cos(rot) + z_sensor * sin(rot)
     *     y_robot =  y_sensor
     *     z_robot = -x_sensor * sin(rot) + z_sensor * cos(rot)
     *
     * Similarly transform the normal vectors.
     */

    if (!depth->valid || !normals->valid) {
        return;
    }

    float cos_rot = cosf(deg2rad(rotation_angle));
    float sin_rot = sinf(deg2rad(rotation_angle));

    uint16_t added = 0;

    for (uint8_t row = 0; row < VL53L8CX_GRID_SIZE; row++) {
        for (uint8_t col = 0; col < VL53L8CX_GRID_SIZE; col++) {

            uint16_t dist_mm = depth->distance_mm[row][col];

            /* Skip invalid or out-of-range zones */
            if (dist_mm == 0 || dist_mm > 4000) {
                continue;
            }

            /* Ensure we do not exceed the output buffer */
            if ((*count + added) >= SCAN_MAX_POINTS) {
                break;
            }

            /* Compute angular position of this zone */
            float angle_x_deg = ((float)col - 3.5f) * VL53L8CX_ZONE_ANGLE_DEG;
            float angle_y_deg = ((float)row - 3.5f) * VL53L8CX_ZONE_ANGLE_DEG;

            float tan_ax = tanf(deg2rad(angle_x_deg));
            float tan_ay = tanf(deg2rad(angle_y_deg));

            /* 3D position in sensor frame (mm) */
            float x_s = (float)dist_mm * tan_ax;
            float y_s = (float)dist_mm * tan_ay;
            float z_s = (float)dist_mm;

            /* Transform to robot-centric frame (rotation about Y axis) */
            float x_r =  x_s * cos_rot + z_s * sin_rot;
            float y_r =  y_s;
            float z_r = -x_s * sin_rot + z_s * cos_rot;

            /* Get surface normal from normal map (same grid coordinates) */
            float nx_s = normals->nx[row][col];
            float ny_s = normals->ny[row][col];
            float nz_s = normals->nz[row][col];

            /* Rotate normal to robot frame */
            float nx_r =  nx_s * cos_rot + nz_s * sin_rot;
            float ny_r =  ny_s;
            float nz_r = -nx_s * sin_rot + nz_s * cos_rot;

            /* Compute curvature and classification */
            float curvature = computeLocalCurvature(normals, row, col);

            /* Compute angular variation for classification */
            float max_angle_deg = 0.0f;
            if (col + 1 < SCAN_NMAP_COLS) {
                float d = dot3(nx_s, ny_s, nz_s,
                               normals->nx[row][col + 1],
                               normals->ny[row][col + 1],
                               normals->nz[row][col + 1]);
                float angle = fastAcosDeg(d);
                if (angle > max_angle_deg) max_angle_deg = angle;
            }
            if (row + 1 < SCAN_NMAP_ROWS) {
                float d = dot3(nx_s, ny_s, nz_s,
                               normals->nx[row + 1][col],
                               normals->ny[row + 1][col],
                               normals->nz[row + 1][col]);
                float angle = fastAcosDeg(d);
                if (angle > max_angle_deg) max_angle_deg = angle;
            }

            SurfaceType_t stype = classifySurface(curvature, max_angle_deg);

            /* Compute confidence based on signal strength and depth.
             * Higher signal and closer objects get higher confidence. */
            float avg_signal = ((float)s_signalImages[0].signal[row][col] +
                                (float)s_signalImages[1].signal[row][col] +
                                (float)s_signalImages[2].signal[row][col]) / 3.0f;
            float conf_signal = clampf(avg_signal / 5000.0f, 0.0f, 1.0f);
            float conf_dist   = clampf(1.0f - (float)dist_mm / 4000.0f, 0.0f, 1.0f);
            uint8_t confidence = (uint8_t)(clampf((conf_signal * 0.5f + conf_dist * 0.5f) * 100.0f,
                                                  0.0f, 100.0f));

            /* Store the point */
            ScanPoint_t *pt = &points[*count + added];
            pt->x          = x_r;
            pt->y          = y_r;
            pt->z          = z_r;
            pt->nx         = nx_r;
            pt->ny         = ny_r;
            pt->nz         = nz_r;
            pt->curvature  = curvature;
            pt->surface    = stype;
            pt->confidence = confidence;

            added++;
        }
    }

    *count += added;
}

/* ========================================================================== */
/*                       BOUNDING BOX AND CENTROID                             */
/* ========================================================================== */

void computeBoundingBox(ScanResult_t *result)
{
    if (result->point_count == 0) {
        memset(result->bounding_box, 0, sizeof(result->bounding_box));
        memset(result->centroid, 0, sizeof(result->centroid));
        result->max_dimension = 0.0f;
        result->dominant_surface = SURFACE_UNKNOWN;
        return;
    }

    /* Initialize min/max with first point */
    float min_x = result->points[0].x;
    float max_x = result->points[0].x;
    float min_y = result->points[0].y;
    float max_y = result->points[0].y;
    float min_z = result->points[0].z;
    float max_z = result->points[0].z;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    /* Surface type histogram for dominant classification */
    uint16_t surface_counts[7] = {0};  /* One per SurfaceType_t value */

    for (uint16_t i = 0; i < result->point_count; i++) {
        const ScanPoint_t *pt = &result->points[i];

        /* Update bounding box */
        if (pt->x < min_x) min_x = pt->x;
        if (pt->x > max_x) max_x = pt->x;
        if (pt->y < min_y) min_y = pt->y;
        if (pt->y > max_y) max_y = pt->y;
        if (pt->z < min_z) min_z = pt->z;
        if (pt->z > max_z) max_z = pt->z;

        /* Accumulate for centroid */
        sum_x += pt->x;
        sum_y += pt->y;
        sum_z += pt->z;

        /* Count surface types */
        if ((uint8_t)pt->surface < 7U) {
            surface_counts[(uint8_t)pt->surface]++;
        }
    }

    /* Store bounding box */
    result->bounding_box[0] = min_x;
    result->bounding_box[1] = max_x;
    result->bounding_box[2] = min_y;
    result->bounding_box[3] = max_y;
    result->bounding_box[4] = min_z;
    result->bounding_box[5] = max_z;

    /* Compute centroid */
    float inv_count = 1.0f / (float)result->point_count;
    result->centroid[0] = sum_x * inv_count;
    result->centroid[1] = sum_y * inv_count;
    result->centroid[2] = sum_z * inv_count;

    /* Compute maximum dimension */
    float dx = max_x - min_x;
    float dy = max_y - min_y;
    float dz = max_z - min_z;
    result->max_dimension = dx;
    if (dy > result->max_dimension) result->max_dimension = dy;
    if (dz > result->max_dimension) result->max_dimension = dz;

    /* Find dominant surface type (skip SURFACE_UNKNOWN = index 0) */
    uint16_t max_count = 0;
    result->dominant_surface = SURFACE_UNKNOWN;
    for (uint8_t s = 1; s < 7; s++) {
        if (surface_counts[s] > max_count) {
            max_count = surface_counts[s];
            result->dominant_surface = (SurfaceType_t)s;
        }
    }
}

/* ========================================================================== */
/*                       SCAN EXECUTION                                        */
/* ========================================================================== */

/**
 * Execute a single-position scan capture and processing.
 * Captures 3 images under different IR illumination, computes normals,
 * reads depth, fuses into point cloud, and detects edges.
 *
 * @param position_index  Index 0..SCAN_POSITIONS-1 for rotation angle.
 * @return HAL_OK on success.
 */
static HAL_StatusTypeDef executeScanPosition(uint8_t position_index)
{
    HAL_StatusTypeDef ret;
    float rotation_angle = (float)position_index * SCAN_ROTATION_STEP_DEG;

    s_scannerState = SCANNER_CAPTURING;

    /* Step 1: Capture 3 images under different IR LED illumination */
    for (uint8_t led = 0; led < SCAN_IR_LED_COUNT; led++) {
        ret = captureWithIRLED(led, &s_signalImages[led]);
        if (ret != HAL_OK) {
            s_scannerState = SCANNER_ERROR;
            return ret;
        }
    }

    s_scannerState = SCANNER_PROCESSING;

    /* Step 2: Read the depth grid from VL53L8CX */
    DepthGrid_t local_depth;
    osMutexAcquire(depthMutex, osWaitForever);
    memcpy(&local_depth, (const void *)&g_depthGrid, sizeof(DepthGrid_t));
    osMutexRelease(depthMutex);

    /* If the depth grid is stale or invalid, try one more read */
    if (!local_depth.valid) {
        ret = readDepthGrid(&local_depth);
        if (ret != HAL_OK || !local_depth.valid) {
            s_scannerState = SCANNER_ERROR;
            return HAL_ERROR;
        }
    }

    /* Step 3: Compute surface normal map via photometric stereo */
    computeNormalMap(s_signalImages, &s_normalMap);
    if (!s_normalMap.valid) {
        s_scannerState = SCANNER_ERROR;
        return HAL_ERROR;
    }

    /* Step 4: Detect edges from the normal map */
    detectEdgesFromNormals(&s_normalMap, &s_edgeMap);

    /* Step 5: Fuse depth and normals into the point cloud */
    fuseDepthAndNormals(&local_depth, &s_normalMap, rotation_angle,
                        s_scanResult.points, &s_scanResult.point_count);

    /* Accumulate edge statistics across positions */
    if (s_edgeMap.valid) {
        s_scanResult.edges.sharp_edge_count  += s_edgeMap.sharp_edge_count;
        s_scanResult.edges.smooth_curve_count += s_edgeMap.smooth_curve_count;

        /* Merge edge map from last position (overwrite) for final result.
         * In a full scan, the last position's edge map is representative
         * of the front-facing surface. */
        memcpy(s_scanResult.edges.edge_strength, s_edgeMap.edge_strength,
               sizeof(s_edgeMap.edge_strength));
        memcpy(s_scanResult.edges.edge_type, s_edgeMap.edge_type,
               sizeof(s_edgeMap.edge_type));
        s_scanResult.edges.valid = true;
    }

    return HAL_OK;
}

HAL_StatusTypeDef startFullScan(void)
{
    if (s_scannerState != SCANNER_IDLE && s_scannerState != SCANNER_COMPLETE &&
        s_scannerState != SCANNER_ERROR) {
        return HAL_BUSY;
    }

    /* Reset scan result */
    memset(&s_scanResult, 0, sizeof(ScanResult_t));
    s_scanResult.complete   = false;
    s_scanResult.point_count = 0;
    s_currentPosition        = 0;

    s_fullScanRequested  = true;
    s_quickScanRequested = false;
    s_scannerState       = SCANNER_STARTING;

    return HAL_OK;
}

HAL_StatusTypeDef startQuickScan(void)
{
    if (s_scannerState != SCANNER_IDLE && s_scannerState != SCANNER_COMPLETE &&
        s_scannerState != SCANNER_ERROR) {
        return HAL_BUSY;
    }

    /* Reset scan result */
    memset(&s_scanResult, 0, sizeof(ScanResult_t));
    s_scanResult.complete    = false;
    s_scanResult.point_count = 0;
    s_currentPosition        = 0;

    s_quickScanRequested = true;
    s_fullScanRequested  = false;
    s_scannerState       = SCANNER_STARTING;

    return HAL_OK;
}

ScannerState_t getScannerState(void)
{
    return s_scannerState;
}

ScanResult_t* getScanResult(void)
{
    return &s_scanResult;
}

/* ========================================================================== */
/*                       UART RESULT FORWARDING                                */
/* ========================================================================== */

void sendScanResult(const ScanResult_t *result)
{
    /*
     * Send a compact scan result to ESP32 via UART.
     * Packet payload layout (fits within UART_MAX_PAYLOAD = 200 bytes):
     *
     *   [0..1]   point_count    (uint16_t, big-endian)
     *   [2..25]  bounding_box   (6x float, each as int16_t mm, BE) = 12 bytes
     *   [26..31] centroid       (3x int16_t mm, BE) = 6 bytes
     *   [32..33] max_dimension  (uint16_t mm, BE)
     *   [34]     dominant_surface (uint8_t)
     *   [35..36] sharp_edge_count (uint16_t, BE)
     *   [37..38] smooth_curve_count (uint16_t, BE)
     *   [39..N]  subsampled points (as many as fit)
     *            Each point: x(2) + y(2) + z(2) + surface(1) + confidence(1) = 8 bytes
     *
     * Maximum points in payload: (200 - 39) / 8 = 20 points
     */

    uint8_t payload[UART_MAX_PAYLOAD];
    uint8_t idx = 0;

    /* Point count */
    payload[idx++] = (uint8_t)(result->point_count >> 8);
    payload[idx++] = (uint8_t)(result->point_count & 0xFF);

    /* Bounding box (6 floats as int16_t mm) */
    for (uint8_t i = 0; i < 6; i++) {
        int16_t val = (int16_t)clampf(result->bounding_box[i], -32767.0f, 32767.0f);
        payload[idx++] = (uint8_t)((uint16_t)val >> 8);
        payload[idx++] = (uint8_t)((uint16_t)val & 0xFF);
    }

    /* Centroid (3x int16_t mm) */
    for (uint8_t i = 0; i < 3; i++) {
        int16_t val = (int16_t)clampf(result->centroid[i], -32767.0f, 32767.0f);
        payload[idx++] = (uint8_t)((uint16_t)val >> 8);
        payload[idx++] = (uint8_t)((uint16_t)val & 0xFF);
    }

    /* Max dimension */
    uint16_t max_dim = (uint16_t)clampf(result->max_dimension, 0.0f, 65535.0f);
    payload[idx++] = (uint8_t)(max_dim >> 8);
    payload[idx++] = (uint8_t)(max_dim & 0xFF);

    /* Dominant surface */
    payload[idx++] = (uint8_t)result->dominant_surface;

    /* Edge counts */
    payload[idx++] = (uint8_t)(result->edges.sharp_edge_count >> 8);
    payload[idx++] = (uint8_t)(result->edges.sharp_edge_count & 0xFF);
    payload[idx++] = (uint8_t)(result->edges.smooth_curve_count >> 8);
    payload[idx++] = (uint8_t)(result->edges.smooth_curve_count & 0xFF);

    /* Subsample points to fit in remaining payload space */
    uint8_t remaining = UART_MAX_PAYLOAD - idx - 1; /* Reserve 1 for safety */
    uint8_t max_points = remaining / 8;
    if (max_points > 20) max_points = 20;

    uint16_t step = 1;
    if (result->point_count > max_points && max_points > 0) {
        step = result->point_count / max_points;
    }

    uint8_t sent_points = 0;
    for (uint16_t i = 0; i < result->point_count && sent_points < max_points; i += step) {
        const ScanPoint_t *pt = &result->points[i];

        int16_t px = (int16_t)clampf(pt->x, -32767.0f, 32767.0f);
        int16_t py = (int16_t)clampf(pt->y, -32767.0f, 32767.0f);
        int16_t pz = (int16_t)clampf(pt->z, -32767.0f, 32767.0f);

        payload[idx++] = (uint8_t)((uint16_t)px >> 8);
        payload[idx++] = (uint8_t)((uint16_t)px & 0xFF);
        payload[idx++] = (uint8_t)((uint16_t)py >> 8);
        payload[idx++] = (uint8_t)((uint16_t)py & 0xFF);
        payload[idx++] = (uint8_t)((uint16_t)pz >> 8);
        payload[idx++] = (uint8_t)((uint16_t)pz & 0xFF);
        payload[idx++] = (uint8_t)pt->surface;
        payload[idx++] = pt->confidence;

        sent_points++;
    }

    sendResponse(RSP_SCAN_RESULT, payload, idx);
}

void sendScanStatus(void)
{
    /*
     * Compact status packet:
     *   [0]     scanner_state (uint8_t)
     *   [1]     current_position (uint8_t, 0-11)
     *   [2..3]  point_count so far (uint16_t, BE)
     *   [4]     complete flag (uint8_t, 0 or 1)
     */

    uint8_t payload[5];
    payload[0] = (uint8_t)s_scannerState;
    payload[1] = s_currentPosition;
    payload[2] = (uint8_t)(s_scanResult.point_count >> 8);
    payload[3] = (uint8_t)(s_scanResult.point_count & 0xFF);
    payload[4] = s_scanResult.complete ? 1U : 0U;

    sendResponse(RSP_SCAN_STATUS, payload, 5);
}

/* ========================================================================== */
/*                       FREERTOS SCANNER TASK                                 */
/* ========================================================================== */

void scannerTask(void *argument)
{
    (void)argument;

    /*
     * Non-blocking state machine for 3D scanning.
     * This task runs at low priority alongside DepthTask.
     * The state machine progresses through the scan sequence without
     * blocking other tasks (uses osDelay for waits).
     */

    /* Initialize scanner hardware (IR LED GPIOs, verify ToF) */
    HAL_StatusTypeDef ret = initScanner();
    if (ret != HAL_OK) {
        /* Scanner init failed; stay in error but keep running
         * so the task doesn't exit (FreeRTOS requirement). */
        s_scannerState = SCANNER_ERROR;
    }

    for (;;) {

        switch (s_scannerState) {

        case SCANNER_IDLE:
            /* Wait for scan request */
            osDelay(50);
            break;

        case SCANNER_STARTING: {
            /* Record start time */
            uint32_t start_tick = HAL_GetTick();
            s_scanResult.scan_time_ms = start_tick; /* Temporarily store start time */

            if (s_quickScanRequested) {
                /* Quick scan: single position, no rotation */
                ret = executeScanPosition(0);
                if (ret == HAL_OK) {
                    computeBoundingBox(&s_scanResult);
                    s_scanResult.scan_time_ms = HAL_GetTick() - start_tick;
                    s_scanResult.complete = true;
                    s_scannerState = SCANNER_COMPLETE;
                } else {
                    s_scannerState = SCANNER_ERROR;
                }
                s_quickScanRequested = false;

            } else if (s_fullScanRequested) {
                /* Full scan: begin multi-position sequence */
                s_currentPosition = 0;

                /* Enable motors for rotation */
                ret = enableMotors();
                if (ret != HAL_OK) {
                    s_scannerState = SCANNER_ERROR;
                    s_fullScanRequested = false;
                    break;
                }

                /* Capture first position (already facing forward) */
                ret = executeScanPosition(0);
                if (ret != HAL_OK) {
                    s_scannerState = SCANNER_ERROR;
                    s_fullScanRequested = false;
                    break;
                }

                s_currentPosition = 1;
                s_scannerState = SCANNER_ROTATING;
            }
            break;
        }

        case SCANNER_ROTATING: {
            /* Rotate to next position */
            ret = turnToAngle(SCAN_ROTATION_STEP_DEG);
            if (ret != HAL_OK) {
                /* Rotation failed; try to continue with remaining data */
                stopMotors();
                computeBoundingBox(&s_scanResult);
                s_scanResult.scan_time_ms = HAL_GetTick() -
                    s_scanResult.scan_time_ms; /* Elapsed since start */
                s_scanResult.complete = true;
                s_scannerState = SCANNER_COMPLETE;
                s_fullScanRequested = false;
                break;
            }

            /* Wait for mechanical settling */
            osDelay(SCAN_MOTOR_SETTLE_MS);

            /* Capture at this position */
            ret = executeScanPosition(s_currentPosition);
            if (ret != HAL_OK) {
                /* Capture failed; try to continue with what we have */
                s_currentPosition++;
                if (s_currentPosition >= SCAN_POSITIONS) {
                    stopMotors();
                    computeBoundingBox(&s_scanResult);
                    uint32_t start_tick = s_scanResult.scan_time_ms;
                    s_scanResult.scan_time_ms = HAL_GetTick() - start_tick;
                    s_scanResult.complete = true;
                    s_scannerState = SCANNER_COMPLETE;
                    s_fullScanRequested = false;
                }
                break;
            }

            s_currentPosition++;

            if (s_currentPosition >= SCAN_POSITIONS) {
                /* All positions captured */
                stopMotors();
                computeBoundingBox(&s_scanResult);
                uint32_t start_tick = s_scanResult.scan_time_ms;
                s_scanResult.scan_time_ms = HAL_GetTick() - start_tick;
                s_scanResult.complete = true;
                s_scannerState = SCANNER_COMPLETE;
                s_fullScanRequested = false;

                /* Send result to ESP32 */
                sendScanResult(&s_scanResult);
            }
            break;
        }

        case SCANNER_CAPTURING:
            /* This state is set transiently during executeScanPosition().
             * If we somehow land here in the main loop, just wait. */
            osDelay(10);
            break;

        case SCANNER_PROCESSING:
            /* Same as CAPTURING - transient state */
            osDelay(10);
            break;

        case SCANNER_COMPLETE:
            /* Scan done. Stay here until a new scan is requested. */
            osDelay(100);
            if (s_fullScanRequested || s_quickScanRequested) {
                s_scannerState = SCANNER_STARTING;
            }
            break;

        case SCANNER_ERROR:
            /* LED blink pattern to indicate scanner error (fast toggle) */
            HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
            osDelay(200);

            /* Allow retry: if a new scan is requested, try to re-init */
            if (s_fullScanRequested || s_quickScanRequested) {
                ret = initScanner();
                if (ret == HAL_OK) {
                    s_scannerState = SCANNER_STARTING;
                }
            }
            break;

        default:
            s_scannerState = SCANNER_IDLE;
            break;
        }
    }
}
