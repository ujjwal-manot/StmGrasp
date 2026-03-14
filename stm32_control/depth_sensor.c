/**
 * VL53L8CX 8x8 Multizone Time-of-Flight Sensor Driver
 *
 * Simplified functional wrapper for the VL53L8CX. The full ST ULD driver
 * is a large codebase; this implementation uses the same register-level
 * I2C protocol but keeps only the essentials: init, configure 8x8 at 5Hz,
 * read distance data, and extract object profiles.
 *
 * The VL53L8CX uses 16-bit register addresses. HAL_I2C_Mem_Write/Read
 * with I2C_MEMADD_SIZE_16BIT handles this directly.
 *
 * Reference: ST VL53L8CX ULD driver source (STSW-IMG040).
 */

#include "depth_sensor.h"
#include "uart_protocol.h"

static ToFState_t s_tofState = TOF_STATE_UNINIT;

/* Internal buffer for raw ranging results */
static uint8_t s_resultBuf[VL53L8CX_RESULT_BUF_SIZE];

/* ========================================================================== */
/*  Low-level I2C register access (16-bit addresses)                           */
/* ========================================================================== */

HAL_StatusTypeDef vl53l8cx_write_byte(uint16_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, VL53L8CX_DEFAULT_ADDR, reg,
                             I2C_MEMADD_SIZE_16BIT, &value, 1, 100);
}

HAL_StatusTypeDef vl53l8cx_write_multi(uint16_t reg, const uint8_t *data, uint16_t len)
{
    /* HAL expects non-const pointer, cast is safe for write direction */
    return HAL_I2C_Mem_Write(&hi2c1, VL53L8CX_DEFAULT_ADDR, reg,
                             I2C_MEMADD_SIZE_16BIT, (uint8_t *)data, len, 100);
}

HAL_StatusTypeDef vl53l8cx_read_byte(uint16_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c1, VL53L8CX_DEFAULT_ADDR, reg,
                            I2C_MEMADD_SIZE_16BIT, value, 1, 100);
}

HAL_StatusTypeDef vl53l8cx_read_multi(uint16_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, VL53L8CX_DEFAULT_ADDR, reg,
                            I2C_MEMADD_SIZE_16BIT, data, len, 100);
}

/* ========================================================================== */
/*  Poll for data-ready or command completion                                  */
/* ========================================================================== */

HAL_StatusTypeDef vl53l8cx_poll_ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t status = 0xFF;

    while ((HAL_GetTick() - start) < timeout_ms) {
        HAL_StatusTypeDef ret = vl53l8cx_read_byte(0x0000, &status);
        if (ret != HAL_OK) return ret;
        if (status == 0x00) {
            return HAL_OK; /* Device is ready / command accepted */
        }
        osDelay(5);
    }

    return HAL_TIMEOUT;
}

/* ========================================================================== */
/*  Write a DCI (Device Configuration Interface) command block                 */
/*                                                                             */
/*  The VL53L8CX uses a command-response protocol via specific register banks. */
/*  Configuration changes are written to the DCI area, then a "go" command is  */
/*  issued and completion is polled.                                           */
/* ========================================================================== */

static HAL_StatusTypeDef vl53l8cx_dci_write(uint16_t index, const uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef ret;

    /* Write the data to the command buffer area */
    ret = vl53l8cx_write_multi(index, data, len);
    if (ret != HAL_OK) return ret;

    /* Issue the write-config command */
    uint8_t cmd = 0x01;
    ret = vl53l8cx_write_byte(VL53L8CX_REG_UI_CMD_START, cmd);
    if (ret != HAL_OK) return ret;

    /* Wait for command to be accepted */
    uint32_t start = HAL_GetTick();
    uint8_t status;
    while ((HAL_GetTick() - start) < 500) {
        ret = vl53l8cx_read_byte(VL53L8CX_REG_UI_CMD_STATUS, &status);
        if (ret != HAL_OK) return ret;
        if (status == 0x00 || status == 0x03) {
            return HAL_OK;
        }
        osDelay(2);
    }

    return HAL_TIMEOUT;
}

/* ========================================================================== */
/*  Sensor initialization                                                      */
/* ========================================================================== */

HAL_StatusTypeDef initVL53L8CX(void)
{
    HAL_StatusTypeDef ret;
    uint8_t tmp;

    s_tofState = TOF_STATE_UNINIT;

    /* Verify the sensor is present by reading device ID */
    ret = vl53l8cx_read_byte(VL53L8CX_REG_DEVICE_ID, &tmp);
    if (ret != HAL_OK) {
        s_tofState = TOF_STATE_ERROR;
        return ret;
    }

    if (tmp != VL53L8CX_DEVICE_ID) {
        s_tofState = TOF_STATE_ERROR;
        return HAL_ERROR;
    }

    /* Wait for sensor boot (poll system status) */
    ret = vl53l8cx_poll_ready(5000);
    if (ret != HAL_OK) {
        s_tofState = TOF_STATE_ERROR;
        return ret;
    }

    /* Software reset: write 0x00 then 0x01 to firmware start register */
    ret = vl53l8cx_write_byte(VL53L8CX_REG_FIRMWARE_START, 0x00);
    if (ret != HAL_OK) { s_tofState = TOF_STATE_ERROR; return ret; }

    osDelay(100);

    ret = vl53l8cx_write_byte(VL53L8CX_REG_FIRMWARE_START, 0x01);
    if (ret != HAL_OK) { s_tofState = TOF_STATE_ERROR; return ret; }

    /* Wait for firmware boot */
    osDelay(200);
    ret = vl53l8cx_poll_ready(5000);
    if (ret != HAL_OK) { s_tofState = TOF_STATE_ERROR; return ret; }

    /* ---- Configure resolution: 8x8 (64 zones) ---- */
    uint8_t resolution_cfg[4] = {
        (uint8_t)(VL53L8CX_RESOLUTION_8X8),
        0x00, 0x00, 0x00
    };
    ret = vl53l8cx_dci_write(VL53L8CX_DCI_RESOLUTION, resolution_cfg, 4);
    if (ret != HAL_OK) { s_tofState = TOF_STATE_ERROR; return ret; }

    /* ---- Configure ranging frequency: 5 Hz ---- */
    uint8_t freq_cfg[4] = { 5, 0x00, 0x00, 0x00 };
    ret = vl53l8cx_dci_write(VL53L8CX_DCI_FREQ, freq_cfg, 4);
    if (ret != HAL_OK) { s_tofState = TOF_STATE_ERROR; return ret; }

    /* ---- Configure integration time: 20 ms ---- */
    uint8_t int_time_cfg[4] = {
        (uint8_t)(20 & 0xFF),
        (uint8_t)((20 >> 8) & 0xFF),
        0x00, 0x00
    };
    ret = vl53l8cx_dci_write(VL53L8CX_DCI_INT_TIME, int_time_cfg, 4);
    if (ret != HAL_OK) { s_tofState = TOF_STATE_ERROR; return ret; }

    s_tofState = TOF_STATE_READY;

    /* ---- Start ranging ---- */
    uint8_t start_cmd = 0x01;
    ret = vl53l8cx_write_byte(VL53L8CX_REG_CMD_START, start_cmd);
    if (ret != HAL_OK) { s_tofState = TOF_STATE_ERROR; return ret; }

    /* Wait for ranging to begin */
    osDelay(50);

    uint8_t cmd_status;
    ret = vl53l8cx_read_byte(VL53L8CX_REG_CMD_END, &cmd_status);
    if (ret != HAL_OK) { s_tofState = TOF_STATE_ERROR; return ret; }

    s_tofState = TOF_STATE_RANGING;
    return HAL_OK;
}

/* ========================================================================== */
/*  Read 8x8 depth grid                                                       */
/*                                                                             */
/*  The VL53L8CX outputs results in a structured buffer. Distance values for   */
/*  each zone are 16-bit little-endian at a known offset from the result base. */
/* ========================================================================== */

HAL_StatusTypeDef readDepthGrid(DepthGrid_t *grid)
{
    if (s_tofState != TOF_STATE_RANGING) {
        grid->valid = false;
        return HAL_ERROR;
    }

    HAL_StatusTypeDef ret;

    /* Check if new data is available */
    uint8_t data_ready = 0;
    ret = vl53l8cx_read_byte(VL53L8CX_REG_UI_CMD_STATUS, &data_ready);
    if (ret != HAL_OK) {
        grid->valid = false;
        return ret;
    }

    /* Bit 0 set = new data available */
    if ((data_ready & 0x01) == 0) {
        /* No new data yet; keep previous grid valid state */
        return HAL_OK;
    }

    /* Read the entire result buffer from the distance result register area */
    ret = vl53l8cx_read_multi(VL53L8CX_RESULT_DISTANCE_MM,
                              s_resultBuf,
                              VL53L8CX_ZONE_COUNT * 4); /* 4 bytes per zone: distance(2) + status(1) + pad(1) */
    if (ret != HAL_OK) {
        grid->valid = false;
        return ret;
    }

    /* Parse distance values: each zone has a 4-byte record.
     * Bytes [0:1] = distance in mm (int16_t, little-endian).
     * Bytes [2] = target status.
     * Bytes [3] = padding / signal quality.
     * Caller is responsible for mutex protection of the grid pointer. */
    for (uint8_t zone = 0; zone < VL53L8CX_ZONE_COUNT; zone++) {
        uint16_t offset = zone * 4;
        int16_t raw_dist = (int16_t)((uint16_t)s_resultBuf[offset] |
                                     ((uint16_t)s_resultBuf[offset + 1] << 8));

        uint8_t row = zone / VL53L8CX_GRID_SIZE;
        uint8_t col = zone % VL53L8CX_GRID_SIZE;

        grid->distance_mm[row][col] = (raw_dist > 0) ? (uint16_t)raw_dist : 0;
    }

    grid->timestamp = HAL_GetTick();
    grid->valid = true;

    /* Clear the data-ready flag by writing 0x00 */
    vl53l8cx_write_byte(VL53L8CX_REG_UI_CMD_STATUS, 0x00);

    return HAL_OK;
}

/* ========================================================================== */
/*  Object profile extraction from depth grid                                  */
/* ========================================================================== */

void getObjectProfile(const DepthGrid_t *grid, uint16_t threshold_mm, ObjectProfile_t *obj)
{
    if (!grid->valid) {
        memset(obj, 0, sizeof(ObjectProfile_t));
        return;
    }

    float sum_x = 0.0f, sum_y = 0.0f;
    uint16_t count = 0;
    uint16_t min_dist = 0xFFFF;
    uint8_t min_row = 0xFF, max_row = 0, min_col = 0xFF, max_col = 0;

    for (uint8_t row = 0; row < VL53L8CX_GRID_SIZE; row++) {
        for (uint8_t col = 0; col < VL53L8CX_GRID_SIZE; col++) {
            uint16_t d = grid->distance_mm[row][col];
            if (d > 0 && d < threshold_mm) {
                sum_x += (float)col;
                sum_y += (float)row;
                count++;

                if (d < min_dist) min_dist = d;
                if (row < min_row) min_row = row;
                if (row > max_row) max_row = row;
                if (col < min_col) min_col = col;
                if (col > max_col) max_col = col;
            }
        }
    }

    if (count == 0) {
        memset(obj, 0, sizeof(ObjectProfile_t));
        return;
    }

    obj->center_x   = sum_x / (float)count;
    obj->center_y   = sum_y / (float)count;
    obj->width      = (float)(max_col - min_col + 1);
    obj->height     = (float)(max_row - min_row + 1);
    obj->min_dist   = min_dist;
    obj->zone_count = count;
}

/* ========================================================================== */
/*  Forward depth data to ESP32                                                */
/* ========================================================================== */

void forwardDepthData(const DepthGrid_t *grid)
{
    if (!grid->valid) return;
    sendDepthGrid(grid);
}
