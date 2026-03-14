/**
 * VL53L8CX 8x8 Multizone ToF Sensor Driver
 * Simplified wrapper around ST ULD register-level protocol.
 */

#ifndef DEPTH_SENSOR_H
#define DEPTH_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Simplified VL53L8CX register map (from ULD driver documentation) */
#define VL53L8CX_REG_DEVICE_ID          0x0000U
#define VL53L8CX_REG_REVISION_ID        0x0001U
#define VL53L8CX_REG_SYSTEM_STATUS      0x0006U
#define VL53L8CX_REG_PAD_I2C_HV_CONFIG  0x002DU
#define VL53L8CX_REG_FIRMWARE_START      0x0086U
#define VL53L8CX_REG_CMD_START          0x0100U
#define VL53L8CX_REG_CMD_END           0x0101U
#define VL53L8CX_REG_UI_CMD_STATUS     0x0400U
#define VL53L8CX_REG_UI_CMD_START      0x0401U
#define VL53L8CX_REG_UI_CMD_END        0x0402U

/* Configuration commands */
#define VL53L8CX_RESOLUTION_4X4         16U
#define VL53L8CX_RESOLUTION_8X8         64U
#define VL53L8CX_STATUS_OK              0x00U
#define VL53L8CX_STATUS_TIMEOUT_ERROR   0x01U

/* Expected device ID */
#define VL53L8CX_DEVICE_ID              0xF0U
#define VL53L8CX_REVISION_ID            0x02U

/* Data output registers (simplified offsets into result buffer) */
#define VL53L8CX_START_BH               0x0000U
#define VL53L8CX_METADATA_BH            0x0010U
#define VL53L8CX_RESULT_DISTANCE_MM     0x0100U
#define VL53L8CX_RESULT_STATUS          0x0200U
#define VL53L8CX_RESULT_SIGNAL          0x0300U
#define VL53L8CX_RESULT_NB_TARGET       0x0080U

/* Result buffer size */
#define VL53L8CX_RESULT_BUF_SIZE        1024U

/* Ranging configuration command IDs */
#define VL53L8CX_DCI_FREQ               0x5458U
#define VL53L8CX_DCI_RANGING_MODE       0xAD30U
#define VL53L8CX_DCI_DSS_CONFIG         0xAD38U
#define VL53L8CX_DCI_RESOLUTION         0x5450U
#define VL53L8CX_DCI_INT_TIME           0x545CU

/* Sensor state */
typedef enum {
    TOF_STATE_UNINIT,
    TOF_STATE_READY,
    TOF_STATE_RANGING,
    TOF_STATE_ERROR,
} ToFState_t;

HAL_StatusTypeDef initVL53L8CX(void);
HAL_StatusTypeDef readDepthGrid(DepthGrid_t *grid);
void              getObjectProfile(const DepthGrid_t *grid, uint16_t threshold_mm, ObjectProfile_t *obj);
void              forwardDepthData(const DepthGrid_t *grid);

/* Low-level I2C helpers */
HAL_StatusTypeDef vl53l8cx_write_byte(uint16_t reg, uint8_t value);
HAL_StatusTypeDef vl53l8cx_write_multi(uint16_t reg, const uint8_t *data, uint16_t len);
HAL_StatusTypeDef vl53l8cx_read_byte(uint16_t reg, uint8_t *value);
HAL_StatusTypeDef vl53l8cx_read_multi(uint16_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef vl53l8cx_poll_ready(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* DEPTH_SENSOR_H */
