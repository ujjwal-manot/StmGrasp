#include "acoustic.h"

// Piezo removed from BOM. All acoustic tap analysis now runs on the
// STM32 onboard mic (mic_sensor.c) and IMU (imu_sensor.c).
// Results arrive via UART as RSP_MIC_TAP_DATA and RSP_IMU_TAP_DATA.
// This module is now a stub — initAcoustic() is a no-op.

void initAcoustic() {
    // No local acoustic hardware. Tap data comes from STM32 over UART.
}
