/**
 * Safety Systems
 * Watchdog, force limits, communication timeout, emergency stop.
 */

#ifndef SAFETY_H
#define SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void initSafety(void);
void checkSafetyLimits(void);
void emergencyStop(void);
void kickWatchdog(void);
void setForceLimit(float max_force_n);
void updateForceReading(float force_n);
void recordHeartbeat(void);
bool isEstopActive(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_H */
