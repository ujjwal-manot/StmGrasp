/**
 * Servo/Gripper Control
 * PWM-based servo control with trapezoidal velocity profiling.
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void  initServos(void);
void  setServoAngle(uint8_t channel, float angle);
void  moveServoSmooth(uint8_t channel, float target_angle, uint8_t ms_per_deg);
void  openGripper(void);
void  closeGripper(void);
void  setGripForce(float target_force_n);
void  doTapMotion(void);
float getServoPosition(uint8_t channel);
void  disableAllServos(void);
void  setServoSpeed(uint8_t channel, uint8_t ms_per_deg);

/* Internal helpers */
uint32_t angleToCCR(float angle);
void     servoTaskProcess(void);  /* Called from ServoTask loop */

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONTROL_H */
