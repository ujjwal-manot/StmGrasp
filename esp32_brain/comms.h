#ifndef COMMS_H
#define COMMS_H

#include "config.h"

void initComms();

// STM32 commands
void sendSTM32Command(uint8_t cmd, const uint8_t* data, uint8_t len);
void sendGripOpen();
void sendGripClose();
void sendSetForce(float force_N);
void sendTapCommand();
void sendServoMove(uint8_t servo_id, uint16_t position_us);
void sendEstop();
void sendRequestDepth();

// STM32 response parsing
bool processUARTData();
bool hasNewDepthGrid();
bool hasNewPosition();
DepthGrid getLatestDepthGrid();
uint8_t getSTM32Status();
uint8_t getSTM32Error();

// NodeMCU LED commands
void sendLEDSolid(uint8_t r, uint8_t g, uint8_t b);
void sendLEDAnimate(uint8_t pattern, uint8_t speed);
void sendLEDPixel(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
void sendLEDMaterial(uint8_t material_idx);
void sendLEDHeartbeat(uint8_t bpm);
void sendLEDForceMap(uint8_t f1, uint8_t f2, uint8_t f3);

#endif // COMMS_H
