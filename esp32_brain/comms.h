#ifndef COMMS_H
#define COMMS_H

#include "config.h"

void initComms();

void sendSTM32Command(uint8_t cmd, const uint8_t* data, uint8_t len);
void sendGripOpen();
void sendGripClose();
void sendSetForce(float force_N);
void sendTapCommand();
void sendServoMove(uint8_t servo_id, uint16_t position_us);
void sendEstop();
void sendRequestDepth();
void sendRequestIMUTap();
void sendRequestMicTap();

bool processUARTData();
bool hasNewDepthGrid();
bool hasNewPosition();
DepthGrid getLatestDepthGrid();
uint8_t getSTM32Status();
uint8_t getSTM32Error();

struct IMUTapResult {
    float peak_accel;
    float vibration_rms;
    float dominant_freq;
    bool valid;
};

struct MicTapResult {
    float dominant_freq;
    float spectral_centroid;
    float decay_ratio;
    bool valid;
};

bool hasNewIMUTap();
bool hasNewMicTap();
IMUTapResult getLatestIMUTap();
MicTapResult getLatestMicTap();

#endif
