#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

void initSensors();
bool initADS1115();
int16_t readADS1115Channel(uint8_t channel);
CurvatureResult readIRCurvatureArray();
ForceResult readForces();
bool detectSlip();

#endif // SENSORS_H
