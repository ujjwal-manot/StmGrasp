#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

void initSensors();
ForceResult readForces();
bool detectSlip(float* out_max_rms);

#endif
