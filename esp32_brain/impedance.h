#ifndef IMPEDANCE_H
#define IMPEDANCE_H

#include "config.h"

void initImpedance();
void calibrateBaseline();
ImpedanceResult measureImpedance();
uint8_t classifyMaterial(float magnitude, float phase_deg, float* out_confidence);

#endif // IMPEDANCE_H
