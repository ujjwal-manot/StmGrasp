#ifndef ACOUSTIC_H
#define ACOUSTIC_H

#include "config.h"

void initAcoustic();
bool waitForTap(uint32_t timeout_ms);
void recordTapResponse(float* sample_buf, int num_samples);
AcousticResult analyzeTap();
uint8_t classifyTapMaterial(float dominant_freq, float spectral_centroid,
                            float decay_ratio, float* out_confidence);
uint8_t crossValidateMaterial(const ImpedanceResult& imp, const AcousticResult& aco,
                              float* out_confidence);

#endif // ACOUSTIC_H
