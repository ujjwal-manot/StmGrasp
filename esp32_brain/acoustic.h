#ifndef ACOUSTIC_H
#define ACOUSTIC_H

#include "config.h"

void initAcoustic();
bool waitForTap(uint32_t timeout_ms);
AcousticResult analyzeTapLocal();

#endif
