#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "config.h"

void setupWiFiAP();
void setupWebServer();
void pushSensorData(const ImpedanceResult& imp,
                    const AcousticResult& aco,
                    const ForceResult& force,
                    const CurvatureResult& curv,
                    const DepthGrid& depth,
                    const GraspPlan& plan,
                    GraspState state,
                    float grasp_quality,
                    uint8_t fused_material,
                    float fused_confidence);
void cleanupWebClients();

// Callback type for dashboard commands
typedef void (*WebCommandCallback)(const char* command);
void setWebCommandCallback(WebCommandCallback cb);

#endif // WEB_SERVER_H
