#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "config.h"
#include "ds_fusion.h"

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
                    const DSResult& ds);
void cleanupWebClients();

typedef void (*WebCommandCallback)(const char* command);
void setWebCommandCallback(WebCommandCallback cb);

#endif
