#ifndef GRASP_PLANNER_H
#define GRASP_PLANNER_H

#include "config.h"

void initGraspPlanner();
GraspPlan computeGraspPlan(const ImpedanceResult& imp, const AcousticResult& aco,
                           const CurvatureResult& curv, const DepthGrid& depth,
                           uint8_t fused_material, float fused_confidence);
float computeGraspQuality(const ForceResult& force, const ImpedanceResult& imp,
                          const GraspPlan& plan);
GraspState runStateMachine(GraspState current,
                           const ForceResult& force,
                           const ImpedanceResult& imp,
                           const CurvatureResult& curv,
                           const DepthGrid& depth,
                           const GraspPlan& plan,
                           float grasp_quality);
GraspState getCurrentState();
const char* getStateName(GraspState s);
uint32_t getStateElapsed();
void requestStateTransition(GraspState target);
void triggerEstop();

#endif // GRASP_PLANNER_H
