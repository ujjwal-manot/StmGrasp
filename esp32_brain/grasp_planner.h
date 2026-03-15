#ifndef GRASP_PLANNER_H
#define GRASP_PLANNER_H

#include "config.h"
#include "ds_fusion.h"
#include "success_predictor.h"

void initGraspPlanner();

// Compute grasp plan using DS fusion result and sensor data
GraspPlan computeGraspPlan(const DSResult& ds,
                           const CurvatureResult& curv,
                           const DepthGrid& depth);

// Compute live grasp quality during hold
float computeGraspQuality(const ForceResult& force, const ImpedanceResult& imp,
                          const GraspPlan& plan);

// Run the 12-state grasp FSM
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

// Select best grasp strategy based on material and geometry
GraspStrategy selectStrategy(uint8_t material, uint8_t geometry, float flatness);

// Get the strategy name
const char* getStrategyName(GraspStrategy s);

#endif // GRASP_PLANNER_H
