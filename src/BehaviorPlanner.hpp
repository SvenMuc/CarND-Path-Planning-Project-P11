//
//  BehaviorPlanner.hpp
//  path_planning
//
//  Created by Sven Bone on 19.11.17.
//

#ifndef BehaviorPlanner_hpp
#define BehaviorPlanner_hpp

#include <stdio.h>
#include <vector>
#include "SensorFusion.hpp"

using namespace std;

/**
 Supported FSM states of the behavior planner.
 */
enum class BehaviorState {
  kInitialization = 0,
  kKeepLane,
  kPrepareLaneChangeLeft,
  kLaneChangeLeft,
  kPrepareLaneChangeRight,
  kLaneChangeRight
};

// Sensor fusion index constants [id, x, y, vx, vy, s, d]
const int kSensorFusionIndexId = 0;
const int kSensorFusionIndexX  = 1;
const int kSensorFusionIndexY  = 2;
const int kSensorFusionIndexVx = 3;
const int kSensorFusionIndexVy = 4;
const int kSensorFusionIndexS  = 5;
const int kSensorFusionIndexD  = 6;

/**
 BehaviorPlanner class
 
 The behavior planner suggests the next feasible, safe, legal and most efficient
 maneuver (state) like e.g. keep lane, prepare for left lane change, change the lane to the
 left. It does not consider collision avoidance or any execution details. This is
 typically the task of the trajectory planner.
 */
class BehaviorPlanner {
public:
  BehaviorState current_state_;             // current FSM state (behavior)
  int current_lane_;                        // current lane ID
  double ref_velocity_;                     // reference velocity to target [m/s]
  SensorFusion* sensor_fusion_;             // list of vehicle models based on sensor fusion

  /**
   Default constructor
   */
  BehaviorPlanner();
  
  /**
   Constructor initializes the first state (behavior).
   
   @param sensor_fusion  Pointer to the sensor fusion instance including all vehicle models.
   @param ref_velocity   Reference velocity [m/s].
   @param start_lane     ID of current lane [0=left, 1=center, 2=right].
   */
  BehaviorPlanner(SensorFusion* sensor_fusion, double velocity, const int start_lane = 1);
  
  /**
   Plans the next behavior.
   
   @return Returns the next state (behavior).
   */
  BehaviorState NextBehavior();
  
  /**
   Returns the behavior state as a user readable string.
   
   @return Returns the behavior state as string.
   */
  string GetStateAsString(BehaviorState state);
  
  /**
   Returns the target lane ID for the current behavior (state).
   
   @return Returns the target lane [0-left, 1-center, 2-right].
   */
  int GetTargetLane();
  
private:
  /**
   State Keep Lane: Vehicle keeps the lane as long the reference velocity could be kept.
   
   @return Returns the next state (behavior).
   */
  BehaviorState StateKeepLane();
  
  /**
   State Prepare Lane Change Left: Vehicle prepares for left lane change (e.g. adjusting speed, waiting for a safe gap,...).
   
   @return Returns the next state (behavior).
   */
  BehaviorState StatePrepareLaneChangeLeft();
  
  /**
   State Lane Change Left: Vehicle performs a left lane change.
   
   @return Returns the next state (behavior).
   */

  BehaviorState StateLaneChangeLeft();
};

#endif /* BehaviorPlanner_hpp */
