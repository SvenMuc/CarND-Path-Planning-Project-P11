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
  SensorFusion* sensor_fusion_;                       // list of vehicle models based on sensor fusion
  BehaviorState current_state_;                       // current FSM state (behavior)
  int current_lane_;                                  // current lane ID
  int target_lane_;                                   // target lane ID
  int fastest_lane_;                                  // fastest lane ID
  VehicleModel* next_vehicle_current_lane_;           // next vehicle driving ahead in host lane
  VehicleModel* front_vehicle_target_lane_;           // next vehicle driving ahead in target lane
  VehicleModel* rear_vehicle_target_lane_;            // next vehicle driving behind in target lane
  double target_velocity_;                            // target velocity (selected vehicle) [m/s]
  double d_current_lane_front_;                       // distance to vehicle ahead [m]
  double d_target_lane_front_;                        // distance to vehicle ahead in target lane [m]
  double d_target_lane_rear_;                         // distance to vehicle behind in target lane [m]
  double d_predicted_current_lane_front_;             // predicted distance to vehicle ahead [m]
  double d_predicted_target_lane_front_;              // predicted distance to vehicle ahead in target lane [m]
  double d_predicted_target_lane_rear_;               // predicted distance to vehicle behind in target lane [m]
  double time_gap_current_lane_front_;                // time gap to vehicle ahead [s]
  double time_gap_target_lane_front_;                 // time gap to vehicle ahead in target lane [s]
  double time_gap_target_lane_rear_;                  // time gap to vehicle behind in target lane [s]
  double time_gap_predicted_current_lane_front_;      // predicted time gap to vehicle ahead [s]
  double time_gap_predicted_target_lane_front_;       // predicted time gap to vehicle ahead in target lane [s]
  double time_gap_predicted_target_lane_rear_;        // predicted time gap to vehicle behind in target lane [s]
  double ttc_current_lane_front_;                     // time-to-collision to vehicle ahead [s]
  double ttc_target_lane_front_;                      // time-to-collision to vehicle ahead in target lane [s]
  double ttc_target_lane_rear_;                       // time-to-collision to vehicle behind in target lane [s]
  double ttc_predicted_current_lane_front_;           // predicted time-to-collision to vehicle ahead [s]
  double ttc_predicted_target_lane_front_;            // predicted time-to-collision to vehicle ahead in target lane [s]
  double ttc_predicted_target_lane_rear_;             // predicted time-to-collision to vehicle behind in target lane [s]

  /**
   Constructor initializes the first state (behavior).
   
   @param sensor_fusion  Pointer to the sensor fusion instance including all vehicle models.
   @param ref_velocity   Reference velocity [m/s].
   @param start_lane     ID of current lane [0=left, 1=center, 2=right].
   */
  BehaviorPlanner(SensorFusion* sensor_fusion, const double ref_velocity, const int start_lane = 1);
  
  /**
   Plans the next behavior.
   
   @return Returns the next state (behavior).
   */
  BehaviorState Update();
  
  /**
   Returns the behavior state as a user readable string.
   
   @return Returns the behavior state as string.
   */
  string GetStateAsString(BehaviorState state) const;
  
  /**
   Returns the target lane ID for the current behavior (state).
   
   @return Returns the target lane [0-left, 1-center, 2-right].
   */
  int GetTargetLane();
  
  /**
   Overload standard output stream.
   */
  friend std::ostream& operator<< (std::ostream& os, const BehaviorPlanner& obj);
  
private:
  const double kLowerTimeGap_ = 1.5;                 // min allowed time gap to vehicle ahead [s]
  const double kUpperTimeGap_ = 3.5;                 // time gap [s] to drive with speed limit
  const double kMinTimeGapInitLaneChange_ = 3.0;     // min allowed time gap to intiate a lane change [s]
  const double kMinTimeGapLaneChange_ = 1.0;         // min required time gap to vehicle in target lane [s]
  const double kMinDistanceFrontLaneChange_ = 10.0;  // min required distance to vehicle ahead in target lane [m]
  const double kMinDistanceRearLaneChange_ = 10.0;   // min required distance to vehicle behind in target lane [m]
  const double kMinTTCFrontLaneChange_ = 6.0;        // min required time-to-collision to vehicle ahead in target lane [s]
  const double kMinTTCRearLaneChange_ = 6.0;         // min required time-to-collision to vehicle behind in target lane [s]
  const double kFastestLaneFactor = 0.08;            // the fastest lane velocity need to  be x% faster than the current [%]

  const double kDefaultTimaGab_ = 9999.9;            // default resp. max time gap [s]
  const double kDefaultTTC_ = 9999.9;                // default resp. max time-to-collision [s]
  const double kDefaultDistance_ = 9999.9;           // default resp. max distance to vehicles [m]
  
  /**
   Calculates distance between two vehicles.
   
   @param host_x    x map coordinate of host vehicle [m].
   @param host_y    y map coordinate of host vehicle [m].
   @param target_x  x map coordinate of target vehicle [m].
   @param target_y  y map coordinate of target vehicle [m].
   
   @return Returns the time-to-collision between given vehilces [s].
   */
  double CalculateDistance(const double host_x, const double host_y,  const double target_x, const double target_y);

  /**
   Calculates time gap between two vehicles.
   
   @param host_v    Velocity of host vehicle [m/s].
   @param host_x    x map coordinate of host vehicle [m].
   @param host_y    y map coordinate of host vehicle [m].
   @param target_x  x map coordinate of target vehicle [m].
   @param target_y  y map coordinate of target vehicle [m].
   
   @return Returns the time gap between given vehilces [s].
   */
  double CalculateTimeGap(const double host_v, const double host_x, const double host_y, const double target_x, const double target_y);
  
  /**
   Calculates time-to-collsion between two vehicles.
   
   @param host_v    Velocity of host vehicle [m/s].
   @param host_x    x map coordinate of host vehicle [m].
   @param host_y    y map coordinate of host vehicle [m].
   @param target_v  Velocity of target vehicle [m/s].
   @param target_x  x map coordinate of target vehicle [m].
   @param target_y  y map coordinate of target vehicle [m].
   
   @return Returns the time-to-collision between given vehilces [s].
   */
  double CalculateTTC(const double host_v, const double host_x, const double host_y, const double target_v,  const double target_x, const double target_y);

  /**
   Calculates the safety time gaps, TTC, and distances for the current
   and target lane.
   */
  void CalculateSafetyMeassures();
  
  /**
   Checks whether the target lane is safe to drive or not.
   
   @return Returns true if the lane change is safe.
   */
  bool isTargetLaneSafe();

  /********** FINITE STATE MACHINE **********/
  
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

  /**
   State Prepare Lane Change Right: Vehicle prepares for right lane change (e.g. adjusting speed, waiting for a safe gap,...).
   
   @return Returns the next state (behavior).
   */
  BehaviorState StatePrepareLaneChangeRight();
  
  /**
   State Lane Change Right: Vehicle performs a right lane change.
   
   @return Returns the next state (behavior).
   */
  
  BehaviorState StateLaneChangeRight();
};

#endif /* BehaviorPlanner_hpp */
