//
//  BehaviorPlanner.cpp
//  path_planning
//
//  Created by Sven Bone on 19.11.17.
//

#include "BehaviorPlanner.hpp"
#include <iostream>
#include <math.h>

using namespace std;

BehaviorPlanner::BehaviorPlanner() {
  current_state_ = BehaviorState::kInitialization;
  current_lane_ = 0;
  ref_velocity_ = 0;
  sensor_fusion_ = NULL;
}

BehaviorPlanner::BehaviorPlanner(SensorFusion* sensor_fusion, double ref_velocity, const int start_lane) {
  current_state_ = BehaviorState::kKeepLane;
  current_lane_ = start_lane;
  ref_velocity_ = ref_velocity;
  sensor_fusion_ = sensor_fusion;
}

string BehaviorPlanner::GetStateAsString(BehaviorState state) {
  switch (state) {
    case BehaviorState::kInitialization:
      return "kInitialization";
    case BehaviorState::kKeepLane:
      return "kKeepLane";
    case BehaviorState::kPrepareLaneChangeLeft:
      return "kPrepareLaneChangeLeft";
    case BehaviorState::kLaneChangeLeft:
      return "kLaneChangeLeft";
    case BehaviorState::kPrepareLaneChangeRight:
      return "kPrepareLaneChangeRight";
    case BehaviorState::kLaneChangeRight:
      return "kLaneChangeRight";
    default:
      return "Invalid state";
  }
}

int BehaviorPlanner::GetTargetLane() {
  switch (current_state_) {
    case BehaviorState::kInitialization:
    case BehaviorState::kKeepLane:
      return current_lane_;
    case BehaviorState::kPrepareLaneChangeLeft:
    case BehaviorState::kLaneChangeLeft:
      return max(current_lane_ - 1, 0);
    case BehaviorState::kPrepareLaneChangeRight:
    case BehaviorState::kLaneChangeRight:
      return min(current_lane_ + 1, 2);
    default:
      return current_lane_;
  }
}

BehaviorState BehaviorPlanner::NextBehavior() {
  switch (current_state_) {
    case BehaviorState::kKeepLane:
      current_state_ = StateKeepLane();
      break;
    case BehaviorState::kPrepareLaneChangeLeft:
      current_state_ = StatePrepareLaneChangeLeft();
      break;
    case BehaviorState::kLaneChangeLeft:
      current_state_ = StateLaneChangeLeft();
      break;
    default:
      cout << "Error: Invalid behavior state." << endl;
      break;
  }
  
  return current_state_;
}

BehaviorState BehaviorPlanner::StateKeepLane() {
  VehicleModel* next_vehicle_in_current_lane = sensor_fusion_->GetNextVehicleDrivingAhead(current_lane_);
  
  if (next_vehicle_in_current_lane) {
    // found vehicle driving ahead, check time_gap
    double delta_x = next_vehicle_in_current_lane->x_ - sensor_fusion_->host_vehicle_.x_;
    double delta_y = next_vehicle_in_current_lane->y_ - sensor_fusion_->host_vehicle_.y_;
    double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
    double time_gap = dist / next_vehicle_in_current_lane->v_;
    
    double delta_s = next_vehicle_in_current_lane->s_ - sensor_fusion_->host_vehicle_.s_;
    double time_gap_s = delta_s / next_vehicle_in_current_lane->v_;

    cout << "delta_x=    " << delta_x << " m" << endl;
    cout << "delta_y=    " << delta_y << " m" << endl;
    cout << "dist=       " << dist << " m" << endl;
    cout << "time_gap=   " << time_gap << " s" << endl;
    cout << "delta_s=    " << delta_s << " m" << endl;
    cout << "time_gap_s= " << time_gap_s << " s" << endl;
    
    if (time_gap < 2.0) {
      return BehaviorState::kLaneChangeLeft;
    } else {
      return BehaviorState::kKeepLane;
    }
  } else {
    // free driving, no vehicle ahead
    return BehaviorState::kKeepLane;
  }
}

BehaviorState BehaviorPlanner::StatePrepareLaneChangeLeft() {
  return BehaviorState::kPrepareLaneChangeLeft;
}

BehaviorState BehaviorPlanner::StateLaneChangeLeft() {
  return BehaviorState::kLaneChangeLeft;
}
