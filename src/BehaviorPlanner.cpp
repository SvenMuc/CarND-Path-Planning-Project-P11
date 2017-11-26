//
//  BehaviorPlanner.cpp
//  path_planning
//
//  Created by Sven Bone on 19.11.17.
//

#include "BehaviorPlanner.hpp"
#include <iostream>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

BehaviorPlanner::BehaviorPlanner() {
  current_state_ = BehaviorState::kInitialization;
  current_lane_ = 0;
  target_lane_ = 0;
  fastest_lane_ = 0;
  ref_velocity_ = 0;
  time_gap_ = kDefaultTimaGab_;
  sensor_fusion_ = NULL;
  
  for (int lane=0; lane < sensor_fusion_->number_lanes_; ++lane) {
    weigthedCost_.push_back(0.0);
    costSpeedLimit_.push_back(0.0);
    costHostVelocityCloseToReferenceVelocity_.push_back(0.0);
  }
}

BehaviorPlanner::BehaviorPlanner(SensorFusion* sensor_fusion, const double ref_velocity, const int start_lane) {
  current_state_ = BehaviorState::kKeepLane;
  current_lane_ = start_lane;
  target_lane_ = start_lane;
  ref_velocity_ = ref_velocity;
  time_gap_ = kDefaultTimaGab_;
  sensor_fusion_ = sensor_fusion;
  
  for (int lane=0; lane < sensor_fusion_->number_lanes_; ++lane) {
    weigthedCost_.push_back(0.0);
    costSpeedLimit_.push_back(0.0);
    costHostVelocityCloseToReferenceVelocity_.push_back(0.0);
  }
}

string BehaviorPlanner::GetStateAsString(BehaviorState state) const {
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

BehaviorState BehaviorPlanner::Update() {
  current_lane_ = sensor_fusion_->host_vehicle_.lane_;
  fastest_lane_ = sensor_fusion_->GetReachableFastestLane();
  CalculateWeightedCost();
  
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
    case BehaviorState::kPrepareLaneChangeRight:
      current_state_ = StatePrepareLaneChangeRight();
      break;
    case BehaviorState::kLaneChangeRight:
      current_state_ = StateLaneChangeRight();
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
    double delta_x = sensor_fusion_->host_vehicle_.x_ - next_vehicle_in_current_lane->x_;
    double delta_y = sensor_fusion_->host_vehicle_.y_ - next_vehicle_in_current_lane->y_;
    double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
    time_gap_ = dist / next_vehicle_in_current_lane->v_;
  } else {
    time_gap_ = kDefaultTimaGab_;
  }
  
  if (time_gap_ < kMinTimeGap_) {
    // close vehicle ahead, prepare lane change to fastest lane
    if (fastest_lane_ < current_lane_) {
      target_lane_ = fastest_lane_;
      return BehaviorState::kLaneChangeLeft;
    } else if (fastest_lane_ > current_lane_) {
      target_lane_ = fastest_lane_;
      return  BehaviorState::kLaneChangeRight;
    } else {
      target_lane_ = current_lane_;
      return BehaviorState::kKeepLane;
    }
  } else {
    // free driving, keep lane
    return BehaviorState::kKeepLane;
  }
}

BehaviorState BehaviorPlanner::StatePrepareLaneChangeLeft() {
  return BehaviorState::kPrepareLaneChangeLeft;
}

BehaviorState BehaviorPlanner::StateLaneChangeLeft() {
  double d_left_lane_marker = (target_lane_ + 1) * sensor_fusion_->kLaneWidth_;
  double d_right_host_vehicle = sensor_fusion_->host_vehicle_.d_ + (sensor_fusion_->host_vehicle_.width_ / 2.0);
  
  if (d_right_host_vehicle < d_left_lane_marker) {
    return BehaviorState::kKeepLane;
  } else {
    return BehaviorState::kLaneChangeLeft;
  }
}

BehaviorState BehaviorPlanner::StatePrepareLaneChangeRight() {
    return BehaviorState::kPrepareLaneChangeRight;
}

BehaviorState BehaviorPlanner::StateLaneChangeRight() {
  double d_right_lane_marker = target_lane_ * sensor_fusion_->kLaneWidth_;
  double d_left_host_vehicle = sensor_fusion_->host_vehicle_.d_ - (sensor_fusion_->host_vehicle_.width_ / 2.0);
  
  if (d_left_host_vehicle > d_right_lane_marker) {
    return BehaviorState::kKeepLane;
  } else {
    return BehaviorState::kLaneChangeRight;
  }
}

void BehaviorPlanner::CalculateWeightedCost() {
  double sum_weights = kWeightSpeedLimit_ + kWeightHostVelocityCloseToReferenceVelocity_;
  
  for (int lane=0; lane < sensor_fusion_->number_lanes_; ++lane) {
    costSpeedLimit_[lane] = CostSpeedLimit(lane);
    costHostVelocityCloseToReferenceVelocity_[lane] = CostHostVelocityCloseToReferenceVelocity(lane);
    
    weigthedCost_[lane] = (kWeightSpeedLimit_ * costSpeedLimit_[lane] +
                           kWeightHostVelocityCloseToReferenceVelocity_ * costHostVelocityCloseToReferenceVelocity_[lane]) / sum_weights;
  }
}

/********** SAFETY COSTS FUNCTIONS **********/

/********** LEGALITY COSTS FUNCTIONS **********/

double BehaviorPlanner::CostSpeedLimit(int lane) {
  return double((sensor_fusion_->host_vehicle_.v_ <=  sensor_fusion_->GetSpeedLimitForLane(lane)));
}

/********** COMFORT COSTS FUNCTIONS **********/

/********** EFFICIENCY COSTS FUNCTIONS **********/

double BehaviorPlanner::CostHostVelocityCloseToReferenceVelocity(int lane) {
  double speed_limit = sensor_fusion_->GetSpeedLimitForLane(lane);
  double target_speed = speed_limit - kBufferToSpeedLimit_;
  double v = sensor_fusion_->host_vehicle_.v_;
  
  if (v < target_speed) {
    return kZeroVelocityCost_ * (target_speed - v) / target_speed;
  } else if (v > target_speed && v <= speed_limit) {
    return (v - target_speed) / kBufferToSpeedLimit_;
  } else if (v > speed_limit) {
    return 1.0;
  }
  
  return 1.0;
}

std::ostream& operator<< (std::ostream& os, const BehaviorPlanner& obj) {
  os << "BehaviorPlanner(" << std::endl <<
  " time_gap=             " << obj.time_gap_ << " (<threshold= " << (obj.time_gap_ < obj.kMinTimeGap_) << ")" << std::endl <<
  " fastest_lane=         " << obj.fastest_lane_ << " (" << (obj.sensor_fusion_->average_lane_velocities_ / 0.44704).transpose() << ") mph" << std::endl <<
  " current_lane=         " << obj.current_lane_ << std::endl <<
  " target_lane=          " << obj.target_lane_ << std::endl <<
  " current_state=        " << static_cast<int>(obj.current_state_) << " - " << obj.GetStateAsString(obj.current_state_) << std::endl;
  
  for (int lane=0; lane < obj.sensor_fusion_->number_lanes_; ++lane) {
    os <<
    " Lane[" << lane << "] WeightedCost=                             " << obj.weigthedCost_[lane] << std::endl; // <<
    //" Lane[" << lane << "] CostSpeedLimit=                           " << obj.costSpeedLimit_[lane] << std::endl <<
    //" Lane[" << lane << "] CostHostVelocityCloseToReferenceVelocity= " << obj.costHostVelocityCloseToReferenceVelocity_[lane] << std::endl;
  }
  os << ")" << std::endl;
  
  return os;
}
