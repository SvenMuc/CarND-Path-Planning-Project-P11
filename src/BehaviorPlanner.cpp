//
//  BehaviorPlanner.cpp
//  path_planning
//
//  Created by Sven Bone on 19.11.17.
//

#include "BehaviorPlanner.hpp"
#include <iostream>
#include <iomanip>
#include <math.h>
#include "Utils.hpp"

using namespace std;

BehaviorPlanner::BehaviorPlanner(SensorFusion* sensor_fusion, const double ref_velocity, const int start_lane) {
  sensor_fusion_ = sensor_fusion;
  current_state_ = BehaviorState::kKeepLane;
  current_lane_ = start_lane;
  target_lane_ = start_lane;
  target_velocity_ = ref_velocity;
  next_vehicle_current_lane_ = NULL;
  front_vehicle_target_lane_ = NULL;
  rear_vehicle_target_lane_ = NULL;
  d_predicted_current_lane_front_ = kDefaultDistance_;
  d_predicted_target_lane_front_ = kDefaultDistance_;
  d_predicted_target_lane_rear_ = kDefaultDistance_;
  time_gap_current_lane_front_ = kDefaultTimaGab_;
  time_gap_target_lane_front_ = kDefaultTimaGab_;
  time_gap_target_lane_rear_ = kDefaultTimaGab_;
  time_gap_predicted_current_lane_front_ = kDefaultTimaGab_;
  time_gap_predicted_target_lane_front_ = kDefaultTimaGab_;
  time_gap_predicted_target_lane_rear_ = kDefaultTimaGab_;
  ttc_current_lane_front_ = kDefaultTTC_;
  ttc_target_lane_front_ = kDefaultTTC_;
  ttc_target_lane_rear_ = kDefaultTTC_;
  ttc_predicted_current_lane_front_ = kDefaultTTC_;
  ttc_predicted_target_lane_front_ = kDefaultTTC_;
  ttc_predicted_target_lane_rear_ = kDefaultTTC_;
}

BehaviorState BehaviorPlanner::Update() {
  current_lane_ = sensor_fusion_->host_vehicle_.lane_;
  fastest_lane_ = sensor_fusion_->GetReachableFastestLane(kFastestLaneFactor);
  
  CalculateSafetyMeassures();
  
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

BehaviorState BehaviorPlanner::StateKeepLane() {

  double speed_limit_current_lane = sensor_fusion_->GetSpeedLimitForCurrentLane();
  double host_velocity = sensor_fusion_->host_vehicle_.v_;
  
  // set target velocity
  if (time_gap_current_lane_front_ <= kLowerTimeGap_) {
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else if (time_gap_current_lane_front_ >= kUpperTimeGap_) {
    target_velocity_ = speed_limit_current_lane;
  }

  // determine if left or right lane is free
  bool is_left_lane_free;
  int left_lane = max(0, current_lane_ - 1);
  
  if (left_lane == current_lane_ || sensor_fusion_->lane_occupancy_[current_lane_] == 0) {
    is_left_lane_free = false;
  } else {
    is_left_lane_free = (sensor_fusion_->lane_occupancy_[left_lane] == 0);
  }

  bool is_right_lane_free;
  int right_lane = min(sensor_fusion_->number_lanes_-1, current_lane_ + 1);
  
  if (right_lane == current_lane_ || sensor_fusion_->lane_occupancy_[current_lane_] == 0) {
    is_right_lane_free = false;
  } else {
    is_right_lane_free = (sensor_fusion_->lane_occupancy_[right_lane] == 0);
  }
  
  is_left_lane_free = false;
  is_right_lane_free = false;

  // determine next behavior state
  // close vehicle ahead, prepare lane change to fastest lane
  if (host_velocity < speed_limit_current_lane &&
      time_gap_current_lane_front_ <= kMinTimeGapInitLaneChange_ &&
      /*sensor_fusion_->average_lane_velocities_[current_lane_] < speed_limit_current_lane &&*/
      (fastest_lane_ < current_lane_ || is_left_lane_free)) {
    target_lane_ = fastest_lane_;
    return BehaviorState::kPrepareLaneChangeLeft;
  } else if (host_velocity < speed_limit_current_lane &&
             time_gap_current_lane_front_ <= kMinTimeGapInitLaneChange_ &&
             /*sensor_fusion_->average_lane_velocities_[current_lane_] &&*/
             (fastest_lane_ > current_lane_ || is_right_lane_free)) {
    target_lane_ = fastest_lane_;
    return  BehaviorState::kPrepareLaneChangeRight;
  } else {
    // free driving, keep lane
    target_lane_ = current_lane_;
    return BehaviorState::kKeepLane;
  }
}

bool BehaviorPlanner::isTargetLaneSafe() {
  if (/*time_gap_target_lane_front_ >= kMinTimeGapLaneChange_ &&*/
      time_gap_target_lane_rear_ >= kMinTimeGapLaneChange_ &&
      d_target_lane_front_ >= kMinDistanceFrontLaneChange_ &&
      d_target_lane_rear_ >= kMinDistanceRearLaneChange_ &&
      ttc_target_lane_front_ >= kMinTTCFrontLaneChange_ &&
      ttc_target_lane_rear_ >= kMinTTCRearLaneChange_) {
    return true;
  } else {
    return false;
  }
}

BehaviorState BehaviorPlanner::StatePrepareLaneChangeLeft() {
  CalculateSafetyMeassures();
  
  // set target velocity
  if (next_vehicle_current_lane_) {
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else {
    target_velocity_ = sensor_fusion_->GetSpeedLimitForCurrentLane();
  }
  
  // determine next behavior state
  if (isTargetLaneSafe()) {
    return BehaviorState::kLaneChangeLeft;
//  } else if (sensor_fusion_->average_lane_velocities_[target_lane_] - sensor_fusion_->average_lane_velocities_[current_lane_] >=
//             sensor_fusion_->average_lane_velocities_[target_lane_] * kFastestLaneFactor) {
//    // current lane is faster than target lane, cancel lane change
//    return BehaviorState::kKeepLane;
  } else {
    // still unsafe to change to the left lane, wait
    return BehaviorState::kPrepareLaneChangeLeft;
  }
}

BehaviorState BehaviorPlanner::StateLaneChangeLeft() {
  CalculateSafetyMeassures();
  
  // set target velocity
  if (front_vehicle_target_lane_ && current_lane_ != target_lane_) {
    // vehicle is not yet in the target lane
    target_velocity_ = front_vehicle_target_lane_->v_;
  } else if (next_vehicle_current_lane_ && current_lane_ == target_lane_) {
    // vehicle is already in target lane, the vehicle switched from target to current lane
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else {
    target_velocity_ = sensor_fusion_->GetSpeedLimitForLane(target_lane_);
  }
  
  // determine next state
  double d_left_lane_marker = (target_lane_ + 1) * sensor_fusion_->kLaneWidth_;
  double d_right_host_vehicle = sensor_fusion_->host_vehicle_.d_ + (sensor_fusion_->host_vehicle_.width_ / 2.0);
  
  if (d_right_host_vehicle < d_left_lane_marker) {
    return BehaviorState::kKeepLane;
  } else {
    return BehaviorState::kLaneChangeLeft;
  }
}

BehaviorState BehaviorPlanner::StatePrepareLaneChangeRight() {
  CalculateSafetyMeassures();
  
  // set target velocity
  if (next_vehicle_current_lane_) {
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else {
    target_velocity_ = sensor_fusion_->GetSpeedLimitForCurrentLane();
  }
  
  // determine next state
  if (isTargetLaneSafe()) {
    return BehaviorState::kLaneChangeRight;
//  } else if (sensor_fusion_->average_lane_velocities_[target_lane_] - sensor_fusion_->average_lane_velocities_[current_lane_] >
//             sensor_fusion_->average_lane_velocities_[target_lane_] * kFastestLaneFactor) {
//    // current lane is faster than target lane, cancel lane change
//    return BehaviorState::kKeepLane;
  } else {
    // still unsafe to change to the right lane, wait
    return BehaviorState::kPrepareLaneChangeRight;
  }
}

BehaviorState BehaviorPlanner::StateLaneChangeRight() {
  CalculateSafetyMeassures();

  // set target velocity
  if (front_vehicle_target_lane_ && current_lane_ != target_lane_) {
    // vehicle is not yet in the target lane
    target_velocity_ = front_vehicle_target_lane_->v_;
  } else if (next_vehicle_current_lane_ && current_lane_ == target_lane_) {
    // vehicle is already in target lane, the vehicle switched from target to current lane
    target_velocity_ = next_vehicle_current_lane_->v_;
  } else {
    target_velocity_ = sensor_fusion_->GetSpeedLimitForLane(target_lane_);
  }

  // determine next state
  double d_right_lane_marker = target_lane_ * sensor_fusion_->kLaneWidth_;
  double d_left_host_vehicle = sensor_fusion_->host_vehicle_.d_ - (sensor_fusion_->host_vehicle_.width_ / 2.0);
  
  if (d_left_host_vehicle > d_right_lane_marker) {
    return BehaviorState::kKeepLane;
  } else {
    return BehaviorState::kLaneChangeRight;
  }
}

double BehaviorPlanner::CalculateDistance(const double host_x, const double host_y,  const double target_x, const double target_y) {
  double delta_x = target_x - host_x;
  double delta_y = target_y - host_y;
  
  return sqrt(delta_x * delta_x + delta_y * delta_y);
}

double BehaviorPlanner::CalculateTimeGap(const double host_v, const double host_x, const double host_y, const double target_x, const double target_y) {
  double delta_x = target_x - host_x;
  double delta_y = target_y - host_y;
  double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
  
  return dist / host_v;
}

double BehaviorPlanner::CalculateTTC(const double host_v, const double host_x, const double host_y, const double target_v, const double target_x, const double target_y) {
  double delta_x = target_x - host_x;
  double delta_y = target_y - host_y;
  double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
  double delta_v = host_v - target_v;
  
  return abs(dist / delta_v);
}

void BehaviorPlanner::CalculateSafetyMeassures() {
  
  // Calculate time gap for current lane
  next_vehicle_current_lane_ = sensor_fusion_->GetNextVehicleDrivingAhead(current_lane_);
  
  if (next_vehicle_current_lane_) {
    // found vehicle driving ahead, check time_gap
    d_current_lane_front_ = CalculateDistance(sensor_fusion_->host_vehicle_.x_, sensor_fusion_->host_vehicle_.y_,
                                              next_vehicle_current_lane_->x_, next_vehicle_current_lane_->y_);
    
    d_predicted_current_lane_front_ = CalculateDistance(sensor_fusion_->host_vehicle_.predicted_x_, sensor_fusion_->host_vehicle_.predicted_y_,
                                                        next_vehicle_current_lane_->predicted_x_, next_vehicle_current_lane_->predicted_y_);

    time_gap_current_lane_front_ = CalculateTimeGap(sensor_fusion_->host_vehicle_.v_,
                                                    sensor_fusion_->host_vehicle_.x_, sensor_fusion_->host_vehicle_.y_,
                                                    next_vehicle_current_lane_->x_, next_vehicle_current_lane_->y_);

    time_gap_predicted_current_lane_front_ = CalculateTimeGap(sensor_fusion_->host_vehicle_.predicted_v_,
                                                    sensor_fusion_->host_vehicle_.predicted_x_, sensor_fusion_->host_vehicle_.predicted_y_,
                                                    next_vehicle_current_lane_->predicted_x_, next_vehicle_current_lane_->predicted_y_);
    
    ttc_current_lane_front_ = CalculateTTC(sensor_fusion_->host_vehicle_.v_,
                                                     sensor_fusion_->host_vehicle_.x_, sensor_fusion_->host_vehicle_.y_,
                                                     next_vehicle_current_lane_->v_,
                                                     next_vehicle_current_lane_->x_, next_vehicle_current_lane_->y_);

    ttc_predicted_current_lane_front_ = CalculateTTC(sensor_fusion_->host_vehicle_.predicted_v_,
                                                     sensor_fusion_->host_vehicle_.predicted_x_, sensor_fusion_->host_vehicle_.predicted_y_,
                                                     next_vehicle_current_lane_->predicted_v_,
                                                     next_vehicle_current_lane_->predicted_x_, next_vehicle_current_lane_->predicted_y_);
  } else {
    d_current_lane_front_ = kDefaultDistance_;
    d_predicted_current_lane_front_ = kDefaultDistance_;
    time_gap_current_lane_front_ = kDefaultTimaGab_;
    time_gap_predicted_current_lane_front_ = kDefaultTimaGab_;
    ttc_current_lane_front_ = kDefaultTTC_;
    ttc_predicted_current_lane_front_ = kDefaultTTC_;
  }

  // Calculate time gaps for target lane
  if (target_lane_ == current_lane_) {
    d_target_lane_front_ = d_current_lane_front_;
    d_target_lane_rear_ = kDefaultDistance_;
    d_predicted_target_lane_front_ = d_predicted_current_lane_front_;
    d_predicted_target_lane_rear_ = kDefaultDistance_;
    time_gap_target_lane_front_ = time_gap_current_lane_front_;
    time_gap_target_lane_rear_ = kDefaultTimaGab_;
    time_gap_predicted_target_lane_front_ = time_gap_predicted_current_lane_front_;
    time_gap_predicted_target_lane_rear_ = kDefaultTimaGab_;
    ttc_target_lane_front_ = ttc_current_lane_front_;
    ttc_target_lane_rear_ = kDefaultTTC_;
    ttc_predicted_target_lane_front_ = ttc_predicted_current_lane_front_;
    ttc_predicted_target_lane_rear_ = kDefaultTTC_;
  } else {
    front_vehicle_target_lane_ = sensor_fusion_->GetNextVehicleDrivingAhead(target_lane_);
    
    if (front_vehicle_target_lane_) {
      // found vehicle driving ahead in target lane, check time gap
      d_target_lane_front_ = CalculateDistance(sensor_fusion_->host_vehicle_.x_, sensor_fusion_->host_vehicle_.y_,
                                               front_vehicle_target_lane_->x_, front_vehicle_target_lane_->y_);

      d_predicted_target_lane_front_ = CalculateDistance(sensor_fusion_->host_vehicle_.predicted_x_, sensor_fusion_->host_vehicle_.predicted_y_,
                                                         front_vehicle_target_lane_->predicted_x_, front_vehicle_target_lane_->predicted_y_);

      time_gap_target_lane_front_ = CalculateTimeGap(sensor_fusion_->host_vehicle_.v_,
                                                     sensor_fusion_->host_vehicle_.x_, sensor_fusion_->host_vehicle_.y_,
                                                     front_vehicle_target_lane_->x_, front_vehicle_target_lane_->y_);
      
      time_gap_predicted_target_lane_front_ = CalculateTimeGap(sensor_fusion_->host_vehicle_.predicted_v_,
                                                               sensor_fusion_->host_vehicle_.predicted_x_, sensor_fusion_->host_vehicle_.predicted_y_,
                                                               front_vehicle_target_lane_->predicted_x_, front_vehicle_target_lane_->predicted_y_);
      
      ttc_target_lane_front_ = CalculateTTC(sensor_fusion_->host_vehicle_.v_,
                                            sensor_fusion_->host_vehicle_.x_, sensor_fusion_->host_vehicle_.y_,
                                            front_vehicle_target_lane_->v_,
                                            front_vehicle_target_lane_->x_, front_vehicle_target_lane_->y_);

      ttc_predicted_target_lane_front_ = CalculateTTC(sensor_fusion_->host_vehicle_.predicted_v_,
                                                       sensor_fusion_->host_vehicle_.predicted_x_, sensor_fusion_->host_vehicle_.predicted_y_,
                                                       front_vehicle_target_lane_->predicted_v_,
                                                       front_vehicle_target_lane_->predicted_x_, front_vehicle_target_lane_->predicted_y_);
    } else {
      d_target_lane_front_ = kDefaultDistance_;
      d_predicted_target_lane_front_ = kDefaultDistance_;
      time_gap_target_lane_front_ = kDefaultTimaGab_;
      time_gap_predicted_target_lane_front_ = kDefaultTimaGab_;
      ttc_target_lane_front_ = kDefaultTTC_;
      ttc_predicted_target_lane_front_ = kDefaultTTC_;
    }

    rear_vehicle_target_lane_ = sensor_fusion_->GetNextVehicleDrivingBehind(target_lane_);

    if (rear_vehicle_target_lane_) {
      // found vehicle driving behind in target lane, check time gap
      d_target_lane_rear_ = CalculateDistance(sensor_fusion_->host_vehicle_.x_, sensor_fusion_->host_vehicle_.y_,
                                              rear_vehicle_target_lane_->x_, rear_vehicle_target_lane_->y_);

      d_predicted_target_lane_rear_ = CalculateDistance(sensor_fusion_->host_vehicle_.predicted_x_, sensor_fusion_->host_vehicle_.predicted_y_,
                                                        rear_vehicle_target_lane_->predicted_x_, rear_vehicle_target_lane_->predicted_y_);

      time_gap_target_lane_rear_ = CalculateTimeGap(sensor_fusion_->host_vehicle_.v_,
                                                    sensor_fusion_->host_vehicle_.x_, sensor_fusion_->host_vehicle_.y_,
                                                    rear_vehicle_target_lane_->x_, rear_vehicle_target_lane_->y_);
      
      time_gap_predicted_target_lane_rear_ = CalculateTimeGap(sensor_fusion_->host_vehicle_.predicted_v_,
                                                              sensor_fusion_->host_vehicle_.predicted_x_, sensor_fusion_->host_vehicle_.predicted_y_,
                                                              rear_vehicle_target_lane_->predicted_x_, rear_vehicle_target_lane_->predicted_y_);
      
      ttc_target_lane_rear_ = CalculateTTC(sensor_fusion_->host_vehicle_.v_,
                                           sensor_fusion_->host_vehicle_.x_, sensor_fusion_->host_vehicle_.y_,
                                           rear_vehicle_target_lane_->v_,
                                           rear_vehicle_target_lane_->x_, rear_vehicle_target_lane_->y_);

      ttc_predicted_target_lane_rear_ = CalculateTTC(sensor_fusion_->host_vehicle_.predicted_v_,
                                                      sensor_fusion_->host_vehicle_.predicted_x_, sensor_fusion_->host_vehicle_.predicted_y_,
                                                      rear_vehicle_target_lane_->predicted_v_,
                                                      rear_vehicle_target_lane_->predicted_x_, rear_vehicle_target_lane_->predicted_y_);
    } else {
      d_target_lane_rear_ = kDefaultDistance_;
      d_predicted_target_lane_rear_ = kDefaultDistance_;
      time_gap_target_lane_rear_ = kDefaultTimaGab_;
      time_gap_predicted_target_lane_rear_ = kDefaultTimaGab_;
      ttc_target_lane_rear_ = kDefaultTTC_;
      ttc_predicted_target_lane_rear_ = kDefaultTTC_;
    }
  }
}

std::ostream& operator<< (std::ostream& os, const BehaviorPlanner& obj) {
  os << "BehaviorPlanner(" << std::endl <<
    std::fixed << std::setprecision(2) << std::setfill(' ') <<
//  " host vehicle=                         " << obj.sensor_fusion_->host_vehicle_ << std::endl <<
//  " next vehicle current lane=            " << *obj.next_vehicle_current_lane_ << std::endl <<
//  " front vehicle target lane=            " << *obj.front_vehicle_target_lane_ << std::endl <<
//  " rear vehicle target lane=             " << *obj.rear_vehicle_target_lane_ << std::endl <<
    " d_current_lane=                       " << "          --> " << std::setw(7) << obj.d_current_lane_front_ << " m" << std::endl <<
    " time_gap_current_lane=                " << "          --> " << std::setw(7) << obj.time_gap_current_lane_front_ << " s" << std::endl <<
    " ttc_current_lane=                     " << "          --> " << std::setw(7) << obj.ttc_current_lane_front_ << " s" << std::endl <<
//  " time_gap_current_lane_predicted=      " << "          --> " << std::setw(7) << obj.time_gap_predicted_current_lane_front_ << " s" << std::endl <<
//  " ttc_current_lane_predicted=           " << "          --> " << std::setw(7) << obj.ttc_predicted_current_lane_front_ << " s" << std::endl <<
    " d_target_lane=                        " << std::setw(7) << obj.d_target_lane_rear_ << " m <--> " << std::setw(7) << obj.d_target_lane_front_ << " m" << std::endl <<
    " time_gap_target_lane=                 " << std::setw(7) << obj.time_gap_target_lane_rear_ << " s <--> " << std::setw(7) << obj.time_gap_target_lane_front_ << " s" << std::endl <<
    " ttc_target_lane=                      " << std::setw(7) << obj.ttc_target_lane_rear_ << " s <--> " << std::setw(7) << obj.ttc_target_lane_front_ << " s" << std::endl <<
//  " d_current_lane_predicted=             " << "--> " << std::setw(7) << obj.d_predicted_current_lane_front_ << " m" << std::endl <<
//  " d_target_lane_predicted=              " << std::setw(7) << obj.d_predicted_target_lane_rear_ << " m <--> " << std::setw(7) << obj.d_predicted_target_lane_front_ << " m" << std::endl <<
//  " time_gap_target_lane_predicted=       " << std::setw(7) << obj.time_gap_predicted_target_lane_rear_ << " s <--> " << std::setw(7) << obj.time_gap_predicted_target_lane_front_ << " s" << std::endl <<
//  " ttc_target_lane_predicted=            " << std::setw(7) << obj.ttc_predicted_target_lane_rear_ << " s <--> " << std::setw(7) << obj.ttc_predicted_target_lane_front_ << " s" << std::endl <<
    " fastest_lane=                         " << obj.fastest_lane_ << " (" << (obj.sensor_fusion_->average_lane_velocities_ / 0.44704).transpose() << ") mph" << std::endl <<
    " lane_oocupancy=                       " << obj.sensor_fusion_->lane_occupancy_.transpose() << std::endl <<
    " current_lane=                         " << obj.current_lane_ << std::endl <<
    " target_lane=                          " << obj.target_lane_ << std::endl <<
    " target_velocity=                      " << ms2mph(obj.target_velocity_) << std::endl <<
    " current_state=                        " << static_cast<int>(obj.current_state_) << " - " << obj.GetStateAsString(obj.current_state_) << std::endl;

  os << ")" << std::endl;
  
  return os;
}
