//
//  TrajectoryPlanner.cpp
//  path_planning
//
//  Created by Sven Bone on 22.11.17.
//

#include "TrajectoryPlanner.hpp"
#include "Utils.hpp"
#include "Spline/spline.h"
#include "Eigen-3.3/Eigen/Dense"


TrajectoryPlanner::TrajectoryPlanner(SensorFusion* sensor_fusion, BehaviorPlanner* behavior_planner, const int start_lane) {
  sensor_fusion_ = sensor_fusion;
  behavior_planner_ = behavior_planner;
  current_lane_ = start_lane;
  target_lane_ = start_lane;
  target_velocity_ = sensor_fusion_->GetSpeedLimitForLane(start_lane);
  reference_velocity_ = 0.0;
}

void TrajectoryPlanner::SetMapWaypoints(const vector<double>& x, const vector<double>& y, const vector<double>& s, const vector<double>& dx, const vector<double>& dy) {
  map_waypoints_x_ = x;
  map_waypoints_y_ = y;
  map_waypoints_s_ = s;
  map_waypoints_dx_ = dx;
  map_waypoints_dy_ = dy;
}

void TrajectoryPlanner::SetSpeedLimit(double speed_limit) {
  speed_limit_ = speed_limit;
}

Trajectory TrajectoryPlanner::PlanSplineTrajectory(PreviousTrajectory& previous_trajectory) {

  static double previous_reference_velocity = 0.0;
  
  // determine current, target lane and time gap threshold for speed control
  current_lane_ = behavior_planner_->current_lane_;
  target_velocity_ = behavior_planner_->target_velocity_;
  double time_gap_threshold;
  
  if (behavior_planner_->current_state_ == BehaviorState::kPrepareLaneChangeLeft ||
      behavior_planner_->current_state_ == BehaviorState::kPrepareLaneChangeRight) {
    target_lane_ = current_lane_;
    time_gap_threshold = kMinTimeGapLaneChange_;
    time_gap_ = min(behavior_planner_->time_gap_current_lane_front_, behavior_planner_->time_gap_target_lane_front_);
  } else {
    target_lane_ = behavior_planner_->target_lane_;
    time_gap_ = behavior_planner_->time_gap_current_lane_front_;
    time_gap_threshold = kMinTimeGap_;
  }

  vector<double> points_x;
  vector<double> points_y;
  VehicleModel host_vehicle = sensor_fusion_->host_vehicle_;

  double ref_x = host_vehicle.x_;
  double ref_y = host_vehicle.y_;
  double ref_yaw = host_vehicle.yaw_;

  // control speed depending on time gap to vehicle ahead

  // plan trajectory to target lane
  int prev_trajectory_size = previous_trajectory.points_x.size();

  if(prev_trajectory_size < 2) {
    // use two points that make the path tangent to the vehicle
    double prev_vehicle_x = host_vehicle.x_ - cos(host_vehicle.yaw_);
    double prev_vehicle_y = host_vehicle.y_ - sin(host_vehicle.yaw_);

    points_x.push_back(prev_vehicle_x);
    points_x.push_back(host_vehicle.x_);

    points_y.push_back(prev_vehicle_y);
    points_y.push_back(host_vehicle.y_);
    
    reference_velocity_ = host_vehicle.v_;
  } else {
    // redefine reference state as previous path end point
    ref_x = previous_trajectory.points_x[prev_trajectory_size - 1];
    ref_y = previous_trajectory.points_y[prev_trajectory_size - 1];

    double ref_x_prev = previous_trajectory.points_x[prev_trajectory_size - 2];
    double ref_y_prev = previous_trajectory.points_y[prev_trajectory_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the car
    points_x.push_back(ref_x_prev);
    points_x.push_back(ref_x);

    points_y.push_back(ref_y_prev);
    points_y.push_back(ref_y);
    
    reference_velocity_ = previous_reference_velocity;
  }

  // determine 3 waipoints (30 m, 60 m and 90 m) in frenet coordinates to smooth the trajectory
  double end_point_s;
  
  if (prev_trajectory_size > 0) {
    end_point_s = previous_trajectory.end_point_s;
  } else {
    end_point_s = host_vehicle.s_;
  }
  
  for (auto& offset: {30, 60, 90}) {
    vector<double> next_wp = getXY(end_point_s + offset, (2 + 4 * target_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    points_x.push_back(next_wp[0]);
    points_y.push_back(next_wp[1]);
  }

  for(int i=0; i < points_x.size(); ++i) {
    // shift car reference angle to 0 degrees
    double shift_x = points_x[i] - ref_x;
    double shift_y = points_y[i] - ref_y;

    points_x[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
    points_y[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
  }

  // create a spline to smooth the trajectory
  tk::spline s;
  s.set_points(points_x, points_y);

  // define the actual (x, y) points we will use for the planner
  Trajectory trajectory;

  // start with all of the previous path points from last time
  for(int i=0; i < prev_trajectory_size; ++i) {
    trajectory.points_x.push_back(previous_trajectory.points_x[i]);
    trajectory.points_y.push_back(previous_trajectory.points_y[i]);
  }

  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0.0;

  // fill up the rest of our path planner after filling it with previous points
  // here we will always output 50 points
  for(int i=1; i <  (kNumberPredictionsPoints - prev_trajectory_size); ++i) {
    
    // increase/decrease reference velocity
    if (time_gap_ < time_gap_threshold || reference_velocity_ > target_velocity_) {
      reference_velocity_ = max(0.5, reference_velocity_ - kMaxDeltaVelocity);
    } else {
      reference_velocity_ = min(target_velocity_, reference_velocity_ + kMaxDeltaVelocity);
    }
    
    // Calculate points along new path
    double N = (target_dist / (kControllerCycleTime * reference_velocity_));
    double x_point = x_add_on+(target_x)/N;
    double y_point = s(x_point);
    
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    // rotate and shift back to normal
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
    
    x_point += ref_x;
    y_point += ref_y;
    
    trajectory.points_x.push_back(x_point);
    trajectory.points_y.push_back(y_point);
}
  
  trajectory.current_lane = current_lane_;
  trajectory.target_lane = target_lane_;
  
  previous_reference_velocity = reference_velocity_;
  
  return trajectory;
}

std::ostream& operator<< (std::ostream& os, const TrajectoryPlanner& obj) {
  os << "TrajectoryPlanner(" << std::endl <<
  " current_lane=         " << obj.current_lane_ << std::endl <<
  " target_lane=          " << obj.target_lane_ << std::endl <<
  " target_velocity=      " << ms2mph(obj.target_velocity_) << " mph" << std::endl <<
  " reference_velocity=   " << ms2mph(obj.reference_velocity_) << " mph" << std::endl <<
  " time_gap=             " << obj.time_gap_ << " s" << std::endl <<
  ")" << std::endl;

  return os;
}
