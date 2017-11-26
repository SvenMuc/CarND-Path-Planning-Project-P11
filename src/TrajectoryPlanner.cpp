//
//  TrajectoryPlanner.cpp
//  path_planning
//
//  Created by Sven Bone on 22.11.17.
//

#include "TrajectoryPlanner.hpp"
#include "Utils.hpp"
#include "Spline/spline.h"


TrajectoryPlanner::TrajectoryPlanner() {
  current_lane_ = 0;
  target_lane_ = 0;
  reference_velocity_ = 0.0;
  speed_limit_ = 49.5;
  behavior_planner_ = NULL;
  sensor_fusion_ = NULL;
}

TrajectoryPlanner::TrajectoryPlanner(SensorFusion* sensor_fusion, BehaviorPlanner* behavior_planner, const double speed_limit, const int start_lane) {
  current_lane_ = start_lane;
  target_lane_ = start_lane;
  reference_velocity_ = 0.0;
  speed_limit_ = speed_limit;
  behavior_planner_ = behavior_planner;
  sensor_fusion_ = sensor_fusion;
}

TrajectoryPlanner::~TrajectoryPlanner() {
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

Trajectory TrajectoryPlanner::PlanOptimalTrajectory(PreviousTrajectory& previous_trajectory) {
  current_lane_ = behavior_planner_->current_lane_;
  target_lane_ = behavior_planner_->target_lane_;

  // FIXME: Remove points from class
  points_x_.clear();
  points_y_.clear();

  VehicleModel host_vehicle = sensor_fusion_->host_vehicle_;

  double ref_x = host_vehicle.x_;
  double ref_y = host_vehicle.y_;
  double ref_yaw = host_vehicle.yaw_;
  
  // control speed depending on time gap to vehicle ahead
  VehicleModel* next_vehicle_in_current_lane = sensor_fusion_->GetNextVehicleDrivingAhead(current_lane_);
  
  if (next_vehicle_in_current_lane) {
    // found vehicle driving ahead, check time_gap
    double delta_x = host_vehicle.x_ - next_vehicle_in_current_lane->x_;
    double delta_y = host_vehicle.y_ - next_vehicle_in_current_lane->y_;
    double dist = sqrt(delta_x * delta_x + delta_y * delta_y);
    time_gap_ = dist / next_vehicle_in_current_lane->v_;
  } else {
    time_gap_ = kDefaultTimaGab_;
  }
  
  // FIXME: Check max acceleration rate
  if (time_gap_ < kMinTimeGap_) {
    reference_velocity_ = max(0.0, reference_velocity_ - 0.5);
  } else if (reference_velocity_ < speed_limit_) {
    reference_velocity_ = min(speed_limit_, reference_velocity_ + 0.5);
  }
  
  // plan trajectory to target lane
  int prev_size = previous_trajectory.points_x.size();
  double start_s = host_vehicle.s_;
  
  if(prev_size > 0) {
    start_s = previous_trajectory.end_point_s;
  }
  
  if(prev_size < 2) {
    // use two points that make the path tangent to the vehicle
    double prev_vehicle_x = host_vehicle.x_ - cos(host_vehicle.yaw_);
    double prev_vehicle_y = host_vehicle.y_ - sin(host_vehicle.yaw_);
    
    points_x_.push_back(prev_vehicle_x);
    points_x_.push_back(host_vehicle.x_);
    
    points_y_.push_back(prev_vehicle_y);
    points_y_.push_back(host_vehicle.y_);
  } else {
    // redefine reference state as previous path end point
    ref_x = previous_trajectory.points_x[prev_size-1];
    ref_y = previous_trajectory.points_y[prev_size-1];
    
    double ref_x_prev = previous_trajectory.points_x[prev_size-2];
    double ref_y_prev = previous_trajectory.points_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    
    // use two points that make the path tangent to the car
    points_x_.push_back(ref_x_prev);
    points_x_.push_back(ref_x);
    
    points_y_.push_back(ref_y_prev);
    points_y_.push_back(ref_y);
  }
  
  // in frenet add evenly 30 m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(host_vehicle.s_ + 30, (2+4*target_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp1 = getXY(host_vehicle.s_ + 60, (2+4*target_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  vector<double> next_wp2 = getXY(host_vehicle.s_ + 90, (2+4*target_lane_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  
  points_x_.push_back(next_wp0[0]);
  points_x_.push_back(next_wp1[0]);
  points_x_.push_back(next_wp2[0]);
  
  points_y_.push_back(next_wp0[1]);
  points_y_.push_back(next_wp1[1]);
  points_y_.push_back(next_wp2[1]);
  
  for(int i=0; i < points_x_.size(); ++i) {
    // shift car reference angle to 0 degrees
    double shift_x = points_x_[i] - ref_x;
    double shift_y = points_y_[i] - ref_y;
    
    points_x_[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    points_y_[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
  }
  
  // create a spline
  tk::spline s;
  
  // set (x, y) points to the spline
  s.set_points(points_x_, points_y_);
  
  // define the actual (x, y) points we will use for the planner
  Trajectory trajectory;
  
  // start with all of the previous path points from last time
  for(int i=0; i < previous_trajectory.points_x.size(); ++i) {
    trajectory.points_x.push_back(previous_trajectory.points_x[i]);
    trajectory.points_y.push_back(previous_trajectory.points_y[i]);
  }
  
  // calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  
  double x_add_on = 0;
  
  // fill up the rest of our path planner after filling it with previous points
  // here we will always output 50 points
  for(int i=1; i <=  50-previous_trajectory.points_x.size(); ++i) {
    double N = (target_dist / (0.02 * reference_velocity_));
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);
    
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
    
    x_point += ref_x;
    y_point += ref_y;
    
    trajectory.points_x.push_back(x_point);
    trajectory.points_y.push_back(y_point);
  }
  
  trajectory.current_lane = current_lane_;
  trajectory.target_lane = target_lane_;
  
  return trajectory;
}

std::ostream& operator<< (std::ostream& os, const TrajectoryPlanner& obj) {
  os << "TrajectoryPlanner(" << std::endl <<
  " current_lane=         " << obj.current_lane_ << std::endl <<
  " target_lane=          " << obj.target_lane_ << std::endl <<
  " ref_velocity=         " << obj.reference_velocity_ / 0.44704 << " mph" << std::endl <<
  " time_gap=             " << obj.time_gap_ << " s" << std::endl <<
  ")" << std::endl;

  return os;
}
