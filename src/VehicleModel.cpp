//
//  VehicleModel.cpp
//  path_planning
//
//  Created by Sven Bone on 19.11.17.
//

#include "VehicleModel.hpp"
#include "Utils.hpp"
#include <math.h>

VehicleModel::VehicleModel() {
  id_ = 0;
  x_ = 0.0;
  y_ = 0.0;
  vx_ = 0.0;
  vy_ = 0.0;
  s_ = 0.0;
  d_ = 0.0;
  
  ax_ = 0.0;
  ay_ = 0.0;
  v_ = 0.0;
  a_ = 0.0;
  yaw_ = 0.0;
  width_ = kDefaultVehicleWidth;
  updated_ = false;
  
  lane_ = -1;   // lane unknown
}

VehicleModel::VehicleModel(int id, double x, double y, double vx, double vy, double s, double d) {
  id_ = id;
  x_ = x;
  y_ = y;
  vx_ = vx;
  vy_ = vy;
  s_ = s;
  d_ = d;

  ax_ = 0.0;
  ay_ = 0.0;
  v_ = sqrt(vx_ * vx_ + vy_ * vy_);
  a_ = sqrt(ax_ * ax_ + ay_ * ay_);
  yaw_ = vx_ == 0 ? atan(vy_ / vx_) : 0.0f;
  width_ = kDefaultVehicleWidth;
  updated_ = true;
  
  lane_ = -1;   // lane unknown
}

VehicleModel::VehicleModel(double x, double y, double s, double d, double yaw, double velocity) {
  id_ = 0;
  x_ = x;
  y_ = y;
  vx_ = 0.0f;
  vy_ = 0.0f;
  s_ = s;
  d_ = d;
  
  ax_ = 0.0;
  ay_ = 0.0;
  v_ = velocity;
  a_ = sqrt(ax_ * ax_ + ay_ * ay_);
  yaw_ = yaw;
  width_ = kDefaultVehicleWidth;
  updated_ = true;
  
  lane_ = -1;   // lane unknown
}

void VehicleModel::GeneratePredictions(double prediction_time) {
  // TODO: determine ax_, ay_ and a_
  prediction_time_ = prediction_time;

  // clear previous prediction results
  predicted_trajectory_x_.clear();
  predicted_trajectory_y_.clear();
  predicted_trajectory_vx_.clear();
  predicted_trajectory_vy_.clear();
  predicted_trajectory_ax_.clear();
  predicted_trajectory_ay_.clear();
  predicted_trajectory_v_.clear();
  predicted_trajectory_a_.clear();
  predicted_trajectory_yaw_.clear();
  predicted_trajectory_s_.clear();
  predicted_trajectory_d_.clear();
  
  // predict vehicle trajectory until prediction time
  for (int i=0; i < (prediction_time / kControllerCycleTime); ++i) {
    double dt = i * kControllerCycleTime;
    predicted_trajectory_x_.push_back(x_ + (vx_ * dt + 0.5 * ax_ * dt * dt));
    predicted_trajectory_y_.push_back(y_ + (vy_ * dt + 0.5 * ay_ * dt * dt));
    
    predicted_trajectory_vx_.push_back(vx_ + ax_ * prediction_time);
    predicted_trajectory_vy_.push_back(vy_ + ay_ * prediction_time);
    predicted_trajectory_v_.push_back(v_ + a_ * prediction_time);
  }
  
  // vehicle state at prediction time
  double dt = prediction_time;
  predicted_x_ = x_ + (vx_ * dt + 0.5 * ax_ * dt * dt);
  predicted_y_ = y_ + (vy_ * dt + 0.5 * ay_ * dt * dt);
  
  predicted_vx_ = vx_ + ax_ * prediction_time;
  predicted_vy_ = vy_ + ay_ * prediction_time;
  predicted_v_ = v_ + a_ * prediction_time;
}

std::ostream& operator<< (std::ostream& os, const VehicleModel& vehicleModel) {
  if (&vehicleModel) {
    os << "VehicleModel(" <<
    "id=" << vehicleModel.id_ <<
    " x=" << vehicleModel.x_ <<
    " y=" << vehicleModel.y_ <<
    " vx=" << vehicleModel.vx_ <<
    " vy=" << vehicleModel.vy_ <<
    " ax=" << vehicleModel.ax_ <<
    " ay=" << vehicleModel.ay_ <<
    " v=" << vehicleModel.v_ <<
    " a=" << vehicleModel.a_ <<
    " yaw=" << vehicleModel.yaw_ <<
    " s=" << vehicleModel.s_ <<
    " d=" << vehicleModel.d_ <<
    " lane=" << vehicleModel.lane_ <<
    " updated=" << vehicleModel.updated_ << ")";
  } else {
    os << "VehicleModel(NULL)";
  }
  
  return os;
}
