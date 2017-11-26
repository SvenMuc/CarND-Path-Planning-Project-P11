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
  // clear previous prediction results
  prediction_x_.clear();
  prediction_y_.clear();
  prediction_vx_.clear();
  prediction_vy_.clear();
  prediction_ax_.clear();
  prediction_ay_.clear();
  prediction_v_.clear();
  prediction_a_.clear();
  prediction_yaw_.clear();
  prediction_s_.clear();
  prediction_d_.clear();
  
  // predict vehicle trajectroy
  for (int i=0; i < (prediction_time / kControllerCycleTime); ++i) {
    double dt = i * kControllerCycleTime;
    prediction_x_.push_back(x_ + (vx_ * dt + 0.5 * ax_ * dt * dt));
    prediction_y_.push_back(y_ + (vy_ * dt + 0.5 * ay_ * dt * dt));
    
    //prediction_x_.push_back(x_ + (v_ * dt + 0.5 * a_ * dt * dt) * sin(yaw_));
    //prediction_y_.push_back(y_ + (v_ * dt + 0.5 * a_ * dt * dt) * cos(yaw_));

    // TODO: determine ax_ and ay_
//    prediction_vx_.push_back(vx_ + ax_ * prediction_time);
//    prediction_vy_.push_back(vy_ + ay_ * prediction_time);
//    prediction_a_.push_back(v_ * a_ * prediction_time);
  }
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
