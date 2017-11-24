//
//  VehicleModel.cpp
//  path_planning
//
//  Created by Sven Bone on 19.11.17.
//

#include "VehicleModel.hpp"
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
  updated_ = false;
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
  yaw_ = vx_ < 0 ? atan(vy_ / vx_) : 0.0f;
  
  updated_ = true;
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
  
  updated_ = true;
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
    " updated=" << vehicleModel.updated_ << ")";
  } else {
    os << "VehicleModel(NULL)";
  }
  
  return os;
}
