//
//  VehicleModel.hpp
//  path_planning
//
//  Created by Sven Bone on 19.11.17.
//

#ifndef VehicleModel_hpp
#define VehicleModel_hpp

#include <stdio.h>
#include <iostream>


/**
 VehicleModel class
 
 The vehicle model contains the actual vehicle dynamics and predicts the
 state based on a standard bycicle model.
 */
class VehicleModel {
public:
  int id_;                // vehicle ID
  double x_;              // x position [m]
  double y_;              // y position [m]
  double vx_;             // velocity in x direction [m/s]
  double vy_;             // velocity in y direction [m/s]
  double ax_;             // acceleration in x direction [m/s^2]
  double ay_;             // acceleration in y direction [m/s^2]
  double v_;              // velocity [m/s]
  double a_;              // acceleration [m/s^2]
  double yaw_;            // yaw angle (orientation) [rad]
  double s_;              // frenet s coordinate
  double d_;              // frenet d coordinate
  bool updated_;          // true if attributes are up-to date

  /**
   Constructor initializes a default vehicle model.
   */
  VehicleModel();
  
  /**
   Constructor initializes a vehicle with the standard set of parameters.
   
   @param id  ID of vehicle track.
   @param x   map x-position [m]
   @param y   map y-position [m]
   @param vx  velocity in x direction [m/s]
   @param vy  velocity in y direction [m/s]
   @param s   frenet s coordinate
   @param d   frenet s coordinate
   */
  VehicleModel(int id, double x, double y, double vx, double vy, double s, double d);
  
  /**
   Constructor initializes a vehicle with the standard set of parameters.
   
   @param id  ID of vehicle track.
   @param x        map x-position [m]
   @param y        map y-position [m]
   @param s        frenet s coordinate
   @param d        frenet s coordinate
   @param yaw      yaw angle [rad]
   @param velocity velocity [m/s]
   */
  VehicleModel(double x, double y, double s, double d, double yaw, double velocity);

  /**
   Overload standard output stream.
   */
  friend std::ostream& operator<< (std::ostream& os, const VehicleModel& obj);
};

#endif /* VehicleModel_hpp */
