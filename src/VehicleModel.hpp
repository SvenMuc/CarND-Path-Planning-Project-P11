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
#include <vector>

using namespace std;

const double kDefaultVehicleWidth = 2.0;      // default vehicle width [m]

/**
 VehicleModel class
 
 The vehicle model contains the actual vehicle dynamics and predicts the
 state based on a standard bycicle model.
 */
class VehicleModel {
public:
  int id_;                        // vehicle ID
  double x_;                      // x position [m]
  double y_;                      // y position [m]
  double vx_;                     // velocity in x direction [m/s]
  double vy_;                     // velocity in y direction [m/s]
  double ax_;                     // acceleration in x direction [m/s^2]
  double ay_;                     // acceleration in y direction [m/s^2]
  double v_;                      // velocity [m/s]
  double a_;                      // acceleration [m/s^2]
  double yaw_;                    // yaw angle (orientation) [rad]
  double s_;                      // frenet s coordinate
  double d_;                      // frenet d coordinate
  double width_;                  // vehicle width [m]
  bool updated_;                  // true if attributes are up-to date
  
  int lane_;                      // lane id [-1=unknown, 0-left, 1-center, 2-right]
  
  double prediction_time_;        // number of seconds to predict into the future [s]
  double predicted_x_;            // prediced x positions [m]
  double predicted_y_;            // prediced y positions [m]
  double predicted_vx_;           // prediced velocity in x direction [m/s]
  double predicted_vy_;           // prediced velocity in y direction [m/s]
  double predicted_ax_;           // prediced acceleration in x direction [m/s^2]
  double predicted_ay_;           // prediced acceleration in y direction [m/s^2]
  double predicted_v_;            // prediced velocity [m/s]
  double predicted_a_;            // prediced acceleration [m/s^2]
  double predicted_yaw_;          // prediced yaw angle (orientation) [rad]
  double predicted_s_;            // prediced frenet s coordinates
  double predicted_d_;            // prediced frenet d coordinates

  vector<double> predicted_trajectory_x_;   // prediced trajectory, x positions [m]
  vector<double> predicted_trajectory_y_;   // prediced trajectory, y positions [m]
  vector<double> predicted_trajectory_vx_;  // prediced trajectory, velocity in x direction [m/s]
  vector<double> predicted_trajectory_vy_;  // prediced trajectory, velocity in y direction [m/s]
  vector<double> predicted_trajectory_ax_;  // prediced trajectory, acceleration in x direction [m/s^2]
  vector<double> predicted_trajectory_ay_;  // prediced trajectory, acceleration in y direction [m/s^2]
  vector<double> predicted_trajectory_v_;   // prediced trajectory, velocity [m/s]
  vector<double> predicted_trajectory_a_;   // prediced trajectory, acceleration [m/s^2]
  vector<double> predicted_trajectory_yaw_; // prediced trajectory, yaw angle (orientation) [rad]
  vector<double> predicted_trajectory_s_;   // prediced trajectory, frenet s coordinates
  vector<double> predicted_trajectory_d_;   // prediced trajectory, frenet d coordinates

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
   Predict all vehicle trajectories the given seconds into the future.
   
   @param prediction_time  Number of seconds to predict into the future [s].
   */
  void GeneratePredictions(double prediction_time);

  /**
   Overload standard output stream.
   */
  friend std::ostream& operator<< (std::ostream& os, const VehicleModel& obj);
};

#endif /* VehicleModel_hpp */
