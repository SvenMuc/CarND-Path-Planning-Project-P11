//
//  SensorFusion.hpp
//  path_planning
//
//  Created by Sven Bone on 21.11.17.
//

#ifndef SensorFusion_hpp
#define SensorFusion_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <map>
#include "VehicleModel.hpp"
#include "Eigen-3.3/Eigen/Core"

/**
 SensorFusion class
 
 The sensor fusion holds a list of all detection vehicles and provides methods
 to e.g. find the nearest vehicle in the ego or adjacent lane.
 */
class SensorFusion {
public:
  VehicleModel host_vehicle_;                        // host vehicle model
  std::vector<double> speed_limits_;                 // speed limits per lane [0-left, 1-center, 2-right] [m/s]
  int number_lanes_;                                 // number of lanes
  Eigen::VectorXd average_lane_velocities_;          // average velocity per lane [m/s]
  Eigen::VectorXi lane_occupancy_;                   // number of vehicles driving ahead per lane

  const double kLaneWidth_ = 4.0;                    // default lane width
  double const kMaxDistanceForAvgVelocity_ = 100;    // The average velocity is calucalted for vehicles in range of +/- x m
  double const kMaxDistanceLaneOccupancy_ = 100;     // The average velocity is calucalted for vehicles in range of +/- x m

  /**
   Constructor.
   */
  SensorFusion();
  
  /**
   Destructor.
   */
  ~SensorFusion();
  
  /**
   Update the sensor fusion model (e.g. predictions, lane assignments, etc.).
   
   @param prediction_time  Number of seconds to predict into the future [s].
   */
  void Update(double prediction_time);
  
  /**
   Set host vehicle model.
   
   @param x        x map coordinate
   @param y        y map coordinate
   @param s        s frenet coordinate
   @param d        d frenet coordinate
   @param yaw      yaw angle [rad]
   @param velocity Vehicle's velocity [m/s]
   */
  void SetHostVehicleModel(double x, double y, double s, double d, double yaw, double velocity);
  
  /**
   Set the individual speed limit for each lane.
   
   @param speed_limits  Vector of speed limits for each lane [m/s].
   [0-left, 1-center, 2-right]
   */
  void SetSpeedLimitsForLanes(std::vector<double> speed_limits);
  
  /**
   Get speed limit for current lane.
   
   @return Returns the speed limit [m/s]. In case the host vehicle is
   driving offroad 0 m/s is returned.
   */
  double GetSpeedLimitForCurrentLane();
  
  /**
   Get speed limit for lane.
   
   @param lane  Lane id [0 = left lane, 1 = center lane, 2 = right lane]
   
   @return Returns the speed limit [m/s] for the requested lane. In case the
   lane is not existing 0 m/s is returned.
   */
  double GetSpeedLimitForLane(int lane);

  /**
   Add a vehicle model to the sensor fusion object list.
   
   @param model  Pointer to a vehicle model.
   */
  void AddVehicleModel(VehicleModel* model);
  
  /**
   Reset update flag for all tracked vehicles.
   */
  void ResetUpdateFlagForAllVehicles();
  
  /**
   Update vehicle attributes / dynamics. If the track ID is not existing a new
   vehicle model is added.
   
   @param model  Pointer to a vehicle model.
   */
  void UpdateVehicleModel(VehicleModel* model);
  
  /**
   Removes all outdated vehicle models (updated_ = false).
   */
  void RemoveOutdatedVehicleModels();
  
  /**
   Removes all vehicles models.
   */
  void RemoveAllVehicleModels();
  
  /**
   Return the number of vehicle models in the sensor fusion object list.
   
   @return Number of stored vehicle models.
   */
  int CountVehicleModels();
  
  /**
   Get all vehicle driving in requested lane.

   @param lane  Lane id [0 = left lane, 1 = center lane, 2 = right lane]
   
   @return Returns a vector of vehicles.
   */
  std::vector<VehicleModel*> GetAllVehiclesForLane(int lane);
  
  /**
   Find next vehicle driving ahead in given lane.
   
   @param lane  Lane id [0 = left lane, 1 = center lane, 2 = right lane]
   
   @return Returns the next vehicle driving in the given lane, NULL if no
   vehicle found.
   */
  VehicleModel* GetNextVehicleDrivingAhead(int lane);
  
  /**
   Find next vehicle driving behind in given lane.
   
   @param lane  Lane id [0 = left lane, 1 = center lane, 2 = right lane]
   
   @return Returns the next vehicle driving in the given lane, NULL if no
   vehicle found.
   */
  VehicleModel* GetNextVehicleDrivingBehind(int lane);
  
  /**
   Get fastes lane.
   
   @return Returns the id of the fastest lane [-1=unknown, 0=left, 1=center, 2=right].
   */
  int GetFastestLane();

  /**
   Get fastes lane which is either the adjacent or host lane.
   
   @param factor  Percentage the fastest lane velocity needs to be higher.
   
   @return Returns the id of the fastest reachable lane [-1=unknown, 0=left, 1=center, 2=right].
   */
  int GetReachableFastestLane(const double factor = 1.0);
  
  /**
   Overload standard output stream.
   */
  friend std::ostream& operator<< (std::ostream& os, const SensorFusion& obj);
  
private:
  std::map<int, VehicleModel*> object_map_;            // Map of vehicle models <id, VehicleModel>
  
  /**
   Determines the driving lane for the vehicle.
   
   @param model  Pointer to a vehicle model.
   
   @return Returns the driving lane [-1=unknonw, 0=left, 1=center, 2=right].
   */
  int GetDrivingLaneForVehicle(const VehicleModel* model);
  
  /**
   Predict all vehicle trajectories the given seconds into the future.
   
   @param prediction_time  Number of seconds to predict into the future [s].
   */
  void GenerateVehicleModelPredictions(double prediction_time);

  /**
   Determine average velocity for all laneas based on vehicles driving ahead.
   If a lane is free the speed limit is returned. For the host lane only the
   next vehicle driving ahead is considered.
   
   @return Returns a vector with average velocities [m/s] for left, center and
   right lane.
   */
  Eigen::VectorXd GetAverageVelocityForAllLanes();
  
  /**
   Determin lane occupancy ahead.
   
   @return Returns the number of vehicles driving ahead per lane [0-left, 1-center, 2-right].
   */
  Eigen::VectorXi GetLaneOccupancyForAllLanes();
};

#endif /* SensorFusion_hpp */
