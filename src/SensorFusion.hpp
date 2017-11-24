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

/**
 SensorFusion class
 
 The sensor fusion holds a list of all detection vehicles and provides methods
 to e.g. find the nearest vehicle in the ego or adjacent lane.
 */
class SensorFusion {
public:
  VehicleModel host_vehicle_;                        // host vehicle model
  std::vector<double> speed_limits_;                 // speed limits per lane [0-left, 1-center, 2-right] [m/s]
  const double kLaneWidth = 4.0f;                    // default lane width
  
  /**
   Constructor.
   */
  SensorFusion();
  
  /**
   Destructor.
   */
  ~SensorFusion();
  
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
   Get speed limit for lane.
   
   @param lane  Lane id [0 = left lane, 1 = center lane, 2 = right lane]
   
   @return Returns the speed limit [m/s] for the requested lane. In case the
   lane is not existing 0 m/s is returned.
   */
  double GetSpeedLimitForLane(int lane);
  
  /**
   Overload standard output stream.
   */
  friend std::ostream& operator<< (std::ostream& os, const SensorFusion& obj);
  
private:
  std::map<int, VehicleModel*> object_map_;           // Map of vehicle models <id, VehicleModel>
};

#endif /* SensorFusion_hpp */
