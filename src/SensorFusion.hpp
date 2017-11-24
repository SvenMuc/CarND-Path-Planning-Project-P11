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
#include "VehicleModel.hpp"

/**
 SensorFusion class
 
 The sensor fusion holds a list of all detection vehicles and provides methods
 to e.g. find the nearest vehicle in the ego or adjacent lane.
 */
class SensorFusion {
public:
    VehicleModel host_vehicle_;                        // host vehicle model
    const double lane_width_ = 4.0f;                   // default lane width

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
     Add a vehicle model to the sensor fusion object list.
     
     @param model  Pointer to a vehicle model.
     */
    void AddVehicleModel(VehicleModel* model);
    
    /**
     Removes all vehicles models.
     */
    void RemoveAll();
    
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
     Overload standard output stream.
     */
    friend std::ostream& operator<< (std::ostream& os, const SensorFusion& obj);
    
private:
    std::vector<VehicleModel*> objectList_;             // List of vehicle models
};

#endif /* SensorFusion_hpp */
