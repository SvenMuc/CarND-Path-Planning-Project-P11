//
//  TrajectoryPlanner.hpp
//  path_planning
//
//  Created by Sven Bone on 22.11.17.
//

#ifndef TrajectoryPlanner_hpp
#define TrajectoryPlanner_hpp

#include <stdio.h>
#include <vector>
#include "json.hpp"
#include "BehaviorPlanner.hpp"
#include "VehicleModel.hpp"
#include "SensorFusion.hpp"
#include "BehaviorPlanner.hpp"


using namespace std;
using json = nlohmann::json;

/**
 Optimial and collision free trajectory.
 */
struct Trajectory {
  vector<double> points_x;        // x points of unfiltered trajectory
  vector<double> points_y;        // y points of unfiltered trajectory
  int current_lane;               // current driving lane [0-left, 1-center, 2-right]
  int target_lane;                // target lane of planned trajectory [0-left, 1-center, 2-right]
};

/**
 Previous trajectory's points and frenet end points.
 */
struct PreviousTrajectory {
  json points_x;                  // x points of previous jerk free trajectory
  json points_y;                  // y points of previous jerk free trajectory
  double end_point_s;             // s frenet end coordinate of previous trajectory
  double end_point_d;             // d frenet end coordinate of previous trajectory
};

/**
 TrajectoryPlanner class
 
 The trajectory planner determines the optimal, collision free, jerk free
 and drivable trajectory to realize the desired behavior provided by the
 behavior planner.
 */
class TrajectoryPlanner
{
public:
  
  /**
   Default constructor.
   */
  TrajectoryPlanner();
  
  /**
   Default destructor.
   */
  ~TrajectoryPlanner();
  
  /**
   Contructor initializes the trajectory planner.
   
   @param sensor_fusion    Pointer to the sensor fusion instance.
   @param behavior_planner Pointer to behvior planner instance.
   @param speed_limit      Max allowed velocity [m/s].
   @param start_lane       Current driving lane [0-left, 1-center, 2-right].
   */
  TrajectoryPlanner(SensorFusion* sensor_fusion, BehaviorPlanner* behavior_planner, const double speed_limit, const int start_lane);
  
  /**
   Set the map waypoints.
   */
  void SetMapWaypoints(const vector<double>& x, const vector<double>& y, const vector<double>& s, const vector<double>& dx, const vector<double>& dy);
  
  /**
   Set the target/reference velocity of the host vehicle.
   
   @param speed_limit  Max. allowed velocity [m/s].
   */
  void SetSpeedLimit(double speed_limit);
  
  /**
   Finds the optimal and collision free trajectory to achieve target behavior.
   
   @param previous_trajectory  Previous trajectory provided by the simulator.
   
   @return Returns the trajectory.
   */
  Trajectory PlanOptimalTrajectory(PreviousTrajectory& previous_trajectory);
  
  /**
   Overload standard output stream.
   */
  friend std::ostream& operator<< (std::ostream& os, const TrajectoryPlanner& obj);
  
private:
  const double kMinTimeGap_ = 2.0;                // min allowed time gap to vehicle ahead [s]
  const double kDefaultTimaGab_ = 9999.9;         // default resp. max time gap [s]
  
  PreviousTrajectory previous_trajectory_;        // Previous driven trajectory.
  vector<double> points_x_;                       // x points of unfiltered trajectory
  vector<double> points_y_;                       // y points of unfiltered trajectory
  
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
  vector<double> map_waypoints_dx_;
  vector<double> map_waypoints_dy_;
  
  BehaviorPlanner* behavior_planner_;             // Behavior planner instance
  SensorFusion* sensor_fusion_;                   // Vehicle list incl. host vehicle
  int current_lane_;                              // current ego lane [0-left, 1-center, 2-right]
  int target_lane_;                               // target lane [0-left, 1-center, 2-right]
  double reference_velocity_;                     // reference velocity of host vehicle [m/s]
  double speed_limit_;                            // speed limit in [m/s]
  double time_gap_;                               // time gap to vehicle ahead [s]
};

#endif /* TrajectoryPlanner_hpp */
