//
//  SensorFusion.cpp
//  path_planning
//
//  Created by Sven Bone on 21.11.17.
//

#include "SensorFusion.hpp"
#include "Utils.hpp"


SensorFusion::SensorFusion() {
  number_lanes_ = 0;
}

SensorFusion::~SensorFusion() {
}

void SensorFusion::Update(double prediction_time) {
  RemoveOutdatedVehicleModels();
  average_lane_velocities_ = GetAverageVelocityForAllLanes();
  lane_occupancy_ = GetLaneOccupancyForAllLanes();
  GenerateVehicleModelPredictions(prediction_time);
}

void SensorFusion::SetHostVehicleModel(double x, double y, double s, double d, double yaw, double velocity) {
  host_vehicle_ = VehicleModel(x, y, s, d, yaw, velocity);
  host_vehicle_.lane_ = GetDrivingLaneForVehicle(&host_vehicle_);
}

void SensorFusion::SetSpeedLimitsForLanes(std::vector<double> speed_limits) {
  speed_limits_ = speed_limits;
  number_lanes_ = speed_limits_.size();
}

double SensorFusion::GetSpeedLimitForCurrentLane() {
  int lane = host_vehicle_.lane_;
  
  if (lane >= 0 && lane < speed_limits_.size()) {
    return speed_limits_[lane];
  } else {
    return 0.0;
  }
}

double SensorFusion::GetSpeedLimitForLane(int lane) {
  if (lane >= 0.0 && lane < speed_limits_.size()) {
    return speed_limits_[lane];
  } else {
    return 0.0;
  }
}

void SensorFusion::AddVehicleModel(VehicleModel* model) {
  model->lane_ = GetDrivingLaneForVehicle(model);
  object_map_.insert(std::pair<int, VehicleModel*> (model->id_, model));
}

void SensorFusion::ResetUpdateFlagForAllVehicles() {
  host_vehicle_.updated_ = false;
  
  for (auto& obj: object_map_) {
    obj.second->updated_ = false;
  }
}

void SensorFusion::UpdateVehicleModel(VehicleModel* model) {
  std::map<int, VehicleModel*>::iterator it = object_map_.find(model->id_);
  
  if (it != object_map_.end()) {
    delete (*it).second;
    (*it).second = model;
  } else {
    AddVehicleModel(model);
  }
}

void SensorFusion::RemoveOutdatedVehicleModels() {
  for (auto& obj: object_map_) {
    if (!obj.second->updated_) {
      delete obj.second;
    }
  }
}

void SensorFusion::RemoveAllVehicleModels() {
  for (auto& obj: object_map_) {
    delete obj.second;
  }
  
  object_map_.clear();
}

int SensorFusion::CountVehicleModels() {
  return object_map_.size();
}

std::vector<VehicleModel*> SensorFusion::GetAllVehiclesForLane(int lane) {
  std::vector<VehicleModel*> vehicles;
  
  for (auto& obj: object_map_) {
    if (obj.second->d_ < (2 + kLaneWidth_ * lane + (kLaneWidth_/2.0)) &&
        obj.second->d_ > (2 + kLaneWidth_ * lane - (kLaneWidth_/2.0))) {
      // vehicle found in given lane
      vehicles.push_back(obj.second);
    }
  }
  
  return vehicles;
}

VehicleModel* SensorFusion::GetNextVehicleDrivingAhead(int lane) {
  VehicleModel* next_vehicle = NULL;
  
  for (auto& obj: object_map_) {
    if (obj.second->lane_ == lane) {
      // vehicle found in given lane
      // check if it is the initial one or closer than the last stored one
      double delta_s_object = obj.second->s_ - host_vehicle_.s_;
      
      if (delta_s_object >= 0 && next_vehicle == NULL) {
        next_vehicle = obj.second;
      } else if (delta_s_object >= 0 && next_vehicle) {
        double delta_s_next = next_vehicle->s_ - host_vehicle_.s_;
        
        if (delta_s_object < delta_s_next) {
          next_vehicle = obj.second;
        }
      }
    }
  }
  return next_vehicle;
}

VehicleModel* SensorFusion::GetNextVehicleDrivingBehind(int lane) {
  VehicleModel* next_vehicle = NULL;
  
  for (auto obj: object_map_) {
    if (obj.second->lane_ == lane) {
      // vehicle found in given lane
      // check if it is the initial one or closer than the last stored one
      double delta_s_object = obj.second->s_ - host_vehicle_.s_;
      
      if (delta_s_object < 0 && next_vehicle == NULL) {
        next_vehicle = obj.second;
      } else if (delta_s_object < 0 && next_vehicle) {
        double delta_s_next = next_vehicle->s_ - host_vehicle_.s_;
        
        if (delta_s_object > delta_s_next) {
          next_vehicle = obj.second;
        }
      }
    }
  }
  return next_vehicle;
}

int SensorFusion::GetFastestLane() {
  double max_velocity = average_lane_velocities_[0];
  int lane = 0;
  
  for (int i=1; i < number_lanes_; ++i) {
    if (average_lane_velocities_[i] > max_velocity) {
      max_velocity = average_lane_velocities_[i];
      lane = i;
    }
  }
  
  return lane;
}

int SensorFusion::GetReachableFastestLane(const double factor) {
  int left_lane = max(0, host_vehicle_.lane_ - 1);
  int right_lane = min(number_lanes_ - 1, host_vehicle_.lane_ + 1);
  
  if (average_lane_velocities_[left_lane] - average_lane_velocities_[host_vehicle_.lane_] >
      average_lane_velocities_[host_vehicle_.lane_] * factor) {
    return left_lane;
  } else if (average_lane_velocities_[right_lane] - average_lane_velocities_[host_vehicle_.lane_] >
             average_lane_velocities_[host_vehicle_.lane_] * factor) {
    return right_lane;
  } else {
    return host_vehicle_.lane_;
  }
}

std::ostream& operator<< (std::ostream& os, const SensorFusion& obj) {
  os << "SensorFusion(" << std::endl << " objec_map_=" << std::endl;
  
  for (auto& model: obj.object_map_) {
    os << "  " << *model.second << std::endl;
  }
  
  os <<
  " host_vehicle_= " << std::endl << "  " << obj.host_vehicle_ << std::endl <<
  " avg. lane velocities= " << obj.average_lane_velocities_.transpose() << " m/s (" << (obj.average_lane_velocities_ / 0.44704).transpose() << ") mph" << std::endl <<
  " lane_occupancy=       " << obj.lane_occupancy_.transpose() << std::endl;
  os << ")" << std::endl;
  return os;
}

/*** private methods ***/

int SensorFusion::GetDrivingLaneForVehicle(const VehicleModel* model) {
  
  for (int lane=0; lane < number_lanes_; ++lane) {
    if (model->d_ < (2 + kLaneWidth_ * lane + (kLaneWidth_/2.0)) &&
        model->d_ > (2 + kLaneWidth_ * lane - (kLaneWidth_/2.0))) {
      return lane;
    }
  }
  
  return -1;
}

void SensorFusion::GenerateVehicleModelPredictions(double prediction_time) {
  host_vehicle_.GeneratePredictions(prediction_time);
  
  for (auto& obj: object_map_) {
    obj.second->GeneratePredictions(prediction_time);
    
    // TODO: Remove logging here
    //WriteLogFile(obj.second->id_, obj.second->prediction_x_, obj.second->prediction_y_);
  }
}

Eigen::VectorXd SensorFusion::GetAverageVelocityForAllLanes() {
  std::vector<double> sum_velocities = {0.0, 0.0, 0.0};
  std::vector<int> number_vehicles = {0, 0, 0};
  
  for (auto& obj: object_map_) {
    double delta_s = obj.second->s_ - host_vehicle_.s_;
    
    // ignore vehicles driving in host lane
    // consider all vehicles driving ahead up to a given distance s
    if (obj.second->lane_ != host_vehicle_.lane_ && delta_s > 0.0 && delta_s < kMaxDistanceForAvgVelocity_) {
      sum_velocities[obj.second->lane_] += obj.second->v_;
      number_vehicles[obj.second->lane_] += 1;
    }
  }
  
  // determin host lane velocity on vehicle ahead only
  VehicleModel* next_vehicle = GetNextVehicleDrivingAhead(host_vehicle_.lane_);
  
  if (next_vehicle) {
    sum_velocities[host_vehicle_.lane_] = next_vehicle->v_;
  } else {
    sum_velocities[host_vehicle_.lane_] = speed_limits_[host_vehicle_.lane_];
  }
  number_vehicles[host_vehicle_.lane_] = 1;

  Eigen::VectorXd average_lane_velocities(3);
  average_lane_velocities << (number_vehicles[0] == 0 ? speed_limits_[0] : min(speed_limits_[0], sum_velocities[0] / number_vehicles[0])),
                             (number_vehicles[1] == 0 ? speed_limits_[1] : min(speed_limits_[1], sum_velocities[1] / number_vehicles[1])),
                             (number_vehicles[2] == 0 ? speed_limits_[2] : min(speed_limits_[2], sum_velocities[2] / number_vehicles[2]));
  
  return average_lane_velocities;
}

Eigen::VectorXi SensorFusion::GetLaneOccupancyForAllLanes() {
  Eigen::VectorXi lane_occupancy(3);
  lane_occupancy.setZero();
  
  for (int i=0; i < number_lanes_; ++i) {
    std::vector<VehicleModel*> vehicles = GetAllVehiclesForLane(i);
    
    for (auto& obj: vehicles) {
      double delta_s = obj->s_ - host_vehicle_.s_;
      
      if (delta_s > 0.0 && delta_s < kMaxDistanceLaneOccupancy_) {
        lane_occupancy[i] += 1;
      }
    }
  }
  return lane_occupancy;
}
