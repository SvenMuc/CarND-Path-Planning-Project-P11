//
//  SensorFusion.cpp
//  path_planning
//
//  Created by Sven Bone on 21.11.17.
//

#include "SensorFusion.hpp"

SensorFusion::SensorFusion() {
}

SensorFusion::~SensorFusion() {
}

void SensorFusion::SetHostVehicleModel(double x, double y, double s, double d, double yaw, double velocity) {
  host_vehicle_ = VehicleModel(x, y, s, d, yaw, velocity);
}

void SensorFusion::SetSpeedLimitsForLanes(std::vector<double> speed_limits) {
  speed_limits_ = speed_limits_;
}

void SensorFusion::AddVehicleModel(VehicleModel* model) {
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

VehicleModel* SensorFusion::GetNextVehicleDrivingAhead(int lane) {
  VehicleModel* next_vehicle = NULL;
  
  for (auto& obj: object_map_) {
    if (obj.second->d_ < (2 + kLaneWidth * lane + (kLaneWidth/2.0)) &&
        obj.second->d_ > (2 + kLaneWidth * lane - (kLaneWidth/2.0))) {
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
    if (obj.second->d_ < (2 + kLaneWidth * lane + (kLaneWidth/2.0)) &&
        obj.second->d_ > (2 + kLaneWidth * lane - (kLaneWidth/2.0))) {
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

double SensorFusion::GetSpeedLimitForLane(int lane) {
  if (lane >= 0.0 && lane < speed_limits_.size()) {
    return speed_limits_[lane];
  } else {
    return 0.0;
  }
}

std::ostream& operator<< (std::ostream& os, const SensorFusion& obj) {
  os << "SensorFusion(" << std::endl << " objec_map_=" << std::endl;
  
  for (auto& model: obj.object_map_) {
    os << "  " << *model.second << std::endl;
  }
  
  os << " host_vehicle_=" << std::endl << "  " << obj.host_vehicle_ << std::endl;
  os << ")" << std::endl;
  return os;
}
