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

void SensorFusion::AddVehicleModel(VehicleModel* model) {
    objectList_.push_back(model);
}

void SensorFusion::RemoveAll() {
    for (auto obj: objectList_) {
        delete obj;
    }
    
    objectList_.clear();
}

int SensorFusion::CountVehicleModels() {
    return objectList_.size();
}

VehicleModel* SensorFusion::GetNextVehicleDrivingAhead(int lane) {
    VehicleModel* next_vehicle = NULL;
    
    for (auto obj: objectList_) {
        if (obj->d_ < (2 + lane_width_ * lane + (lane_width_/2.0)) &&
            obj->d_ > (2 + lane_width_ * lane - (lane_width_/2.0))) {
            // vehicle found in given lane
            // check if it is the initial one or closer than the last stored one
            double delta_s_object = obj->s_ - host_vehicle_.s_;
            
            if (delta_s_object >= 0 && next_vehicle == NULL) {
                next_vehicle = obj;
            } else if (delta_s_object >= 0 && next_vehicle) {
                double delta_s_next = next_vehicle->s_ - host_vehicle_.s_;
                
                if (delta_s_object < delta_s_next) {
                    next_vehicle = obj;
                }
            }
        }
    }
    
    return next_vehicle;
}

VehicleModel* SensorFusion::GetNextVehicleDrivingBehind(int lane) {
    VehicleModel* next_vehicle = NULL;
    
    for (auto obj: objectList_) {
        if (obj->d_ < (2 + lane_width_ * lane + (lane_width_/2.0)) &&
            obj->d_ > (2 + lane_width_ * lane - (lane_width_/2.0))) {
            // vehicle found in given lane
            // check if it is the initial one or closer than the last stored one
            double delta_s_object = obj->s_ - host_vehicle_.s_;
            
            if (delta_s_object < 0 && next_vehicle == NULL) {
                next_vehicle = obj;
            } else if (delta_s_object < 0 && next_vehicle) {
                double delta_s_next = next_vehicle->s_ - host_vehicle_.s_;
                
                if (delta_s_object > delta_s_next) {
                    next_vehicle = obj;
                }
            }
        }
    }
    
    return next_vehicle;
}

std::ostream& operator<< (std::ostream& os, const SensorFusion& obj) {
    os << "SensorFusion(" << std::endl << " objectList_=" << std::endl;
    
    for (auto model: obj.objectList_) {
        os << "  " << *model << std::endl;
    }

    os << " host_vehicle_=" << std::endl << "  " << obj.host_vehicle_ << std::endl;
    os << ")" << std::endl;
    return os;
}
