//
//  Utils.hpp
//  path_planning
//
//  Created by Sven Bone on 22.11.17.
//

#ifndef Utils_hpp
#define Utils_hpp

#include <stdio.h>
#include <vector>

using namespace std;

/**
 Returns PI.
 */
constexpr double pi();

/**
 Converts degree to rad.
 */
double deg2rad(double x);

/**
 Converts rad to degree.
 */
double rad2deg(double x);

/**
 */
double distance(double x1, double y1, double x2, double y2);

/**
 */
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

/**
 */
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

/**
 Transform from Cartesian x,y coordinates to Frenet s,d coordinates.
 */
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

/**
 Transform from Frenet s,d coordinates to Cartesian x,y.
 */
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif /* Utils_hpp */
