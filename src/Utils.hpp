//
//  Utils.hpp
//  path_planning
//
//  Created by Sven Bone on 22.11.17.
//

#ifndef Utils_hpp
#define Utils_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

const string logging_file = "logging.txt";                 // logging filename

const double kControllerCycleTime = 0.02;                  // Cycle time of the vehicle controller between two trajectory points [s]
const int    kNumberPredictionsPoints = 50;                // number of prediction points for e.g. the trajectory planner or vehicle model
const double kPredictionTime = kControllerCycleTime * (double)kNumberPredictionsPoints;


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

/**
 Write status, vehicle model predictions, etc. to a csv file.
 */
void WriteLogFile(int id, vector<double>predictions_x, vector<double> predictions_y);

#endif /* Utils_hpp */
