#pragma once

#include <math.h>
#include <cmath>
#include <string>
#include <vector>
#include "Constants.h"
#include <assert.h>
#include <algorithm>

class Road_points;

std::string hasData(std::string s);

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const Road_points& road_points);

int NextWaypoint(double x, double y, double theta, const Road_points& road_points);

std::vector<double> getFrenet(double x, double y, double theta,
	const Road_points& road_points);

std::vector<double> getXY(double s, double d, const Road_points& road_points);

int convert_lane_num_to_d(const int lane_num);

double convert_mph_to_mps(double mph);

const int convert_frenet_d_coord_to_lane_num(const double d);

