#include "helpers.h"
#include "Road_points.h"

#define M_PI 3.14159265358979323846264338327950288

// for convenience
using std::string;
using std::vector;

//   else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const Road_points& road_points) {
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < road_points.get_num_points(); ++i) {
		double map_x = road_points.get_map_waypoints_x(i);
		double map_y = road_points.get_map_waypoints_y(i);
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const Road_points& road_points) {
	int closestWaypoint = ClosestWaypoint(x, y, road_points);

	double map_x = road_points.get_map_waypoints_x(closestWaypoint);
	double map_y = road_points.get_map_waypoints_y(closestWaypoint);

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = std::min(2 * pi() - angle, angle);

	if (angle > pi() / 2) {
		++closestWaypoint;
		if (closestWaypoint == road_points.get_num_points()) {
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
	const Road_points& road_points) {
	int next_wp = NextWaypoint(x, y, theta, road_points);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0) {
		prev_wp = road_points.get_num_points() - 1;
	}

	double n_x = road_points.get_map_waypoints_x(next_wp) - road_points.get_map_waypoints_x(prev_wp);
	double n_y = road_points.get_map_waypoints_y(next_wp) - road_points.get_map_waypoints_y(prev_wp);
	double x_x = x - road_points.get_map_waypoints_x(prev_wp);
	double x_y = y - road_points.get_map_waypoints_y(prev_wp);

	// find the projection of x onto n
	double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000 - road_points.get_map_waypoints_x(prev_wp);
	double center_y = 2000 - road_points.get_map_waypoints_y(prev_wp);
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; ++i) {
		frenet_s += distance(road_points.get_map_waypoints_x(i), road_points.get_map_waypoints_y(i),
			road_points.get_map_waypoints_x(i + 1), road_points.get_map_waypoints_y(i + 1));
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return{ frenet_s,frenet_d };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const Road_points& road_points) {
	int prev_wp = -1;

	while (s > road_points.get_map_waypoints_s(prev_wp + 1) && (prev_wp < (int)(road_points.get_num_points() - 1))) {
		++prev_wp;
	}

	int wp2 = (prev_wp + 1) % road_points.get_num_points();

	double heading = atan2((road_points.get_map_waypoints_y(wp2) - road_points.get_map_waypoints_y(prev_wp)),
		(road_points.get_map_waypoints_x(wp2) - road_points.get_map_waypoints_x(prev_wp)));
	// the x,y,s along the segment
	double seg_s = (s - road_points.get_map_waypoints_s(prev_wp));

	double seg_x = road_points.get_map_waypoints_x(prev_wp) + seg_s*cos(heading);
	double seg_y = road_points.get_map_waypoints_y(prev_wp) + seg_s*sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return{ x,y };
}

/*This function converts the lane number to the (d) part of the Frenet coordinate
*Inputs: lane-num which must have these values [0->most-left ... (NUM_LANES-1)->most-right]
*Outputs: d part of the frenet coordinate
**/
int convert_lane_num_to_d(const int lane_num)
{
	assert(lane_num >= 0 && lane_num < NUM_LANES);
	return((LANE_WIDTH / 2) + LANE_WIDTH * lane_num);
}

//converts miles/hour to meters/second
double convert_mph_to_mps(double mph)
{
	return(mph * 0.44704);
}

const int convert_frenet_d_coord_to_lane_num(const double d)
{
	int car_d = d;
	const int lane_num = d / LANE_WIDTH;
	if (lane_num < 0)return 0;
	if (lane_num >= NUM_LANES)return NUM_LANES - 1;
	return lane_num;
}

