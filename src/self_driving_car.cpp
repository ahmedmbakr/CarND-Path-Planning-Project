#include "self_driving_car.h"
#include "Road_points.h"
#include "helpers.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

std::vector<double> Self_driving_car::convert_frenet_to_cartesian_coordinates(const double s,const double d) const
{
	return getXY(s, d, road_points.map_waypoints_s, road_points.map_waypoints_x, road_points.map_waypoints_y);
}

//car reference yaw must be in radians
//This function changes the first two parameters [ptsx, ptsy]
void Self_driving_car::transform_from_world_to_car_coordinates(std::vector<double>& ptsx, std::vector<double>& ptsy, const double car_ref_x, const double car_ref_y, const double car_ref_yaw)
{
	//shift x, y, theta so that the car is at 0,0 and its heading is at 0 degrees
	for (int i = 0;i < ptsx.size(); ++i)
	{
		double shift_x = ptsx[i] - car_ref_x;
		double shift_y = ptsy[i] - car_ref_y;

		ptsx[i] = (shift_x * cos(0 - car_ref_yaw) - shift_y * sin(0 - car_ref_yaw));
		ptsy[i] = (shift_x * sin(0 - car_ref_yaw) + shift_y * cos(0 - car_ref_yaw));

	}
}

vector<double> Self_driving_car::transform_from_car_to_world_coordinates(const double ptx_car_coordinates,
	const double pty_car_coordinates, const double car_ref_x, const double car_ref_y, const double car_ref_yaw) const
{
	//rotate back to normal after rotating it earlier
	double x_point = car_ref_x + (ptx_car_coordinates * cos(car_ref_yaw) - pty_car_coordinates * sin(car_ref_yaw));
	double y_point = car_ref_y + (ptx_car_coordinates * sin(car_ref_yaw) + pty_car_coordinates * cos(car_ref_yaw));

	return{ x_point, y_point };
}

Self_driving_car::Self_driving_car(const Road_points &road_points) : road_points(road_points)
{
}


Self_driving_car::~Self_driving_car()
{
}

vector<vector<double>> Self_driving_car::move_forward_in_current_lane()
{
	int lane_num = 1;//lane numbers are 0->left-lane, 1 middle-lane, 2->right-lane
	float ref_velocity = CAR_MAX_VELOCITY;

	int prev_size = previous_path_x.size();
	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	if (prev_size < 2)
	{
		double prev_car_x = car_x - cos(ref_yaw);
		double prev_car_y = car_y - sin(ref_yaw);
		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);
	}
	else
	{
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];
		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}
	vector<double> next_mp0 = convert_frenet_to_cartesian_coordinates(car_s + 30, car_d);
	vector<double> next_mp1 = convert_frenet_to_cartesian_coordinates(car_s + 60, car_d);
	vector<double> next_mp2 = convert_frenet_to_cartesian_coordinates(car_s + 90, car_d);

	ptsx.push_back(next_mp0[0]);
	ptsx.push_back(next_mp1[0]);
	ptsx.push_back(next_mp2[0]);

	ptsy.push_back(next_mp0[1]);
	ptsy.push_back(next_mp1[1]);
	ptsy.push_back(next_mp2[1]);

	transform_from_world_to_car_coordinates(ptsx, ptsy, ref_x, ref_y, ref_yaw);

	tk::spline s;//create a spline that creates a cubic equation that passes to all points, we can also use interpolation
	s.set_points(ptsx, ptsy);//set x, y points to the spline

	//define actual x, y points that we will use for the planner
	vector<double> next_x_vals;
	vector<double> next_y_vals;
	//fill next_x, next_y with the previous points that is not used by the simulator to move the car yet
	for (int i = 0;i < previous_path_x.size(); ++i)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	//calculate how to break up spline points so that we travel at our desired reference velocity
	const double target_x = 30.0;//Draw a trajectory for 30 meters ahead the car's current position
	const double target_y = s(target_x);
	const double target_dist = sqrt(target_x * target_x + target_y * target_y);

	double x_add_on = 0;
	const double N = target_dist / (CAR_UPDATE_POSITION_RATE * convert_mph_to_mps(ref_velocity));

	//fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
	for (int i = 0; i < NUM_POINTS_FOR_TRAJECTORY - previous_path_x.size();++i)
	{
		double x_point = x_add_on + (target_x / N);
		double y_point = s(x_point);

		x_add_on = x_point;

		vector<double> x_y_point = transform_from_car_to_world_coordinates(x_point, y_point, ref_x, ref_y,
			ref_yaw);
		next_x_vals.push_back(x_y_point[0]);
		next_y_vals.push_back(x_y_point[1]);
		
	}
	return{ next_x_vals, next_y_vals };
}
