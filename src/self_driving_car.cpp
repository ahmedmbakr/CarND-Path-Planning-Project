#include "self_driving_car.h"
#include "Road_points.h"
#include "helpers.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

Self_driving_car::Self_driving_car(const Road_points &road_points) : road_points(road_points)
{
}


Self_driving_car::~Self_driving_car()
{
}

vector<vector<double>> Self_driving_car::move_forward_in_current_lane()
{
	int lane_num = 1;//lane numbers are 0->left-lane, 1 middle-lane, 2->right-lane
	float ref_velocity = 49.5;

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
	vector<double> next_mp0 = getXY(car_s + 30, car_d, road_points.map_waypoints_s, road_points.map_waypoints_x, road_points.map_waypoints_y);
	vector<double> next_mp1 = getXY(car_s + 60, car_d, road_points.map_waypoints_s, road_points.map_waypoints_x, road_points.map_waypoints_y);
	vector<double> next_mp2 = getXY(car_s + 90, car_d, road_points.map_waypoints_s, road_points.map_waypoints_x, road_points.map_waypoints_y);

	ptsx.push_back(next_mp0[0]);
	ptsx.push_back(next_mp1[0]);
	ptsx.push_back(next_mp2[0]);

	ptsy.push_back(next_mp0[1]);
	ptsy.push_back(next_mp1[1]);
	ptsy.push_back(next_mp2[1]);

	//shift x, y, theta so that the car is at 0,0 and its heading is at 0 degrees
	for (int i = 0;i < ptsx.size(); ++i)
	{
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

	}
	tk::spline s;//create a spline that creates a cubic equation that passes to all points, we can also use interpolation

	s.set_points(ptsx, ptsy);//set x, y points to the spline


							 //define actual x, y points that we will use for the planner
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	for (int i = 0;i < previous_path_x.size(); ++i)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	//calculate how to break up spline points so that we travel at our desired reference velocity
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x * target_x + target_y * target_y);

	double x_add_on = 0;

	//fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
	for (int i = 1; i <= 50 - previous_path_x.size();++i)
	{
		double N = (target_dist / (0.02 * ref_velocity / 2.24));
		double x_point = x_add_on + (target_x) / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		//rotate back to normal after rotating it earlier
		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
		
	}
	return{ next_x_vals, next_y_vals };
}
