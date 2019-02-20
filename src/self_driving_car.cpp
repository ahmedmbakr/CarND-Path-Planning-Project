#include "self_driving_car.h"
#include "Road_points.h"
#include "helpers.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "sensor_fusion_car.h"
#include <iostream>
#include <openssl/rsa.h>
#include <functional>

using namespace std;

std::vector<std::vector<double>> Self_driving_car::move()
{
	std::vector<std::vector<double>> x_y_trajectory_points;
	double target_velocity_at_end_of_trajectory = 0;//TODO: dummy and may need to be removed

	switch(this->state.get_current_state())
	{
	default:
	{//TODO: to be changed later when the preparation states are implemented
		cout << "ERROR: Undefined state !!! I will keep lane\n";
		this->state.update_current_state(KEEP_LANE);
	}
	case KEEP_LANE: 
		cout << "execute keep_lane\n";
		x_y_trajectory_points = move_forward_in_current_lane();
		break;
	case CHANGE_LANE_LEFT:
		cout << "execute change_lane_left\n";
		x_y_trajectory_points = move_to_change_lane_left();
		break;
	case CHANGE_LANE_RIGHT:
		cout << "execute change_lane_right\n";
		x_y_trajectory_points = move_to_change_lane_right();
		break;
	case PREP_CHANGE_LANE_LEFT:
		cout << "execute prep_change_lane_left\n";
		x_y_trajectory_points = move_to_prep_change_lane_left();
		break;
	case PREP_CHANGE_LANE_RIGHT:
		cout << "execute prep_change_lane_right\n";
		x_y_trajectory_points = move_to_prep_change_lane_right();
		break;
	}
	return x_y_trajectory_points;
}

std::vector<double> Self_driving_car::convert_frenet_to_cartesian_coordinates(const double s,const double d) const
{
	return getXY(s, d, road_points);
}

std::vector<double> Self_driving_car::convert_cartesian_to_frenet_coordinates(const double x, const double y) const
{
	return getFrenet(x, y, deg2rad(this->car_yaw), road_points);
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

Self_driving_car::Self_driving_car(const Road_points &road_points) : car_x(0), car_y(0), car_s(0), car_d(0), car_yaw(0),
                                                                     car_speed(0), end_path_s(0), end_path_d(0),
                                                                     road_points(road_points), ref_velocity(0)
{
}


Self_driving_car::~Self_driving_car()
{
}

std::vector<std::vector<double>> Self_driving_car::move_to_change_lane_left( )
{
	double target_lane = convert_frenet_d_coord_to_lane_num(this->car_d) - 1;
	vector<vector<double>> x_y_trajectory = change_to_lane(target_lane);
	return x_y_trajectory;
}

std::vector<std::vector<double>> Self_driving_car::move_to_change_lane_right( )
{
	double target_lane = convert_frenet_d_coord_to_lane_num(this->car_d) + 1;
	vector<vector<double>> x_y_trajectory = change_to_lane(target_lane);
	return x_y_trajectory;
}

std::vector<std::vector<double>> Self_driving_car::move_to_prep_change_lane_left( )
{
	static int counter = 0;//number of times the state remains in prepare change lane left
	const int time_in_sec = counter * CAR_UPDATE_POSITION_RATE;
	if(time_in_sec == 5)//if I am in this mode for 5 seconds
	{
		counter = 0;//reset the counter
		this->state.update_current_state(KEEP_LANE);//Abort and get back to follow the car in front of us
		cout << "Finished prep_lane_shift, and going to keep lane because of timeout\n";
	}
	else
	{
		counter++;
	}
	const int current_lane = convert_frenet_d_coord_to_lane_num(this->get_car_d());
	const int intended_lane = current_lane - 1;
	Sensor_fusion_car* nearest_car_in_intended_lane =  get_nearest_car_in_lane(intended_lane, CAR_SAFE_DISTANCE_M);
	Sensor_fusion_car* nearest_car_in_current_lane = get_car_exist_in_front_of_us(CAR_SAFE_DISTANCE_M, current_lane);
	double target_velocity_at_end_of_trajectory = this->ref_velocity;
	if(nearest_car_in_current_lane != nullptr)
	{
		//we want the speed to be less than the car in the other lane so that we can pass it
		target_velocity_at_end_of_trajectory = nearest_car_in_current_lane->get_car_speed();
	}
	if(nearest_car_in_intended_lane == nullptr)
	{
		counter = 0;//reset the counter
		this->state.update_current_state(CHANGE_LANE_LEFT);//you are safe to change to right lane
		target_velocity_at_end_of_trajectory = nearest_car_in_intended_lane->get_car_speed();
		cout << "Finished prep_lane_shift, and going to change_lane_left\n";
	}
	else 
	{
		target_velocity_at_end_of_trajectory = min(target_velocity_at_end_of_trajectory, nearest_car_in_intended_lane->get_car_speed());
	}
	//TODO: we may need to add the speed here to use target_velocity_at_end_of_trajectory variable
	return move_forward_in_current_lane();
}

std::vector<std::vector<double>> Self_driving_car::move_to_prep_change_lane_right()
{
	static int counter = 0;//number of times the state remains in prepare change lane left
	const int time_in_sec = counter * CAR_UPDATE_POSITION_RATE;
	if (time_in_sec == 5)//if I am in this mode for 5 seconds
	{
		counter = 0;//reset the counter
		this->state.update_current_state(KEEP_LANE);//Abort and get back to follow the car in front of us
		cout << "Finished prep_lane_shift, and going to keep lane because of timeout\n";
	}
	else
	{
		counter++;
	}
	const int current_lane = convert_frenet_d_coord_to_lane_num(this->get_car_d());
	const int intended_lane = current_lane + 1;
	Sensor_fusion_car* nearest_car_in_intended_lane = get_nearest_car_in_lane(intended_lane, CAR_SAFE_DISTANCE_M);
	Sensor_fusion_car* nearest_car_in_current_lane = get_car_exist_in_front_of_us(CAR_SAFE_DISTANCE_M, current_lane);
	double target_velocity_at_end_of_trajectory = this->ref_velocity;
	if (nearest_car_in_current_lane != nullptr)
	{
		//we want the speed to be less than the car in the other lane so that we can pass it
		target_velocity_at_end_of_trajectory = nearest_car_in_current_lane->get_car_speed() - 2;
	}
	if (nearest_car_in_intended_lane == nullptr)
	{
		counter = 0;//reset the counter
		this->state.update_current_state(CHANGE_LANE_RIGHT);//you are safe to change to right lane
		cout << "Finished prep_lane_shift, and going to change_lane_right\n";
	}
	else
	{
		target_velocity_at_end_of_trajectory = min(target_velocity_at_end_of_trajectory, nearest_car_in_intended_lane->get_car_speed()) ;
	}
	//TODO: we may need to add the speed here to use target_velocity_at_end_of_trajectory variable
	return move_forward_in_current_lane();
}

vector<vector<double>> Self_driving_car::move_forward_in_current_lane( )
{
	int our_lane = convert_frenet_d_coord_to_lane_num(this->get_car_d());
	double target_velocity_at_end_of_trajectory = get_traffic_speed_in_lane(our_lane, CAR_SAFE_DISTANCE_M);
	Sensor_fusion_car* car_exist_in_front_of_us = get_car_exist_in_front_of_us(CAR_SAFE_DISTANCE_M, our_lane);
	if (car_exist_in_front_of_us != nullptr)
	{
		states new_state = is_eligible_to_change_lane();
		this->state.update_current_state(new_state);
	}

	assert(our_lane >= 0 && our_lane < NUM_LANES);
	return move_to_lane(our_lane, target_velocity_at_end_of_trajectory);
}

vector<vector<double>> Self_driving_car::change_to_lane(double target_lane)
{
	static int intended_lane = -1;//-1 means that there is no intended lane yet
	int lane_num;//lane numbers are 0->left-lane, 1 middle-lane, 2->right-lane
	if (intended_lane == -1)
	{
		lane_num = target_lane;
		intended_lane = lane_num;
		/*Sensor_fusion_car* car_exist_in_front_of_us = get_car_exist_in_front_of_us(CAR_SAFE_DISTANCE_M, intended_lane);
		if (car_exist_in_front_of_us != nullptr)
		{
			target_velocity_at_end_of_trajectory = min(target_velocity_at_end_of_trajectory,
				car_exist_in_front_of_us->get_car_speed());
		}*/
	}
	else
	{
		lane_num = intended_lane;
	}
	double target_velocity_at_end_of_trajectory = get_traffic_speed_in_lane(intended_lane, CAR_SAFE_DISTANCE_M);
	assert(lane_num >= 0 && lane_num < NUM_LANES);
	vector<std::vector<double>> x_y_trajectory = move_to_lane(lane_num, target_velocity_at_end_of_trajectory);
	static int count_num_times_car_is_intended_lane = 0;
	if (convert_frenet_d_coord_to_lane_num(this->get_car_d()) == intended_lane)
	{
		count_num_times_car_is_intended_lane++;
	}
	if (count_num_times_car_is_intended_lane == static_cast<int>((1 / CAR_UPDATE_POSITION_RATE) / 10))
	{
		count_num_times_car_is_intended_lane = 0;
		intended_lane = -1;
		this->state.update_current_state(KEEP_LANE);
		cout << "finished implementing lane change, and moving back to keep_lane\n";
	}
	return x_y_trajectory;
}

std::vector<std::vector<double>> Self_driving_car::move_to_lane(int lane_num,
	double target_velocity_at_end_of_trajectory)
{
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

	const double d_val = convert_lane_num_to_d(lane_num);
	for(int i = 1; i <= 3;++i)
	{
		vector<double> next_mp = convert_frenet_to_cartesian_coordinates(car_s + i * 30 + 15, d_val);//TODO: you need to change this to 60 or even higher if you change NUM_POINTS_FOR_TRAJECTORY or you will get an error
		ptsx.push_back(next_mp[0]);
		ptsy.push_back(next_mp[1]);
	}

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
	const double target_x = 60.0;//Draw a trajectory for 60 meters ahead the car's current position
	const double target_y = s(target_x);
	const double target_dist = sqrt(target_x * target_x + target_y * target_y);
	const int num_remaining_points_to_compute = NUM_POINTS_FOR_TRAJECTORY - previous_path_x.size();
	double x_add_on = 0;
	const double velocity_inc_mps = get_velocity_increment_rate(ref_velocity, target_velocity_at_end_of_trajectory,
		num_remaining_points_to_compute);
	//fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
	for (int i = 0; i < num_remaining_points_to_compute;++i)
	{
		ref_velocity += velocity_inc_mps;
		const double N = target_dist / (CAR_UPDATE_POSITION_RATE * convert_mph_to_mps(ref_velocity));
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

//Get the speed of the car in the lane specified within range[-safe_dist ... safe_dist]
double Self_driving_car::get_traffic_speed_in_lane(int lane_num, double safe_dist_m)
{
	double car_s = this->car_s;
	if (previous_path_x.size() > 0)
	{
		car_s = end_path_s;
	}
	for (int i = 0; i < this->sensor_fusion_cars.size(); ++i)
	{
		Sensor_fusion_car a_sensor_fusion_car = this->sensor_fusion_cars[i];
		double other_d = a_sensor_fusion_car.get_d();
		int other_lane = convert_frenet_d_coord_to_lane_num(other_d);
		if (lane_num == other_lane)
		{
			double other_vx = a_sensor_fusion_car.get_vx();
			double other_vy = a_sensor_fusion_car.get_vy();
			double other_speed_magnitude = sqrt(other_vx * other_vx + other_vy * other_vy);
			double other_s = a_sensor_fusion_car.get_s();

			other_s += this->previous_path_x.size() * CAR_UPDATE_POSITION_RATE * other_speed_magnitude;
			if ( (other_s < car_s && car_s - other_s < safe_dist_m)
				|| (other_s > car_s && other_s - car_s < safe_dist_m))
			{
				return other_speed_magnitude;
			}
		}
	}
	return CAR_MAX_VELOCITY;
}

Sensor_fusion_car* Self_driving_car::get_car_exist_in_front_of_us(const float safe_dist_m, int lane_to_search_in)
{
	double car_s = this->car_s;
	if(previous_path_x.size() > 0)
	{
		car_s = end_path_s;
	}
	for(int i = 0; i < this->sensor_fusion_cars.size(); ++i)
	{
		Sensor_fusion_car a_sensor_fusion_car = this->sensor_fusion_cars[i];
		double other_d = a_sensor_fusion_car.get_d();
		int other_lane = convert_frenet_d_coord_to_lane_num(other_d);
		if(lane_to_search_in == other_lane)
		{
			double other_speed_magnitude = a_sensor_fusion_car.get_car_speed();
			double other_s = a_sensor_fusion_car.get_s();

			other_s += this->previous_path_x.size() * CAR_UPDATE_POSITION_RATE * other_speed_magnitude;
			if(other_s > car_s && other_s - car_s < safe_dist_m)
			{
				return &this->sensor_fusion_cars[i];
			}
		}
	}
	return nullptr;
}

//The nearest car can be behind or above us in a specific lane
Sensor_fusion_car* Self_driving_car::get_nearest_car_in_lane(const int lane_num, const double safe_dist_m)
{
	double car_s = this->car_s;
	if (previous_path_x.size() > 0)
	{
		car_s = end_path_s;
	}
	for (int i = 0; i < this->sensor_fusion_cars.size(); ++i)
	{
		Sensor_fusion_car a_sensor_fusion_car = this->sensor_fusion_cars[i];
		double other_d = a_sensor_fusion_car.get_d();
		int other_lane = convert_frenet_d_coord_to_lane_num(other_d);
		if (lane_num == other_lane)
		{
			double other_speed_magnitude = a_sensor_fusion_car.get_car_speed();
			double other_s = a_sensor_fusion_car.get_s();

			other_s += this->previous_path_x.size() * CAR_UPDATE_POSITION_RATE * other_speed_magnitude;
			if (abs(other_s - car_s) < safe_dist_m)
			{
				return &this->sensor_fusion_cars[i];
			}
		}
	}
	return nullptr;
}

double Self_driving_car::get_velocity_increment_rate(double v0, double vt, int n)
{
	double velocity_inc = (vt - v0) / n;
	int sign = velocity_inc < 0 ? -1 : 1;
	if (abs(velocity_inc) >= CAR_MAX_VELOCITY_INCREMENT_RATE)
		velocity_inc = sign * CAR_MAX_VELOCITY_INCREMENT_RATE;
	return velocity_inc;
}

Self_driving_car::states Self_driving_car::is_eligible_to_change_lane()
{
	const int current_lane_num = convert_frenet_d_coord_to_lane_num(this->car_d);
	vector<double> lane_speeds(NUM_LANES);
	int max_lane_speed_idx = 0;
	double max_lane_speed = 0;
	Sensor_fusion_car* nearest_car_in_the_fastest_lane = nullptr;
	for(int lane_num = 0;lane_num < NUM_LANES; ++lane_num)
	{
		double lane_speed = CAR_MAX_VELOCITY;
		Sensor_fusion_car* car_in_a_lane = get_nearest_car_in_lane(lane_num, CAR_SAFE_DISTANCE_M);
		lane_speed = car_in_a_lane == nullptr ? CAR_MAX_VELOCITY : car_in_a_lane->get_car_speed();
		if(lane_num == current_lane_num)
		{
			lane_speed = min(lane_speed, this->ref_velocity);
		}

		if (max_lane_speed < lane_speed 
			|| (max_lane_speed == lane_speed 
			    && abs(max_lane_speed_idx - current_lane_num) > abs(lane_num - current_lane_num)))
		{
			max_lane_speed = lane_speed;
			max_lane_speed_idx = lane_num;
			nearest_car_in_the_fastest_lane = car_in_a_lane;
		}
		cout << "lane "<< lane_num <<" speed: " << lane_speed << endl;
		lane_speeds[lane_num] = lane_speed;
	}
	cout << "..................\n";
	if(max_lane_speed_idx != current_lane_num && nearest_car_in_the_fastest_lane != nullptr)
	{
		return KEEP_LANE;
	}
	if(max_lane_speed_idx < current_lane_num)
	{//We should try to go left
		if (max_lane_speed_idx + 1 == current_lane_num)
			return CHANGE_LANE_LEFT;
		else
			return PREP_CHANGE_LANE_LEFT;
	}
	else if (max_lane_speed_idx > current_lane_num)
	{//We should try to go right
		if (max_lane_speed_idx - 1 == current_lane_num)
			return CHANGE_LANE_RIGHT;
		else
			return PREP_CHANGE_LANE_RIGHT;
	}
	return KEEP_LANE;
}

Self_driving_car::State::State()
{
	this->current_state = KEEP_LANE;
	this->previous_state = CHANGE_LANE_LEFT;//dummy init state which is different from the current state
}

void Self_driving_car::State::update_current_state(const states new_state)
{
	if(new_state != this->current_state)
	{
		this->previous_state = this->current_state;
		this->current_state = new_state;
	}
}