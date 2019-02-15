#pragma once
#include <vector>

class Road_points;

class Self_driving_car
{
public:
	double get_car_x() const
	{
		return car_x;
	}

	void set_car_x(double car_x)
	{
		this->car_x = car_x;
	}

	double get_car_y() const
	{
		return car_y;
	}

	void set_car_y(double car_y)
	{
		this->car_y = car_y;
	}

	double get_car_s() const
	{
		return car_s;
	}

	void set_car_s(double car_s)
	{
		this->car_s = car_s;
	}

	double get_car_d() const
	{
		return car_d;
	}

	void set_car_d(double car_d)
	{
		this->car_d = car_d;
	}

	double get_car_yaw() const
	{
		return car_yaw;
	}

	void set_car_yaw(double car_yaw)
	{
		this->car_yaw = car_yaw;
	}

	double get_car_speed() const
	{
		return car_speed;
	}

	void set_car_speed(double car_speed)
	{
		this->car_speed = car_speed;
	}

	std::vector<double> get_previous_path_x() const
	{
		return previous_path_x;
	}

	void set_previous_path_x(const std::vector<double>& previous_path_x)
	{
		this->previous_path_x = previous_path_x;
	}

	std::vector<double> get_previous_path_y() const
	{
		return previous_path_y;
	}

	void set_previous_path_y(const std::vector<double>& previous_path_y)
	{
		this->previous_path_y = previous_path_y;
	}

	double get_end_path_s() const
	{
		return end_path_s;
	}

	void set_end_path_s(double end_path_s)
	{
		this->end_path_s = end_path_s;
	}

	double get_end_path_d() const
	{
		return end_path_d;
	}

	void set_end_path_d(double end_path_d)
	{
		this->end_path_d = end_path_d;
	}

private:
	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;

	std::vector<double> previous_path_x;
	std::vector<double> previous_path_y;

	double end_path_s;
	double end_path_d;

	//auto sensor_fusion;

	const Road_points &road_points;

	std::vector<double> convert_frenet_to_cartesian_coordinates(const double s, const double d) const;
	void transform_from_world_to_car_coordinates(std::vector<double>& ptsx, std::vector<double>& ptsy, const double car_ref_x, const double car_ref_y, const double car_ref_yaw);
	std::vector<double> transform_from_car_to_world_coordinates(const double ptx_car_coordinates, const double pty_car_coordinates, const double car_ref_x, const double car_ref_y, const double car_ref_yaw) const;
public:
	Self_driving_car(const Road_points& road_points);
	~Self_driving_car();

	std::vector<std::vector<double>> move_forward_in_current_lane();
};

