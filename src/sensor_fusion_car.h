#pragma once
#include <vector>

class Sensor_fusion_car
{
	int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;

public:

	Sensor_fusion_car(const int id, const double x, const double y, const double vx, const double vy, const double s,
		const double d)
		: id(id),
		  x(x),
		  y(y),
		  vx(vx),
		  vy(vy),
		  s(s),
		  d(d)
	{
	}


	int get_id() const
	{
		return id;
	}

	double get_x() const
	{
		return x;
	}

	double get_y() const
	{
		return y;
	}

	double get_vx() const
	{
		return vx;
	}

	double get_vy() const
	{
		return vy;
	}

	double get_s() const
	{
		return s;
	}

	double get_d() const
	{
		return d;
	}
};

class Sensor_fusion_builder
{
public:
	const static std::vector<Sensor_fusion_car> parse_sensor_fusion_vec(const std::vector<std::vector<double>> &sensor_fusion);
};