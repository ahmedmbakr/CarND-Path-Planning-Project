#pragma once
#include <vector>

class Road_points
{
	std::vector<double> map_waypoints_x;
	std::vector<double> map_waypoints_y;
	std::vector<double> map_waypoints_s;
	std::vector<double> map_waypoints_dx;
	std::vector<double> map_waypoints_dy;
public:

	Road_points(const std::vector<double>& map_waypoints_x, const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s, const std::vector<double>& map_waypoints_dx,
		const std::vector<double>& map_waypoints_dy)
		: map_waypoints_x(map_waypoints_x),
		map_waypoints_y(map_waypoints_y),
		map_waypoints_s(map_waypoints_s),
		map_waypoints_dx(map_waypoints_dx),
		map_waypoints_dy(map_waypoints_dy)
	{
	}

	double get_map_waypoints_x(const int idx) const
	{
		return map_waypoints_x[idx % map_waypoints_x.size()];
	}

	double get_map_waypoints_y(const int idx) const
	{
		return map_waypoints_y[idx % map_waypoints_x.size()];
	}

	double get_map_waypoints_s(const int idx) const
	{
		return map_waypoints_s[idx % map_waypoints_x.size()];
	}

	double get_map_waypoints_dx(const int idx) const
	{
		return map_waypoints_dx[idx % map_waypoints_x.size()];
	}

	double get_map_waypoints_dy(const int idx) const
	{
		return map_waypoints_dy[idx % map_waypoints_x.size()];
	}

	int get_num_points() const
	{
		return map_waypoints_x.size();
	}
};
