#include "sensor_fusion_car.h"
using namespace std;

const std::vector<Sensor_fusion_car> Sensor_fusion_builder::parse_sensor_fusion_vec(
	const std::vector<std::vector<double>>& sensor_fusion)
{
	vector<Sensor_fusion_car> other_cars;
	for (vector<double> a_sensor_fusion : sensor_fusion)
	{
		int id = a_sensor_fusion[0];
		int x = a_sensor_fusion[1];
		int y = a_sensor_fusion[2];
		int vx = a_sensor_fusion[3];
		int vy = a_sensor_fusion[4];
		int s = a_sensor_fusion[5];
		int d = a_sensor_fusion[6];

		Sensor_fusion_car a_car(id, x, y, vx, vy, s, d);
		other_cars.push_back(a_car);
	}
	return other_cars;
}
