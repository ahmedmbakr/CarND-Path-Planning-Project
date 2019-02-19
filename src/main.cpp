#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "Road_points.h"
#include "self_driving_car.h"
#include "sensor_fusion_car.h"
#include "helpers.h"

// for convenience
using nlohmann::json;
using namespace std;

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors

	// Waypoint map to read from
	string file_path = __FILE__;
	string dir_path = file_path.substr(0, file_path.rfind("\\"));//The path of the main.cpp file
	string map_file_ = dir_path + "/../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
	string line;
	while (getline(in_map_, line)) {
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
	const Road_points road_points(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
	Self_driving_car car(road_points);

	h.onMessage([&car]
		(uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
			uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner that the simulator does not use to move the car yet
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side 
					//   of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];
					const vector<Sensor_fusion_car> sensor_fusion_cars = Sensor_fusion_builder::parse_sensor_fusion_vec(
						sensor_fusion);
					car.set_sensor_fusion_cars(sensor_fusion_cars);

					json msgJson;

					car.set_car_x(car_x);
					car.set_car_y(car_y);
					car.set_car_s(car_s);
					car.set_car_d(car_d);
					car.set_car_yaw(car_yaw);
					car.set_car_speed(car_speed);
					car.set_previous_path_x(previous_path_x);
					car.set_previous_path_y(previous_path_y);
					car.set_end_path_s(end_path_s);
					car.set_end_path_d(end_path_d);

					auto x_y_vector_vals = car.move();

					msgJson["next_x"] = x_y_vector_vals[0];
					msgJson["next_y"] = x_y_vector_vals[1];

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}  // end "telemetry" if
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}  // end websocket if
	}); // end h.onMessage

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
		char *message, size_t length) {
		ws->close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	auto host = "127.0.0.1";
	if (h.listen(host, port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}