#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//to smooth out the path
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//treating first lane after yellow lane as lane 0, next is lane 1 and lane 2
//start in lane 1
//int lane = 1;
int current_lane;
double curr_lead_vehicle_speed = 22.352 - 0.5;
double target_vehicle_speed;
vector<double> avg_points = { 0,0,0 };

//reference velocity to target, to be as close as speed limit as possible
//double ref_vel = 0.0; //mph

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = min(2 * pi() - angle, angle);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x + x_y * n_y) / (n_x*n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return { frenet_s,frenet_d };

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return { x,y };

}

int assignLane(double d) {
	// Check which lane the d-value comes from
	// Left lane = 0, middle lane = 1, right lane = 2
	int lane;
	if (d < 4) {
		lane = 0;
	}
	else if (d < 8) {
		lane = 1;
	}
	else {
		lane = 2;
	}
	return lane;
}

vector<double> closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction) {
	double dist = 10000;
	double velocity = 22.352 - 0.5; // Set in case of no cars
	double vehicle_s;
	double vehicle_d;
	double vehicle_v;
	int vehicle_lane;

	// Check each vehicle in sensor range
	for (int vehicle = 0; vehicle < sensor_fusion.size(); vehicle++) {
		vehicle_s = sensor_fusion[vehicle][5];
		vehicle_d = sensor_fusion[vehicle][6];
		vehicle_v = sqrt((sensor_fusion[vehicle][3], 2) * (sensor_fusion[vehicle][3], 2) + (sensor_fusion[vehicle][4], 2) * (sensor_fusion[vehicle][4], 2));
		vehicle_lane = assignLane(vehicle_d);

		if (vehicle_lane == lane) { // if same lane
			if (direction == true) {
				if (vehicle_s > s and (vehicle_s - s) < dist) { // and ahead of my vehicle
					dist = vehicle_s - s;
					velocity = vehicle_v;
				}
			}
			else {
				if (s >= vehicle_s and (s - vehicle_s) < dist) { // if behind my vehicle
					dist = s - vehicle_s;
					velocity = vehicle_v;
				}
			}
		}
	}
	if (dist <= 0) { // Avoid dividing by zero in assignPoints()
		dist = 1.0;
	}
	if (lane == current_lane and direction == true) {
		curr_lead_vehicle_speed = velocity;
	}
	return { dist, velocity };
}

int assignPoints(double s, int lane, vector<vector<double>> sensor_fusion) {
	vector <double> lane_points = { 0,0,0 };
	vector <double> front_vehicle;
	vector <double> back_vehicle;

	for (int i = 0; i < 3; i++) {
		if (i == lane) {  // assign positive points for keeping lane
			lane_points[i] += 0.5;
		}
		front_vehicle = closestVehicle(s, i, sensor_fusion, true);
		back_vehicle = closestVehicle(s, i, sensor_fusion, false);
		if (front_vehicle[0] > 1000 and back_vehicle[0] > 1000) {
			lane_points[i] += 5; // if wide open lane, move into that lane
		}
		else {
			if (front_vehicle[0] < 10 || back_vehicle[0] < 10) {
				lane_points[i] -= 5; // if car is too close in front or too close in back, assign negative points
			}
			lane_points[i] += 1 - (10 / (front_vehicle[0] / 3)); // points for large open distance in lane in front
			lane_points[i] += 1 - (10 / (back_vehicle[0] / 3)); // points for large open distance in lane in back
			lane_points[i] += 1 - (10 / (front_vehicle[1] / 2)); // points for faster car speed in lane in front
			lane_points[i] += 1 / (back_vehicle[1] / 2); // points for slower car speed in lane in back
		}
		// Simple in-exact calculation for points over the last ten iterations
		avg_points[i] = (avg_points[i] * 10) - avg_points[i];
		avg_points[i] += lane_points[i];
		avg_points[i] /= 10;
	}

	// Only compare applicable lanes
	if (lane == 0) {
		return max_element(avg_points.begin(), avg_points.end() - 1) - avg_points.begin();
	}
	else if (lane == 1) {
		return max_element(avg_points.begin(), avg_points.end()) - avg_points.begin();
	}
	else {
		return max_element(avg_points.begin() + 1, avg_points.end()) - avg_points.begin();
	}
}

int lanePlanner(double s, double d, vector<vector<double>> sensor_fusion) {
	int lane = assignLane(d);
	int new_lane;
	double distance = closestVehicle(s, lane, sensor_fusion, true)[0];

	current_lane = lane; // Keep the current lane to later calculate desired move

						 // check if blocked, i.e. car is within 20 meters
	if (distance > 20) { // if lots of space, stay in lane and go near the speed limit
		new_lane = lane;
		target_vehicle_speed = 22.352 - 0.5;
		avg_points = { 0,0,0 }; // Reset average points for assignPoints()
		return 0;
	}
	else {
		new_lane = assignPoints(s, lane, sensor_fusion);
		vector <double> vehicle = closestVehicle(s, new_lane, sensor_fusion, true);
		target_vehicle_speed = vehicle[1];
	}

	// Space between middle of each lane is four meters, so move accordingly
	if (new_lane == lane) {
		return 0;
	}
	else if (new_lane < lane) {
		return -4;
	}
	else {
		return 4;
	}
}

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
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

	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
		uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
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

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

					//define the actual (x,y) points we will use for the planner
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					//to help transition
					int prev_size = previous_path_x.size();

					/*if (prev_size > 0)
					car_s = end_path_s;*/

					//bool too_close = false;

					//use sensor fusion to avoid hitting cars infront of us

					////find ref_vel to use
					//for (int i = 0; i < sensor_fusion.size(); i++)
					//{
					//	//car is in my lane
					//	float d = sensor_fusion[i][6];
					//	if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2))
					//	{
					//		double vx = sensor_fusion[i][3];
					//		double vy = sensor_fusion[i][4];
					//		double check_speed = sqrt(vx*vx + vy * vy);
					//		double check_car_s = sensor_fusion[i][5];

					//		//if using previous points can project s value out
					//		check_car_s += ((double)prev_size*.02*check_speed);

					//		//check s values greater than mine and s gap
					//		if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
					//		{
					//			//we can have logic here to have flag to change lanes or lower the ref_vel to not hit the car infront of us
					//			//ref_vel = 29.5; //mph
					//			too_close = true;
					//			if (lane > 0)
					//				lane = 0;
					//		}
					//	}
					//}

					//if (too_close)
					//	ref_vel -= .224;
					//else if (ref_vel < 49.5)
					//	ref_vel += .224;

					//double dist_inc = 0.4; //points spaced apart
					//start with all of the previous path points from last time
					for (int i = 0; i < prev_size; i++)
					{
						//double next_s = car_s + (i +1) * dist_inc;
						//we are in middle lane, waypoints are measured from double yellow line and the lane
						//lanes are 4 meters wide (from clasroom), 1 1/2 times 4 mts wide is 6
						//double next_d = 6; 
						//vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

						//next_x_vals.push_back(xy[0]);
						//next_y_vals.push_back(xy[1]);
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					vector<double> ptsx;
					vector<double> ptsy;

					//reference x,y, yaw rates
					//either we will reference the starting point or where the car is or as the previous paths and point
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);
					double ref_vel;

					//if previous size is almost empty, use the car as starting preference
					if (prev_size < 2)
					{
						//use two points that make the path tangent to the car
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);

						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);

						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);

						ref_vel = car_speed;
					}
					//use the previous path's end point as starting reference
					else
					{
						//Redirect refence state as previous path end point
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];

						double ref_x_prev = previous_path_x[prev_size - 2];
						double ref_y_prev = previous_path_y[prev_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

						//Use two points that make the path tangent to the previous path's end points
						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);

						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);

						ref_vel = target_vehicle_speed;
					}

					// Plan the rest of the path based on calculations
					vector<double> frenet_vec = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);

					double move = lanePlanner(frenet_vec[0], frenet_vec[1], sensor_fusion);
					double lane = current_lane;
					double next_d = 2 + 4 * lane + move;

					// Double-check that the car has not incorrectly chose a blocked lane
					int check_lane = assignLane(next_d);
					vector<double> front_vehicle = closestVehicle(frenet_vec[0], check_lane, sensor_fusion, true);
					vector<double> back_vehicle = closestVehicle(frenet_vec[0], check_lane, sensor_fusion, false);

					// Reset to current lane and leading vehicle if not enough room
					if (front_vehicle[0] < 10 or back_vehicle[0] < 10 or avg_points[check_lane] <= -5) {
						next_d = 2 + 4 * lane;
						if (check_lane != lane) {
							target_vehicle_speed = curr_lead_vehicle_speed;
						}
					}

					//In Frenet add evenly 50m spaced points ahead of the starting reference
					vector<double> next_wp0 = getXY(car_s + 50, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s + 100, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s + 150, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					if (ptsx.size() > 2) {  // Spline fails if not greater than two points - Otherwise just use rest of old path

						for (int i = 0; i < ptsx.size(); i++)
						{
							//shift car reference angle to 0 degrees
							double shift_x = ptsx[i] - ref_x;
							double shift_y = ptsy[i] - ref_y;

							ptsx[i] = (shift_x *cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
							ptsy[i] = (shift_x *sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
						}

						//create a spline
						tk::spline s;

						//set (x,y) points to the spline
						s.set_points(ptsx, ptsy);

						//calculate how to break up spline points so that we travel at our desired reference velocity
						double target_x = 30.0;
						double target_y = s(target_x);
						double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

						double x_add_on = 0;
						const int MAX_ACCEL = 10; // m/s/s
						const double accel = (MAX_ACCEL) * 0.02 * 0.8; // Limit acceleration within acceptable range

																	   //fill up the rest of the path planner after filling it with starting points, here we will always output 50 points
						for (int i = 1; i <= 50 - prev_size; i++)
						{
							if (ref_vel < target_vehicle_speed - accel) {  // Accelerate if under target speed
								ref_vel += accel;
							}
							else if (ref_vel > target_vehicle_speed + accel) { // Brake if below target
								ref_vel -= accel;
							}

							//mph to meters/sec = divide by 2.24
							double N = (target_dist / (.02*ref_vel));// / 2.24));
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
					}
					target_vehicle_speed = ref_vel;  // Save the end speed to be used for the next path

					json msgJson;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

				}
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
		size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		}
		else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
		char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
