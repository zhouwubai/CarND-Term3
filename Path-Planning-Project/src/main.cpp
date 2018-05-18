#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "helper.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  map<string, vector<double>> map_waypoints;

  // Waypoint map to read from
  string map_file_ = "../../../data/highway_map.csv";

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

  map_waypoints["x"] = map_waypoints_x;
  map_waypoints["y"] = map_waypoints_y;
  map_waypoints["s"] = map_waypoints_s;
  map_waypoints["dx"] = map_waypoints_dx;
  map_waypoints["dy"] = map_waypoints_dy;

  // some states for ego vehicle
  string ego_state = "KL";

  h.onMessage([&map_waypoints, &ego_state](
        uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

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
            // 2D vector [id, x, y, vx, vy, s, d]
          	auto sensor_fusion = j[1]["sensor_fusion"];
           
            json msgJson;
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            for(int i = 0; i < previous_path_x.size(); i++){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            int prev_size = previous_path_x.size();
            
            // we plan 50 points everytime and update when the size is less than 20
            if(prev_size < 20){
                map<int, Vehicle> vehicles;
                for(int i = 0; i < sensor_fusion.size(); i ++){
                    // initial sensor fusion vehicle state as "CS"
                    double id = sensor_fusion[i][0];
                    double x = sensor_fusion[i][1];
                    double y = sensor_fusion[i][2];
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double s = sensor_fusion[i][5];
                    double d =sensor_fusion[i][6];

                    double yaw = atan2(vy, vx);
                    double vel = sqrt(vx * vx + vy * vy);
                
                    Vehicle vehicle = Vehicle(x, y, yaw, vel, s, d, "CS");
                    // check the end of previous_path
                    vehicles[id] = vehicle.position_at(prev_size);
                } // end_for
            
                map<int, vector<Vehicle>> predictions;
                for(map<int, Vehicle>::iterator it = vehicles.begin(); it != vehicles.end(); it ++){
                    int v_id = it->first;
                    // constant speed are used in frenet system to generate predictions
                    vector<Vehicle> preds = it->second.generate_predictions(50);
                    predictions[v_id] = preds;
                } // end_for
                
                // ref state when prev_size < 2, only true at the start of the program
                double ref_x = car_x;
                double ref_y = car_y;
                double ref_yaw = deg2rad(car_yaw);
                double ref_speed = 0.0;
                
                if(prev_size >= 2){
                    ref_x = previous_path_x[prev_size - 1];
                    ref_y = previous_path_y[prev_size - 1];
                    
                    double ref_x_prev = previous_path_x[prev_size - 2];
                    double ref_y_prev = previous_path_y[prev_size - 2];
                    double dx = ref_x - ref_x_prev;
                    double dy = ref_y - ref_y_prev;
                    
                    ref_yaw = atan2(dy, dx);
                    ref_speed = sqrt(dx*dx + dy*dy) / 0.02;
                }
                
                Vehicle ego_car;
                vector<double> sd = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints["x"], map_waypoints["y"]);
                ego_car = Vehicle(ref_x, ref_y, ref_yaw, ref_speed, sd[0], sd[1], ego_state);
                ego_car.configure(previous_path_x, previous_path_y, end_path_s, end_path_d, map_waypoints);
            
                vector<Vehicle> trajectory;
                string next_state = ego_car.choose_next_state(predictions, trajectory);
                
                ego_state = next_state;
                //TODO: add all to next_x_values
                for(int i = 0; i < trajectory.size(); i ++){
                    next_x_vals.push_back(trajectory[i].x);
                    next_y_vals.push_back(trajectory[i].y);
                }
            }
           
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
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
    } else {
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
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
