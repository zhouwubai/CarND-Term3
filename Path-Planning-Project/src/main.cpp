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
// #include "vehicle.h"
#include "helper.h"
#include "spline.h"

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

  map_waypoints["x"] = map_waypoints_x;
  map_waypoints["y"] = map_waypoints_y;
  map_waypoints["s"] = map_waypoints_s;
  map_waypoints["dx"] = map_waypoints_dx;
  map_waypoints["dy"] = map_waypoints_dy;

  // some states for ego vehicle
  int lane = 1;
  //double ref_vel = 49.5; // mph
  double ref_vel = 0.0; // mph
  bool changing_lane = false;

  h.onMessage([&map_waypoints,&lane,&ref_vel, &changing_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

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

            int prev_size = previous_path_x.size();
            // check whether lane changing can be set as finished
            if(fabs(car_d - (2 + 2 * lane)) < 0.2){
                changing_lane = false;
            }

            if(prev_size > 0){
                car_s = end_path_s;
            }
            
            bool too_close = false;

            // find ref_v to use
            for(int i = 0; i < sensor_fusion.size(); i ++){
                //car is in my lane
                float d = sensor_fusion[i][6];
                if(d < (2+4*lane+2) && d > (2+4*lane-2)){
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];

                    check_car_s += (double)prev_size * 0.02 * check_speed;
                    if(check_car_s > car_s && check_car_s - car_s < 20){
                        //ref_vel = 29.5;
                        too_close = true;
                        
                        /*
                        if(lane > 0){
                            lane = 0;
                        }*/
                        
                        break;
                    }
                }
            }
            
            
            // change change module, choose a good lane to go and keep lane if not avaliable.
            // if lane changed, we add all points to avoid lane change interruption
            vector<int> next_lanes;
            
            // only change lane when last lane change finished
            if(too_close and not changing_lane){
                if(lane > 0) next_lanes.push_back(lane - 1);
                if(lane < 2) next_lanes.push_back(lane + 1);
                
                for(int i = 0; i < next_lanes.size(); i ++){
                    int cur_lane = next_lanes[i];
                    bool car_ahead = false;
                    bool car_behind = false;
                    cout << "check lane: " << cur_lane << endl;
                    for(int i = 0; i < sensor_fusion.size(); i ++){
                        // car in my lane
                        float d = sensor_fusion[i][6];
                        if(d < (2+4*cur_lane+2) && d > (2+4*cur_lane-2)){
                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_speed = sqrt(vx*vx + vy*vy);
                            double check_car_s = sensor_fusion[i][5];

                            check_car_s += (double)prev_size * 0.02 * check_speed;
                            // change numbers smaller to make car more aggressive in lane changing
                            if(check_car_s > car_s && check_car_s - car_s < 20){
                                car_ahead = true;
                                cout<< "car ahead " << check_car_s - car_s << endl;
                            }
                        
                            if(check_car_s < car_s && car_s - check_car_s < 10){
                                car_behind = true;
                                cout<< "car behind " << car_s - check_car_s << endl;
                            }
                        }
                    }// end for
                    
                    // if no car ahead and behind, change the first lane feasible
                    if(not car_ahead and not car_behind){
                        cout << "chane lane from " << lane << " to " << cur_lane << endl;
                        lane = cur_lane;
                        changing_lane = true;
                        break;
                    }
                    
                }// end for
            }// end if
            
            
            if(too_close){
                ref_vel -= .224;
            }
            else if(ref_vel < 49.5){
                ref_vel += .224;
            }

            vector<double> ptsx;
            vector<double> ptsy;

            // reference x, y, yaw states
            // either we will reference the starting points as where the car is or at the previous path end point
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // if previous size is empty, use the car as starting reference
            if(prev_size < 2){
                // Use two points that make the path tangent to the car
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            // use the previous path's end point as starting reference
            else{
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                //Use two points that make the path tangent to the previous path's end point
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }


            // in frenet add evenly 30m spaced points ahead of the starting reference
            vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints["s"], map_waypoints["x"], map_waypoints["y"]);
            vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints["s"], map_waypoints["x"], map_waypoints["y"]);
            vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints["s"], map_waypoints["x"], map_waypoints["y"]);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for(int i = 0; i < ptsx.size(); i ++){
                // shift car reference angle to 0 degrees
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // create spline
            tk::spline s;
            s.set_points(ptsx, ptsy);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
           for(int i = 0; i < previous_path_x.size(); i++){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
           }

           // calculate how to break up spline points so that we travel at our desired reference speed
           double target_x = 30.0;
           double target_y = s(target_x);
           double target_dist = sqrt(target_x * target_x + target_y * target_y);

           double x_add_on = 0;
           double N = (target_dist / (0.02 * ref_vel / 2.24));
           
            for(int i = 0; i < 50 - previous_path_x.size(); i ++){

                double x_point = x_add_on + target_x / N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double ref_xi = x_point;
                double ref_yi = y_point;

                // transform x_point, y_point back to global XY coordinate
                x_point = ref_xi * cos(ref_yaw) - ref_yi * sin(ref_yaw);
                y_point = ref_xi * sin(ref_yaw) + ref_yi * cos(ref_yaw);

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
              }
            
            json msgJson;
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
