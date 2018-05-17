//
//  vehicle.h
//  path_planning
//
//  Created by Wubai Zhou on 5/6/18.
//

#ifndef vehicle_h
#define vehicle_h

#include <iostream>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {

public:
    map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};
    
    struct collider{
        bool collision;  // is there a collision
        int time;  // time collision happens
    };
    
    int preferred_buffer = 6;
    
    //car information
    double x;
    double y;
    double yaw;
    double v;
    double a;
    double s;
    double d;
    int lane;
    string state;
    
    // constant value
    int num_lanes;
    double max_speed;
    double max_acceleration;
    double max_jerk;
    double max_s;
    
    // set internal
    double goal_s;
    double goal_lane;
    
    // configure data
    double target_speed;
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;
    map<string, vector<double>> map_waypoints;
    
    Vehicle();
    Vehicle(int lane, double x, double y, double yaw, double v, double s, double d, string state);
    
    virtual ~Vehicle();
    
    vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);
    
    vector<string> successor_states();
    
    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);
    
    vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);
    
    vector<Vehicle> constant_speed_trajectory();

    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

    vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    vector<double> position_at(double t);

    bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    vector<Vehicle> generate_predictions(int horizon=2);

    void configure(double target_speed, int lane, vector<double> previous_path_x,vector<double> previous_path_y,
        double end_path_s, double end_path_d, map<string, vector<double>> map_waypoints);

};

#endif /* vehicle_h */
