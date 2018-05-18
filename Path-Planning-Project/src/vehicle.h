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
    double dt;
    
    // set internal
    double goal_s;
    double goal_lane;
    
    // configure data
    double target_speed;
    map<string, vector<double>> map_waypoints;
    
    Vehicle();
    Vehicle(double x, double y, double yaw, double v, double s, double d, string state, map<string, vector<double>> map_waypoints);
    
    virtual ~Vehicle();
    
    string choose_next_state(map<int, vector<Vehicle>> predictions, vector<Vehicle>& trajectory);
    
    vector<string> successor_states();
    
    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);
    
    vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);
    
    vector<Vehicle> constant_speed_trajectory();

    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

    vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    Vehicle position_at(int pos);

    bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    vector<Vehicle> generate_predictions(int horizon=2);
};

#endif /* vehicle_h */
