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
    map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};
    
    struct collider{
        bool collision;  // is there a collision
        int time;  // time collision happens
    };
    
    int preferred_buffer = 6;
    
    int lane;
    
    //car information
    float x;
    float y;
    float yaw;
    float v;
    float a;
    float s;
    float d;
    
    // goal information
    float target_speed;
    int goal_lane;
    int goal_s;
    string state;
    
    // road information
    int num_lanes;
    float max_acceleration;
    float max_speed;
    float max_jerk;
    
    // just const reference
    map<string, vector<double>> map_waypoints;
    
    
    Vehicle();
    Vehicle(int lane, float s, float v, float a, string state="CS");
    Vehicle(float x, float y, float yaw, float v, float s, float d, string state, map<string, vector<double>> map_waypoints);
    
    virtual ~Vehicle();
    
    vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);
    
    vector<string> successor_states();
    
    vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);
    
    vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);
    
    vector<Vehicle> constant_speed_trajectory();

    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

    vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

    void increment(int dt);

    float position_at(int t);

    bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

    vector<Vehicle> generate_predictions(int horizon=2);

    void realize_next_state(vector<Vehicle> trajectory);

    void configure(vector<int> road_data);

};

#endif /* vehicle_h */
