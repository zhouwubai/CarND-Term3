//
//  vehicle.cpp
//  path_planning
//
//  Created by Wubai Zhou on 5/6/18.
//

#include <stdio.h>
#include "vehicle.h"
#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "helper.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}
Vehicle::Vehicle(double x, double y, double yaw, double v, double s, double d, string state){
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->v = v;
    this->s = s;
    this->d = d;
    this->state = state;
    
    max_speed = 22.35;  // m/s
    max_acceleration = 10;
    max_jerk = 10;
    max_s = 6945.554;
    goal_s = 60; // lookahead 60 m
    goal_lane = 1;
    
    a = 0;
}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = successor_states();
    float cost;
    vector<double> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        if (lane != num_lanes - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /*
    [x, y, yaw, v, s, d, a]
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = this->max_acceleration + this->v;
    double new_s;
    double new_velocity;
    double new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    new_s = this->s + new_velocity + new_accel / 2.0;
    double new_d = getDFromLane(lane);
    vector<double> new_xy = getXY(new_s, new_d, map_waypoints["s"], map_waypoints["x"], map_waypoints["y"]);
    
    return{new_s, new_d, this->yaw, new_velocity, new_xy[0], new_xy[1], new_accel};

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    vector<double> next_pos = position_at(1);
    vector<Vehicle> trajectory =
        {Vehicle(this->x, this->y, this->yaw, this->v, this->s, this->d, this->state),
         Vehicle(next_pos[0], next_pos[1], next_pos[2], next_pos[3], next_pos[4], next_pos[5], this->state)
        };
    
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(this->x, this->y, this->yaw, this->v, this->s, this->d, this->state)};
    vector<double> new_pos = get_kinematics(predictions, this->lane);
    Vehicle new_vehicle = Vehicle(new_pos[0], new_pos[1], new_pos[2], new_pos[3], new_pos[4], new_pos[5], "KL");
    new_vehicle.a = new_pos[6];
    trajectory.push_back(new_vehicle);
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    double new_s;
    double new_v;
    double new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->x, this->y, this->yaw, this->v, this->s, this->d, this->state)};
    vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];

    } else {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }
    
    /*
    Vehicle new_vehicle = Vehicle(new_pos[0], new_pos[1], new_pos[2], new_pos[3], new_pos[4], new_pos[5], "KL");
    new_vehicle.a = new_pos[6];
    trajectory.push_back(new_vehicle);
    */
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->x, this->y, this->yaw, this->v, this->s, this->d, this->state));
    vector<double> new_pos = get_kinematics(predictions, new_lane);
    Vehicle new_vehicle = Vehicle(new_pos[0], new_pos[1], new_pos[2], new_pos[3], new_pos[4], new_pos[5], "KL");
    new_vehicle.a = new_pos[6];
    trajectory.push_back(new_vehicle);
    return trajectory;
}

vector<double> Vehicle::position_at(double t) {
    // assume constant speed (a = 0) for other cars in frenet system
    // return [x, y, yaw, v, s, d]
    double new_s = this->s + this->v*t + 0.5 * this->a*t*t;
    double new_v = this->v + this->a*t;
    vector<double> xy = getXY(new_s, this->d, map_waypoints["s"], map_waypoints["x"], map_waypoints["y"]);
    return {xy[0], xy[1], this->yaw, new_v, new_s, this->d};
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = this->s + this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
  vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      // [x, y, yaw, v, s, d]
      vector<double> next_pos = position_at(i);
      predictions.push_back(Vehicle(next_pos[0], next_pos[1], next_pos[2], next_pos[3], next_pos[4], next_pos[5], this->state));
    }
    return predictions;
}

void Vehicle::configure(double target_speed, int lane, vector<double> previous_path_x, vector<double> previous_path_y,
        double end_path_s, double end_path_d, map<string, vector<double>> map_waypoints) {
    this->target_speed = target_speed;
    this->lane = lane;
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
    this->end_path_s = end_path_s;
    this->end_path_d = end_path_d;
    this->map_waypoints = map_waypoints;
}
