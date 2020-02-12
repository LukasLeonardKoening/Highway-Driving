//
//  vehicle.cpp
//  Highway-Driving
//
//  Created by Lukas Leonard Köning on 07.02.20.
//  Copyright © 2020 Lukas Leonard Köning. All rights reserved.
//

#include "vehicle.hpp"
#include <string>
#include <vector>
#include "trajectory_gen.hpp"

using std::string;
using std::vector;
using STATE = Vehicle::STATE;

// HYPERVARS
double MAX_VELOCITY = 49.5;    // not using maximum!
double MAX_ACC = 9.5;           // m/s^2, also not using maximum
double REFRESH_RATE = 0.02;     // 20 ms
double MILES_TO_METERS = 1/2.24;

Vehicle::Vehicle(){}
Vehicle::~Vehicle() {}

Vehicle::Vehicle(int lane, float x, float y, float s, float d, float speed, float yaw, STATE state) {
    this->lane = lane;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->speed = speed;
    this->yaw = yaw;
    this->state = state;
}

vector<Vehicle> Vehicle::select_successor_state(map<int, vector<Vehicle>> &predictions, vector<double> previous_x, vector<double> previous_y, double previous_speed, vector<vector<double>> map) {
    Trajectory_Generator gen = Trajectory_Generator(map[0], map[1], map[2]);
    
    // TODO: Control target-velocity of car
    
    
    return gen.generate_keep_lane(*this, previous_x, previous_y, previous_speed, MAX_VELOCITY);
}

vector<STATE> Vehicle::get_possible_next_states(STATE &current_state) {
    vector<STATE> states;
    states.push_back(KEEP_LANE);
    switch (current_state) {
        case KEEP_LANE:
            states.push_back(PREP_LANE_CHANGE_LEFT);
            states.push_back(PREP_LANE_CHANGE_RIGHT);
            break;
        case PREP_LANE_CHANGE_RIGHT:
            if (this->lane != 0) {
                states.push_back(PREP_LANE_CHANGE_RIGHT);
                states.push_back(CHANGE_LANE_RIGHT);
            }
            break;
        case PREP_LANE_CHANGE_LEFT:
            if (this->lane != num_lanes-1) {
                states.push_back(PREP_LANE_CHANGE_LEFT);
                states.push_back(CHANGE_LANE_LEFT);
            }
            break;
        case CHANGE_LANE_LEFT:
            break;
        case CHANGE_LANE_RIGHT:
            break;
    }
    return states;
}
