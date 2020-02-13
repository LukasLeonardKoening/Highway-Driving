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

vector<Vehicle> Vehicle::select_successor_state(vector<vector<double>> &sensor_data, vector<double> previous_x, vector<double> previous_y, double previous_speed, vector<vector<double>> map) {
    Trajectory_Generator gen = Trajectory_Generator(map[0], map[1], map[2]);
    double target_vel = MAX_VELOCITY;
    bool vehicle_ahead = false;
    
    // TODO: Control target-velocity of car
    

    // sensorfusion data [id,x,y,vx,vy,s,d]
    for (int i=0; i < sensor_data.size(); i++) {
        
        // Check for car ahead
        vector<double> car = sensor_data[i];
        int car_lane = round((car[6]-2.0)/4.0);
        if (car_lane == this->lane && car[5] > this->s) {
            double distance_to_ego = distance(this->x, this->y, car[1], car[2]);
            double car_velocity = sqrt(car[3]*car[3] + car[4]*car[4]);
            double security_distance = car_velocity*2;
            if (distance_to_ego < security_distance) {
                std::cout << "Vehicle upfront... Slow down! v=" << car_velocity/MILES_TO_METERS << std::endl;
                target_vel = car_velocity/MILES_TO_METERS;
                vehicle_ahead = true;
                
            }
        }
        
        // Check if car is left or right
        
    }
    
    if (vehicle_ahead && this->state == KEEP_LANE) {
        this->state = PREP_LANE_CHANGE;
    } else if (this->state == PREP_LANE_CHANGE) {
        // Check if lane change is possible and reasonable
        std::cout << "Prep lane change" << std::endl;
    }
    
    
    return gen.generate_trajectory(*this, previous_x, previous_y, previous_speed, target_vel);
}

vector<STATE> Vehicle::get_possible_next_states(STATE &current_state) {
    vector<STATE> states;
    states.push_back(KEEP_LANE);
    switch (current_state) {
        case KEEP_LANE:
            states.push_back(PREP_LANE_CHANGE);
            break;
        case PREP_LANE_CHANGE:
            if (this->lane != 0) {
                states.push_back(PREP_LANE_CHANGE);
                states.push_back(CHANGE_LANE_RIGHT);
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
