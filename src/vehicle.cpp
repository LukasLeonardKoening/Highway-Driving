//
//  vehicle.cpp
//  Highway-Driving
//
//  Created by Lukas Leonard Köning on 07.02.20.
//  Copyright © 2020 Lukas Leonard Köning. All rights reserved.
//

#include "vehicle.hpp"
#include "trajectory_gen.hpp"
#include <string>
#include <vector>
#include <math.h>
#include <iostream>
#include "helpers.h"

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
    bool vehicle_left = false;
    bool vehicle_right = false;
    bool lane_change_complete = false;
    
    //std::cout << "d=" << this->d << std::endl;
    
    if ((this->d > 1.8 && this->d < 2.2) || (this->d > 5.8 && this->d < 6.2) || (this->d > 9.8 && this->d < 10.2)) {
        lane_change_complete = true;
        std::cout << "Lane change complete" << std::endl;
    }
    
    
    //vector<float> lane_velocities = {0.0, 0.0, 0.0};
    
    // TODO: Control target-velocity of car
    

    // sensorfusion data [id,x,y,vx,vy,s,d]
    for (int i=0; i < sensor_data.size(); i++) {
        
        // Check for car ahead
        vector<double> car = sensor_data[i];
        int car_lane = round((car[6]-2.0)/4.0);
        double car_velocity = sqrt(car[3]*car[3] + car[4]*car[4]);
        bool car_behind = car[5] < this->s;
        if (car_lane == this->lane && car[5] > this->s) {
            double distance_to_ego = distance(this->x, this->y, car[1], car[2]);
            double security_distance = car_velocity*2;
            if (distance_to_ego < security_distance) {
                std::cout << "Vehicle upfront... Slow down! v=" << car_velocity/MILES_TO_METERS << std::endl;
                //lane_velocities[car_lane] = car_velocity;
                target_vel = car_velocity/MILES_TO_METERS;
                vehicle_ahead = true;
                
            }
        }
        
        // Check if car is left or right
        if ((!car_behind && abs(car[5] - this->s) <= 35) || (car_behind && abs(car[5] - this->s) <= 15)) {
            if (car_lane == (this->lane - 1)) { // left
                //lane_velocities[car_lane] = car_velocity;
                vehicle_left = true;
                //std::cout << "Vehicle on my left..." << std::endl;
            } else if (car_lane == (this->lane + 1)) { // right
                //lane_velocities[car_lane] = car_velocity;
                vehicle_right = true;
                //std::cout << "Vehicle on my right..." << std::endl;
            }
        }
        //std::cout << "Lane velocities: " << lane_velocities[0] << "," << lane_velocities[1] << "," << lane_velocities[2] << std::endl;
        
    }
    
    if (lane_change_complete && vehicle_ahead && this->state == KEEP_LANE) {
        this->state = PREP_LANE_CHANGE;
    } else if (this->state == PREP_LANE_CHANGE) {
        // Check if lane change is possible and reasonable
        //std::cout << "Prep lane change" << std::endl;
        int lane_left = this->lane - 1;
        int lane_right = this->lane + 1;
        
        if (lane_left >= 0 && !vehicle_left) { //&& lane_velocities[lane_left] == 0.0) {
            std::cout << "Change left" << std::endl;
            this->state = CHANGE_LANE_LEFT;
        } else if (lane_right < 3 && !vehicle_right) {//} && lane_velocities[lane_right] == 0.0) {
            std::cout << "Change right" << std::endl;
            this->state = CHANGE_LANE_RIGHT;
        }
    } else if (this->state == CHANGE_LANE_LEFT) {
            this->state = KEEP_LANE;
            this->lane -= 1;
        
        //this->state = KEEP_LANE;
    } else if (this->state == CHANGE_LANE_RIGHT) {
            this->state = KEEP_LANE;
            this->lane += 1;
        
        //this->state = KEEP_LANE;
    }
    
    std::cout << "Target lane: " << this->lane << std::endl;
    
    
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
