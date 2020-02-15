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
    double target_vel = 39;
    bool vehicle_ahead = false;
    bool vehicle_left = false;
    bool vehicle_right = false;
    bool lane_change_complete = false;
    
    if ((this->d > 1.5 && this->d < 2.5 && this->lane == 0) || (this->d > 5.5 && this->d < 6.5 && this->lane == 1) || (this->d > 9.5 && this->d < 10.5 && this->lane == 2)) {
        if (state == KEEP_LANE) {
            target_vel = MAX_VELOCITY;
        }
        lane_change_complete = true;
    }

    // sensorfusion data [id,x,y,vx,vy,s,d]
    for (int i=0; i < sensor_data.size(); i++) {
        
        // Check for car ahead
        vector<double> car = sensor_data[i];
        int car_lane = round((car[6]-2.0)/4.0);
        double car_velocity = sqrt(car[3]*car[3] + car[4]*car[4]);
        if (car_lane == this->lane && car[5] > this->s) {
            double distance_to_ego = distance(this->x, this->y, car[1], car[2]);
            double security_distance = car_velocity*2;
            if (distance_to_ego < security_distance) {
                std::cout << "Vehicle upfront... Slow down! v=" << car_velocity/MILES_TO_METERS << std::endl;
                target_vel = car_velocity/MILES_TO_METERS;
                vehicle_ahead = true;
            }
        }
        
        // Check if car is left or right
        bool car_behind = car[5] < this->s;
        if ((!car_behind && abs(car[5] - this->s) <= 35) || (car_behind && abs(car[5] - this->s) <= 15)) {
            if (car_lane == (this->lane - 1)) { // left
                vehicle_left = true;
            } else if (car_lane == (this->lane + 1)) { // right
                vehicle_right = true;
            }
        }
        
    }
    
    if (lane_change_complete && vehicle_ahead && this->state == KEEP_LANE) {
        this->state = PREP_LANE_CHANGE;
    } else if (this->state == PREP_LANE_CHANGE) {
        // Check if lane change is possible
        int lane_left = this->lane - 1;
        int lane_right = this->lane + 1;
        
        if (lane_left >= 0 && !vehicle_left) {
            // Check if velocity is low enough
            if (speed < 39.1) {
                std::cout << "Change left" << std::endl;
                this->state = CHANGE_LANE_LEFT;
            } else {
                target_vel = 39;
            }
        } else if (lane_right < 3 && !vehicle_right) {
            // Check if velocity is low enough
            if (speed < 39.1) {
                std::cout << "Change right" << std::endl;
                this->state = CHANGE_LANE_RIGHT;
            } else {
                target_vel = 39;
            }
        }
    } else if (this->state == CHANGE_LANE_LEFT) {
        // Last safety check, if there is no vehicle...
        if (!vehicle_left) {
            this->lane -= 1;
            target_vel = 40;
        }
        this->state = KEEP_LANE;
    } else if (this->state == CHANGE_LANE_RIGHT) {
        // Last safety check, if there is no vehicle...
        if (!vehicle_right) {
            this->lane += 1;
            target_vel = 40;
        }
        this->state = KEEP_LANE;
    }
    
    return gen.generate(*this, previous_x, previous_y, previous_speed, target_vel);
}
