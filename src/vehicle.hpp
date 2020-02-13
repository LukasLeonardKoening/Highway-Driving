//
//  vehicle.hpp
//  Highway-Driving
//
//  Created by Lukas Leonard Köning on 07.02.20.
//  Copyright © 2020 Lukas Leonard Köning. All rights reserved.
//

#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

using std::string;
using std::vector;
using std::map;

class Vehicle {
public:
    enum STATE {
        KEEP_LANE, PREP_LANE_CHANGE, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT
    };
    
    // Constructors
    Vehicle();
    Vehicle(int lane, float x, float y, float s, float d, float speed, float yaw, STATE state=KEEP_LANE);

    //Deconstructor
    ~Vehicle();
    
    //Functions
    vector<Vehicle> select_successor_state(vector<vector<double>> &sensor_data, vector<double> previous_x, vector<double> previous_y, double previous_speed, vector<vector<double>> map);
    vector<STATE> get_possible_next_states(STATE &current_state);
    
    // static variables
    int num_lanes = 3;
    
    // vehicle variables
    int lane;
    float x;
    float y;
    float s;
    float d;
    float speed;
    float yaw;
    STATE state;
    
    
private:
    //private variables
    
};


#endif /* VEHICLE_H */
