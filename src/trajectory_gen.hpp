//
//  trajectory_gen.hpp
//  Highway-Driving
//
//  Created by Lukas Leonard Köning on 07.02.20.
//  Copyright © 2020 Lukas Leonard Köning. All rights reserved.
//

#ifndef TRAJECTORY_GEN_H
#define TRAJECTORY_GEN_H

#include <stdio.h>
#include <vector>
#include "vehicle.hpp"

using std::vector;

class Trajectory_Generator {
public:
    // Con- / Destructor
    Trajectory_Generator(vector<double> map_x, vector<double> map_y, vector<double> map_s);
    ~Trajectory_Generator();
    
    // Functions
    vector<Vehicle> generate_keep_lane(Vehicle current, vector<double> prev_x, vector<double> prev_y, double s, double d);
    vector<double> JMT(vector<double> &start, vector<double> &end, double T);
    
private:
    vector<double> mappoints_x;
    vector<double> mappoints_y;
    vector<double> mappoints_s;
};

#endif /* TRAJECTORY_GEN_H */
