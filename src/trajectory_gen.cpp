//
//  trajectory_gen.cpp
//  Highway-Driving
//
//  Created by Lukas Leonard Köning on 07.02.20.
//  Copyright © 2020 Lukas Leonard Köning. All rights reserved.
//

#include "trajectory_gen.hpp"
#include <vector>
#include "helpers.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include <iostream>

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

double MAX_VEL = 50*0.44704;

double evaluatePoly(vector<double> coefs, double t) {
    return coefs[0] + coefs[1]*t + coefs[2]*t*t + coefs[3]*t*t*t + coefs[4]*t*t*t*t + coefs[5]*t*t*t*t*t;
}

Trajectory_Generator::Trajectory_Generator(vector<double> map_x, vector<double> map_y, vector<double> map_s) {
    this->mappoints_x = map_x;
    this->mappoints_y = map_y;
    this->mappoints_s = map_s;
}
Trajectory_Generator::~Trajectory_Generator() {}

vector<Vehicle> Trajectory_Generator::generate_keep_lane(Vehicle current, vector<double> prev_x, vector<double> prev_y, double prev_s, double prev_d) {
    
    vector<Vehicle> trajectory;
    double dist_inc = 0.5;
    unsigned long path_size = prev_x.size();
    
    vector<double> splinepoints_x, splinepoints_y;
    
    double pos_x, pos_y, pos_s, pos_d, angle;
    
//    for (int i = 0; i < path_size; ++i) {
//        // TODO: Calculate new s and d values?
//        float s = current.s;
//        float d = current.d;
//        trajectory.push_back(Vehicle(current.lane, prev_x[i], prev_y[i], s, d, current.speed, current.yaw));
//    }
    
    if (path_size < 2) {
        pos_x = current.x;
        pos_y = current.y;
        pos_s = current.s;
        pos_d = current.d;
        angle = deg2rad(current.yaw);
    } else {
//        for (int i = 0; i < path_size; ++i) {
//            // TODO: Calculate new s and d values?
//            float s = current.s;
//            float d = current.d;
//            trajectory.push_back(Vehicle(current.lane, prev_x[i], prev_y[i], s, d, current.speed, current.yaw));
//        }
        
        pos_x = prev_x[path_size-1];
        pos_y = prev_y[path_size-1];

        double pos_x2 = prev_x[path_size-2];
        double pos_y2 = prev_y[path_size-2];
        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
        
        pos_s = prev_s;
        pos_d = prev_d;
        
        
    }
    int next_waypoint = NextWaypoint(pos_x, pos_y, angle, this->mappoints_x, this->mappoints_y);
    double next_waypoint_s = this->mappoints_s[next_waypoint];
    std::cout << next_waypoint << std::endl;
    
    vector<double> start_vec;
    start_vec.push_back(pos_s);
    start_vec.push_back(current.speed);
    start_vec.push_back(0);
    vector<double> goal_vec;
    goal_vec.push_back(next_waypoint_s);
    goal_vec.push_back(MAX_VEL);
    goal_vec.push_back(0);
    
    // t = s/v
    std::cout << "Calculating coeffs..." << std::endl;
    double delta_dis = distance(pos_x, pos_y, this->mappoints_x[next_waypoint], this->mappoints_y[next_waypoint]);
    double time = delta_dis / (MAX_VEL);
    std::cout << "Calculated delta s:" << delta_dis << std::endl;
    std::cout << "Calculated t:" << time << std::endl;
    time = 1;
    
    vector<double> s_poly_coeffs = JMT(start_vec, goal_vec, time);
    
    

    for (int i = 0; i < 50-path_size; ++i) {
        double s = evaluatePoly(s_poly_coeffs, i*0.02);
        pos_s = s;
        
        // Keep lane
        double d = pos_d;
        vector<double> cartesian = getXY(s, d, this->mappoints_s, this->mappoints_x, this->mappoints_y);

        trajectory.push_back(Vehicle(current.lane, cartesian[0], cartesian[1], s, d, current.speed, current.yaw));
        pos_x = cartesian[0];
        pos_y = cartesian[1];
    }
    return trajectory;
}



vector<double> Trajectory_Generator::JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
    vector<double> coefs;
    coefs.push_back(start[0]);
    coefs.push_back(start[1]);
    coefs.push_back(0.5*start[2]);
    
    double T_2 = T*T;
    double T_3 = T_2 * T;
    double T_4 = T_3 * T;
    double T_5 = T_4 * T;
    
    MatrixXd A = MatrixXd(3, 3);
    A << T_3,   T_4,    T_5,
         3*T_2, 4*T_3,  5*T_4,
         6*T,   12*T_2, 20*T_3;
    
    VectorXd alpha(3);
    VectorXd b(3);
    b(0) = end[0] - (start[0] + start[1]*T + 1.0/2 * start[2]*T*T);
    b(1) = end[1] - (start[1] + start[2]*T);
    b(2) = end[2] - (start[2]);
    
    alpha = A.inverse() * b;
    
    coefs.push_back(alpha[0]);
    coefs.push_back(alpha[1]);
    coefs.push_back(alpha[2]);

  return coefs;
}
