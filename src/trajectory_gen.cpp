//
//  trajectory_gen.cpp
//  Highway-Driving
//
//  Created by Lukas Leonard Köning on 07.02.20.
//  Copyright © 2020 Lukas Leonard Köning. All rights reserved.
//

#include <vector>
#include <iostream>
#include "trajectory_gen.hpp"
#include "helpers.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

//// Helper functions
//double evaluatePoly(vector<double> coefs, double t) {
//    return coefs[0] + coefs[1]*t + coefs[2]*t*t + coefs[3]*t*t*t + coefs[4]*t*t*t*t + coefs[5]*t*t*t*t*t;
//}
//
//bool nearlySame(double x1, double x2) {
//    return x1 == x2 || abs(x1-x2) < 0.01;
//}

Trajectory_Generator::Trajectory_Generator(vector<double> map_x, vector<double> map_y, vector<double> map_s) {
    this->mappoints_x = map_x;
    this->mappoints_y = map_y;
    this->mappoints_s = map_s;
}
Trajectory_Generator::~Trajectory_Generator() {}

vector<Vehicle> Trajectory_Generator::generate_trajectory(Vehicle current, vector<double> prev_x, vector<double> prev_y, double trajec_last_speed, double target_speed) {
    
    // HYPERVARS
    float SPLINE_POINT_SHIFT = 30.0;
    double MILES_TO_METERS = 1/2.24;
    double REFRESH_RATE = 0.02;     // 20ms
    double MAX_ACC = 9.5;           // m/s^2, not using maximum
    
    vector<Vehicle> trajectory;
    vector<double> spline_x_points, spline_y_points;
    double ref_x, ref_y;

    unsigned long path_size = prev_x.size();
    double yaw = current.yaw;
    double lane_d = 4*current.lane + 2;

    if (path_size < 2) {
        double previous_x = current.x - cos(yaw);
        double previous_y = current.y - sin(yaw);

        spline_x_points.push_back(previous_x);
        spline_x_points.push_back(current.x);

        spline_y_points.push_back(previous_y);
        spline_y_points.push_back(current.y);

        ref_x = current.x;
        ref_y = current.y;
    } else {
        double old_x, prev_old_x, old_y, prev_old_y;
        old_x = prev_x[path_size-1];
        prev_old_x = prev_x[path_size-2];
        old_y = prev_y[path_size-1];
        prev_old_y = prev_y[path_size-2];

        spline_x_points.push_back(prev_old_x);
        spline_x_points.push_back(old_x);

        spline_y_points.push_back(prev_old_y);
        spline_y_points.push_back(old_y);

        yaw = atan2(old_y-prev_old_y, old_x-prev_old_x);
        ref_x = old_x;
        ref_y = old_y;

        // Add old trajectory points
        for (int i = 0; i < path_size; i++) {
            trajectory.push_back(Vehicle(current.lane, prev_x[i], prev_y[i], 0, 0, 0, 0));
        }
        
        // Set current speed to trajectories last speed
        current.speed = trajec_last_speed;
    }
    

    // Add 3 extra points
    vector<double> point1 = getXY(current.s + SPLINE_POINT_SHIFT, lane_d, this->mappoints_s, this->mappoints_x, this->mappoints_y);
    vector<double> point2 = getXY(current.s + 2*SPLINE_POINT_SHIFT, lane_d, this->mappoints_s, this->mappoints_x, this->mappoints_y);
    vector<double> point3 = getXY(current.s + 3*SPLINE_POINT_SHIFT, lane_d, this->mappoints_s, this->mappoints_x, this->mappoints_y);
    //vector<double> point4 = getXY(current.s + 4*SPLINE_POINT_SHIFT, lane_d, this->mappoints_s, this->mappoints_x, this->mappoints_y);

    spline_x_points.push_back(point1[0]);
    spline_x_points.push_back(point2[0]);
    spline_x_points.push_back(point3[0]);
    //spline_x_points.push_back(point4[0]);

    spline_y_points.push_back(point1[1]);
    spline_y_points.push_back(point2[1]);
    spline_y_points.push_back(point3[1]);
    //spline_y_points.push_back(point4[1]);
    

    // Convert spline points to vehicle coordinate system
    for (int i=0; i < spline_x_points.size(); i++) {
        double car_x = spline_x_points[i] - ref_x;
        double car_y = spline_y_points[i] - ref_y;

        spline_x_points[i] = car_x * cos(0-yaw) - car_y * sin(0-yaw);
        spline_y_points[i] = car_x * sin(0-yaw) + car_y * cos(0-yaw);
    }

    // Create spline for trajectory
    tk::spline spline;
    spline.set_points(spline_x_points, spline_y_points);

    // Create trajectory
    double x_target = SPLINE_POINT_SHIFT;
    double y_target = spline(x_target);
    double distance_target = sqrt(x_target*x_target + y_target*y_target);
    double current_x = 0;

    for (int i=0; i < (50-path_size); i++) {
        // Handle speed
        double vel_inc = REFRESH_RATE*MAX_ACC / MILES_TO_METERS;
        if (current.speed < target_speed) {
            current.speed = current.speed + vel_inc;
        } else if (current.speed > target_speed) {
            current.speed = current.speed - vel_inc;
        }
        
        // Split spline into points
        double delta = distance_target / (REFRESH_RATE*(current.speed*MILES_TO_METERS));
        current_x += x_target/delta;
        double current_y = spline(current_x);

        // Convert back to world coordinates
        double x = ref_x + (current_x * cos(yaw) - current_y * sin(yaw));
        double y = ref_y + (current_x * sin(yaw) + current_y * cos(yaw));

        trajectory.push_back(Vehicle(current.lane, x, y, 0, 0, current.speed, 0, current.state));
    }
    
    
    
//    //double dist_inc = 0.5;
//    double dist_inc = MAX_VEL/50;
//    unsigned long path_size = prev_x.size();
//
//    double pos_x, pos_y, pos_s, pos_d, angle;
//
//    pos_d = 4*current.lane + 2;
//
////pos_d = current.d;
//
//    if (path_size < 2) {
//        pos_x = current.x;
//        pos_y = current.y;
//        pos_s = current.s;
//        pos_d = current.d;
//        angle = deg2rad(current.yaw);
//    } else {
//        for (int i = 0; i < path_size; ++i) {
//            // TODO: Calculate new s and d values?
//            float s = current.s;
//            float d = current.d;
//            trajectory.push_back(Vehicle(current.lane, prev_x[i], prev_y[i], s, d, current.speed, current.yaw));
//        }
//
//        pos_x = prev_x[path_size-1];
//        pos_y = prev_y[path_size-1];
//
//        double pos_x2 = prev_x[path_size-2];
//        double pos_y2 = prev_y[path_size-2];
//        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
//
//        pos_s = prev_s;
//        //pos_d = prev_d;
//    }
//
//    for (int i = 0; i < 50-path_size; ++i) {
//        double s = pos_s + (i+1)*dist_inc;
//        //pos_s = s;
//
//        // Keep lane
//        double d = pos_d;
//        vector<double> cartesian = getXY(s, d, this->mappoints_s, this->mappoints_x, this->mappoints_y);
//
//        trajectory.push_back(Vehicle(current.lane, cartesian[0], cartesian[1], s, d, current.speed, current.yaw));
////        std::cout << "S:" << s << std::endl;
////        std::cout << "D:" << d << std::endl;
//        pos_x = cartesian[0];
//        pos_y = cartesian[1];
//    }
    
//    int next_waypoint = NextWaypoint(pos_x, pos_y, angle, this->mappoints_x, this->mappoints_y);
//    double next_waypoint_s = this->mappoints_s[next_waypoint];
//    std::cout << "Next waypoint: " << next_waypoint << std::endl;
//    std::cout << "Next waypoint s: " << next_waypoint_s << std::endl;
//    std::cout << "My s: " << pos_s << std::endl;
//
//    vector<double> start_vec;
//    start_vec.push_back(pos_s);
//    start_vec.push_back(current.speed);
//    start_vec.push_back(0);
//    vector<double> goal_vec;
//    goal_vec.push_back(next_waypoint_s);
//    goal_vec.push_back(MAX_VEL);
//    goal_vec.push_back(0);
//
//    // t = s/v
//    double delta_dis = next_waypoint_s - pos_s; // distance(pos_x, pos_y, this->mappoints_x[next_waypoint], this->mappoints_y[next_waypoint]);
//    double time = delta_dis / (MAX_VEL);
//    std::cout << "Calculated delta s: " << delta_dis << std::endl;
//    std::cout << "Calculated t: " << time << std::endl;
//    //time = 10;
//
//    vector<double> s_poly_coeffs = JMT(start_vec, goal_vec, time);
//
//    double dt = time / (50-path_size);
//    std::cout << "Calculated dt: " << dt << std::endl;
//
//
//    for (int i = 0; i < 50-path_size; ++i) {
//
//        double s = evaluatePoly(s_poly_coeffs, i*dt);
//
//        if (!nearlySame(s, pos_s)) {
//            std::cout << "s: " << s << std::endl;
//            pos_s = s;
//            // Keep lane
//            double d = pos_d;
//            vector<double> cartesian = getXY(s, d, this->mappoints_s, this->mappoints_x, this->mappoints_y);
//
//            trajectory.push_back(Vehicle(current.lane, cartesian[0], cartesian[1], s, d, current.speed, current.yaw));
//            pos_x = cartesian[0];
//            pos_y = cartesian[1];
//        }
//    }
    std::cout << "=================================================================================================" << std::endl;
    
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
