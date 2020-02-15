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

Trajectory_Generator::Trajectory_Generator(vector<double> map_x, vector<double> map_y, vector<double> map_s) {
    this->mappoints_x = map_x;
    this->mappoints_y = map_y;
    this->mappoints_s = map_s;
}
Trajectory_Generator::~Trajectory_Generator() {}

vector<Vehicle> Trajectory_Generator::generate(Vehicle current, vector<double> prev_x, vector<double> prev_y, double trajec_last_speed, double target_speed) {
    
    // HYPERVARS
    float SPLINE_POINT_SHIFT = 35.0;
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

    spline_x_points.push_back(point1[0]);
    spline_x_points.push_back(point2[0]);
    spline_x_points.push_back(point3[0]);

    spline_y_points.push_back(point1[1]);
    spline_y_points.push_back(point2[1]);
    spline_y_points.push_back(point3[1]);
    

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
        if (current.speed < target_speed) {
            double vel_inc = REFRESH_RATE * 0.5 * MAX_ACC / MILES_TO_METERS;
            current.speed = current.speed + vel_inc;
        } else if (current.speed > target_speed) {
            double vel_inc = REFRESH_RATE*MAX_ACC / MILES_TO_METERS;
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
    return trajectory;
}
