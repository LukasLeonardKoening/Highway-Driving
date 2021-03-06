#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <typeinfo>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "vehicle.cpp"
#include "vehicle.hpp"
#include "trajectory_gen.cpp"
#include "trajectory_gen.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::map;

double last_speed = 0; // trajectorys last speed
Vehicle::STATE last_state = Vehicle::KEEP_LANE;
int last_lane = 1;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
            
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
            
            // *** IMPLEMENTATION ***

            // calculate current lane
            int lane = round((car_d-2.0)/4.0);
            
            // init current state
            Vehicle current_state = Vehicle(last_lane, car_x, car_y, car_s, car_d, car_speed, car_yaw, last_state);
            
            // prepare sensor data
            vector<vector<double>> sensor_data;
            for (int i=0; i<sensor_fusion.size(); i++) {
                sensor_data.push_back({sensor_fusion[i][0],sensor_fusion[i][1],sensor_fusion[i][2],sensor_fusion[i][3],sensor_fusion[i][4],sensor_fusion[i][5],sensor_fusion[i][6]});
            }
            
            // create trajectory and push it into the given vectors
            vector<Vehicle> trajectory = current_state.select_successor_state(sensor_data, previous_path_x, previous_path_y, last_speed, {map_waypoints_x, map_waypoints_y, map_waypoints_s});
            for (int i=0; i < trajectory.size(); i++) {
                next_x_vals.push_back(trajectory[i].x);
                next_y_vals.push_back(trajectory[i].y);
            }
            // save last speed and lane of trajectory and state of vehicle's state maschine
            last_speed = trajectory[trajectory.size()-1].speed;
            last_state = trajectory[trajectory.size()-1].state;
            last_lane = trajectory[trajectory.size()-1].lane;

            // *** END IMPLEMENTATION ***

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
