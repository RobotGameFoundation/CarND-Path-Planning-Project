#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int findLine(double d) {
  if (d>=0 && d<=4) {
    return 0;
  } else if (d>4 && d<=8) {
    return  1;
  } else if (d>8 && d<=12) {
    return 2;
  } else {
    return -1;
  }
}

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

  double ref_v = 0.0;
  int lane = 1;
  h.onMessage([&ref_v, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
            
          int prev_size = previous_path_x.size();
          if (prev_size >0) {
              car_s = end_path_s;
          }
          double fps = 0.02;
          
          double safe_distance_front = 30;
          double safe_distance_back = 15;
          bool lane0_full=false;
          bool lane1_full=false;
          bool lane2_full=false;
          bool lane0_front=false;
          bool lane1_front=false;
          bool lane2_front=false;
          
          bool change_lane=false;
          bool slow_down = false;
         
          for(int i=0; i<sensor_fusion.size(); i++) {
            double sense_s = sensor_fusion[i][5];
            double sense_d = sensor_fusion[i][6];
            double sense_v_x = sensor_fusion[i][3];
            double sense_v_y = sensor_fusion[i][4];
            double speed = sqrt(sense_v_x*sense_v_x + sense_v_y*sense_v_y);
            int sense_lane;
            sense_s += (double)prev_size*fps*speed;
            sense_lane = findLine(sense_d);
            
          
            if (sense_s-car_s > 0 && sense_s-car_s < safe_distance_front) {
                if (sense_lane == lane ){
                  change_lane = true;
                }
            
                
                
            }
            if (sense_s-car_s > 0 && sense_s-car_s < safe_distance_front) {
              if (sense_lane == 0) {
                  lane0_front = true;
                }
                else if (sense_lane == 1) {
                  lane1_front = true;
                }
                else if (sense_lane == 2) {
                  lane2_front = true;
                }
            }
            if (((car_s-sense_s< safe_distance_back) && (car_s-sense_s>0)) || 
                ((sense_s-car_s < safe_distance_front) &&  (sense_s-car_s>0))){
                if (sense_lane == 0) {
                  lane0_full = true;
                }
                else if (sense_lane == 1) {
                  lane1_full = true;
                }
                else if (sense_lane == 2) {
                  lane2_full = true;
                }
                
            }
           
          
          }
          if (change_lane == true) {
            if (lane == 1) {
              if (lane2_full == false) {
                lane = 2;
              }
              else if (lane0_full == false) {
                lane = 0;
              }
              else if (lane1_front == true){
                slow_down = true;
                
              }
            }

            else if (lane == 0) {
              if (lane1_full == false) {
                lane = 1;
              }
              else if (lane0_front == true){
                slow_down = true;
                
              }
            }

            else if (lane == 2) {
              if (lane1_full == false) {
                lane = 1;
              }
              else if (lane2_front == true){
          
                  slow_down = true;
                
              }
            }
          }
          else{
            if (lane == 0 || lane == 2) {
              if (lane1_full == false) {
                lane = 1;
              }
            }
          }
            
          
          if (slow_down) {
            ref_v -= 0.24;
          }
          else {
            if (ref_v < 49) {
              ref_v += 0.24;
            }
            else {
              ref_v = 49;
            }
          }
          //std::cout<<ref_v<<std::endl;
          //std::cout<<change_lane<<std::endl;
          //std::cout<<slow_down<<std::endl;
          //std::cout<<lane0_full<<lane1_full<<lane2_full<<std::endl;
          //std::cout<<lane0_front<<lane1_front<<lane2_front<<std::endl;
         

          vector<double> path_x;
          vector<double> path_y;

         
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            path_x.push_back(prev_car_x); path_x.push_back(car_x);
            path_y.push_back(prev_car_y); path_y.push_back(car_y);
          }
          
          else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            path_x.push_back(ref_x_prev); path_x.push_back(ref_x);
            path_y.push_back(ref_y_prev); path_y.push_back(ref_y);
          }

          vector<double> next_wp0 = getXY(car_s + 30, (4*lane+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 50, (4*lane+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 60, (4*lane+2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          path_x.push_back(next_wp0[0]); path_x.push_back(next_wp1[0]); path_x.push_back(next_wp2[0]);
          path_y.push_back(next_wp0[1]); path_y.push_back(next_wp1[1]); path_y.push_back(next_wp2[1]);

          for (int i = 0; i < path_x.size(); i++) {
            double shift_x = path_x[i] - ref_x;
            double shift_y = path_y[i] - ref_y;
            path_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            path_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create a spline
          tk::spline s;
          s.set_points(path_x, path_y);
          
          
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }


          double target_x = 30.0;
          double target_y = s(target_y);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);


          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

            double N = target_dist / (fps * ref_v /2.24);
            double x_p =  target_x / N * i;
            double y_p = s(x_p);
            double x_ref = x_p;
            double y_ref = y_p;

            // Rotate back into previous coordinate system
            x_p = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_p = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_p += ref_x;
            y_p += ref_y;

            next_x_vals.push_back(x_p);
            next_y_vals.push_back(y_p);
          }

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