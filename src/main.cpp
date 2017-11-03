#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "World.h"
#include "BehaviourPlanner.h"
#include "Car.h"
#include "spline.h"
#include "tools.h"


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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
  
  World world(map_waypoints_x,map_waypoints_y,map_waypoints_s,map_waypoints_dx,map_waypoints_dy);
  Car car;
  BehaviourPlanner bp(world, car);
  world.car = &car;
  double ref_vel_ms = 0;
  double target_speed = 0;
  double last_change = now_ms();
  
  h.onMessage([&bp,&world,&car,&ref_vel_ms, &target_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    double max_vel_mph = 49.5; // miles/hour
    double max_vel_ms = max_vel_mph / 2.24; // meters/sec
    double ancor_spacing = 30;
    double ancor_cnt = 3;
    double lane = 1;
    int path_size = 50;
    double path_length = 30; //meters
    double sim_upd_freq = 0.02;
    double max_accel = 0.1;
    double accel = max_accel;
    double safety_distance = 30;
    
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          auto data=j[1];
          double x = data["x"];
          double y = data["y"];
          double s = data["s"];
          double d = data["d"];
          double yaw_deg = data["yaw"];
          double speed_mph = data["speed"];
          double speed_ms = speed_mph / 2.24;
          
          // j[1] is the data JSON object
          // Previous path data given to the Planner
          auto previous_path_x = data["previous_path_x"];
          auto previous_path_y = data["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = data["end_path_s"];
          double end_path_d = data["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = data["sensor_fusion"];
          int prev_size = previous_path_x.size();
          
          car.update(data["x"], data["y"], data["s"], data["d"], data["yaw"], data["speed"]);
          world.update_predictions(sensor_fusion);
          auto lane_info = bp.best_lane();
          lane = lane_info[0][0];
          
          /*
          auto trajectory = bp.get_trajectory({previous_path_x,previous_path_y});
          vector<double> next_x_vals = trajectory[0];
          vector<double> next_y_vals = trajectory[1];
           */
          
          
          /*Collision*/
          bool limit_speed = false;
          
          
          double lim_s = s;
          if (prev_size > 0){
            lim_s = end_path_s;
          }
          
          for (int i = 0; i< sensor_fusion.size(); i++){
            float check_d = sensor_fusion[i][6];
            if (check_d > lane*4 && check_d < (lane+1)*4 ){
              double check_vx = sensor_fusion[i][3];
              double check_vy = sensor_fusion[i][4];
              double check_speed = sqrt(check_vx*check_vx+check_vy*check_vy);
              double check_s = sensor_fusion[i][5];
              check_s += prev_size*sim_upd_freq*check_speed;
              double distance = check_s - lim_s;
              if ( distance > 0 ){
                cout<<"distance:"<<distance<<endl;
                if ( distance < safety_distance ){
                  accel = max_accel * (1-(distance/safety_distance));
                  limit_speed = true;
                  prev_size = 2;
                }
              }
              
            }
          }
          
          if (limit_speed && ref_vel_ms - accel > 0){
            ref_vel_ms -= accel; // ms
          }else{
            target_speed = max_vel_ms;
            if ( ref_vel_ms + accel < max_vel_ms ){
              ref_vel_ms += accel; // ms
            }
          }
          
          /*Trajectory*/
        
          
          double ref_yaw = deg2rad(yaw_deg);
          double ref_x = x;
          double ref_y = y;
          vector<double> ancor_x;
          vector<double> ancor_y;
          
          
          if (prev_size < 2){
            ancor_x.push_back(ref_x - cos(ref_yaw));
            ancor_y.push_back(ref_y - sin(ref_yaw));
            
            ancor_x.push_back(ref_x);
            ancor_y.push_back(ref_y);
          }else{
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double prev_ref_x = previous_path_x[prev_size-2];
            double prev_ref_y = previous_path_y[prev_size-2];
            
            ancor_x.push_back(prev_ref_x);
            ancor_y.push_back(prev_ref_y);
            
            ancor_x.push_back(ref_x);
            ancor_y.push_back(ref_y);
            
            ref_yaw = atan2(ref_y-prev_ref_y,ref_x-prev_ref_x);
          }
          
          
          for( int i = 1; i<= ancor_cnt; i++ ){
            vector<double> next_ancor = world.getXY(s+(ancor_spacing*i), 4*lane+2);
            ancor_x.push_back(next_ancor[0]);
            ancor_y.push_back(next_ancor[1]);
          }
          
          for(int i =0; i < ancor_x.size(); i++){
            double rel_x = ancor_x[i] - ref_x;
            double rel_y = ancor_y[i] - ref_y;
            
            ancor_x[i] = rel_x * cos(-ref_yaw) - rel_y * sin(-ref_yaw);
            ancor_y[i] = rel_x * sin(-ref_yaw) + rel_y * cos(-ref_yaw);
            
          }
          
          tk::spline spl;
          spl.set_points(ancor_x, ancor_y);
          
          
        
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          for(int i = 0; i < prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = path_length;
          double target_y = spl(target_x);
          double target_distance = sqrt( (target_x*target_x) + (target_y*target_y) );
          double chunk_num = target_distance/(ref_vel_ms * sim_upd_freq);
          double chunk_size = path_length/chunk_num;
          
          double last_x = 0;
          for(int i = 0; i<path_size-prev_size; i++){
            double rel_point_x = last_x + chunk_size;
            double rel_point_y = spl(rel_point_x);
            last_x = rel_point_x;
            
            //Revert rotation
            double point_x = (rel_point_x*cos(ref_yaw) - rel_point_y*sin(ref_yaw));
            double point_y = (rel_point_x*sin(ref_yaw) + rel_point_y*cos(ref_yaw));
            
            //Revert translation
            point_x += ref_x;
            point_y += ref_y;
            
            next_x_vals.push_back(point_x);
            next_y_vals.push_back(point_y);
            
          }
        
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(200));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
