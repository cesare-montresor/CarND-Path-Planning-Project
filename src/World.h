//
//  World.hpp
//  path_planning
//
//  Created by Cesare on 23/10/2017.
//

#ifndef World_h
#define World_h

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <map>
#include <algorithm>
#include "Car.h"

using namespace std;

struct Vehicle {
  int id;
  double last_update;
  double s;
  double d;
  double vs;
  double vd;
  double as;
  double ad;
  double lane;
  vector<vector<double>> prediction;
};

#define VehicleList map<int, Vehicle>

class World{
public:
  int lane_size = 4;
  int lane_cnt = 3;
  int horizon = 10;
  
  vector<double> map_x;
  vector<double> map_y;
  vector<double> map_s;
  vector<double> map_dx;
  vector<double> map_dy;
  
  VehicleList vehicles;
  Car *car;
  
  World(DoubleV map_x, DoubleV map_y, DoubleV map_s, DoubleV map_dx, DoubleV map_dy);
  
  virtual ~World(){};
  
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  vector<double> getFrenet(double x, double y, double theta);
  vector<double> getXY(double s, double d);
  vector<vector<double>> getXYTrajectory(DoubleV2 fernet_trajectory);
  
  void update_predictions(DoubleV2 sensor_fusion);
  VehicleList filter_predictions(bool in_front, double s, int lane, VehicleList vehicle_list);
  Vehicle get_closest(double s, VehicleList vehicle_list );
  Vehicle get_farthest(double s, VehicleList vehicle_list );
  
  Vehicle get_slowest( VehicleList vehicle_list );
  Vehicle get_fastest( VehicleList vehicle_list );
  
  double coordsFromLane(int lane);
  int laneFromCoords(double coords);
  
};


#endif /* World_hpp */
