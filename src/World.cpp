//
//  World.cpp
//  path_planning
//
//  Created by Cesare on 23/10/2017.
//

#include "tools.h"
#include "World.h"
#include <limits>

using namespace std;


World::World(DoubleV map_x, DoubleV map_y, DoubleV map_s, DoubleV map_dx, DoubleV map_dy){
  this->map_x = map_x;
  this->map_y = map_y;
  this->map_s = map_s;
  this->map_dx = map_dx;
  this->map_dy = map_dy;
}


void World::update_predictions(DoubleV2 sensor_fusion){
  vector<int> id_list = {};
  //cout<<"update_predictions:"<<endl;
  // Add/Update
  for(auto vehicle:sensor_fusion){
    auto now = now_ms();
    const int id = vehicle[0];
    const double x = vehicle[1];
    const double y = vehicle[2];
    const double vx = vehicle[3];
    const double vy = vehicle[4];
    const double s = vehicle[5];
    const double d = vehicle[6];
    
    /*
    const double theta = atan2(vy,vx);
    
    
    DoubleV sd = this->getFrenet(x, y, theta);
    DoubleV next_sd = this->getFrenet(x+vx, y+vy, theta);
    const double vs = next_sd[0]-sd[0];
    const double vd = next_sd[1]-sd[1];
     
    int dist = (int)(s-car->s);
    if (dist > 0)
    cout<<"Car:\t"<<id<<"\t"<<dist<<"\t"<<(int)ms2mph(vs)<<"\t"<<"\t"<<(int)d<<endl;
     */
    Vehicle v;
    auto it = vehicles.find(id);
    if ( it == vehicles.end() ){
      v.id = id;
      v.vs = 0;
      v.vd = 0;
      v.as = 0;
      v.ad = 0;
    }else{
      v = it->second;
      double dt = now - v.last_update;
      double vs = (s - v.s)/dt;
      double vd = (d - v.d)/dt;
      v.as = (vs - v.vs)/dt;
      v.ad = (vs - v.vd)/dt;
      v.vs = vs;
      v.vd = vd;
    }
    v.s = s;
    v.d = d;
    v.lane = laneFromCoords(v.d);
    v.last_update = now;
    v.prediction = {};
    for(int t=0;t<=horizon; t++){
      v.prediction.push_back({s+v.s*t,d+v.d*t});
    }
    //cout<<"Car:\t"<<id<<"\t"<<dist<<"\t"<<(int)ms2mph(v.vs)<<"\t"<<"\t"<<v.vd<<endl;
    vehicles[id] = v;
  }
  
  //Cleanup old
  vector<int> to_remove = {};
  for( auto v:vehicles ){
    if( std::find(id_list.begin(), id_list.end(), v.first) != id_list.end() ){
      to_remove.push_back(v.first);
    }
  }
  for(auto i: to_remove){
    vehicles.erase(i);
  }
  //cout<<"------------------------"<<endl;
}


VehicleList World::filter_predictions(bool in_front, double s, int lane, VehicleList vehicle_list){
  VehicleList filtered = {};
  for(auto vehicle:vehicle_list){
    auto id = vehicle.first;
    auto v = vehicle.second;
    if (lane == -1 || v.lane == lane){ // filter by d coords
      if ( ( in_front && v.s >= s ) || ( !in_front && v.s <= s ) ){
        filtered[id] = v;
      }
    }
  }
  return filtered;
}

VehicleList World::vehicle_in_gap(int lane, int min_s, int max_s, VehicleList vehicle_list){
  VehicleList filtered = {};
  for(auto vehicle:vehicle_list){
    auto id = vehicle.first;
    auto v = vehicle.second;
    if (v.lane == lane && v.s > min_s && v.s<max_s){ // filter by d coords
      filtered[id] = v;
    }
  }
  return filtered;
}


Vehicle World::get_closest(double s, VehicleList vehicle_list ){
  double min_distance = std::numeric_limits<float>::max();
  int car_id = -1;
  for(auto vehicle:vehicle_list){
    auto id = vehicle.first;
    auto v = vehicle.second;
    double dist = v.s - s;
    if ( dist < min_distance ){
      min_distance = dist;
      car_id = id;
    }
  }
  if (car_id > -1){
    return vehicle_list[car_id];
  }
  
  // return an "invalid vehicle" for empty lists, list size should be checked before calling this function.
  Vehicle v;
  v.id = -1;
  return v;
}

Vehicle World::get_farthest(double s, VehicleList vehicle_list ){
  double max_distance = 0;
  int car_id = -1;
  for(auto vehicle:vehicle_list){
    auto id = vehicle.first;
    auto v = vehicle.second;
    double dist = v.s - s;
    if ( dist > max_distance ){
      max_distance = dist;
      car_id = id;
    }
  }
  if (car_id > -1){
    return vehicle_list[car_id];
  }
  
  // return an "invalid vehicle" for empty lists, list size should be checked before calling this function.
  Vehicle v;
  v.id = -1;
  return v;
}


Vehicle World::get_slowest( VehicleList vehicle_list ){
  double min_speed = std::numeric_limits<float>::max();
  int car_id = -1;
  for(auto vehicle:vehicle_list){
    auto id = vehicle.first;
    auto v = vehicle.second;
    if ( v.vs < min_speed ){
      min_speed = v.vs;
      car_id = id;
    }
  }
  if (car_id > -1){
    return vehicle_list[car_id];
  }
  
  // return an "invalid vehicle" for empty lists, list size should be checked before calling this function.
  Vehicle v;
  v.id = -1;
  return v;
}

Vehicle World::get_fastest(VehicleList vehicle_list ){
  double max_speed = 0;
  int car_id = -1;
  for(auto vehicle:vehicle_list){
    auto id = vehicle.first;
    auto v = vehicle.second;
    if ( v.vs > max_speed ){
      max_speed = v.vs;
      car_id = id;
    }
  }
  if (car_id > -1){
    return vehicle_list[car_id];
  }
  
  // return an "invalid vehicle" for empty lists, list size should be checked before calling this function.
  Vehicle v;
  v.id = -1;
  return v;
}



double World::coordsFromLane(int lane){
  if (lane < 0){lane = 0;}
  if (lane > lane_cnt -1){lane = lane_cnt-1;}
  return lane*lane_size + lane_size/2;
}

int World::laneFromCoords(double coords){
  int lane = (int)(coords / lane_size);
  if (lane < 0){lane = 0;}
  if (lane > lane_cnt-1){lane = lane_cnt-1;}
  return lane;
}

int World::ClosestWaypoint(double x, double y)
{
  
  double closestLen = 100000; //large number
  int closestWaypoint = 0;
  
  for(int i = 0; i < this->map_x.size(); i++)
  {
    double map_x = this->map_x[i];
    double map_y = this->map_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
    
  }
  
  return closestWaypoint;
  
}

DoubleV2 World::getXYTrajectory(DoubleV2 fernet_trajectory){
  DoubleV2 trajectory = {{},{}};
  for(int i=0;i<fernet_trajectory[0].size();i++){
    DoubleV xy = getXY(fernet_trajectory[0][i], fernet_trajectory[1][i]);
    trajectory[0].push_back(xy[0]);
    trajectory[1].push_back(xy[1]);
  }
  return trajectory;
}

int World::NextWaypoint(double x, double y, double theta)
{
  
  int closestWaypoint = ClosestWaypoint(x,y);
  
  double map_x = this->map_x[closestWaypoint];
  double map_y = this->map_y[closestWaypoint];
  
  double heading = atan2( (map_y-y),(map_x-x) );
  
  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);
  //double angle = abs(theta-heading);
  
  if(angle > pi()/4)
  {
    closestWaypoint++;
    closestWaypoint %= this->map_x.size();
  }
  
  return closestWaypoint;
  
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates

DoubleV World::getFrenet(double x, double y, double theta)
{
  int next_wp = NextWaypoint(x,y, theta);
  
  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = map_x.size()-1;
  }
  
  double n_x = this->map_x[next_wp]-this->map_x[prev_wp];
  double n_y = this->map_y[next_wp]-this->map_y[prev_wp];
  double x_x = x - this->map_x[prev_wp];
  double x_y = y - this->map_y[prev_wp];
  
  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;
  
  double frenet_d = distance(x_x,x_y,proj_x,proj_y);
  
  //see if d value is positive or negative by comparing it to a center point
  
  double center_x = 1000-this->map_x[prev_wp];
  double center_y = 2000-this->map_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);
  
  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }
  
  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(this->map_x[i],this->map_y[i],this->map_x[i+1],this->map_y[i+1]);
  }
  
  frenet_s += distance(0,0,proj_x,proj_y);
  
  return {frenet_s,frenet_d};
  
}

// Transform from Frenet s,d coordinates to Cartesian x,y

DoubleV World::getXY(double s, double d)
{
  int prev_wp = -1;
  
  while(s > map_s[prev_wp+1] && (prev_wp < (int)(map_s.size()-1) ))
  {
    prev_wp++;
  }
  
  int wp2 = (prev_wp+1)%map_x.size();
  
  double heading = atan2((this->map_y[wp2]-this->map_y[prev_wp]),(this->map_x[wp2]-this->map_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-map_s[prev_wp]);
  
  double seg_x = this->map_x[prev_wp]+seg_s*cos(heading);
  double seg_y = this->map_y[prev_wp]+seg_s*sin(heading);
  
  double perp_heading = heading-pi()/2;
  
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  
  return {x,y};
  
}



