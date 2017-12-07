//
//  BehaviourPlanner.cpp
//  path_planning
//
//  Created by Cesare on 23/10/2017.
//

#include "BehaviourPlanner.h"
#include "World.h"

BehaviourPlanner::BehaviourPlanner(World &w, Car &c){
  this->world = &w;
  this->car = &c;
  
  for(int i=0; i<world->lane_cnt;i++){
    evaluations[i] = {}; // lane, cost
  }
}

bool BehaviourPlanner::should_change_lane(){
  VehicleList vehicles_head = world->filter_predictions(true, car->s, car->lane, world->vehicles);
  if (vehicles_head.size() == 0){
    return false;
  }
  Vehicle closest = world->get_closest(car->s, vehicles_head);
  double distance = abs(car->s - closest.s);
  return distance < CHANGE_LANE_DISTANCE;
}

bool BehaviourPlanner::can_move_left(){
  return can_move_to_lane(car->lane-1);
}

bool BehaviourPlanner::can_move_right(){
  return can_move_to_lane(car->lane+1);
}

bool BehaviourPlanner::can_move_to_lane(int lane){
  if (lane < 0 || lane > world->lane_cnt ){
    return false;
  }
  float min_s = car->s - SAFE_GAP_BEFORE;
  float max_s = car->s + SAFE_GAP_AFTER;
  VehicleList vehicles_in_gap = world->vehicle_in_gap(lane, min_s, max_s, world->vehicles);
  return vehicles_in_gap.size() == 0;
}

int BehaviourPlanner::evaluate_manuver(){
  auto lane_info = best_lane();
  auto now = now_ms();
  if (cooldown_timer == -1){
    cooldown_timer = now + COOLDOWN_TIME;
  }
  
  if (target_lane == -1){
    target_lane = car->lane;
  }
  
  if (now > cooldown_timer){
    
    if (should_change_lane()){
      if ( lane_info.lane < car->lane && can_move_left() ){
        target_lane -= 1;
        cooldown_timer = now + COOLDOWN_TIME;
      }else if ( lane_info.lane > car->lane && can_move_right() ){
        target_lane += 1;
        cooldown_timer = now + COOLDOWN_TIME;
      }
    }
  }
  cout<<"target_lane:"<<target_lane<<endl;
  return target_lane;
}

LaneInfo BehaviourPlanner::best_lane(){
  double now = now_ms();
  map<int, LaneInfo> latest_lane_info;
  VehicleList in_front = world->filter_predictions(true, car->s, -1, world->vehicles);
  double max_distance = 250;
  /*
  auto in_front_cnt = in_front.size();
  if (in_front_cnt > 0){
    Vehicle futherst = world->get_farthest(car->s, in_front);
    max_distance = futherst.s - car->s;
  }
  */
  //cout<<"In front:"<<in_front_cnt<<endl;
  
  int current_lane = car->lane;
  for(int lane=0; lane<world->lane_cnt; lane++ ){
    LaneInfo lane_info;
    VehicleList in_lane = world->filter_predictions(true, car->s, lane, in_front);
    auto in_lane_cnt = in_lane.size();
    
    double cost = 0;
    double lane_speed = 0;
    double free_space = 0;
    double distance_cost = 0;
    double distance_cost_new = 0;
    if ( in_lane_cnt > 0 ){
      Vehicle closest = world->get_closest(car->s, in_lane);
      lane_speed = min(closest.vs,car->MAX_SPEED);
      free_space = closest.s - car->s;
      
      distance_cost = (1-(free_space/max_distance)) * COST_DISTANCE;
      distance_cost_new = exp((1-(free_space/50))) * 20;
      cost += distance_cost + distance_cost_new;
    }else{
      free_space = max_distance * 2;
      lane_speed = car->MAX_SPEED;
    }
    
    if (lane != current_lane ){
      cost += COST_SHIFT;
    }
    cost += (1-(lane_speed/car->MAX_SPEED))*COST_SPEED;
    cout<<(int)lane<<"\t"<<in_lane_cnt<<"\t"<<(int)free_space<<"\t"<<(int)distance_cost<<"\t"<<(int)distance_cost_new<<endl;
    
    lane_info.lane = lane;
    lane_info.cost = cost;
    lane_info.lane_speed = lane_speed;
    lane_info.free_space = free_space;
    lane_info.timestamp = now;
    
    evaluations[lane].push_back(lane_info);
    latest_lane_info[lane] = lane_info;
    //auto l = lane_info;
    //cout<<(int)l.lane<<"\t"<<(int)l.cost<<"\t"<<(int)l.lane_speed<<"\t"<<(int)l.free_space<<"\t"<<"\n";
  }
  
  DoubleV costs = {0,0,0};
  cout<<"Evaluations queue size:\t";
  for(auto &ev:evaluations){
    auto lane = ev.first;
    auto lane_info_list = &ev.second;
    cout<<lane_info_list->size()<<"\t";
    for (auto lane_info=lane_info_list->begin(); lane_info!=lane_info_list->end(); ){
      if ( lane_info->timestamp + EVALUATION_TIME >= now ){
        costs[lane] += lane_info->cost;
        lane_info++;
      }else{
        lane_info_list->erase(lane_info);
      }
    }
  }
  cout<<endl;
  cout<<"\n--------------------------\n";
  
  double min_cost = MAXFLOAT;
  int lane_idx = -1;
  for(int i =0; i < costs.size(); i++ ){
    if (costs[i]<min_cost){
      min_cost = costs[i];
      lane_idx = i;
    }
  }
  
  
  return latest_lane_info[lane_idx];
}

