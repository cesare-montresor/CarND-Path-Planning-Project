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
  
  for(double i=0; i<world->lane_cnt;i++){
    evaluations[i] = {i,0}; // lane, cost
  }
}

DoubleV2 BehaviourPlanner::best_lane(){
  DoubleV2 lane_info={};
  
  VehicleList in_front = world->filter_predictions(true, car->s, -1, world->vehicles);
  Vehicle futherst = world->get_farthest(car->s, in_front);
  
  double max_distance = futherst.s - car->s;
  auto in_front_cnt = in_front.size();
  int current_lane = world->laneFromCoords(car->d);
  
  for(int lane=0; lane<world->lane_cnt; lane++ ){
    
    VehicleList in_lane = world->filter_predictions(true, car->s, lane, in_front);
    auto in_lane_cnt = in_lane.size();
    
    double cost = 0;
    double lane_speed = 0;
    double free_space = 0;
    if ( in_lane_cnt > 0 ){
      Vehicle closest = world->get_closest(car->s, in_lane);
      lane_speed = closest.vs;
      free_space = closest.s - car->s;
      
      cost += (in_lane_cnt / (double)in_front_cnt) * COST_COUNT;
      cost += (1-(free_space/max_distance)) * COST_DISTANCE;
    }else{
      free_space = max_distance * 2;
      lane_speed = car->MAX_SPEED;
    }
    
    if (lane != current_lane ){
      cost += COST_SHIFT;
    }
    cost += (1-(lane_speed/car->MAX_SPEED))*COST_SPEED;
    
    lane_info.push_back({(double)lane, cost, lane_speed, free_space});
  }
  
  sort(lane_info.begin(), lane_info.end(), [](const DoubleV &a, const DoubleV &b) -> bool
  {
    return a[1] < b[1];
  });
  cout<<endl<<endl;
  for (auto li: lane_info){
    cout<<li[0]<<"\t"<<li[1]<<"\t"<<li[2]<<"\t"<<li[3]<<endl;
  }
  cout<<"----------------------------------"<<endl;
  return lane_info;
}

DoubleV2 BehaviourPlanner::evaluate_manuver(){
  double now = now_ms();
  if (evaluate_timer == -1){
    evaluate_timer = now;
  }
  auto lane_info = best_lane();
  if (evaluate_timer + EVALUATION_TIME < now ){ //Aggregate
    for(auto lane: lane_info){
      evaluations[lane[0]][1] += lane[1];
    }
  }else{ // Execute
    double min_cost = MAXFLOAT;
    int lane = -1;
    for(auto info: lane_info){
      
    }
  }
  
  
  
  
}

DoubleV2 BehaviourPlanner::get_trajectory(DoubleV2 previous_path){
  auto next_states = possible_states();
  
  DoubleV costs = {};
  DoubleV2 final_trajectory;
  DoubleV3 trajectories = {};
  
  
  
  for(auto state: next_states ){
    if (state == "KL"){
      auto t = trajectory_for_state(state);
      trajectories.push_back(t);
      costs.push_back(0);
    }
  }
  
  double min_cost = MAXFLOAT;
  double best_traj = -1;
  for(int i = 0 ; i<trajectories.size(); i++){
    if (costs[i]<min_cost){
      best_traj = i;
    }
  }
  
  auto xy_trajectory = world->getXYTrajectory(trajectories[best_traj]);
  return xy_trajectory;
}

vector<vector<double>> BehaviourPlanner::trajectory_for_state(string state){
  vector<vector<double>> trajectory;
  if (state == "KL"){
    auto filtered = world->filter_predictions(true, car->s, world->laneFromCoords(car->d), world->vehicles);
    auto closest = world->get_closest(car->s, filtered);
    
    double max_vs = target_speed; //v_info[2];
    double s = closest.s - buffer_distance;
    double d = world->coordsFromLane(world->laneFromCoords(car->d));
    
    //trajectory = t.generate_trajectory_cs(s, d, max_vs, max_acc); //TODO: fix acce
  }
  
  return trajectory;
}

vector<string> BehaviourPlanner::possible_states(){
  int lane = this->world->laneFromCoords(car->d);
  
  if (car->state == "KL"){
    vector<string> states = {"KL"};
    if ( lane > 0 ){ states.push_back("PL"); }
    if ( lane < this->world->lane_cnt-1 ){ states.push_back("PR"); }
    return states;
  }
  
  if (car->state == "PL"){
    return {"KL","PL","L"};
  }
  
  if (car->state == "PR"){
    return {"KL","PR","R"};
  }
  
  if (car->state == "L"){
    return {"KL","L"};
  }
  
  if (car->state == "R"){
    return {"KL","R"};
  }
  
  return {"KL"};
}
