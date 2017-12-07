//
//  BehaviourPlanner.hpp
//  path_planning
//
//  Created by Cesare on 23/10/2017.
//

#ifndef BehaviourPlanner_h
#define BehaviourPlanner_h

#include <stdio.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <stdio.h>
#include "World.h"
#include "Car.h"
#include <map>
#include "tools.h"

using namespace std;


struct LaneInfo{
  int lane;
  float cost;
  float lane_speed;
  float free_space;
  double timestamp;
};

typedef vector<LaneInfo> LaneInfoV;
typedef map<int, LaneInfoV> LaneEvaluations;

class BehaviourPlanner{
  
public:
  const double CHANGE_LANE_DISTANCE = 50;
  const double SAFE_GAP_BEFORE = 10;
  const double SAFE_GAP_AFTER = 20;
  
  const double EVALUATION_TIME = 2;
  const double COOLDOWN_TIME = 3;
  
  const double COST_COUNT = 10;
  const double COST_DISTANCE = 100;
  const double COST_SHIFT = 10;
  const double COST_SPEED = 30;
  const double COST_SPEED_MAX = 20;
  
  LaneEvaluations evaluations = {};
  
  double evaluate_timer = -1;
  double cooldown_timer = -1;
  
  int target_lane = -1;
  
  
  int horizon = 10;
  double t_res = 0.02; // 20 ms, simulator time resolution
  double target_speed = mph2ms(50.0);
  double buffer_distance = 2.0;
  double max_acc = 25.0; // 24.0 ?
  double manuver_s = 30; // fixed size for
  
  World *world;
  Car *car;
  
  BehaviourPlanner(World &w, Car &c);
  vector<string> possible_states();
  virtual ~BehaviourPlanner(){};
  bool can_move_to_lane(int lane);
  bool can_move_right();
  bool can_move_left();
  bool should_change_lane();
  void decide();
  LaneInfo best_lane();
  int evaluate_manuver();
  
  
};


#endif /* BehaviourPlanner_h */
