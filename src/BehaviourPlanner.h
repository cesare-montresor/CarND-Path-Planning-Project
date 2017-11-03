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
#include "Trajectory.h"

using namespace std;

class BehaviourPlanner{
  
public:
  const double EVALUATION_TIME = 1;
  const double COOLDOWN_TIME = 3;
  
  const double COST_COUNT = 10;
  const double COST_DISTANCE = 50;
  const double COST_SHIFT = 20;
  const double COST_SPEED = 30;
  const double COST_SPEED_MAX = 20;
  
  map<int, DoubleV> evaluations = {};
  
  double evaluate_timer = -1;
  double cooldown_timer = -1;
  
  
  int horizon = 10;
  double t_res = 0.02; // 20 ms, simulator time resolution
  double target_speed = mph2ms(50.0);
  double buffer_distance = 2.0;
  double max_acc = 25.0; // 24.0 ?
  double manuver_s = 30; // fixed size for
  
  World *world;
  Car *car;
  Trajectory t;
  
  
  BehaviourPlanner(World &w, Car &c);
  vector<string> possible_states();
  virtual ~BehaviourPlanner(){};
  DoubleV2 best_lane();
  DoubleV2 get_trajectory(DoubleV2 previous_path);
  DoubleV2 trajectory_for_state(string state);
  DoubleV2 BehaviourPlanner::evaluate_manuver();
  
  
};


#endif /* BehaviourPlanner_h */
