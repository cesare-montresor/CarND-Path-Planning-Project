//
//  Trajectory.hpp
//  Path_Planning
//
//  Created by Cesare on 28/10/2017.
//
//

#ifndef Trajectory_h
#define Trajectory_h


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
#include "Car.h"

class Trajectory{
public:
  vector<vector<double>> constant_speed(vector<double> start, vector<double> end, vector<double> speed);
  vector<vector<double>> generate_trajectory_jmt(Car car, double final_s, double final_d, double max_vs, double max_acc_);
};

#endif /* Trajectory_h */
