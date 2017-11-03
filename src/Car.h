//
//  Car.hpp
//  path_planning
//
//  Created by Cesare on 23/10/2017.
//

#ifndef Car_h
#define Car_h


#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <stdio.h>
#include "tools.h"


using namespace std;

class Car{
public:
  string state = "KL";
  const double MAX_SPEED = mph2ms(50);
  const double MAX_ACC = 10;
  
  long last_update=-1;
  
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  
  double speed_s;
  double speed_d;
  
  double acc_s;
  double acc_d;
  
  Car();
  virtual ~Car(){};
  
  void update(double x,double y,double s,double d,double yaw,double speed);
  void adjust_speed(double speed=-1);
};


#endif /* Car_h */
