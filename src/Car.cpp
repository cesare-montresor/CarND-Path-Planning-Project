//
//  Car.cpp
//  path_planning
//
//  Created by Cesare on 23/10/2017.
//

#include "Car.h"

Car::Car(){
  
}

void Car::adjust_speed(double speed){
  if (speed < 0){speed = 0; }
  double max_speed = min(speed,MAX_SPEED);
  
  
}

void Car::update(double x,double y,double s,double d,double yaw,double speed){
  if (last_update == -1){
    last_update = now_ms();
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;
    this->speed_s = 0;
    this->speed_d = 0;
    
    this->acc_s = 0;
    this->acc_d = 0;
    return;
  }
  
  double dt = now_ms() - last_update;
  last_update = now_ms();
  
  double delta_s = s-this->s;
  double delta_d = d-this->d;
  //double delta_yaw = yaw-this->yaw;
  //double delta_speed = speed-this->speed;
  
  double speed_s = 0;
  double speed_d = 0;
  
  if (dt > 0.001 ){
    speed_s = delta_s / dt;
    speed_d = delta_d / dt;
  }
  
  double delta_speed_s = speed_s-this->speed_s;
  double delta_speed_d = speed_d-this->speed_d;
  
  double acc_s = 0;
  double acc_d = 0;
  
  if (dt > 0.001 ){
    acc_s = delta_speed_s / dt;
    acc_d = delta_speed_d / dt;
  }
  
  
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
  
  this->speed_s = speed_s;
  this->speed_d = speed_d;
  
  this->acc_s = acc_s;
  this->acc_d = acc_d;
}


