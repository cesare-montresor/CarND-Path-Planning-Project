//
//  Trajectory.cpp
//  Path_Planning
//
//  Created by Cesare on 28/10/2017.
//
//

#include "Trajectory.h"



vector<vector<double>> Trajectory::constant_speed(vector<double> start, vector<double> end, vector<double> speed){
  return {{},{}};
}

  
vector<vector<double>> Trajectory::generate_trajectory_jmt(Car car, double final_s, double final_d, double max_vs, double max_acc_s){
  Car *c = &car;
  double horizon = 10;
  double manuver_s = 10;
  double t_res = 0.02;
  
  vector<double> start_s = { c->s, c->speed_s,c->acc_s };
  vector<double> start_d = { c->d, c->speed_d,c->acc_d };
  
  vector<double> end_s = { c->s+manuver_s, c->speed_s, 1 }; //TODO: fix final speed and acc
  vector<double> end_d = { final_d, 0,0 };
  
  vector<double> as = JMT(start_s, end_s, horizon);
  vector<double> ad = JMT(start_d, end_d, horizon);
  
  vector<vector<double>> trajectory = {{},{}};
  for(double t1=0;t1<horizon;t1+=t_res){
    const double t2 = t1*t1;
    const double t3 = t2*t1;
    const double t4 = t3*t1;
    const double t5 = t4*t1;
    const double s = as[0] + as[1]*t1 + as[2]*t2 + as[3]*t3 + as[4]*t4 + as[5]*t5;
    const double d = ad[0] + ad[1]*t1 + ad[2]*t2 + ad[3]*t3 + ad[4]*t4 + ad[5]*t5;
    trajectory[0].push_back(s);
    trajectory[1].push_back(d);
  }
  return trajectory;
}
