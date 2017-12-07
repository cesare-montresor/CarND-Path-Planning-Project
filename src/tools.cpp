//
//  tools.cpp
//  path_planning
//
//  Created by Cesare on 26/10/2017.
//

#include <stdio.h>
#include "tools.h"

double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double mph2ms(double speed)
{
  return speed/2.224;
}

double ms2mph(double speed)
{
  return speed*2.224;
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double now_ms(){
  auto now = high_resolution_clock::now();
  long ms = time_point_cast<milliseconds>(now).time_since_epoch().count();
  return (double)ms/1000;
}

DoubleV JMT(vector<double> start, vector <double> end, double T)
{
  const double t1 = T;
  const double t2 = t1*T;
  const double t3 = t2*T;
  const double t4 = t3*T;
  const double t5 = t4*T;
  
  const double s = start[0];
  const double s_ = start[1];
  const double s__ = start[2];
  
  const double e = end[0];
  const double e_ = end[1];
  const double e__ = end[2];
  
  const double r0 = e - (s + s_*T + 0.5*s__*t2);
  const double r1 = e_ - (s_ + s__*t1);
  const double r2 = e__ - s__;
  
  MatrixXd B(3,3);
  B <<t3,   t4,    t5,
  3*t2, 4*t3,  5*t4,
  6*t1, 12*t2, 20*t3;
  
  VectorXd R(3);
  R << r0,r1,r2;
  
  VectorXd A = B.inverse() * R;
  
  return {s,s_,0.5*s__,A[0],A[1],A[2] };
}
