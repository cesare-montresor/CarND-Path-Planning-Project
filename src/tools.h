//
//  tools.h
//  Path_Planning
//
//  Created by Cesare on 23/10/2017.
//


#ifndef tools_h
#define tools_h

#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include <chrono>
#include <vector>

using namespace std;
using namespace std::chrono;
using namespace Eigen;

#define DoubleV vector<double>
#define DoubleV2 vector<DoubleV>
#define DoubleV3 vector<DoubleV2>


double pi();
double deg2rad(double x);
double rad2deg(double x);
double mph2ms(double speed);
double ms2mph(double speed);

double distance(double x1, double y1, double x2, double y2);

double now_ms();

DoubleV JMT(vector<double> start, vector <double> end, double T);


#endif /* tools_h */
