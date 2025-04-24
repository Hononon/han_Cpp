#pragma

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;
using namespace Eigen;

double factorial(int n);

Vector2d bezierCommon(vector<Vector2d> Ps, double t);