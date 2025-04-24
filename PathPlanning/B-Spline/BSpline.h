#pragma

#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
using namespace std;
using namespace Eigen;

double baseFunction(int i, int k, double u, vector<double>node_vector);

vector<double> u_quasi_uniform(int n,int k);