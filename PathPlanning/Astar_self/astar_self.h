#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <climits>
using namespace std;

void astar(int s, int e, vector<vector<int>> &graph, vector<int> &g, vector<int> &f,
           vector<int> &h, vector<bool> &visited, vector<int> &previous);

void print_path(int s, int e, const vector<int> &previous);
