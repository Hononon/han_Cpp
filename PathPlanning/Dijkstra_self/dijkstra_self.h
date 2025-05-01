#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <climits>
using namespace std;

void dijkstra(int s, int e, vector<vector<int>> &graph, vector<int> &g, vector<bool> &visited, vector<int> &previous);

void print_path(int s, int e, const vector<int> &previous);