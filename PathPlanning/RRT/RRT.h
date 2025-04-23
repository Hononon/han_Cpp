#pragma

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <time.h>

#include "../../matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;

#define PI 3.14159265354

class RRT
{
public:
    struct Node
    {
        double x, y;
        vector<double> path_x, path_y;
        Node *parent;
        double cost;
        Node(double x, double y);
    };

public:
    vector<vector<double>> obstacle_list; // 障碍物列表
    vector<double> rand_area;             // 采样区域[x_min,x_max,y_min,y_max]
    vector<double> play_area;             // 可行区域[x_min,x_max,y_min,y_max]
    double expand_dis;                    // 拓展步长
    double goal_sample_rate;              // 采样概率
    Node *begin;                          // 根节点
    Node *end;                            // 终结点
    vector<Node *> node_list;             // 树
    int max_iter;                         // 最大采样数
    double robot_radius;                  // 机器人半径

public:
    RRT(const vector<vector<double>> &obstacle_list,
        const vector<double> &rand_area,const vector<double> &play_area, double robot_radius,
        double expand_dis, double goal_sample_rate, int max_iter);

    Node *sampleFree(); // 随机采样

    bool isInsidePlayArea(Node *node); // 判断节点是否在可行域

    int getNearestNodeIndex(vector<Node *> node_list, Node *rnd_node); // 计算离采样点最近的节点

    Node *steer(Node *from_node, Node *to_node, double extend_dis = numeric_limits<double>::max()); // 拓展出x_new

    double calDistanceToGoal(double x, double y); // 计算离目标的距离

    vector<double> calDistanceAngle(Node *from_node, Node *end_node); // 计算距离和角度

    bool obstacleFree(Node *node); // 判断节点x_new和from_node之间的路径是否有障碍物
    
    pair<vector<double>, vector<double>> planning();

    pair<vector<double>, vector<double>> generateFinalCourse(double goal_index);

    void setBegin(Node *begin);
    void setEnd(Node *end);
    void plotCircle(double x, double y, double size, string color = "b");
    void draw(Node *node = nullptr);
};
