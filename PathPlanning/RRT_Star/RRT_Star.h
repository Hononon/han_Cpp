#pragma

#include "../RRT/RRT.h"

class RRT_Star : public RRT
{
public:
    double connect_circle_radius;
    bool search_until_max_iter;

    RRT_Star(const vector<vector<double>> &obstacleList, const vector<double> &randArea, const vector<double> &playArea,
             double robotRadius, double expandDis, double goalSampleRate, int maxIter, double connect_circle_radius, bool search_until_max_iter);

    vector<int> findNearIndexs(Node *new_node);

    double calNewCost(Node *near_node, Node *new_node);

    Node *chooseParent(Node *new_node, vector<int> near_node_indexs);

    void propagateCostToLeaves(Node *parent_node);

    void rewire(Node *node_with_updated_parent, vector<int> near_node_indexs);

    int findBestGoalId();

    pair<vector<double>, vector<double>> planning();
};