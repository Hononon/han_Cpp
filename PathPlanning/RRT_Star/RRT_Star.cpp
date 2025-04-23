#include "RRT_Star.h"
// #include "../utils/geometry_utils.h"

RRT_Star::RRT_Star(const vector<vector<double>> &obstacleList, const vector<double> &randArea, const vector<double> &playArea,
                   double robotRadius, double expandDis, double goalSampleRate, int maxIter, double connect_circle_radius,
                   bool search_until_max_iter) : RRT(obstacleList, randArea, playArea, robotRadius, expandDis, goalSampleRate, maxIter),
                                                 connect_circle_radius(connect_circle_radius), search_until_max_iter(search_until_max_iter) {};
// 找到x_new附近的所有点的索引
vector<int> RRT_Star::findNearIndexs(RRT::Node *new_node)
{
    vector<int> near_node_indexs;
    int nnode = node_list.size() + 1;
    int r = connect_circle_radius * sqrt((log(nnode) / nnode));
    for (int i = 0; i < node_list.size(); i++)
    {

        int d = calDistanceAngle(node_list[i], new_node)[0];
        if (d <= r)
        {
            near_node_indexs.push_back(i);
        }
    }
    return near_node_indexs;
}

// 计算to_node以from_node为跳板的cost
double RRT_Star::calNewCost(RRT::Node *from_node, RRT::Node *to_node)
{
    int d = calDistanceAngle(from_node, to_node)[0];
    return from_node->cost + d;
}

// 为x_new选择cost最小的节点作为父节点
RRT::Node *RRT_Star::chooseParent(RRT::Node *new_node, vector<int> near_node_indexs)
{
    if (near_node_indexs.empty())
        return nullptr;
    vector<double> costs;
    for (int i = 0; i < near_node_indexs.size(); i++)
    {
        Node *temp_node = steer(node_list[i], new_node);
        // Todo:只判断了node_list[i]和temp_node之间是否有碰撞, 没判断temp_node和new_node之间是否有碰撞
        if (temp_node && obstacleFree(temp_node))
        {
            costs.push_back(calNewCost(node_list[i], new_node));
        }
        else
        {
            costs.push_back(numeric_limits<double>::max());
        }
    }
    double min_cost = *min_element(costs.begin(), costs.end());
    if (min_cost == numeric_limits<double>::max())
    {
        cout << "There is no good path.(min_cost is inf)" << endl;
        return nullptr;
    }

    int min_cost_index = near_node_indexs[min_element(costs.begin(), costs.end()) - costs.begin()];

    // Todo: 当拓展距离大于 nearest_node和rnd_node间的距离(此时new_node就是rnd_node)
    // 且 拓展距离大于 minCostIndexNode和newNode间的距离时. determined_node才是new_node,
    // 否则determined_node是位于minCostIndexNode和newNode间的一个节点;
    Node *determined_node = steer(node_list[min_cost_index], new_node);
    determined_node->cost = min_cost;
    return determined_node;
}

// 让parent_node的所有子节点 以及 子节点的子节点 都重新更新cost
void RRT_Star::propagateCostToLeaves(RRT::Node *parent_node)
{
    for (int i = 0; i < node_list.size(); i++)
    {
        if (node_list[i]->parent = parent_node)
        {
            node_list[i]->cost = calNewCost(parent_node, node_list[i]);
            propagateCostToLeaves(node_list[i]);
        }
    }
}

// 为选择完父节点的x_new的所有邻居节点重新布线, 如果以x_new为跳板的代价小于以前的代价, 则重新布线
void RRT_Star::rewire(RRT::Node *node_with_updated_parent, vector<int> near_node_indexs)
{
    for (int i = 0; i < near_node_indexs.size(); i++)
    {
        Node *near_node = node_list[i];
        Node *edge_node = steer(node_with_updated_parent, near_node);
        if (!edge_node)
        {
            continue;
        }
        edge_node->cost = calNewCost(node_with_updated_parent, near_node);
        // 同样的问题, 只判断了edgeNode和node_with_updated_paren之间有无障碍, 没判断edgeNode和near_node间有无碰撞
        // near_node的cost大于node_with_updated_parent的cost+node_with_updated_parent和near节点的cost
        // 即从起点到near_node > 从起点到node_with_updated_parent到near_node
        if (obstacleFree(edge_node) && near_node->cost > edge_node->cost)
        {
            near_node->x = edge_node->x;
            near_node->y = edge_node->y;
            near_node->path_x = edge_node->path_x;
            near_node->path_y = edge_node->path_y;
            near_node->parent = edge_node->parent;
            near_node->cost = edge_node->cost;
            // 让node_with_updated_parent的所有子节点 以及 子节点的子节点 都重新更新cost
            propagateCostToLeaves(node_with_updated_parent);
        }
    }
}

int RRT_Star::findBestGoalId()
{
    vector<int> goal_indexs;
    vector<int> safe_goal_indexs;
    for (int i = 0; i < node_list.size(); i++)
    {
        if (calDistanceToGoal(node_list[i]->x, node_list[i]->y) < expand_dis)
        {
            goal_indexs.push_back(i);
        }
    }
    for (int goal_index : goal_indexs)
    {
        Node *temp = steer(node_list[goal_index], end);
        if (obstacleFree(temp))
        {
            safe_goal_indexs.push_back(goal_index);
        }
    }
    if (safe_goal_indexs.empty())
    {
        return -1;
    }
    double min_cost = numeric_limits<double>::max();
    int min_index = -1;
    for (auto safe_goal_index : safe_goal_indexs)
    {
        if (node_list[safe_goal_index]->cost <= min_cost)
        {
            min_cost = node_list[safe_goal_index]->cost;
            min_index = safe_goal_index;
        }
    }
    return min_index;
}

pair<vector<double>, vector<double>> RRT_Star::planning()
{
    node_list.push_back(begin);
    for (int i = 0; i < max_iter; i++)
    {
        // 1. 随机采样
        Node *rnd_node = sampleFree();
        cout << "随机树节点个数：" << node_list.size() << endl;
        // 2. 找到最近的节点
        int nearest_node_index = getNearestNodeIndex(node_list, rnd_node);
        Node *nearest_node = node_list[nearest_node_index];
        // 3. 拓展新节点x_new
        Node *new_node = steer(nearest_node, rnd_node, expand_dis);
        if (isInsidePlayArea(new_node) && obstacleFree(new_node))
        {
            new_node->cost = calNewCost(nearest_node, new_node);
            // 4. 从x_new周围的邻近节点中找到cost最小的节点作为父节点
            vector<int> near_node_indexs = findNearIndexs(new_node);

            Node *node_with_updated_parent = chooseParent(new_node, near_node_indexs);
            // 5. 为x_new周围的邻近节点重新布线
            if (node_with_updated_parent)
            {
                rewire(node_with_updated_parent, near_node_indexs);
                node_list.push_back(node_with_updated_parent);
            }
            else
            {
                node_list.push_back(new_node);
            }
        }
        // 6. 画出rnd节点
        draw(rnd_node);
        // 7. 判断是否到达终点
        int last_index = findBestGoalId();
        if (last_index != -1)
        {
            cout << "reaches the goal!" << endl;
            return generateFinalCourse(last_index);
        }
    }
    // 8. 遍历最大轮数后, 有无到达终点
    cout << "达到最大回合数" << endl;
    int last_index = findBestGoalId();
    if (last_index != -1)
        return generateFinalCourse(last_index);
    return {};
}
