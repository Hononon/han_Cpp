#include "RRT.h"
#include "../utils/geometry_utils.h"

RRT::Node::Node(double x, double y) : x(x), y(y) {};

RRT::RRT(vector<vector<double>> &obstacle_list,
         vector<double> &rand_area, vector<double> &play_area, double robot_radius,
         double expand_dis, double goal_sample_rate, int max_iter) : obstacle_list(obstacle_list), rand_area(rand_area),
                                                                     play_area(play_area), robot_radius(robot_radius),
                                                                     expand_dis(expand_dis), goal_sample_rate(goal_sample_rate),
                                                                     max_iter(max_iter) {};

RRT::Node *RRT::sampleFree()
{
    Node *rnd = nullptr;
    if ((rand() % 100) > goal_sample_rate)
    {
        double rnd_x = rand() / double(RAND_MAX) * (rand_area[1] - rand_area[0]) + rand_area[0];
        double rnd_y = rand() / double(RAND_MAX) * (rand_area[1] - rand_area[0]) + rand_area[0];
        return new Node(rnd_x, rnd_y);
    }
    else
    {
        return new Node(end->x, end->y);
    }
}

bool RRT::isInsidePlayArea(RRT::Node *node)
{
    if (node->x < play_area[0] ||
        node->x > play_area[1] ||
        node->y < play_area[2] ||
        node->y > play_area[3])
    {
        return false;
    }
    return true;
}

int RRT::getNearestNodeIndex(vector<RRT::Node *> node_list, RRT::Node *rnd_node)
{
    int min_index = -1;
    double d = numeric_limits<double>::max();
    for (int i = 0; i < node_list.size(); i++)
    {
        double d_temp = pow(rnd_node->x - node_list[i]->x, 2) + pow(rnd_node->y - node_list[i]->y, 2);
        if (d_temp < d)
        {
            d = d_temp;
            min_index = i;
        }
    }
    return min_index;
}

RRT::Node *RRT::steer(RRT::Node *from_node, RRT::Node *to_node, double extend_dis)
{
    vector<double> dis_angle = calDistanceAngle(from_node, to_node);
    double d = dis_angle[0];
    double angle = dis_angle[1];
    double new_x, new_y;
    if (extend_dis >= d)
    {
        new_x = to_node->x;
        new_y = to_node->y;
    }
    else
    {
        new_x = from_node->x + d * cos(angle) * extend_dis;
        new_y = from_node->y + d * sin(angle) * extend_dis;
    }
    Node *new_node = new Node(new_x, new_y);
    new_node->path_x.push_back(from_node->x);
    new_node->path_x.push_back(new_x);
    new_node->path_y.push_back(from_node->y);
    new_node->path_y.push_back(new_y);
    new_node->parent = from_node;
    return new_node;
}

vector<double> RRT::calDistanceAngle(RRT::Node *from_node, RRT::Node *end_node)
{
    double dx = end_node->x - from_node->x;
    double dy = end_node->y - from_node->y;
    double angle = atan2(dy, dx);
    double distance = sqrt(pow(dx, 2) + pow(dy, 2));
    return {distance, angle};
}

double RRT::calDistanceToGoal(double x, double y)
{
    double dx = x - end->x;
    double dy = y - end->y;
    return sqrt(pow(dx, 2) + pow(dy, 2));
}

bool RRT::obstacleFree(RRT::Node *node)
{
    // auto isSegmentIntersectingObstacle = [this](const Point &p1, const Point &p2)
    // {
    //     for (const auto &obs : obstacle_list)
    //     {
    //         Point obstacleCenter = {obs[0], obs[1]};
    //         double distance = distanceBetweenPoints(p1, p2);
    //         // 障碍点中心到两点连线直线的投影
    // const Point closestPointOnLine = closestPointOnSegment(p1, p2, obstacleCenter);
    //         double dist_to_center = distanceBetweenPoints(obstacleCenter, closestPointOnLine);
    //         // obs[2]障碍物半径
    //         if (dist_to_center <= obs[2] + robot_radius && dist_to_center >= 0 && dist_to_center <= distance)
    //         {
    //             return true;
    //         }
    //     }
    //     return false;
    // };

    // for (int i = 0; i < node->path_x.size() - 1; i++)
    // {
    //     Point point1 = {node->path_x[i], node->path_y[i]};
    //     Point point2 = {node->path_x[i + 1], node->path_y[i + 1]};
    //     if (isSegmentIntersectingObstacle(point1, point2))
    //     {
    //         return false;
    //     }
    // }
    // return true;
    vector<double> point1 = {node->path_x[0], node->path_y[0]};
    vector<double> point2 = {node->path_x[1], node->path_y[1]};
    double l2 = sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
    // double d =
    for (const auto &obs : obstacle_list)
    {
        vector<double> obstacleCenter = {obs[0], obs[1]};
        double dist_to_center = abs((point1[1] - point2[1]) * obstacleCenter[0] + (point2[0] - point1[0]) * obstacleCenter[1] + point1[0] * point2[1] - point1[1] * point2[0]) / l2;
        // obs[2]障碍物半径
        if (dist_to_center <= obs[2] + robot_radius && dist_to_center >= 0 && dist_to_center <= l2)
        {
            return false;
        }
    }
    return true;
}

pair<vector<double>, vector<double>> RRT::planning()
{
    node_list.push_back(begin);
    for (int i = 0; i < max_iter; i++)
    {
        Node *rnd_node = sampleFree();
        int nearest_index = getNearestNodeIndex(node_list, rnd_node);
        Node *nearest_node = node_list[nearest_index];
        Node *new_node = steer(nearest_node, rnd_node, expand_dis);
        if (isInsidePlayArea(new_node) && obstacleFree(new_node))
        {
            node_list.push_back(new_node);
        }
        Node *latest_node = node_list[node_list.size() - 1];
        if (calDistanceToGoal(latest_node->x, latest_node->y) < expand_dis)
        {
            Node *final_node = steer(latest_node, end, expand_dis);
            if (obstacleFree(final_node))
            {
                return generateFinalCourse(node_list.size() - 1);
            }
        }
        draw(rnd_node);
    }
    return {};
}

pair<vector<double>, vector<double>> RRT::generateFinalCourse(double goal_ind)
{
    vector<double> x_, y_;
    x_.push_back(end->x);
    y_.push_back(end->y);
    Node *node = node_list[goal_ind];
    while (node->parent != nullptr)
    {
        x_.push_back(node->x);
        y_.push_back(node->y);
        node = node->parent;
        // cout<<node->x<<","<<node->y<<endl;
    }
    x_.push_back(node->x);
    y_.push_back(node->y);
    return {x_, y_};
}

void RRT::setBegin(RRT::Node *begin)
{
    RRT::begin = begin;
}

void RRT::setEnd(RRT::Node *End)
{
    end = End;
}

void RRT::draw(Node *node)
{
    plt::clf();
    // 画随机点
    if (node)
    {
        plt::plot(vector<double>{node->x}, vector<double>{node->y}, "^k");
        if (robot_radius > 0)
        {
            plotCircle(node->x, node->y, robot_radius, "-r");
        }
    }

    // 画已生成的树
    for (Node *node1 : node_list)
    {
        if (node1->parent)
        {
            plt::plot(node1->path_x, node1->path_y, "-g");
        }
    }
    // 画障碍物
    for (vector<double> ob : obstacle_list)
    {
        plotCircle(ob[0], ob[1], ob[2]);
    }

    plt::plot(vector<double>{play_area[0], play_area[1], play_area[1], play_area[0], play_area[0]},
              vector<double>{play_area[2], play_area[2], play_area[3], play_area[3], play_area[2]}, "k-");

    // 画出起点和目标点
    plt::plot(vector<double>{begin->x}, vector<double>{begin->y}, "xr");
    plt::plot(vector<double>{end->x}, vector<double>{end->y}, "xr");
    plt::axis("equal");
    plt::grid(true);
    plt::xlim(play_area[0] - 1, play_area[1] + 1);
    plt::ylim(play_area[2] - 1, play_area[3] + 1);
    plt::pause(0.01);
}

void RRT::plotCircle(double x, double y, double size, string color)
{
    vector<double> x_t, y_t;
    for (double i = 0.; i <= 2 * PI; i += 0.01)
    {
        x_t.push_back(x + size * cos(i));
        y_t.push_back(y + size * sin(i));
    }
    plt::plot(x_t, y_t, color);
}