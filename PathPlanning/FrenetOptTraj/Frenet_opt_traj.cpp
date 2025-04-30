#include "Frenet_opt_traj.h"
#include "../utils/quartic_polynomial.h"
#include "../utils/quintic_polynomial.h"

using namespace hancpp;

#define MAX_ROAD_WIDTH 7.0
#define D_ROAD_WIDTH 1.0
#define MINT 4.0
#define MAXT 5.0
#define DT 0.2
#define TARGET_SPEED 30.0 / 3.6 // target speed [m/s]
#define D_T_S 5.0 / 3.6         // target speed sampling length [m/s]
#define N_S_SAMPLE 1            // sampling number of target speed

#define KJ 0.1
#define KT 0.1
#define KD 1.0
#define KLAT 1.0
#define KLON 1.0

#define ROBOT_RADIUS 1.5 // robot radius [m]

#define MAX_SPEED 50.0 / 3.6 // maximum speed [m/s]
#define MAX_ACCEL 2.0        // maximum acceleration [m/ss]
#define MAX_CURVATURE 1.0    // maximum curvature [1/m]

float sum_of_power(std::vector<float> list)
{
    float sum = 0;
    for (auto val : list)
    {
        sum += val * val;
    }
    return sum;
}

Vec_Path calc_frenet_paths(float current_speed, float current_l, float current_l_d, float current_l_dd, float current_s)
{
    // 1. fp_lits存储所有的 frenet路径
    std::vector<FrenetPath> fp_lists;
    // 2. 遍历每一个侧向距离 li 和 侧向时间 ti, 生成当前侧向(l,v,a)到目标侧向(li,0,0)共ti时间的五次多项式曲线
    // 3. 遍历每一个侧向距离 li 和 侧向时间 ti, 遍历每一个目标纵向速度 target_v, 生成当前纵向 (s,v,a)到目标纵向(target_v,a_end)共 ti时间的四次多项式曲线
    for (float li = -MAX_ROAD_WIDTH; li < MAX_ROAD_WIDTH; li += D_ROAD_WIDTH)
    {
        for (float ti = MINT; ti < MAXT; ti += DT)
        {
            // 横向五次多项式
            QuinticPolynomial lat_qp = QuinticPolynomial(current_l, current_l_d, current_l_dd, li, 0, 0, ti);
            FrenetPath fp;
            // 4. 计算五次多项式曲线上每一点的 l,v,a,jerk
            for (float t = 0; t < ti; t += DT)
            {
                fp.t.push_back(t);
                fp.l.push_back(lat_qp.calc_point(t));
                fp.l_d.push_back(lat_qp.calc_first_derivative(t));
                fp.l_dd.push_back(lat_qp.calc_second_derivative(t));
                fp.l_ddd.push_back(lat_qp.calc_third_derivative(t));
            }
            // 纵向四次多项式
            for (float target_v = TARGET_SPEED - D_T_S * N_S_SAMPLE; target_v < TARGET_SPEED + D_T_S * N_S_SAMPLE; target_v += D_T_S)
            {
                QuarticPolynomial long_qp = QuarticPolynomial(current_s, current_speed, 0, target_v, 0, ti);
                FrenetPath fp_bot = fp;
                fp_bot.max_speed = std::numeric_limits<float>::min();
                fp_bot.max_accel = std::numeric_limits<float>::min();
                // 5. 计算三次多项式曲线上每一点的 s,v,a,jerk
                for (auto t : fp_bot.t)
                {
                    fp_bot.s.push_back(long_qp.calc_point(t));
                    fp_bot.s_d.push_back(long_qp.calc_first_derivative(t));
                    fp_bot.s_dd.push_back(long_qp.calc_second_derivative(t));
                    fp_bot.s_ddd.push_back(long_qp.calc_third_derivative(t));
                    if (fp_bot.s_d.back() > fp_bot.max_speed)
                    {
                        fp_bot.max_speed = fp_bot.s_d.back();
                    }
                    if (fp_bot.s_dd.back() > fp_bot.max_accel)
                    {
                        fp_bot.max_accel = fp_bot.s_dd.back();
                    }
                }

                // 6. 计算每一条frenet路径的代价
                // 横向代价: 横向加速度的累计 + 时间 + 最后偏离目标的横向距离
                fp_bot.cost_l = KJ * sum_of_power(fp_bot.l_ddd) + KT * ti + KD * std::pow(fp_bot.l.back(), 2);
                // 纵向代价: 纵向加速的的累计 + 时间 + 最后偏离目标速度
                fp_bot.cost_s = KJ * sum_of_power(fp_bot.s_ddd) + KT * ti + KD * (TARGET_SPEED - fp_bot.s_d.back());
                // 总代价
                fp_bot.cost_total = KLAT * fp_bot.cost_l + KLON * fp_bot.cost_s;
                fp_lists.push_back(fp_bot);
            }
        }
    }
    return fp_lists;
}

void calc_global_paths(Vec_Path &path_list, Spline2D csp)
{
    // 将 frenet 坐标系转换到 全局坐标系中
    // 1. 遍历每一条 frenet 路径
    for (auto path = path_list.begin(); path != path_list.end(); path++)
    {
        // 2. 遍历每一条路径里的每一个点,弧线 s 转到 x,y
        for (int i = 0; i < path->s.size(); i++)
        {
            // 3. 判断是否全部转换完成
            if (path->s[i] >= csp.s.back())
            {
                break;
            }
            std::array<float, 2> position = csp.calc_postion(path->s[i]);
            float iyaw = csp.calc_yaw(path->s[i]);
            path->x.push_back(position[0] - path->l[i] * std::sin(iyaw));
            path->y.push_back(position[1] + path->l[i] * std::cos(iyaw));
        }

        // 3. 求全局坐标系下的航向角 和 弧长
        for (int i = 0; i < path->x.size() - 1; i++)
        {
            float dx = path->x[i + 1] - path->x[i];
            float dy = path->y[i + 1] - path->y[i];
            path->yaw.push_back(std::atan2(dy, dx));
            path->ds.push_back(std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)));
        }
        path->yaw.push_back(path->yaw.back());
        path->ds.push_back(path->ds.back());

        // 4. 求曲率
        path->max_curvature = std::numeric_limits<float>::min();
        for (int i = 0; i < path->x.size() - 1; i++)
        {
            path->curvature.push_back((path->yaw[i + 1] - path->yaw[i]) / path->ds[i]);
            if (path->curvature.back() > path->max_curvature)
            {
                path->max_curvature = path->curvature.back();
            }
        }
    }
}

bool check_collision(FrenetPath path, const Vec_Poi obstacle)
{
    for (auto point : obstacle)
    {
        for (int i = 0; i < path.x.size(); i++)
        {
            float distance = std::sqrt(std::pow(path.x[i] - point[0], 2) + std::pow(path.y[i] - point[1], 2));
            if (distance <= ROBOT_RADIUS)
            {
                return false;
            }
        }
    }
    // true表示未碰撞
    return true;
}

// 返回 与障碍物未碰撞, 且最大速度, 加速度, 曲率满足要求的路径
Vec_Path check_paths(Vec_Path path_list, const Vec_Poi obstacle)
{
    Vec_Path output_fp_lists;
    for (auto path : path_list)
    {
        if (check_collision(path, obstacle) && path.max_speed < MAX_SPEED && path.max_accel < MAX_ACCEL && path.max_curvature < MAX_CURVATURE)
        {
            output_fp_lists.push_back(path);
        }
    }
    return output_fp_lists;
}

FrenetPath frenet_optimal_planning(Spline2D csp, float current_s, float current_speed, float current_l, float current_l_d, float current_l_dd, std::vector<Poi_f> obstacle)
{
    // 1. 获取所有路径
    Vec_Path all_paths = calc_frenet_paths(current_speed, current_l, current_l_d, current_l_dd, current_s);
    calc_global_paths(all_paths, csp);
    // 2. 挑选出与障碍物未碰撞, 且最大速度, 加速度, 曲率满足要求的路径
    Vec_Path selected_paths = check_paths(all_paths, obstacle);
    // 3. 挑选出 代价 最小的路径
    float min_cost = std::numeric_limits<float>::max();
    FrenetPath min_cost_path;
    for (auto path : selected_paths)
    {
        if (path.cost_total < min_cost)
        {
            min_cost = path.cost_total;
            min_cost_path = path;
        }
    }
    return min_cost_path;
}

cv::Point2i cv_offset(
    float x, float y, int image_width, int image_height)
{
    cv::Point2i output;
    output.x = int(x * 100) + 300;
    output.y = image_height - int(y * 100) - image_height / 3;
    return output;
};