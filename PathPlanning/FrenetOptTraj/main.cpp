#include "Frenet_opt_traj.h"

#define SIM_LOOP 500

int main()
{
    Vec_f x({0.0, 10.0, 20.5, 35.0, 70.5});
    Vec_f y({0.0, -6.0, 5.0, 6.5, 0.0});
    std::vector<Poi_f> obstcles{
        {{20.0, 10.0}},
        {{30.0, 6.0}},
        {{30.0, 8.0}},
        {{35.0, 8.0}},
        {{50.0, 3.0}}};

    Spline2D cubicSpline(x, y);
    Vec_f x_ref;
    Vec_f y_ref;
    Vec_f yaw_ref;
    Vec_f curvature_ref;
    Vec_f s_ref;

    for (float i = 0; i < cubicSpline.s.back(); i += 0.1)
    {
        std::array<float, 2> point_ = cubicSpline.calc_postion(i);
        x_ref.push_back(point_[0]);
        y_ref.push_back(point_[1]);
        yaw_ref.push_back(cubicSpline.calc_yaw(i));
        curvature_ref.push_back(cubicSpline.calc_curvature(i));
        s_ref.push_back(i);
    }

    float current_speed = 10.0 / 3.6;
    float current_l = 2.0;
    float current_l_d = 0.0;
    float current_l_dd = 0.0;
    float current_s = 0.0;

    float area = 20.0;

    cv::namedWindow("frenet", cv::WINDOW_NORMAL);
    int count = 0;

    for (int i = 0; i < SIM_LOOP; i++)
    {
        // final_path.s[0]是t=0时刻的数据, 表示当前时刻, 程序需要的是下一个时刻
        FrenetPath final_path = frenet_optimal_planning(cubicSpline, current_s, current_speed, current_l, current_l_d, current_l_dd, obstcles);
        current_s = final_path.s[1];
        current_l = final_path.l[1];
        current_l_d = final_path.l_d[1];
        current_l_dd = final_path.l_dd[1];
        current_speed = final_path.s_d[1];

        // 判断是否到达终点
        if (std::pow((final_path.x[1] - x_ref.back()), 2) + std::pow((final_path.y[1] - y_ref.back()), 2) <= 1.0)
        {
            break;
        }

        // 绘制参考路径(黑色线条)
        cv::Mat bg(2000, 8000, CV_8UC3, cv::Scalar(255, 255, 255));
        for (unsigned int i = 1; i < x_ref.size(); i++)
        {
            cv::line(
                bg,
                cv_offset(x_ref[i - 1], y_ref[i - 1], bg.cols, bg.rows),
                cv_offset(x_ref[i], y_ref[i], bg.cols, bg.rows),
                cv::Scalar(0, 0, 0),
                10);
        }
        
        // 绘制规划的路径(蓝色实心圆)
        for (unsigned int i = 0; i < final_path.x.size(); i++)
        {
            cv::circle(
                bg,
                cv_offset(final_path.x[i], final_path.y[i], bg.cols, bg.rows),
                40, cv::Scalar(255, 0, 0), -1);
        }

        // 绘制规划的路径的起点(绿色实心圆), 也就是当前位置
        cv::circle(
            bg,
            cv_offset(final_path.x.front(), final_path.y.front(), bg.cols, bg.rows),
            50, cv::Scalar(0, 255, 0), -1);
        
        // 标记障碍物(红色空心圆)
        for (unsigned int i = 0; i < obstcles.size(); i++)
        {
            cv::circle(
                bg,
                cv_offset(obstcles[i][0], obstcles[i][1], bg.cols, bg.rows),
                40, cv::Scalar(0, 0, 255), 5);
        }

        cv::putText(
            bg,
            "Speed: " + std::to_string(current_speed * 3.6).substr(0, 4) + "km/h",
            cv::Point2i((int)bg.cols * 0.5, (int)bg.rows * 0.1),
            cv::FONT_HERSHEY_SIMPLEX,
            5,
            cv::Scalar(0, 0, 0),
            10);

        cv::imshow("frenet", bg);
        cv::waitKey(5);
    }
    return 0;
};
