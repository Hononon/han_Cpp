#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include "../CubicSpline/types.h"

namespace hancpp
{
    class FrenetPath
    {
    public:
        float cost_l = 0;
        float cost_s = 0;
        float cost_total = 0;

        Vec_f t;
        Vec_f l;     // 横向位移
        Vec_f l_d;   // 横向速度
        Vec_f l_dd;  // 横向加速度
        Vec_f l_ddd; // 横向jerk

        Vec_f s;     // 纵向位移
        Vec_f s_d;   // 纵向速度
        Vec_f s_dd;  // 纵向加速度
        Vec_f s_ddd; // 纵向jerk

        // 笛卡尔坐标系
        Vec_f x;
        Vec_f y;
        Vec_f yaw;
        Vec_f ds;
        Vec_f curvature;

        float max_speed;
        float max_accel;
        float max_curvature;
    };

    using Vec_Path = std::vector<FrenetPath>;
}
