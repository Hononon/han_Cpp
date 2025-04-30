#include "../utils/frenet_path.h"
#include "../CubicSpline/cubic_spline.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace hancpp;

float sum_of_power(std::vector<float> list);

Vec_Path calc_frenet_paths(float current_speed, float current_l, float current_l_d, float current_l_dd, float current_s);

void calc_global_paths(Vec_Path &path_list, Spline2D csp);

bool check_collision(FrenetPath path, const Vec_Poi obstacle);

Vec_Path check_paths(Vec_Path path_list, const Vec_Poi obstacle);

FrenetPath frenet_optimal_planning(Spline2D csp, float current_s, float current_speed, float current_l, float current_l_d, float current_l_dd, Vec_Poi ob);

cv::Point2i cv_offset(float x, float y, int image_width = 2000, int image_height = 2000);