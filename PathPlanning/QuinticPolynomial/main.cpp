#include "../../matplotlibcpp.h"
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

Matrix<double, 6, 1> quinticPolynomial(Vector3d start, Vector3d end, double T)
{
    Matrix<double, 6, 1> result;
    result[0] = start[0];
    result[1] = start[1];
    result[2] = start[2] / 2;

    Vector3d right;
    right << end[0] - start[0] - start[1] - 0.5 * start[2] * T * T,
        end[1] - start[1] - start[2] * T,
        end[2] - start[1];
    Matrix3d left;
    left << pow(T, 3), pow(T, 4), pow(T, 5),
        3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
        6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
    Vector3d solution = left.inverse() * right;
    result.segment(3, 3) = solution;
    return result;
}

double evaluateQuinticPolynomial(Matrix<double, 6, 1> coeffs, double t)
{
    return coeffs[0] + coeffs[1] * t + coeffs[2] * pow(t, 2) + coeffs[3] * pow(t, 3) + coeffs[4] * pow(t, 4) + coeffs[5] * pow(t, 5);
}

int main(int argc, char const *argv[])
{
    // 起始和结束条件
    Vector3d start(0, 0, 0);
    Vector3d end(1, 0, 0);
    double T = 2.0;

    // 计算五次多项式的系数
    Matrix<double, 6, 1> coeffs = quinticPolynomial(start, end, T);

    // 生成绘图数据
    std::vector<double> x_vals;
    std::vector<double> y_vals;
    for (double t = 0; t <= T; t += 0.01)
    {
        x_vals.push_back(t);
        y_vals.push_back(evaluateQuinticPolynomial(coeffs, t));
    }

    // 绘图
    plt::plot(x_vals, y_vals);
    plt::xlabel("Time");
    plt::ylabel("Position");
    plt::title("Quintic Polynomial Trajectory");
    plt::grid(true);
    plt::show();

    return 0;
}
