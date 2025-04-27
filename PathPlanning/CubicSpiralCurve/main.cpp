#include <iostream>
#include <Eigen/Dense>
#include "../../matplotlibcpp.h"

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

double fKappka(Matrix<double, 3, 1> coeffs, double s, Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    // coeffs [p_1,p_2,s_f]
    double a = start_state[3];
    double b = -(11 * start_state[3] - 18 * coeffs[0] + 9 * coeffs[1] - 2 * end_state[3]) / (2 * coeffs[2]);
    double c = 9 * (2 * start_state[3] - 5 * coeffs[0] + 4 * coeffs[1] - end_state[3]) / (2 * pow(coeffs[2], 2));
    double d = -9 * (start_state[3] - 3 * coeffs[0] + 3 * coeffs[1] - end_state[3]) / (2 * pow(coeffs[2], 3));
    return a + b * s + c * pow(s, 2) + d * pow(s, 3);
}

double fTheta(Matrix<double, 3, 1> coeffs, double s, Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    double a = start_state[3];
    double b = -(11 * start_state[3] - 18 * coeffs[0] + 9 * coeffs[1] - 2 * end_state[3]) / (2 * coeffs[2]);
    double c = 9 * (2 * start_state[3] - 5 * coeffs[0] + 4 * coeffs[1] - end_state[3]) / (2 * pow(coeffs[2], 2));
    double d = -9 * (start_state[3] - 3 * coeffs[0] + 3 * coeffs[1] - end_state[3]) / (2 * pow(coeffs[2], 3));
    return start_state[2] + a * s + b * pow(s, 2) / 2 + c * pow(s, 3) / 3 + d * pow(s, 4) / 4;
}

// simpson方法
function<double(function<double(double)>, double, double, int)> simpson_method()
{
    return [](function<double(double)> func, double a, double b, int n)
    {
        if (n % 2 != 0)
        {
            throw std::invalid_argument("子区间数量 n 必须为偶数。");
        }
        double h = (b - a) / n;
        double s = func(a) + func(b);
        for (int i = 1; i < n; ++i)
        {
            if (i % 2 == 0)
            {
                s += 2 * func(a + i * h);
            }
            else
            {
                s += 4 * func(a + i * h);
            }
        }
        return s * h / 3;
    };
}

double fX(Matrix<double, 3, 1> coeffs, double s, Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    double a = start_state[3];
    double b = -(11 * start_state[3] - 18 * coeffs[0] + 9 * coeffs[1] - 2 * end_state[3]) / (2 * coeffs[2]);
    double c = 9 * (2 * start_state[3] - 5 * coeffs[0] + 4 * coeffs[1] - end_state[3]) / (2 * pow(coeffs[2], 2));
    double d = -9 * (start_state[3] - 3 * coeffs[0] + 3 * coeffs[1] - end_state[3]) / (2 * pow(coeffs[2], 3));
    auto simpson = simpson_method();
    auto f = [coeffs, start_state, end_state](double x)
    {
        return cos(fTheta(coeffs, x, start_state, end_state));
    };
    return start_state[0] + simpson(f, 0, s, 50);
}

double fY(Matrix<double, 3, 1> coeffs, double s, Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    double a = start_state[3];
    double b = -(11 * start_state[3] - 18 * coeffs[0] + 9 * coeffs[1] - 2 * end_state[3]) / (2 * coeffs[2]);
    double c = 9 * (2 * start_state[3] - 5 * coeffs[0] + 4 * coeffs[1] - end_state[3]) / (2 * pow(coeffs[2], 2));
    double d = -9 * (start_state[3] - 3 * coeffs[0] + 3 * coeffs[1] - end_state[3]) / (2 * pow(coeffs[2], 3));
    auto simpson = simpson_method();
    auto f = [coeffs, start_state, end_state](double x)
    {
        return sin(fTheta(coeffs, x, start_state, end_state));
    };
    return start_state[1] + simpson(f, 0, s, 50);
}

// 定义误差函数向量
VectorXd error_function(const Matrix<double, 3, 1> &coeffs, const Matrix<double, 4, 1> &start_state, const Matrix<double, 4, 1> &end_state)
{
    VectorXd error(4);
    double s_f = coeffs[2];
    error[0] = fX(coeffs, s_f, start_state, end_state) - end_state[0];
    error[1] = fY(coeffs, s_f, start_state, end_state) - end_state[1];
    error[2] = fKappka(coeffs, s_f, start_state, end_state) - end_state[2];
    error[3] = fTheta(coeffs, s_f, start_state, end_state) - end_state[3];
    cout << "误差向量: " << error.transpose() << endl;
    return error;
}

// 计算雅可比矩阵
MatrixXd jacobian(const Matrix<double, 3, 1> &coeffs, const Matrix<double, 4, 1> &start_state, const Matrix<double, 4, 1> &end_state)
{
    const double eps = 1e-6;
    MatrixXd jac(4, 3);
    for (int i = 0; i < 3; ++i)
    {
        Matrix<double, 3, 1> coeffs_plus = coeffs;
        Matrix<double, 3, 1> coeffs_minus = coeffs;
        coeffs_plus[i] += eps;
        coeffs_minus[i] -= eps;
        VectorXd error_plus = error_function(coeffs_plus, start_state, end_state);
        VectorXd error_minus = error_function(coeffs_minus, start_state, end_state);
        jac.col(i) = (error_plus - error_minus) / (2 * eps);
    }
    return jac;
}

Matrix<double, 3, 1> cubicSpiral(Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    Matrix<double, 3, 1> coeffs;
    coeffs << 0.5, 0.5, 0.5;
    const int max_iter = 1000;
    const double tol = 0.01;
    for (int iter = 0; iter < max_iter; ++iter)
    {
        VectorXd error = error_function(coeffs, start_state, end_state);
        if (error.norm() < tol)
        {
            break;
        }
        MatrixXd J = jacobian(coeffs, start_state, end_state);
        VectorXd delta = J.fullPivLu().solve(-error);
        coeffs += delta;
    }
    return coeffs;
}

Vector2d evaluateCubicSpiral(Matrix<double, 3, 1> coeffs, double s, Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    double x = fX(coeffs, s, start_state, end_state);
    double y = fY(coeffs, s, start_state, end_state);
    return Vector2d(x, y);
}

int main()
{
    // 初始化起始状态和结束状态
    Matrix<double, 4, 1> start_state;
    start_state << 0, 0, 0, 0;

    Matrix<double, 4, 1> end_state;
    end_state << 20, 20, 0, 0;

    // 调用 cubicSpiral 函数求解参数
    Matrix<double, 3, 1> coeffs = cubicSpiral(start_state, end_state);

    // 输出求解得到的参数
    cout << "求解得到的参数 coeffs:" << endl;
    cout << coeffs << endl;

    // for (int s = 0; s < coeffs[2]; s++)
    // {
    //     /* code */
    // }
    

    return 0;
}