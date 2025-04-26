#include <iostream>
#include <Eigen/Dense>
#include "../../matplotlibcpp.h"

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

double fKappka(Matrix<double, 5, 1> paras, double s)
{
    double a = paras[0];
    double b = -(11 * paras[0] - 18 * paras[1] + 9 * paras[2] - 2 * paras[3]) / (2 * paras[4]);
    double c = 9 * (2 * paras[0] - 5 * paras[1] + 4 * paras[2] - paras[3]) / (2 * pow(paras[4], 2));
    double d = -9 * (paras[0] - 3 * paras[1] + 3 * paras[2] - paras[3]) / (2 * pow(paras[4], 3));
    return a + b * s + c * pow(s, 2) + d * pow(s, 3);
}

double fTheta(Matrix<double, 5, 1> paras, double s, Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    double a = paras[0];
    double b = -(11 * paras[0] - 18 * paras[1] + 9 * paras[2] - 2 * paras[3]) / (2 * paras[4]);
    double c = 9 * (2 * paras[0] - 5 * paras[1] + 4 * paras[2] - paras[3]) / (2 * pow(paras[4], 2));
    double d = -9 * (paras[0] - 3 * paras[1] + 3 * paras[2] - paras[3]) / (2 * pow(paras[4], 3));
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

double fX(Matrix<double, 5, 1> paras, double s, Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    double a = paras[0];
    double b = -(11 * paras[0] - 18 * paras[1] + 9 * paras[2] - 2 * paras[3]) / (2 * paras[4]);
    double c = 9 * (2 * paras[0] - 5 * paras[1] + 4 * paras[2] - paras[3]) / (2 * pow(paras[4], 2));
    double d = -9 * (paras[0] - 3 * paras[1] + 3 * paras[2] - paras[3]) / (2 * pow(paras[4], 3));
    auto simpson = simpson_method();
    auto f = [paras, start_state, end_state](double x)
    {
        return cos(fTheta(paras, x, start_state, end_state));
    };
    return start_state[0] + simpson(f, 0, s, 50);
}

double fY(Matrix<double, 5, 1> paras, double s, Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    double a = paras[0];
    double b = -(11 * paras[0] - 18 * paras[1] + 9 * paras[2] - 2 * paras[3]) / (2 * paras[4]);
    double c = 9 * (2 * paras[0] - 5 * paras[1] + 4 * paras[2] - paras[3]) / (2 * pow(paras[4], 2));
    double d = -9 * (paras[0] - 3 * paras[1] + 3 * paras[2] - paras[3]) / (2 * pow(paras[4], 3));
    auto simpson = simpson_method();
    auto f = [paras, start_state, end_state](double x)
    {
        return sin(fTheta(paras, x, start_state, end_state));
    };
    return start_state[1] + simpson(f, 0, s, 50);
}

// 定义误差函数向量
VectorXd error_function(const Matrix<double, 5, 1> &paras, const Matrix<double, 4, 1> &start_state, const Matrix<double, 4, 1> &end_state)
{
    VectorXd error(8);
    double s_f = paras[4];
    error[0] = fKappka(paras, 0) - start_state[3];
    error[1] = fTheta(paras, 0, start_state, end_state) - start_state[2];
    error[2] = fX(paras, 0, start_state, end_state) - start_state[0];
    error[3] = fY(paras, 0, start_state, end_state) - start_state[1];
    error[4] = fKappka(paras, s_f) - end_state[3];
    error[5] = fTheta(paras, s_f, start_state, end_state) - end_state[2];
    error[6] = fX(paras, s_f, start_state, end_state) - end_state[0];
    error[7] = fY(paras, s_f, start_state, end_state) - end_state[1];
    cout << "误差向量: " << error.transpose() << endl;
    return error;
}

// 计算雅可比矩阵
MatrixXd jacobian(const Matrix<double, 5, 1> &paras, const Matrix<double, 4, 1> &start_state, const Matrix<double, 4, 1> &end_state)
{
    const double eps = 1e-6;
    MatrixXd jac(8, 5);
    for (int i = 0; i < 5; ++i)
    {
        Matrix<double, 5, 1> paras_plus = paras;
        Matrix<double, 5, 1> paras_minus = paras;
        paras_plus[i] += eps;
        paras_minus[i] -= eps;
        VectorXd error_plus = error_function(paras_plus, start_state, end_state);
        VectorXd error_minus = error_function(paras_minus, start_state, end_state);
        jac.col(i) = (error_plus - error_minus) / (2 * eps);
    }
    return jac;
}

Matrix<double, 5, 1> cubicSpiral(Matrix<double, 4, 1> start_state, Matrix<double, 4, 1> end_state)
{
    Matrix<double, 5, 1> paras = Matrix<double, 5, 1>::Zero();
    const int max_iter = 100;
    const double tol = 1;
    for (int iter = 0; iter < max_iter; ++iter)
    {
        VectorXd error = error_function(paras, start_state, end_state);
        if (error.norm() < tol)
        {
            break;
        }
        MatrixXd J = jacobian(paras, start_state, end_state);
        VectorXd delta = J.fullPivLu().solve(-error);
        paras += delta;
    }
    return paras;
}

int main()
{
    // 初始化起始状态和结束状态
    Matrix<double, 4, 1> start_state;
    start_state << 0, 0, 0, 0;

    Matrix<double, 4, 1> end_state;
    end_state << 2, 2, 0, 0;

    // 调用 cubicSpiral 函数求解参数
    Matrix<double, 5, 1> paras = cubicSpiral(start_state, end_state);

    // 输出求解得到的参数
    cout << "求解得到的参数 paras:" << endl;
    cout << paras << endl;

    return 0;
}