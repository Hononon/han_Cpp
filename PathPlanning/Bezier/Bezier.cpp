#include "Bezier.h"

double factorial(int n)
{
    if (n <= 1)
    {
        return 1;
    }
    else
    {
        return n * factorial(n - 1);
    }
}

Vector2d bezierCommon(vector<Vector2d> Ps, double t)
{
    Vector2d result(0.0, 0.0);
    // n个控制点, 需要有n个多项式, 但是每个多项式的排列组合里只有n-1
    int n = Ps.size() - 1;
    for (int i = 0; i < Ps.size(); i++)
    {
        int C_n_i = factorial(n) / (factorial(i) * factorial(n - i));
        result += C_n_i * pow(1 - t, n - i) * pow(t, i) * Ps[i];
    }
    return result;
}