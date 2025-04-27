#include "BSpline.h"

double baseFunction(int i, int k, double u, vector<double> node_vector)
{
    double B_i_k;
    // 1. 如果k=1, 计算基元
    if (k == 1)
    {
        if (u >= node_vector[i] && u < node_vector[i + 1])
        {
            B_i_k = 1;
        }
        else
        {
            B_i_k = 0;
        }
    }
    // 2. 如果k>1, 迭代计算
    else
    {
        double dens1 = node_vector[i + k - 1] - node_vector[i];
        double nums1 = u - node_vector[i];
        double dens2 = node_vector[i + k] - node_vector[i + 1];
        double nums2 = node_vector[i + k] - u;
        if (dens1 == 0)
        {
            dens1 = 1;
        }
        if (dens2 == 0)
        {
            dens2 = 1;
        }
        B_i_k = nums1 / dens1 * baseFunction(i, k - 1, u, node_vector) + nums2 / dens2 * baseFunction(i + 1, k - 1, u, node_vector);
    }
    return B_i_k;
}

vector<double> u_quasi_uniform(int n, int k)
{
    // 1. 节点矢量的共有 n+k+1个, n为控制点个数, k为阶数
    vector<double> node_vector(n + k + 1);
    // 2. 段数:n-k+2, 控制点数减u的次数 n-k+2, 有几条bezier曲线
    double piecewise = n - k + 2;
    // n=k-1, n+k+1 = 2k, 前k个为0,后k个为1
    if (piecewise == 1)
    {
        for (int i = k; i < n + k + 1; i++)
        {
            node_vector[i] = 1;
        }
    }
    else
    {
        // 包含 划分0的节点, 共有0,...,k-1个节点为0,
        // 包含 划分1的节点, 共有n+k,...,n+1个节点为1
        // 所以 包含划分0和1的两个节点, 中间共有 n+k+1-2k+2=n-k+3个节点, 索引为k-1,...,n+1,
        // 其中最后一个0的索引为k-1, 第一个1的索引为n+1
        // 将完整的[0,1]区间按照n-k+3个节点划分, 共需要划分n-k+2个区间, 即段数
        for (int i = k; i < n + 2; i++)
        {
            // node[k] = node[k-1]+1/p = 0+1/p
            node_vector[i] = node_vector[i - 1] + 1 / piecewise;
            // node[n+1] = node[n]+1/p,结果为1
        }
        for (int i = n + 2; i < n + k + 1; i++)
        {
            node_vector[i] = 1;
        }
    }
    return node_vector;
}