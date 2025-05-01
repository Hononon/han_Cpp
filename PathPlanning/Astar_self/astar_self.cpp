#include "astar_self.h"
#include <algorithm>

void astar(int s, int e, vector<vector<int>> &graph, vector<int> &g, vector<int> &f,
           vector<int> &h, vector<bool> &visited, vector<int> &previous)
{
    // 1. 初始化openlist,起点到节点的代价g,visit,previous
    int n = graph.size();
    // g[i]表示 起点 到 i 的代价, 初始化时除起点为0外, 其余表示未探索区域, 所以要设置无穷大
    fill(g.begin(), g.end(), 99);
    fill(f.begin(), f.end(), 99);
    fill(visited.begin(), visited.end(), false);
    fill(previous.begin(), previous.end(), -1);

    g[s] = 0;
    f[s] = g[s] + h[s];

    map<int, int> open_list;
    open_list[f[s]] = s;

    while (!open_list.empty())
    {
        // openlist: cost,nodeid, 第一个为cost最小的节点
        auto it = open_list.begin();
        int current_cost = it->first;
        int current = it->second;
        open_list.erase(it);

        if (visited[current])
        {
            continue;
        }
        visited[current] = true;

        if (current == e)
        {
            break;
        }

        for (int next = 0; next < n; next++)
        {
            // 遍历 未参观 的 邻居节点(!=999)
            if (!visited[next] && graph[current][next] != 999)
            {
                // 仍然拿实际代价比较
                int next_cost = g[current] + graph[current][next];
                if (next_cost < g[next])
                {
                    g[next] = next_cost;
                    // 更新时多一个启发式代价的更新, 将带有启发式的代价加入openlist
                    f[next] = next_cost + h[next];
                    open_list[f[next]] = next;

                    previous[next] = current;
                }
            }
        }
    }
}

void print_path(int s, int e, const vector<int> &previous)
{
    if (e == -1)
    {
        cout << "No path found" << endl;
        return;
    }

    vector<int> path;
    for (int at = e; at != -1; at = previous[at])
    {
        path.push_back(at);
    }
    reverse(path.begin(), path.end());

    cout << "Shortest path: ";
    for (int node : path)
    {
        cout << node << " ";
    }
    cout << endl;
}
