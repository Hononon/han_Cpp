#include "astar_self.h"

int main(int argc, char const *argv[])
{
    int n = 6;
    vector<vector<int>> graph(n, vector<int>(n));
    vector<int> g(n);
    vector<int> f(n);
    vector<int> h(n);
    vector<bool> visited(n);
    vector<int> previous(n);

    const int max = 999;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            graph[i][j] = (i == j ? 0 : max);
        }
    }
    graph[0][1] = 10;
    graph[0][2] = 12;
    graph[0][3] = 5;

    graph[1][0] = 10;
    graph[1][4] = 11;

    graph[2][0] = 12;
    graph[2][3] = 6;
    graph[2][4] = 11;
    graph[2][5] = 8;

    graph[3][0] = 5;
    graph[3][2] = 6;
    graph[3][5] = 14;

    graph[4][1] = 11;
    graph[4][2] = 11;

    graph[5][2] = 8;
    graph[5][3] = 14;

    h.assign({10, 15, 5, 5, 10, 5});

    int s = 0;
    int e = 5;

    astar(s, e, graph, g, f, h, visited, previous);

    cout << "minumum distance bewteen " << s << " and " << e << " is " << g[e] << endl;

    print_path(s, e, previous);

    return 0;
}