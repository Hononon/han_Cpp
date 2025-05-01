#include "dijkstra_self.h"

int main(int argc, char const *argv[])
{
    int n = 5;
    vector<vector<int>> graph(n, vector<int>(n));
    vector<int> g(n);
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
    graph[0][1] = 8;
    graph[0][2] = 5;
    graph[1][0] = 8;
    graph[1][2] = 3;
    graph[1][3] = 1;
    graph[2][0] = 5;
    graph[2][1] = 3;
    graph[2][3] = 6;
    graph[2][4] = 9;
    graph[3][1] = 1;
    graph[3][2] = 6;
    graph[3][4] = 2;
    graph[4][2] = 9;
    graph[4][3] = 2;

    int s = 0;
    int e = 4;

    dijkstra(s, e, graph, g, visited, previous);

    cout << "minumum distance bewteen " << s << " and " << e << " is " << g[e] << endl;
    print_path(s, e, previous);

    return 0;
}