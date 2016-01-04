
#include <queue>
#include <cmath>
#include <algorithm>
#include "tca/weight.hpp"
#include "tca/graph.hpp"

/*
 * Adds an edge to the graph and adds a path and cost to the the associated
 * EdgeData
 */
void Graph::add_edge(Index a, Index b, vector<Index> path, double dist)
{
    if (edges.count(Edge(a, b)) == 0)
    {
        EdgeData ed;
        ed.add_path(path, dist);
        add_edge(a, b, ed);
    }
    else
    {
        get_edge(a, b)->add_path(path, dist);
    }
}

/*
 * Adds an edge represented as two nodes with a given EdgeData to the graph
 */
void Graph::add_edge(Index a, Index b, EdgeData ed)
{
    if (nodes.count(a) == 0)
    {
        nodes[a] = unordered_set<Index, IndexHash>();
    }

    nodes[a].insert(b);
    edges[Edge(a, b)] = ed;
}

/*
 * Adds an edge with a given EdgeData to the graph
 */
void Graph::add_edge(Edge edge, EdgeData ed)
{
    this->add_edge(edge.a, edge.b, ed);
}

/*
 * Gets an edge from the graph
 */
EdgeData *Graph::get_edge(Edge edge)
{
    if (edges.count(edge) > 0)
    {
        return &edges[edge];
    }
    else
    {
        return NULL;
    }
}

/*
 * Gets an edge from the graph
 */
EdgeData *Graph::get_edge(Index a, Index b)
{
    return this->get_edge(Edge(a, b));
}

/*
 * Removes an edge from the graph
 */
void Graph::remove_edge(Index a, Index b)
{
    Edge edge(a, b);
    edges.erase(edge);
    if (nodes[a].count(b) > 0)
    {
        nodes[a].erase(b);
    }
}

/*
 * Euclidean distance estimate used as a heuristic for A*
 */
double dist_estimate(Index a, Index b)
{
    return sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}

/*
 * Shortest path between two nodes in the graph
 */
double Graph::shortest_path(Index start, Index goal, vector<Index>& path)
{
    priority_queue<Weight<Index>> pq;
    unordered_set<Index, IndexHash> closed, open;
    unordered_map<Index, Index, IndexHash> came_from;
    unordered_map<Index, double, IndexHash> gscore;
    gscore[start] = 0;
    pq.push(Weight<Index>(start, 0));
    open.insert(start);
    while (not pq.empty())
    {
        Weight<Index> wcur = pq.top();
        Index cur = wcur.get_val();
        open.erase(cur);
        pq.pop();
        if (cur == goal)
        {
            // bro beans, this bit is not entirely correct
            Index idx = goal;
            while (came_from.count(idx) > 0)
            {
                path.push_back(idx);
                idx = came_from[idx];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            return path.size();
        }
        closed.insert(cur);
        for (auto nbr : nodes[cur])
        {
            if (closed.count(nbr) > 0)
            {
                continue;
            }
            Weight<vector<Index> > wpath = get_edge(cur, nbr)->wpaths.top();
            float min_dist = wpath.get_weight();
            float tscore = gscore[cur] + min_dist;
            if (open.count(nbr) == 0 || tscore <= gscore[nbr])
            {
                came_from[nbr] = cur;
                gscore[nbr] = tscore;
                pq.push(Weight<Index>(nbr, gscore[nbr] +
                            dist_estimate(nbr, goal)));
                open.insert(nbr);
            }
        }
    }
}
