
#include <queue>
#include <cmath>
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

double dist_estimate(Index a, Index b)
{
    return sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}

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
            return 0;
        }
        closed.insert(cur);
        for (auto nbr : nodes[cur])
        {
            if (closed.count(nbr) > 0)
            {
                continue;
            }
            float min_dist = 0;
            for (int i = 0; i < get_edge(cur, nbr)->dists.size(); i++)
            {
                if (i == 0)
                {
                    min_dist = get_edge(cur, nbr)->dists[i];
                }
                else if (get_edge(cur, nbr)->dists[i] < min_dist)
                {
                    min_dist = get_edge(cur, nbr)->dists[i];
                }
            }
            float tscore = gscore[cur] + min_dist;
            if (open.count(nbr) == 0 || tscore >= gscore[nbr])
            {
                came_from[nbr] = cur;
                gscore[nbr] = tscore;
                pq.push(Weight<Index>(nbr, gscore[nbr] + dist_estimate(nbr, goal)));
                open.insert(nbr);
            }
        }
    }
}
