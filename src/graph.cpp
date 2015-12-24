
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
