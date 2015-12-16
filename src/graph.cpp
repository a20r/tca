
#include <functional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include "tca/graph.hpp"


Index::Index(int i, int j) : i(i), j(j)
{
}

Edge::Edge(Index a, Index b) : a(a), b(b)
{
}

size_t IndexHash::operator() (const Index& index) const
{
    stringstream buffer;
    buffer << index.i << " " << index.j;
    return hash<string>()(buffer.str());
}

size_t EdgeHash::operator() (const Edge& edge) const {
    stringstream buffer;
    buffer << edge.a.i << " " << edge.a.j << " ";
    buffer << edge.b.i << " " << edge.b.j << " ";
    return hash<string>()(buffer.str());
}

EdgeData::EdgeData(vector<IndexPath> paths, vector<double> dists) :
    paths(paths), dists(dists)
{
}

void EdgeData::add_path(vector<Index> path, double dist)
{
    this->paths.push_back(path);
    this->dists.push_back(dist);
}

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

void Graph::add_edge(Index a, Index b, EdgeData ed)
{
    if (nodes.count(a) == 0)
    {
        nodes[a] = unordered_set<Index, IndexHash>();
    }

    nodes[a].insert(b);
    edges[Edge(a, b)] = ed;
}

void Graph::add_edge(Edge edge, EdgeData ed)
{
    this->add_edge(edge.a, edge.b, ed);
}

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

EdgeData *Graph::get_edge(Index a, Index b)
{
    return this->get_edge(Edge(a, b));
}

