
#include <functional>
#include <sstream>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include "tca/graph.hpp"

using namespace std;

/*
 * Constructs an Index. This class is used to denote an index of a map
 * in integer coordinates and does not represent the physical positions
 * with respect to a generic origin
 */
Index::Index(int i, int j) : i(i), j(j)
{
}

/*
 * Allows us to print an index
 */
ostream& operator<<(ostream& os, const Index& idx)
{
    os << "Index(i=" << idx.i << ", j=" << idx.j << ")";
    return os;
}

/*
 * Checks if two indices are equal in value
 */
bool operator== (Index const& lhs, Index const& rhs)
{
    return (lhs.i == rhs.i) && (lhs.j == rhs.j);
}

/*
 * Constructs an edge which is just two indices. This is used for the data
 * structures in the graph
 */
Edge::Edge(Index a, Index b) : a(a), b(b)
{
}

/*
 * Checks if two edges are equal in value
 */
bool operator== (Edge const& lhs, Edge const& rhs)
{
    return (lhs.a == rhs.a) && (lhs.b == rhs.b);
}

/*
 * Hashes an index using a string hash.
 */
size_t IndexHash::operator() (const Index& index) const
{
    stringstream buffer;
    buffer << index.i << " " << index.j;
    return hash<string>()(buffer.str());
}

/*
 * Hashes an edge using a string hash.
 */
size_t EdgeHash::operator() (const Edge& edge) const {
    stringstream buffer;
    buffer << edge.a.i << " " << edge.a.j << " ";
    buffer << edge.b.i << " " << edge.b.j << " ";
    return hash<string>()(buffer.str());
}

/*
 * Constructs an EdgeData object. This is used as a extensible key
 * association object that is used to be a value in a map.
 */
EdgeData::EdgeData(vector<IndexPath> paths, vector<double> dists) :
    paths(paths), dists(dists)
{
}

/*
 * Adds a path and its cost to the EdgeData object
 */
void EdgeData::add_path(vector<Index> path, double dist)
{
    this->paths.push_back(path);
    this->dists.push_back(dist);
}

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
 * Converts an Index to JSON
 */
string json_index(Index ind)
{
    stringstream buffer;
    buffer << "{\"i\": " << ind.i << ",";
    buffer << "\"j\": " << ind.j << "}";
    return buffer.str();
}

/*
 * Converts an index and EdgeData to JSON
 */
string json_index_paths(Index ind, EdgeData *ed)
{
    stringstream buffer;
    buffer << "{\"i\": " << ind.i << ",";
    buffer << "\"j\": " << ind.j << ",";
    buffer << "\"paths\": [";
    for (int i = 0; i < ed->paths.size(); i++)
    {
        buffer << "[";
        for (int j = 0; j < ed->paths[i].size(); j++)
        {
            buffer << json_index(ed->paths[i][j]);
            if (j + 1 < ed->paths[i].size())
            {
                buffer << ",";
            }
        }
        buffer << "]";
        if (i + 1 < ed->paths.size())
        {
            buffer << ",";
        }
    }
    buffer << "]}";
    return buffer.str();
}

/*
 * Converts a graph object to JSON
 */
string Graph::json() {
    stringstream buffer;
    buffer << "[";
    int node_c = 0;
    for (auto np : nodes)
    {
        buffer << "{\"index\":" <<  json_index(np.first) << ",";
        buffer << "\"neighbours\": [";
        int nbr_c = 0;
        for (auto nbr : np.second)
        {
            buffer << json_index_paths(nbr, get_edge(np.first, nbr));
            if (++nbr_c < np.second.size())
            {
                buffer << ",";
            }
        }
        buffer << "]}";
        if (++node_c < nodes.size())
        {
            buffer << ",";
        }
    }
    buffer << "]";
    return buffer.str();
}

/*
 * Writes the graph to a file
 */
void Graph::write_to_file(string filename) {
    ofstream out_file;
    out_file.open(filename);
    out_file << this->json();
    out_file.close();
}
