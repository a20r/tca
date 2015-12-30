
#include "tca/graph.hpp"

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
 * Allows us to print a vector of indices
 */
ostream& operator<<(ostream& os, const vector<Index>& idcs)
{
    os << "[";
    for (int i = 0; i < idcs.size(); i++)
    {
        os << idcs[i];
        if (i < idcs.size() - 1)
        {
            os << ", ";
        }
    }
    os << "]";
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
    this->wpaths.push(Weight<vector<Index> >(path, dist));
    this->paths.push_back(path);
    this->dists.push_back(dist);
}

