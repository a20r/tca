
#ifndef GRAPH_H
#define GRAPH_H

#include <functional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

class Index {

    public:
        unsigned int i, j;

        Index() {};
        ~Index() {};
        Index(int i, int j);
};

typedef vector<Index> IndexPath;

inline bool operator== (Index const& lhs, Index const& rhs)
{
    return (lhs.i == rhs.i) && (lhs.j == rhs.j);
}

class IndexHash {

    public:
        size_t operator() (const Index& index) const;
};

class Edge {

    public:
        Index a, b;
        Edge() {};
        ~Edge() {};
        Edge(Index a, Index b);
};

inline bool operator== (Edge const& lhs, Edge const& rhs)
{
    return (lhs.a == rhs.a) && (lhs.b == rhs.b);
}

class EdgeHash {

    public:
        size_t operator() (const Edge& edge) const;
};

class EdgeData {

    public:
        vector<IndexPath> paths;
        vector<double> dists;

        EdgeData() {};
        ~EdgeData() {};
        EdgeData(vector<IndexPath> paths, vector<double> dists);
        void add_path(vector<Index> path, double dist);
};

class Graph {

    private:
        unordered_map<Index, unordered_set<Index, IndexHash>, IndexHash> nodes;
        unordered_map<Edge, EdgeData, EdgeHash> edges;

    public:

        Graph() {}
        ~Graph() {}

        void add_edge(Index a, Index b, vector<Index> path, double dist);
        void add_edge(Index a, Index b, EdgeData ed);
        void add_edge(Edge edge, EdgeData ed);
        EdgeData *get_edge(Edge edge);
        EdgeData *get_edge(Index a, Index b);
};

#endif
