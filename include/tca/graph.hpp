
#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <functional>
#include <sstream>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>

using namespace std;

class Index
{
    public:
        int i, j;

        Index() {};
        ~Index() {};
        Index(int i, int j);
};

ostream& operator<<(ostream& os, const Index& idx);

bool operator== (Index const& lhs, Index const& rhs);

typedef vector<Index> IndexPath;

class IndexHash
{
    public:
        size_t operator() (const Index& index) const;
};

class Edge
{
    public:
        Index a, b;
        Edge() {};
        ~Edge() {};
        Edge(Index a, Index b);
};

bool operator== (Edge const& lhs, Edge const& rhs);

class EdgeHash
{
    public:
        size_t operator() (const Edge& edge) const;
};

class EdgeData
{
    public:
        vector<IndexPath> paths;
        vector<double> dists;

        EdgeData() {};
        ~EdgeData() {};
        EdgeData(vector<IndexPath> paths, vector<double> dists);
        void add_path(vector<Index> path, double dist);
};

class Graph
{
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
        void remove_edge(Index a, Index b);
        string json();
        void write_to_file(string filename);
};

#endif
