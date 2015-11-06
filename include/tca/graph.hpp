
#ifndef GRAPH_H
#define GRAPH_H

#include <functional>
#include <sstream>
#include <unordered_map>

using namespace std;

class Index {

    public:
        unsigned int i, j;

        Index() {}
        ~Index() {}
        Index(int i, int j) : i(i), j(j) {}
};

class IndexHash {

    public:
        size_t operator() (const Index& index) const {
            stringstream buffer;
            buffer << index.i << " " << index.j;
            return hash<string>()(buffer.str());
        }
};

class Edge {

    Index a, b;

    public:
        Edge() {}
        ~Edge() {}
        Edge(Index a, Index b) : a(a), b(b) {}
};

class EdgeHash {

    public:
        size_t operator() (const Edge& edge) const {
            stringstream buffer;
            buffer << edge.a.i << " " << edge.a.j << " ";
            buffer << edge.b.i << " " << edge.b.j << " ";
            return hash<string>()(buffer.str());
        }
};

class EdgeData {

    public:
        double distance;

        EdgeData() {};
        ~EdgeData() {};
        EdgeData(double distance) : distance(distance) {}
};

class Graph {

    private:
        unordered_map<Index, vector<Index>, IndexHash> nodes;
        unordered_map<Edge, EdgeData, EdgeHash> edges;

};

#endif
