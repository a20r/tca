
#ifndef GRAPH_H
#define GRAPH_H

#include <functional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

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

    public:
        Index a, b;
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

typedef vector<Index> IndexPath;

class EdgeData {

    public:
        double dist;
        vector<IndexPath> paths;

        EdgeData() {};
        ~EdgeData() {};
        EdgeData(vector<IndexPath> paths, double dist) : paths(paths),
            dist(dist) {}
};

class Graph {

    private:
        unordered_map<Index, unordered_set<Index, IndexHash>, IndexHash> nodes;
        unordered_map<Edge, EdgeData, EdgeHash> edges;

    public:
        void add_edge(Index i, Index j) {
            if (nodes.count(i) > 0) {
                nodes[i].insert(j);
                nodes[j].insert(i);
            } else {
                nodes[i] = unordered_set<Index, IndexHash>();
                nodes[j] = unordered_set<Index, IndexHash>();
            }
        }
};

#endif
