
#ifndef PATHGRAPH_H
#define PATHGRAPH_H

#include <set>
#include <utility>
#include <map>
#include <vector>
#include <list>
#include <string>
#include "searchpath.hpp"
#include "types.hpp"

namespace NrlEvg {

    using namespace std;

    class PathGraph {

        public:
            PathGraph() {};
            ~PathGraph() {};
            set<Node> *get_nodes();
            set<Node> *get_neighbours(Node node);
            Path *get_path(Node n0, Node n1, int i);
            vector<Path> *get_paths(Node n0, Node n1);
            double get_weight(Node n0, Node n1, int i);
            pair<Edge, int> *get_associated_edge(Point p);
            bool has_point(Point p);
            bool has_node(Node node);
            bool has_edge(Node n0, Node n1);
            bool has_path(Node n0, Node n1);
            bool is_edge_muted(Node n0, Node n1);
            void add_node(Node node);
            void add_edge(Node n0, Node n1);
            void add_path(Node n0, Node n1, Path ps);
            void add_path(Node n0, Node n1, Path ps, double weight);
            void mute_node(Node n);
            void mute_edge(Node n0, Node n1);
            void restore_muted_edges();
            map<Node, vector<Path> > *operator[](Node n);
            string json();
            void write_to_file(string filename);

            /* Fancy algorithms */
            Node get_nearest(Point check);
            Node get_nearest(int x, int y);
            SearchPath shortest_tour(vector<Node> nodes);
            SearchPath shortest_path(Node source, Node sink);
            vector<SearchPath> k_shortest_paths(Node source,
                    Node sink, int K);

        protected:
            double node_dist(Point n0, Node n1);
            SearchPath backtrack_edge_chain(Node goal,
                    map<Node, Node> parents, map<Edge, int> routes);

        private:
            set<Edge> muted_edges;
            set<Node> nodes;
            map<Node, map<Node, vector<Path> > > paths;
            map<Node, set<Node> > neighbours;
            map<Point, pair<Edge, int> > edge_map;
            map<Node, map<Node, map<int, double> > > weights;
    };
}

#endif
