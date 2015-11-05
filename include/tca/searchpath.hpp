
#ifndef SEARCHPATH_H
#define SEARCHPATH_H

#include <vector>
#include <map>
#include <string>
#include "tca/types.hpp"

using namespace std;

class SearchPath {

    public:
        SearchPath() {};
        ~SearchPath() {};
        double get_weight();
        void set_weight(double weight);
        vector<Node> *get_nodes();
        vector<Node> get_nodes(int start, int end);
        SearchPath slice(int start, int end);
        Node get_node(int i);
        void set_nodes(vector<Node> nodes);
        vector<Path> *get_paths();
        vector<Path> get_paths(int start, int end);
        Path *get_path(int i);
        int get_num_paths();
        void set_paths(vector<Path> paths);
        double get_component_weight(Node n0, Node n1);
        void set_component_weight(Node n0, Node n1, double weight);
        SearchPath append(SearchPath& rhs, Path path, double weight);
        SearchPath append(SearchPath& rhs, Path path);
        SearchPath append(SearchPath& rhs);
        int size();
        string json();
        void write_to_file(string filename);

        static string json(vector<SearchPath> s_paths);
        static void write_to_file(vector<SearchPath> s_paths,
                string filename);

    private:
        vector<Node> nodes;
        vector<Path> paths;
        double total_weight;
        map<Node, map<Node, double> > component_weights;
};

#endif
