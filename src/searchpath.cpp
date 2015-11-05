
#include <iostream>
#include <sstream>
#include <fstream>
#include "tca/searchpath.hpp"

namespace NrlEvg {

    double SearchPath::get_weight() {
        return this->total_weight;
    }

    void SearchPath::set_weight(double weight) {
        this->total_weight = weight;
    }

    vector<Node> *SearchPath::get_nodes() {
        return &this->nodes;
    }

    vector<Node> SearchPath::get_nodes(int start, int end) {
        vector<Node> ret_nodes;
        for (int i = start; i <= end; i++) {
            ret_nodes.push_back(this->nodes[i]);
        }

        return ret_nodes;
    }

    SearchPath SearchPath::slice(int start, int end) {
        SearchPath sp;
        vector<Node> ns = this->get_nodes(start, end);
        vector<Path> ps = this->get_paths(start, end);

        double t_weight = 0, weight;
        for (size_t i = 1; i < ns.size(); i++) {
            weight = this->get_component_weight(ns[i - 1], ns[i]);
            sp.set_component_weight(ns[i - 1], ns[i], weight);
            t_weight += weight;
        }

        sp.set_weight(t_weight);
        sp.set_nodes(ns);
        sp.set_paths(ps);

        return sp;
    }

    Node SearchPath::get_node(int i) {
        return this->nodes[i];
    }

    void SearchPath::set_nodes(vector<Node> nodes) {
        this->nodes = nodes;
    }

    vector<Path> *SearchPath::get_paths() {
        return &this->paths;
    }

    Path *SearchPath::get_path(int i) {
        return &this->paths[i];
    }

    int SearchPath::get_num_paths() {
        return this->paths.size();
    }

    vector<Path> SearchPath::get_paths(int start, int end) {
        vector<Path> ps;
        for (int i = 0; i < end; i++) {
            ps.push_back(this->paths[i]);
        }
        return ps;
    }

    void SearchPath::set_paths(vector<Path> paths) {
        this->paths = paths;
    }

    double SearchPath::get_component_weight(Node n0, Node n1) {
        return this->component_weights[n0][n1];
    }

    void SearchPath::set_component_weight(Node n0, Node n1, double weight) {
        this->component_weights[n0][n1] = weight;
    }

    SearchPath SearchPath::append(SearchPath& rhs, Path path, double wht) {
        // 'path' is the list of points that link the last element
        // of 'this' and the first element of 'rhs'

        SearchPath sp;
        vector<Node> rhs_nodes = *rhs.get_nodes();
        vector<Path> rhs_paths = *rhs.get_paths();
        vector<Node> ns = this->nodes;
        vector<Path> ps = this->paths;
        ns.insert(ns.end(), rhs_nodes.begin(), rhs_nodes.end());
        ps.push_back(path);
        ps.insert(ps.end(), rhs_paths.begin(), rhs_paths.end());

        double t_weight = 0, weight;
        for (int i = 1; i < this->size(); i++) {
            weight = this->get_component_weight(this->get_node(i - 1),
                    this->get_node(i));
            sp.set_component_weight(this->get_node(i - 1), this->get_node(i),
                    weight);
            t_weight += weight;
        }

        for (int i = 1; i < rhs.size(); i++) {
            weight = rhs.get_component_weight(rhs.get_node(i - 1),
                    rhs.get_node(i));
            sp.set_component_weight(rhs.get_node(i - 1), rhs.get_node(i),
                    weight);
            t_weight += weight;
        }

        if (this->size() > 0 && rhs.size() > 0) {
            sp.set_component_weight(
                this->get_nodes()->back(), rhs.get_nodes()->front(), wht);
        }

        sp.set_weight(this->get_weight() + rhs.get_weight() + wht);
        sp.set_nodes(ns);
        sp.set_paths(ps);

        return sp;

    }

    SearchPath SearchPath::append(SearchPath& rhs, Path path) {
        return this->append(rhs, path, path.size());
    }

    SearchPath SearchPath::append(SearchPath& rhs) {
        return this->append(rhs, Path(), 0);
    }

    int SearchPath::size() {
        return this->nodes.size();
    }

    string SearchPath::json() {
        stringstream buffer;

        buffer << "[";
        int i = 0, j;
        for (Path path : this->paths) {
            j = 0;
            i++;
            for (Point point : path) {
                if (!(i == 1 and j++ == 0)) {
                    buffer << ",";
                }
                buffer << "{\"x\": " << get<0>(point) << ", \"y\":";
                buffer << get<1>(point) << "}";
            }
        }

        buffer << "]";

        return buffer.str();
    }

    string SearchPath::json(vector<SearchPath> s_paths) {
        stringstream buffer;

        buffer << "[";
        size_t i = 0;
        for (SearchPath s_path : s_paths) {
            if (s_path.size() > 0) {
                buffer << s_path.json();
                if (++i == s_paths.size()) {
                    buffer << "]";
                } else {
                    buffer << ",";
                }
            }
        }
        return buffer.str();
    }

    void SearchPath::write_to_file(string filename) {
        ofstream out_file;
        out_file.open(filename);
        out_file << this->json();
        out_file.close();
    }

    void SearchPath::write_to_file(vector<SearchPath> s_paths,
            string filename) {
        ofstream out_file;
        out_file.open(filename);
        out_file << SearchPath::json(s_paths);
        out_file.close();
    }
}
