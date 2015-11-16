
#include <cmath>
#include <iostream>
#include <queue>
#include <algorithm>
#include "tca/searchpath.hpp"
#include "tca/weight.hpp"
#include "tca/pathgraph.hpp"

set<Node> *PathGraph::get_nodes() {
    return &this->nodes;
}

set<Node> *PathGraph::get_neighbours(Node node) {
    if (this->has_node(node)) {
        return &this->neighbours[node];
    } else {
        cerr << "Node does not exist in graph (1)" << endl;
        cerr << "Node: ";
        cerr << get<0>(node) << ", " << get<1>(node) << endl;
        throw 1;
    }
}

Path *PathGraph::get_path(Node n0, Node n1, int i) {
    if (this->has_path(n0, n1) && !this->is_edge_muted(n0, n1)) {
        if (this->paths[n0][n1].size() > 0) {
            return &this->paths[n0][n1][i];
        } else {
            return NULL;
        }
    } else {
        cerr << "Supplied nodes are not neighbours (1)" << endl;
        throw 2;
    }
}

vector<Path> *PathGraph::get_paths(Node n0, Node n1) {
    if (this->has_path(n0, n1)) {
        return &this->paths[n0][n1];
    } else {
        cerr << "Supplied nodes are not neighbours (2)." << endl;
        throw 1;
    }
}

pair<Edge, int> *PathGraph::get_associated_edge(Point p) {
    if (this->has_point(p)) {
        return &this->edge_map[p];
    } else {
        cerr << "Point does not lay on an edge" << endl;
        throw 1;
    }
}

bool PathGraph::is_edge_muted(Node n0, Node n1) {
    return this->muted_edges.count(Edge(n0, n1)) > 0;
}

bool PathGraph::has_node(Node node) {
    return this->nodes.count(node) > 0;
}

bool PathGraph::has_edge(Node n0, Node n1) {
    bool exists = this->neighbours.count(n0) > 0
        and this->neighbours[n0].count(n1) > 0;
    bool muted = this->is_edge_muted(n0, n1);
    return exists && !muted;
}

bool PathGraph::has_path(Node n0, Node n1) {
    return this->paths.count(n0) > 0
        && this->paths[n0].count(n1) > 0;
}

bool PathGraph::has_point(Point p) {
    return this->edge_map.count(p) > 0;
}

void PathGraph::add_node(Node node) {
    if (!this->has_node(node)) {
        map<Node, vector<Path>> path_map;
        set<Node> neighs;
        this->nodes.insert(node);
        this->paths[node] = path_map;
        this->neighbours[node] = neighs;
    }
}

void PathGraph::add_edge(Node n0, Node n1) {
    if (!this->has_edge(n0, n1)) {
        vector<Path> empty_list;
        this->add_node(n0);
        this->add_node(n1);
        this->neighbours[n0].insert(n1);
        this->neighbours[n1].insert(n0);
        this->paths[n0][n1] = empty_list;
        this->paths[n1][n0] = empty_list;
    }
}

void PathGraph::add_path(Node n0, Node n1, Path path, double weight) {
    Edge curr_edge = Edge(n0, n1);
    Path::iterator iter;

    this->add_edge(n0, n1);
    this->paths[n0][n1].push_back(path);
    path.reverse();
    this->paths[n1][n0].push_back(path);
    this->weights[n0][n1][this->paths[n0][n1].size() - 1] = weight;
    this->weights[n1][n0][this->paths[n1][n0].size() - 1] = weight;

    for (iter = path.begin(); iter != path.end(); ++iter) {
        this->edge_map[*iter] = pair<Edge, int>(curr_edge,
                this->paths[n0][n1].size());
    }
}

void PathGraph::add_path(Node n0, Node n1, Path path) {
    this->add_path(n0, n1, path, path.size());
}

void PathGraph::mute_node(Node n) {
    this->nodes.erase(n);
    for (Node nbr : this->neighbours[n]) {
        this->neighbours[nbr].erase(n);
        this->muted_edges.insert(Edge(n, nbr));
        this->muted_edges.insert(Edge(nbr, n));
    }

    this->neighbours.erase(n);
}

void PathGraph::mute_edge(Node n0, Node n1) {
    if (this->neighbours.count(n0) > 0) {
        if (this->neighbours[n0].count(n1) > 0) {
            this->neighbours[n0].erase(n1);

            if (this->neighbours[n0].size() == 0) {
                this->neighbours.erase(n0);
                this->nodes.erase(n0);
            }
        }
    }

    if (this->neighbours.count(n1) > 0) {
        if (this->neighbours[n1].count(n0) > 0) {
            this->neighbours[n1].erase(n0);

            if (this->neighbours[n1].size() == 0) {
                this->neighbours.erase(n1);
                this->nodes.erase(n1);
            }
        }
    }

    this->muted_edges.insert(Edge(n0, n1));
    this->muted_edges.insert(Edge(n1, n0));
}

void PathGraph::restore_muted_edges() {
    Node n0, n1;
    for (Edge e : this->muted_edges) {
        n0 = get<0>(e);
        n1 = get<1>(e);
        this->nodes.insert(n0);
        this->nodes.insert(n1);

        if (this->neighbours.count(n0) == 0) {
            this->neighbours[n0] = set<Node>();
        }

        if (this->neighbours.count(n1) == 0) {
            this->neighbours[n1] = set<Node>();
        }

        this->neighbours[n0].insert(n1);
        this->neighbours[n1].insert(n0);
    }

    this->muted_edges = set<Edge>();
}

double PathGraph::get_weight(Node n0, Node n1, int i) {
    return this->weights[n0][n1][i];
}

map<Node, vector<Path>> *PathGraph::operator[](Node n) {
    if (this->has_node(n)) {
        return &this->paths[n];
    } else {
        cerr << "Supplied node is not in the graph (2)" << endl;
        throw 1;
    }
}


double PathGraph::node_dist(Point n0, Node n1) {
    double xp = pow(get<0>(n0) - get<0>(n1), 2);
    double yp = pow(get<1>(n0) - get<1>(n1), 2);
    return sqrt(xp + yp);
}

Node PathGraph::get_nearest(Point check) {
    double min_dist = -1, dist;
    Node min_node;
    for (Node node : this->nodes) {
        dist = this->node_dist(check, node);
        if (min_dist < 0 || dist < min_dist) {
            min_dist = dist;
            min_node = node;
        }
    }

    return min_node;
}

Node PathGraph::get_nearest(int x, int y) {
    return this->get_nearest(Node(x, y));
}

SearchPath PathGraph::shortest_tour(vector<Node> nodes) {
    SearchPath s_path, t_path;
    for (size_t i = 0; i < nodes.size() - 1; i++) {
        t_path = this->shortest_path(nodes[i], nodes[i + 1]);
        if (i == 0) {
            s_path = t_path;
        } else {
            s_path = s_path.append(t_path);
        }
    }
    return s_path;
}

// graph search always looks ugly, now shut up.
SearchPath PathGraph::shortest_path(Node source, Node sink) {
    priority_queue<Weight<Node> > open_set;
    Node c_node;
    set<Node> marked;
    set<Node> neighbours;
    map<Node, Node> parents;
    map<Edge, int> routes;
    map<Node, double> dist;
    vector<Path> c_paths;
    double wght, min_weight, alt;
    int i, min_route;

    if (get<0>(source) == get<0>(sink) and
            get<1>(source) == get<1>(sink)) {
        return SearchPath();
    }

    for (Node node : this->nodes) {
        dist[node] = INFINITY;
    }

    open_set.push(Weight<Node>(source, 0));
    dist[source] = 0;
    while (!open_set.empty()) {
        c_node = open_set.top().get_val();
        open_set.pop();
        marked.insert(c_node);
        // cerr << this->nodes.size() << endl;
        neighbours = *this->get_neighbours(c_node);
        for (Node neighbour : neighbours) {
            c_paths = this->paths[c_node][neighbour];
            if (marked.count(neighbour) == 0) {
                i = 0;
                min_route = 0;
                min_weight = 0;

                for (Path path : c_paths) {
                    wght = this->get_weight(c_node, neighbour, i);
                    if (i++ == 0 || wght < min_weight) {
                        min_weight = wght;
                        min_route = i - 1;
                    }
                }

                alt = dist[c_node] + min_weight;
                if (alt < dist[neighbour]) {
                    dist[neighbour] = alt;
                    parents[neighbour] = c_node;
                    routes[Edge(c_node, neighbour)] = min_route;
                    open_set.push(Weight<Node>(neighbour, -alt));
                }

                if (neighbour == sink) {
                    return this->backtrack_edge_chain(sink, parents,
                            routes);
                }
            }
        }
    }

    // cerr << "No path found" << endl;
    throw 1;
}

// using Yen's algorithm
vector<SearchPath> PathGraph::k_shortest_paths(Node source,
        Node sink, int K) {
    vector<SearchPath> k_paths; // A
    k_paths.push_back(this->shortest_path(source, sink));
    priority_queue<Weight<SearchPath> > open_set; // B
    Node spur_node;
    Path res_path;
    SearchPath spur_path, total_path, root_path;
    vector<Node> root_path_nodes, sp_nodes;
    vector<Node>::iterator root_iter, sp_iter;
    bool path_eq;

    for (int k = 1; k < K; k++) {
        for (int i = 0; i < k_paths.back().size() - 1; i++) {

            spur_node = k_paths.back().get_node(i);
            root_path = k_paths.back().slice(0, i);
            root_path_nodes = *root_path.get_nodes();

            for (SearchPath sp : k_paths) {
                sp_nodes = sp.get_nodes(0, i);
                root_iter = root_path_nodes.begin();
                sp_iter = sp_nodes.begin();
                path_eq = equal(root_iter, root_iter + i, sp_iter);
                if (path_eq) {
                    this->mute_edge(sp.get_node(i), sp.get_node(i + 1));
                }
            }

            try {
                spur_path = this->shortest_path(spur_node, sink);
                if (root_path.get_paths()->size() == 0) {
                    root_path = root_path.slice(0, root_path.size() - 2);
                    total_path = root_path.append(spur_path);
                } else {
                    res_path = root_path.get_paths()->back();
                    root_path = root_path.slice(0, root_path.size() - 2);
                    total_path = root_path.append(spur_path, res_path);
                }

                open_set.push(Weight<SearchPath>(total_path,
                    -total_path.get_weight()));

            } catch (int e) {}

            this->restore_muted_edges();
        }

        if (open_set.empty()) {
            break;
        } else {
            k_paths.push_back(open_set.top().get_val());
            open_set.pop();
        }
    }

    return k_paths;
}

SearchPath PathGraph::backtrack_edge_chain(Node goal,
        map<Node, Node> parents, map<Edge, int> routes) {

    vector<Node> nodes;
    vector<Path> paths;
    Node current = goal;
    Edge c_edge;
    Node parent;
    int route;
    double total_weight = 0, c_weight = 0;
    Path path;
    SearchPath sp;

    while (true) {
        parent = parents[current];
        c_edge = Edge(parent, current);
        route = routes[c_edge];
        c_weight = this->get_weight(parent, current, route);
        nodes.push_back(current);
        paths.push_back(*this->get_path(parent, current, route));
        sp.set_component_weight(parent, current, c_weight);
        total_weight += c_weight;
        if (parents.count(parent) > 0) {
            current = parent;
        } else {
            nodes.push_back(parent);
            reverse(nodes.begin(), nodes.end());
            reverse(paths.begin(), paths.end());
            sp.set_weight(total_weight);
            sp.set_nodes(nodes);
            sp.set_paths(paths);
            return sp;
        }
    }
}
