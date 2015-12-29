
#ifndef TCA_VORONOI_H
#define TCA_VORONOI_H
#include "dynamicvoronoi.h"
#include "tca/graph.hpp"

using namespace std;

void determine_nodes(DynamicVoronoi& dv, vector<Index>& nodes,
        unordered_set<Index, IndexHash>& node_set);
bool neighbourhood(Index ind, DynamicVoronoi& dv, bool nbrs[4]);
Index index_lookup(Index a, int i);
void find_enclosing_nodes(Index ind, DynamicVoronoi& dv,
        vector<Index>& nodes);
void find_enclosing_nodes(Index ind, DynamicVoronoi& dv,
        vector<Index>& nodes, unordered_set<Index, IndexHash>& seen);
void connect_start_and_goal(Index& start, Index& goal, DynamicVoronoi& dv,
        Graph& g);
void generate_connectivity_graph(DynamicVoronoi& dv, Graph& g);
void generate_graph(Index& start, Index& goal, DynamicVoronoi& dv, Graph& g);

#endif
