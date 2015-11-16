
#include <vector>
#include <unordered_set>
#include "dynamicvoronoi.h"
#include "tca/graph.hpp"
#include "tca/voronoi.hpp"

#define NUM_NBRS 4

vector<Index> determine_nodes(DynamicVoronoi *dv) {
    vector<Index> nodes;
    int nn;
    bool is_alt;
    for (int i = 0; i < dv->getSizeX(); i++) {
        for (int j = 0; j < dv->getSizeY(); j++) {
            nn = dv->getNumVoronoiNeighborsAlternative(i, j);
            is_alt = dv->isVoronoiAlternative(i, j);
            if (is_alt || nn >= 3) {
                nodes.push_back(Index(i, j));
            }
        }
    }
    return nodes;
}

inline bool neighbourhood(Index ind, DynamicVoronoi *dv, bool nbrs[4]) {
    int i = ind.i;
    int j = ind.j;

    for (int k = 0; k < NUM_NBRS; k++) {
        nbrs[k] = false;
    }

    if (dv->isVoronoiAlternative(i, j)) {
        if (j > 0 and dv->isVoronoiAlternative(i, j - 1)) {
            nbrs[0] = true;
        }

        if (i < dv->getSizeX() - 1 and dv->isVoronoiAlternative(i + 1, j)) {
            nbrs[1] = true;
        }

        if (j < dv->getSizeY() - 1 and dv->isVoronoiAlternative(i, j + 1)) {
            nbrs[2] = true;
        }

        if (i > 0 and dv->isVoronoiAlternative(i - 1, j)) {
            nbrs[3] = true;
        }
        return true;
    }

    return false;
}

inline Index index_lookup(Index a, int i) {
    switch (i) {
        case 0:
            return Index(a.i, a.j - 1);
        case 1:
            return Index(a.i + 1, a.j);
        case 2:
            return Index(a.i, a.j + 1);
        case 3:
            return Index(a.i - 1, a.j);
        default:
            return a;
    }
}

Graph *generate_graph(DynamicVoronoi *dv) {
    vector<Index> nodes = determine_nodes(dv);
    bool nbrs[4];

    #pragma omp parallel for
    for (int k = 0; k < nodes.size(); k++) {
        unordered_set<Index, IndexHash> seen;
        seen.insert(nodes[k]);
        Index cur = nodes[k];
        bool in_vor = neighbourhood(cur, dv, nbrs);
        for (int i = 0; i < NUM_NBRS; i++) {
            vector<Index> path;
            if (nbrs[i]) {
                Index nbr = index_lookup(cur, i);
                if (seen.count(nbr) == 0) {
                    path.push_back(nbr);
                }
            }
        }
    }
}
