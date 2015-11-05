
#include <vector>
#include "dynamicvoronoi.h"
#include "tca/pathgraph.hpp"
#include "tca/voronoi.hpp"

vector<Index> determine_nodes(DynamicVoronoi *dv) {
    vector<Index> nodes;
    int nn;
    for (int i = 0; i < dv->getSizeX(); i++) {
        for (int j = 0; j < dv->getSizeY(); j++) {
            nn = dv->getNumVoronoiNeighborsAlternative(i, j);
            if (nn >= 3) {
                nodes.push_back(Index(i, j));
            }
        }
    }
    return nodes;
}

PathGraph *generate_graph(DynamicVoronoi *dv) {
    for (int i = 0; i < dv->getSizeX(); i++) {
        for (int j = 0; j < dv->getSizeY(); j++) {

        }
    }
}
