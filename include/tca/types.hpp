
#ifndef TYPES_H
#define TYPES_H

#include <utility>
#include <list>

namespace NrlEvg {

    using namespace std;

    typedef pair<int, int> Node;
    typedef pair<int, int> Point;
    typedef pair<Node, Node> Edge;
    typedef pair<Edge, int> PathEdge;
    typedef list<Point> Path;
    typedef list<PathEdge> EdgeChain;
}

#endif
