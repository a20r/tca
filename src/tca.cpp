
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include "tca/graph.hpp"
#include "tca/io.hpp"
#include "tca/voronoi.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tca");
    ros::NodeHandle n;

    bool doPruneAlternative = true;
    string pre = "/home/wallarelvo/Projects/catkin_ws/src/tca/";

    ifstream is(pre + "/resources/dynamicvoronoi/strongly_connected.pgm");
    if (!is) {
        std::cerr << "Could not open map file for reading.\n";
        exit(-1);
    }

    bool **map = NULL;
    int sizeX, sizeY;
    loadPGM( is, &sizeX, &sizeY, &map );
    is.close();

    DynamicVoronoi voronoi;
    voronoi.initializeMap(sizeX, sizeY, map);

    Graph G;
    Index start(525, 30), goal(525, 400);
    clock_t t1, t2;
    t1 = clock();
    vector<Index> path;
    generate_graph(start, goal, voronoi, G);
    double cost = G.shortest_path(start, goal, path);
    t2 = clock();
    float diff = (float) t2 - (float) t1;
    cout << "Time: " << diff / CLOCKS_PER_SEC << endl;
    cout << "Start: " << start << " | Goal: " << goal << endl;
    cout << "Path: " << path << endl;
    cout << "Path Cost: " << cost << endl;
    G.write_to_file(pre + "/sandbox/graph.json");
}
