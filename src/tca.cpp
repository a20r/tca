
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include "tca/graph.hpp"
#include "tca/io.hpp"
#include "tca/voronoi.hpp"

using namespace std;

string pre = "/home/wallarelvo/Projects/catkin_ws/src/tca/";
string img = "/resources/dynamicvoronoi/strongly_connected.pgm";

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tca");
    ros::NodeHandle n;
    DynamicVoronoi dv;
    Graph G;
    clock_t t1, t2;
    vector<Index> path;
    Index start(525, 30), goal(525, 400);

    load_dynamic_voronoi(pre + img, dv);
    t1 = clock();
    generate_graph(start, goal, dv, G);
    double cost = G.shortest_path(start, goal, path);
    t2 = clock();
    float diff = (float) t2 - (float) t1;
    cout << "Time: " << diff / CLOCKS_PER_SEC << endl;
    cout << "Start: " << start << " | Goal: " << goal << endl;
    cout << "Path: " << path << endl;
    cout << "Path Cost: " << cost << endl;
    G.write_to_file(pre + "/sandbox/graph.json");
}
