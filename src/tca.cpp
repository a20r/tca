
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <fstream>
#include "tca/graph.hpp"
#include "tca/voronoi.hpp"

using namespace std;

void loadPGM( std::istream &is, int *sizeX, int *sizeY, bool ***map )
{
    std::string tag;
    is >> tag;
    if (tag != "P5")
    {
        std::cerr << "Awaiting 'P5' in pgm header, found " << tag << std::endl;
        exit(-1);
    }

    while (is.peek() == ' ' || is.peek() == '\n')
    {
        is.ignore();
    }
    while (is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> *sizeX;
    while (is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> *sizeY;
    while (is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> tag;
    if (tag != "255") {
        cerr << "Awaiting '255' in pgm header, found " << tag << endl;
        exit(-1);
    }
    is.ignore(255, '\n');

    *map = new bool*[*sizeX];

    for (int x = 0; x < *sizeX; x++)
    {
        (*map)[x] = new bool[*sizeY];
    }
    for (int y = *sizeY - 1; y >= 0; y--)
    {
        for (int x = 0; x < *sizeX; x++)
        {
            int c = is.get();
            if ((double) c < 255 - 255 * 0.2)
            {
                (*map)[x][y] = true; // cell is occupied
            }
            else
            {
                (*map)[x][y] = false; // cell is free
            }
            if (!is.good())
            {
                std::cerr << "Error reading pgm map.\n";
                exit(-1);
            }
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tca");
    ros::NodeHandle n;

    bool doPruneAlternative = true;

    ifstream is("/home/wallarelvo/Projects/catkin_ws/src/tca/resources/dynamicvoronoi/strongly_connected.pgm");
    if (!is) {
        std::cerr << "Could not open map file for reading.\n";
        exit(-1);
    }

    bool **map = NULL;
    int sizeX, sizeY;
    loadPGM( is, &sizeX, &sizeY, &map );
    is.close();
    // fprintf(stderr, "Map loaded (%dx%d).\n", sizeX, sizeY);

    DynamicVoronoi voronoi;
    voronoi.initializeMap(sizeX, sizeY, map);
    voronoi.update();
    if (doPruneAlternative)
    {
        voronoi.updateAlternativePrunedDiagram();
    }

    voronoi.visualize("/home/wallarelvo/Projects/catkin_ws/src/tca/sandbox/initial.ppm");
    Graph G;
    generate_graph(voronoi, G);
    cout << G.json() << endl;
}
