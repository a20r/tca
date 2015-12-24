
#include <iostream>
#include <fstream>

using namespace std;

void loadPGM(std::istream &is, int *sizeX, int *sizeY, bool ***map)
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

