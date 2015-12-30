#ifndef TCA_IO_HPP
#define TCA_IO_HPP

#include "dynamicvoronoi.h"

void loadPGM(std::istream &is, int *sizeX, int *sizeY, bool ***map);
void load_dynamic_voronoi(string filename, DynamicVoronoi& dv);

#endif
