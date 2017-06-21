//
// Created by s132054 on 18-6-2017.
//

#ifndef FLUIDS_MARCHINGCUBES_H
#define FLUIDS_MARCHINGCUBES_H

#include "Eigen/Dense"
#include <vector>

using namespace Eigen;
using namespace std;

typedef struct {
    Vector3f p[3];
} TRIANGLE;

typedef struct {
    Vector3f p[8];
    double val[8];
} GRIDCELL;

int Polygonise(GRIDCELL &grid,double isolevel,TRIANGLE *triangles);
bool operator<(const Vector3f &left, const Vector3f &right);
Vector3f VertexInterp(double isovalue, Vector3f &p1, Vector3f &p2, double val1, double val2);
void updateGradient(float grid[], int dim[3], int index, Vector3f gradients[]);
Vector3f getEdgeNormal(Vector3f &edgePoint, Vector3f &gridStart, Vector3f &gridEnd, int gridDim[3], float gridStep, Vector3f gradients[]);

#endif //FLUIDS_MARCHINGCUBES_H
