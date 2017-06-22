//
// Created by Ruud Andriessen on 14/06/2017.
//

#ifndef FLUIDS_UNIFORMGRID_H
#define FLUIDS_UNIFORMGRID_H


#include "../Particle.h"
#include <vector>

using namespace std;

class Cell {
public:
    Cell() : particles(vector<Particle*>()) {};
    void insert(Particle* p);
    void clear();

    vector<Particle*> particles;
};

class UniformGrid {
public:
    UniformGrid(int x, int y, int z, float delta, Vector3f offset = Vector3f(0,0,0));

    void insert(const vector<Particle*> &particles);
    vector<Particle*> query(const Vector3f &pos);
    void clear();

private:
    void insert(Particle *p);

    float delta;
    Vector3f offset;
    int xMax, yMax, zMax;
    vector< vector< vector< Cell > > > grid;     // 3d Grid with vectors
};


#endif //FLUIDS_UNIFORMGRID_H
