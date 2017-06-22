//
// Created by Ruud Andriessen on 14/06/2017.
//

#include "UniformGrid.h"
#include <math.h>

UniformGrid::UniformGrid(int x, int y, int z, float delta, Vector3f offset): xMax(x), yMax(y), zMax(z), delta(delta), offset(offset),
     grid(
        vector<vector<vector<Cell>>>
            (x, vector<vector<Cell>>
                    (y, vector<Cell>
                            (z, Cell())
                    )
            )
     ) {}

void UniformGrid::insert(const vector<Particle *> &particles) {
    for (Particle * p : particles) {
        this->insert(p);
    }
}

void UniformGrid::insert(Particle *p) {
    int xC = floorf((p->position[0] + offset[0]) / delta);
    int yC = floorf((p->position[1] + offset[1]) / delta);
    int zC = floorf((p->position[2] + offset[2]) / delta);

    if (xC >= 0 && yC >= 0 && zC >= 0 && xC < xMax && yC < yMax && zC < zMax)
        grid[xC][yC][zC].insert(p);
}

void appendVect(vector<Particle*> &target, vector<Particle*> &from) {
    target.insert(target.end(), make_move_iterator(from.begin()),
                  make_move_iterator(from.end()));
}

vector<Particle *> UniformGrid::query(const Vector3f &pos) {
    int xC = floorf((pos[0] + offset[0]) / delta);
    int yC = floorf((pos[1] + offset[1]) / delta);
    int zC = floorf((pos[2] + offset[2]) / delta);

    vector<Particle*> result;

    if (xC < 0 || yC < 0 || zC < 0 || xC >= xMax || yC >= yMax || zC >= zMax)
        return result;

    unsigned long count = 0;
    for (int x = -1; x < 1; x++) {
        if (xC + x < xMax && xC + x >= 0)
            for (int y = -1; y < 1; y++) {
                if (yC + y < yMax && yC + y >= 0)
                    for (int z = -1; z < 1; z++) {
                        if (zC + z < zMax && zC + z >= 0)
                            count += grid[xC + x][yC + y][zC + z].particles.size();
                    }
            }
    }
    result.reserve(count);
    for (int x = -1; x <= 1; x++) {
        if (xC + x < xMax && xC + x >= 0)
            for (int y = -1; y <= 1; y++) {
                if (yC + y < yMax && yC + y >= 0)
                    for (int z = -1; z <= 1; z++) {
                        if (zC + z < zMax && zC + z >= 0) {
                            appendVect(result, grid[xC + x][yC + y][zC + z].particles);
                        }
                    }
            }
    }
    return result;
}

void UniformGrid::clear() {
    for (int x = 0; x < xMax; x++) {
        for (int y = 0; y < yMax; y++) {
            for (int z = 0; z < zMax; z++) {
                grid[x][y][z].clear();
            }
        }
    }
}

void Cell::insert(Particle *p) {
    particles.push_back(p);
}

void Cell::clear() {
    particles.clear();
}