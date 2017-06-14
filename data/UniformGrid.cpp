//
// Created by Ruud Andriessen on 14/06/2017.
//

#include "UniformGrid.h"
#include <math.h>

UniformGrid::UniformGrid(int x, int y, int z, float delta, Vector3f offset): xMax(x), yMax(y), zMax(z), delta(delta), offset(offset) {
    this->grid = vector<vector<vector<Cell>>>
            (x, vector<vector<Cell>>
                    (y, vector<Cell>
                            (z, Cell())
                    )
            );
}

void UniformGrid::insert(vector<Particle *> &particles) {
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

vector<Particle *> UniformGrid::query(const Vector3f pos) {
    int xC = floorf((pos[0] + offset[0]) / delta);
    int yC = floorf((pos[1] + offset[1]) / delta);
    int zC = floorf((pos[2] + offset[2]) / delta);

    vector<Particle*> result;

    if (xC < 0 || yC < 0 || zC < 0 || xC >= xMax || yC >= yMax || zC >= zMax)
        return result;

    unsigned long count = grid[xC][yC][zC].particles.size() +
                xC + 1 < xMax ? grid[xC + 1][yC][zC].particles.size() : 0 +
                xC - 1 >= 0 ? grid[xC - 1][yC][zC].particles.size() : 0 +
                yC + 1 < xMax ? grid[xC][yC + 1][zC].particles.size() : 0 +
                yC - 1 >= 0 ? grid[xC][yC - 1][zC].particles.size() : 0 +
                zC + 1 < zMax ? grid[xC][yC][zC + 1].particles.size() : 0 +
                zC - 1 >= 0 ? grid[xC][yC][zC - 1].particles.size() : 0;
    result.reserve(count);

    appendVect(result, grid[xC][yC][zC].particles);
    if (xC + 1 < xMax)
        appendVect(result, grid[xC + 1][yC][zC].particles);
    if (xC - 1 >= 0)
        appendVect(result, grid[xC - 1][yC][zC].particles);
    if (yC + 1 < yMax)
        appendVect(result, grid[xC][yC + 1][zC].particles);
    if (yC - 1 >= 0)
        appendVect(result, grid[xC][yC - 1][zC].particles);
    if (zC + 1 < zMax)
        appendVect(result, grid[xC][yC][zC + 1].particles);
    if (zC - 1 >= 0)
        appendVect(result, grid[xC][yC][zC - 1].particles);

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
    this->particles.push_back(p);
}

void Cell::clear() {
    this->particles.clear();
}