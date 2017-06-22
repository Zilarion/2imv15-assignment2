//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "DensityField.h"
#include "../Kernels.h"
#include "../System.h"

float DensityField::eval(Particle* pi) {
    float density = 0;

    vector<Particle*> targets = sys->grid.query(pi->position);
    for (Particle* pj : targets) {
        density += pj->mass * Poly6::W(pi->position - pj->position);
    }
    return density;
}