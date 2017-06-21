//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "DensityField.h"
#include "../Kernels.h"
#include "../System.h"

float DensityField::eval(Particle* pi) {
    float density = 0;
    for (Particle* pj : sys->grid.query(pi->position)) {
        density += pj->mass * Poly6::W(pi->position - pj->position, .1f);
    }

    return density;
}