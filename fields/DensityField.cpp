//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "DensityField.h"
#include "../Kernels.h"
#include "../System.h"

float DensityField::eval(Particle* pi) {
    float density = 0;
    for (Particle* pj : sys->particles) {
        if (pj->index != pi->index) {
            density += pj->mass * Poly6::W(pi->position - pj->position, 10.f);
        }
    }
    return density;
}