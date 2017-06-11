//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "DensityField.h"
#include "../Kernels.h"
#include "../System.h"

float DensityField::eval(Particle* pi) {
    float density = 0;
    for (Particle* p : sys->particles) {
        density += p->mass * Poly6::W((pi->position - p->position).norm());
    }
    return density;
}