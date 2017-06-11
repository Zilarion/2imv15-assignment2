//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "PressureField.h"
#include "../Kernels.h"
#include "../System.h"

Vector3f PressureField::eval(Particle* pi) {
    Vector3f force = Vector3f(0, 0, 0);
    for (Particle* pj : sys->particles) {
        if (pi->index != pj->index) {
            for (int i = 0; i < 3; i++) {
                force[i] -= pj->mass * (pi->pressure[i] + pj->pressure[i]) / (2 * pj->density) *
                            Spiky::dW(pi->position[i] - pj->position[i]);
            }
        }
    }
    return force;
}