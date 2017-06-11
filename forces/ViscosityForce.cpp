//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "ViscosityForce.h"
#include "../Kernels.h"

ViscosityForce::ViscosityForce(vector<Particle*> particles) {
    this->setTarget(particles);
}

void ViscosityForce::setTarget(std::vector<Particle*> particles) {
    this->particles = particles;
}

void ViscosityForce::apply(System *s) {
    float u = 1.0f;
    // Evaluate pressure force for every particle
    for (Particle* pi : particles) {
        Vector3f viscosityForce = Vector3f(0, 0, 0);
        for (Particle* pj : particles) {
            if (pi->index != pj->index) {
                for (int i = 0; i < 3; i++) {
                    viscosityForce[i] = pj->mass * (pi->velocity[i] - pj->velocity[i]) / pj->density
                                        * Viscosity::ddW(pi->position[i] - pj->position[i], 200.0f);
                }
            }
        }
        pi->force += u * viscosityForce;
    }
}

void ViscosityForce::draw() {
    // TODO
}
