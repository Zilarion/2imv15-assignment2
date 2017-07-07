//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "ViscosityForce.h"
#include "../Kernels.h"
#include "../System.h"

ViscosityForce::ViscosityForce(const vector<Particle *> &particles) {
    this->setTarget(particles);
}

void ViscosityForce::setTarget(const vector<Particle *> &particles) {
    this->particles = particles;
}

void ViscosityForce::apply(System *s) {
    float u = 1000.f;
    // Evaluate viscosity force for every particle
    for (Particle *pi : particles) {
        if (pi->rigid || pi->cloth) continue;
        Vector3f viscosityForce = Vector3f(0, 0, 0);

        vector<Particle*> targets = s->grid.query(pi->position);
        for (Particle *pj : targets) {
            viscosityForce = pj->mass * (pj->velocity - pi->velocity) / pj->density
                             * Viscosity::ddW(pi->position - pj->position);
        }
        pi->vForce = u * viscosityForce;
        pi->force += u * viscosityForce;
    }
}

void ViscosityForce::draw() {
    // TODO
}
