//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "PressureForce.h"
#include "../System.h"

PressureForce::PressureForce(vector<Particle *, allocator<Particle *>> particles) {
    this->setTarget(particles);
}

void PressureForce::setTarget(std::vector<Particle*> particles) {
    this->particles = particles;
}

void PressureForce::apply(System *s) {
    // Compute all densities
    for (Particle *p : particles) {
        p->density = s->densityField->eval(p);
    }

    float k = 1.0f, restDensity = 1.0f;

    // Compute all pressures at each particle
    for (Particle* p : particles) {
        p->pressure = k * (p->density - restDensity);
    }

    // Evaluate pressure force for every particle
    for (Particle* p : particles) {
        p->force += s->pressureField->eval(p);
    }
}
void PressureForce::draw() {
    // TODO draw a sample of the pressure force field at random points in the field or something
    // Maybe on particles instead, not sure
}