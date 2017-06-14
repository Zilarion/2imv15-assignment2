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
    // Evaluate pressure force for every particle
    for (Particle* p : particles) {
        p->force += s->pressureField->eval(p, s->grid);
    }
}
void PressureForce::draw() {
    // TODO draw a sample of the pressure force field at random points in the field or something
    // Maybe on particles instead, not sure
}