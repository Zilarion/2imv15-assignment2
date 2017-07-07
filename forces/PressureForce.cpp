//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "PressureForce.h"
#include "../System.h"

PressureForce::PressureForce(const vector<Particle*> &particles) {
    this->setTarget(particles);
}

void PressureForce::setTarget(const vector<Particle*> &particles) {
    this->particles = particles;
}

void PressureForce::apply(System *s) {
    // Evaluate pressure force for every particle
    for (Particle* p : particles) {
        Vector3f val = s->pressureField->eval(p);
        if(p->rigid){
            p->force -= .02f*val;
            p->pForce = -.02f*val;
        } else {
            p->force -= val;
            p->pForce = -val;
        }
    }
}
void PressureForce::draw() {
    // TODO draw a sample of the pressure force field at random points in the field or something
    // Maybe on particles instead, not sure
}