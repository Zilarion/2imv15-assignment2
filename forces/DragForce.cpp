//
// Created by Ruud Andriessen on 06/05/2017.
//

#include "DragForce.h"
#include "../System.h"

DragForce::DragForce(const vector<Particle*> &particles, float amount) : amount(amount)
{
    this->setTarget(particles);
}

void DragForce::setTarget(const vector<Particle*> &particles)
{
    this->particles = particles;
}

void DragForce::apply(System* s)
{
    for (Particle* p : particles) {
        p->force -= amount * p->density * p->velocity;
    }
    for (RigidBody* r : s->rigidBodies) {
        r->force -= amount * r->v;
    }
}

void DragForce::draw()
{
}