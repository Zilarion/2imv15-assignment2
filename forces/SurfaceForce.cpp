
#include "SurfaceForce.h"
#include "../System.h"
#include "../Kernels.h"

SurfaceForce::SurfaceForce(const vector<Particle*> &particles) {
    this->setTarget(particles);
}

void SurfaceForce::setTarget(const vector<Particle *> &particles) {
    this->particles = particles;
}

void SurfaceForce::apply(System *s) {
    //Tension coefficient water-air
    float sigma = 12.75f;
    float certainThreshold = .001f;

    // Evaluate surface force for every particle
    for (Particle *pi : particles) {
        if (pi->rigid || pi->cloth) continue;
        Vector3f n = s->colorField->dEval(pi->position);
        if (n.norm() > certainThreshold) {
            pi->force += -sigma * s->colorField->ddEval(pi->position) * n / n.norm();
            pi->sForce = -sigma * s->colorField->ddEval(pi->position) * n / n.norm();
        }
    }
}

void SurfaceForce::draw() {
    // TODO draw a sample of the pressure force field at random points in the field or something
    // Maybe on particles instead, not sure
}