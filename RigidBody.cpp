

#include "RigidBody.h"

RigidBody::RigidBody(Vector3f startPos, Vector3f size, Vector3f numParticles, float particleMass) :
        startPos(startPos), size(size) {
    initializeVariables();

    //generate particles with body coordinates
    int index = 0;
    for (int x = 0; x < numParticles[0]; x++) {
        for (int y = 0; y < numParticles[1]; y++) {
            for (int z = 0; z < numParticles[2]; z++) {
                float xStart = -size[0] / 2 + size[0] * (float) x / float(numParticles[0]);
                float yStart = -size[1] / 2 + size[1] * (float) y / float(numParticles[1]);
                float zStart = -size[2] / 2 + size[2] * (float) z / float(numParticles[2]);
                particles.push_back(new Particle(Vector3f(xStart, yStart, zStart), particleMass, index, true));
                index++;
            }
        }
    }

    //Calculate total mass
    for (Particle *p : particles) {
        M += p->mass;
    }

    //Calculate Ibody
    for (Particle *p : particles) {
        Vector3f r0i = p->position;              //position in body coordinates
        RowVector3f r0iT = r0i.transpose();     //position in body coordinates
        Ibody += p->mass * (r0i * r0iT);
    }
    IbodyInv = Ibody.inverse();
}

RigidBody::~RigidBody(void) {

}

void RigidBody::reset() {
    initializeVariables();
    for (Particle *p : particles) {
        p->reset();
    }
}

void RigidBody::initializeVariables() {
    x = startPos;
    R = Matrix3f::Identity();
    P = Vector3f(0, 0, 0);
    L = Vector3f(0, 0, 0);
    Iinv = IbodyInv;
    v = Vector3f(0, 0, 0);
    omega = Iinv * L;
    force = Vector3f(0, 0, 0);
    torque = Vector3f(0, 0, 0);
}

void RigidBody::draw(bool drawVelocity, bool drawForce) {
    //TODO
}

void RigidBody::updateForce() {
    force = Vector3f(0, 0, 0);
    for (Particle *p : particles) {
        force += p->force;
    }
}

void RigidBody::updateTorque() {
    force = Vector3f(0, 0, 0);
    for (Particle *p : particles) {
        Vector3f ri = R * p->position + x;
        torque += (ri - x).cross(p->force);
    }

}
