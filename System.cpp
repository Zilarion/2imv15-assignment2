//
// Created by Ruud Andriessen on 03/05/2017.
//

#include "System.h"
#include "solvers/Solver.h"
#include "solvers/ConstraintSolver.h"
#include "fields/PressureField.h"
#include "fields/ColorField.h"
#include "MarchingCubes.h"
#include "forces/DirectionalForce.h"
#include <iostream>
#include "parameters.h"


const int RigidBody::STATE_SIZE;

#if defined(_WIN32) || defined(WIN32)

#include <GL/glut.h>

#else
#include <GLUT/glut.h>
#endif

System::System(Solver *solver) : solver(solver), time(0.0f), dt(0.005),
                                 grid(40, 40, 40, 0.05f, Vector3f(1.f, 1.f, 1.f)) {
    densityField = new DensityField(this);
    pressureField = new PressureField(this);
    colorField = new ColorField(this);
    marchingCubes = new MarchingCubes(this);
}

System::~System() {
    delete densityField;
    delete pressureField;
    delete colorField;
}

/**
 * Adds a given particle to the system
 * @param p The particle to add
 */
void System::addParticle(Particle *p) {
    particles.push_back(p);
    for (Force *f : forces)
        f->addAsTarget(p);
}

/**
 * Adds a given particle to the system
 * @param p The particle to add
 */
void System::addRigidBody(RigidBody *r) {
    rigidBodies.push_back(r);
    for (Particle * p: r->particles)
        addParticle(p);
}

/**
 * Adds a force to use in the system when advancing a time step
 * @param f The new force to use in the system
 */
void System::addForce(Force *f) {
    forces.push_back(f);
}

/**
 * Adds a constraint to use in the system when advancing a time step
 * @param c The new constraint to use in the system
 */
void System::addConstraint(Constraint *c) {
    constraints.push_back(c);
}

/**
 * Frees all system data
 */
void System::free() {
    particles.clear();
    forces.clear();
}

/**
 * Resets all the system to it's initial state
 */
void System::reset() {
    for (Particle *p : particles) {
        p->reset();
    }
    for (RigidBody *r : rigidBodies) {
        r->reset();
    }
}

/**
 * Draws the forces
 */
void System::draw(bool drawVelocity, bool drawForce, bool drawConstraint, bool drawMarchingCubes) {
    if (!drawMarchingCubes)
        drawParticles(drawVelocity, drawForce, drawMarchingCubes);
    drawRigidBodies(drawVelocity, drawForce);

    if (drawForce) {
        drawForces();
    }
    if (drawConstraint) {
        drawConstraints();
    }

    // draw marching cubes
    if (drawMarchingCubes) {
        marchingCubes->drawMarching();
    }
}

/**
 * Runs the active solver on the system to progress it's state by dt time
 * @param dt the amount of time to advance the system
 */
void System::step(bool adaptive) {
    if (adaptive) {
        VectorXf before = this->getState();
        solver->simulateStep(this, dt);
        VectorXf xa = this->getState();
        this->setState(before);

        solver->simulateStep(this, dt / 2);
        solver->simulateStep(this, dt / 2);
        VectorXf xb = this->getState();

        float err = (xa - xb).norm();
        if (err > 0)
            dt *= pow(0.001f / err, .5f);

        this->setState(before);
    }

    solver->simulateStep(this, dt);

}


unsigned long System::getParticleDim() {
    return particles.size() * 3 * 2; // 3 dimensions, velocity and position
}

/**
 * Constructs a state given the current system
 * @return A copy of the current state of the system
 */
VectorXf System::getState() {
    VectorXf state(this->getParticleDim() + rigidBodies.size() * RigidBody::STATE_SIZE);

    for (int i = 0; i < this->particles.size(); i++) {
        Particle *p = particles[i];
        state[i * 6 + 0] = p->position[0];
        state[i * 6 + 1] = p->position[1];
        state[i * 6 + 2] = p->position[2];
        state[i * 6 + 3] = p->velocity[0];
        state[i * 6 + 4] = p->velocity[1];
        state[i * 6 + 5] = p->velocity[2];
    }

    for (int i = 0; i < rigidBodies.size(); i++) {
        RigidBody *r = rigidBodies[i];
        VectorXf rState = r->getState();
        for (int j = 0; j < rState.size(); j++) {
            state[getParticleDim() + RigidBody::STATE_SIZE * i + j] = rState[j];
        }
    }
    return state;
}

float System::getTime() {
    return time;
}

/**
 * Evaluates a derivative
 * @param dst The destination vector
 */
VectorXf System::derivEval() {
    clearForces();
    computeForces();
    ConstraintSolver::solve(this, 100.0f, 10.0f);
    return computeDerivative();
}

void System::setState(VectorXf src) {
    this->setState(src, this->getTime());
}

void System::setState(VectorXf src, float t) {
    for (int i = 0; i < particles.size(); i++) {
        if (particles[i]->movable) {
            particles[i]->position[0] = src[i * 6 + 0];
            particles[i]->position[1] = src[i * 6 + 1];
            particles[i]->position[2] = src[i * 6 + 2];
            particles[i]->velocity[0] = src[i * 6 + 3];
            particles[i]->velocity[1] = src[i * 6 + 4];
            particles[i]->velocity[2] = src[i * 6 + 5];
        }
    }
    for (int i = 0; i < rigidBodies.size(); i++) {
        RigidBody *r = rigidBodies[i];
        VectorXf rState(RigidBody::STATE_SIZE);
        for (int j = 0; j < rState.size(); j++) {
            rState[j] = src[getParticleDim() + RigidBody::STATE_SIZE * i + j];
        }
        r->setState(rState);
    }
    this->time = t;
}

/// Private ///

void System::computeForces() {
    grid.clear();
    grid.insert(particles);

    // Compute all densities
    for (Particle *p : particles) {
        if (p->rigid || p->cloth) continue;
        p->density = densityField->eval(p);
    }

    // Compute all pressures at each particle
    for (Particle *p : particles) {
        p->pressure = k * (p->density - restDensity);
    }

    // Apply all forces
    for (Force *f : forces) {
        f->apply(this);
    }

    float vfmean = 0, pfmean = 0, sfmean = 0;
    for (Particle *p : particles) {
        float sum = (p->vForce).norm() + (p->pForce).norm() + (p->sForce).norm();
        if (sum == 0) sum = 0.01f;
        float vf = (p->vForce).norm() / sum;
        float pf = (p->pForce).norm() / sum;
        float sf = (p->sForce).norm() / sum;

        vfmean += vf;
        pfmean += pf;
        sfmean += sf;
    }
    vfmean /= particles.size();
    pfmean /= particles.size();
    sfmean /= particles.size();

//    std::cout << std::endl << vfmean << std::endl << pfmean << std::endl << sfmean << std::endl << std::endl;
}

void System::clearForces() {
    for (Particle *p : particles) {
        p->force = Vector3f(0.0f, 0.0f, 0.0f);
    }
    for (RigidBody *r : rigidBodies) {
        for (Particle *p : r->particles) {
            p->force = Vector3f(0.0f, 0.0f, 0.0f);
        }
    }
}

VectorXf System::computeDerivative() {
    VectorXf dst(this->getParticleDim() + rigidBodies.size() * RigidBody::STATE_SIZE);
    for (int i = 0; i < particles.size(); i++) {
        Particle *p = particles[i];
        dst[i * 6 + 0] = p->velocity[0];            /* Velocity */
        dst[i * 6 + 1] = p->velocity[1];
        dst[i * 6 + 2] = p->velocity[2];
        if (p->cloth) {
            dst[i * 6 + 3] = p->force[0] / p->mass;  /* "old" acceleration is F/density */
            dst[i * 6 + 4] = p->force[1] / p->mass;
            dst[i * 6 + 5] = p->force[2] / p->mass;
        } else {
            dst[i * 6 + 3] = p->force[0] / p->density;  /* new acceleration is F/density */
            dst[i * 6 + 4] = p->force[1] / p->density;
            dst[i * 6 + 5] = p->force[2] / p->density;
        }
    }
    for (int i = 0; i < rigidBodies.size(); i++) {
        RigidBody *r = rigidBodies[i];
        VectorXf rDeriv = r->getDerivativeState();
        for (int j = 0; j < rDeriv.size(); j++) {
            dst[getParticleDim() + RigidBody::STATE_SIZE * i + j] = rDeriv[j];
        }
    }
    return dst;
}

void System::drawParticles(bool drawVelocity, bool drawForce, bool drawMarchingCubes) {
    for (Particle *p : particles) {
        //if (!p->movable || !drawMarchingCubes)
            p->draw(drawVelocity, drawForce);
    }
}

void System::drawRigidBodies(bool drawVelocity, bool drawForce) {
    for (RigidBody *r:rigidBodies) {
        r->draw(drawVelocity, drawForce);
    }
}

void System::drawForces() {
    for (Force *f : forces) {
        f->draw();
    }
}

void System::drawConstraints() {
    for (Constraint *c : constraints) {
        c->draw();
    }
}

//vector<Contact *> System::findContacts(VectorXf newState) {
//    vector<Contact *> contacts;
//    //sweep sort
//    //bool indicates start or end, start=true
//    map<float, pair<Object *, bool>> xMap;
//    map<float, pair<Object *, bool>> yMap;
//    map<float, pair<Object *, bool>> zMap;
//    for (RigidBody *r:rigidBodies) {
//        VectorXf boundingBox = r->getBoundingBox();
//        xMap[boundingBox[0]] = make_pair(r, true);
//        xMap[boundingBox[3]] = make_pair(r, false);
//        yMap[boundingBox[1]] = make_pair(r, true);
//        yMap[boundingBox[4]] = make_pair(r, false);
//        zMap[boundingBox[2]] = make_pair(r, true);
//        zMap[boundingBox[5]] = make_pair(r, false);
////        printf("minY: %f\n",boundingBox[1]);
//    }
//    for (Particle *p:particles) {
//        xMap[p->position[0]] = make_pair(p, true);
//        yMap[p->position[1]] = make_pair(p, true);
//        zMap[p->position[2]] = make_pair(p, true);
//    }
//    vector<RigidBody *> activeRigidBodies;
//    //keep track of particles that are in x/y/z range of a rigid body
//    //there is a collision if a  particle is present in all 3 ranges
//    vector<pair<RigidBody *, Particle *>> xRange;
//    vector<pair<RigidBody *, Particle *>> yRange;
//    vector<pair<RigidBody *, Particle *>> zRange;
//    for (pair<int, pair<Object *, bool>> xPair:xMap) {
//        xPair.second.first->handleSweep(xPair.second.second, &activeRigidBodies, &xRange);
//    }
//    for (pair<int, pair<Object *, bool>> yPair:yMap) {
//        yPair.second.first->handleSweep(yPair.second.second, &activeRigidBodies, &yRange);
//    }
//    for (pair<int, pair<Object *, bool>> zPair:zMap) {
//        zPair.second.first->handleSweep(zPair.second.second, &activeRigidBodies, &zRange);
//    }
//    for (pair<RigidBody *, Particle *> xPair:xRange) {
//        //check if there is a collision in all three directions x,y,z
//        if (find(yRange.begin(), yRange.end(), xPair) != yRange.end() &&
//            find(zRange.begin(), zRange.end(), xPair) != zRange.end()) {
//            contacts.push_back(new Contact(xPair.first, xPair.second, xPair.first->getNormal(xPair.second->position)));
//        }
//    }
//    return contacts;
//}
