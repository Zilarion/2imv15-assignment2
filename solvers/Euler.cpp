//
// Created by Ruud Andriessen on 03/05/2017.
//

#include "Euler.h"
#include "../forces/DirectionalForce.h"
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

using namespace Eigen;

Euler::Euler(Euler::TYPE type) : type(type) {}

void Euler::simulateStep(System *system, float h) {
    // Get the old state
    VectorXf oldState = system->getState();
    float epsilon = 0;
    for (RigidBody *r :system->rigidBodies) {
        for (Particle *p:system->particles) {
            Vector3f c = r->getBodyCoordinates(p->position);
//            printf("bodyx: %f, bodyy: %f, bodyz: %f\n", c[0], c[1], c[2]);
            if (r->isPenetrating(epsilon, p)) {
//                printf("fx: %f, fy: %f, fz: %f\n", p->force[0], p->force[1], p->force[2]);
                Vector3f n = r->getNormal(p->position);
                if((n.dot(p->force))>0) {
                    system->addForce(new DirectionalForce({p}, abs(n.dot(p->force)) * n));
//                    printf("nx: %f, ny: %f, nz: %f\n", n[0], n[1], n[2]);
                }
            }
        }
        r->recomputeAuxiliaryVars();
    }
    // Evaluate derivative
    VectorXf deriv = system->derivEval();

    // Compute the new state
    VectorXf newState = oldState + h * deriv;

    if (type == SEMI) {
        system->setState(newState, system->getTime() + h);
        VectorXf derivNew = system->derivEval();
        // If we are running semi implicit euler, use the new velocity instead
        VectorXf semiImpl(system->getParticleDim() + system->rigidBodies.size() * 13);
        for (int i = 0; i < system->particles.size(); i++) {
            semiImpl[i * 6 + 0] = oldState[i * 6 + 0] + h * derivNew[i * 6 + 0];  // Xnew implicit, using Vnew
            semiImpl[i * 6 + 1] = oldState[i * 6 + 1] + h * derivNew[i * 6 + 1];  // Xold + h * Vnew
            semiImpl[i * 6 + 2] = oldState[i * 6 + 2] + h * derivNew[i * 6 + 2];
            semiImpl[i * 6 + 3] = newState[i * 6 + 3];  // Keep Vnew
            semiImpl[i * 6 + 4] = newState[i * 6 + 4];
            semiImpl[i * 6 + 5] = newState[i * 6 + 5];
        }

        for (int i = 0; i < system->rigidBodies.size(); i++) {
            unsigned long startIndex = i * 13 + system->getParticleDim();
            semiImpl[startIndex + 0] =
                    oldState[startIndex + 0] + h * derivNew[startIndex + 0];  // Xnew implicit, using Vnew
            semiImpl[startIndex + 1] = oldState[startIndex + 1] + h * derivNew[startIndex + 1];  // Xold + h * Vnew
            semiImpl[startIndex + 2] = oldState[startIndex + 2] + h * derivNew[startIndex + 2];
            semiImpl[startIndex + 3] = oldState[startIndex + 3] + h * derivNew[startIndex + 3];
            semiImpl[startIndex + 4] = oldState[startIndex + 4] + h * derivNew[startIndex + 4];
            semiImpl[startIndex + 5] = oldState[startIndex + 5] + h * derivNew[startIndex + 5];
            semiImpl[startIndex + 6] = oldState[startIndex + 6] + h * derivNew[startIndex + 6];
            semiImpl[startIndex + 7] = newState[startIndex + 7];  // Keep Vnew
            semiImpl[startIndex + 8] = newState[startIndex + 8];
            semiImpl[startIndex + 9] = newState[startIndex + 9];
            semiImpl[startIndex + 10] = newState[startIndex + 10];  // Keep Vnew
            semiImpl[startIndex + 11] = newState[startIndex + 11];
            semiImpl[startIndex + 12] = newState[startIndex + 12];
        }

        // Set the new state, using semi implicit computation
        system->setState(semiImpl, system->getTime() + h);


//        vector<Contact *> contacts = system->findContacts(semiImpl);
//        float epsilon = 0.01f;
//        if (contacts.size() > 0) {
//            for (Contact *contact:contacts) {
//                if (contact->isPenetrating(epsilon)) {
//                    hasCollisions = true;
//                }
//            }
//            if (hasCollisions) {
//                simulateStep(system, h / 2);
//            }
//        } else if (hasCollisions) {
//            simulateStep(system, 1.5f * h);
//        }
    } else {
        system->setState(newState, system->getTime() + h);
    }
}
