//
// Created by s130604 on 18-6-2017.
//

#include "LeapFrog.h"

using namespace Eigen;

void LeapFrog::simulateStep(System *system, float h) {
    float oldTime = system->getTime();
    // Get the old state
    VectorXf oldState = system->getState();

    // Evaluate derivative
    VectorXf deriv = system->derivEval();

    VectorXf newState(system->getParticleDim() + system->rigidBodies.size() * RigidBody::STATE_SIZE);

    for (int i = 0; i < system->particles.size(); i++) {
        newState[i * 6 + 0] += h * (newState[i * 6 + 0] + 0.5f * h * deriv[i * 6 + 0]);
        newState[i * 6 + 1] += h * (newState[i * 6 + 1] + 0.5f * h * deriv[i * 6 + 1]);
        newState[i * 6 + 2] += h * (newState[i * 6 + 2] + 0.5f * h * deriv[i * 6 + 2]);
        newState[i * 6 + 3] = newState[i * 6 + 0] + 0.5f * h * deriv[i * 6 + 0] + h * deriv[i * 6 + 3];
        newState[i * 6 + 4] = newState[i * 6 + 1] + 0.5f * h * deriv[i * 6 + 1] + h * deriv[i * 6 + 4];
        newState[i * 6 + 5] = newState[i * 6 + 2] + 0.5f * h * deriv[i * 6 + 2] + h * deriv[i * 6 + 5];
    }
    for (int i = 0; i < system->rigidBodies.size(); i++) {
        unsigned long startIndex = i * RigidBody::STATE_SIZE + system->getParticleDim();
        newState[startIndex * 6 + 0] += h * (newState[startIndex * 6 + 0] + 0.5f * h * deriv[startIndex * 6 + 0]);
        newState[startIndex * 6 + 1] += h * (newState[startIndex * 6 + 1] + 0.5f * h * deriv[startIndex * 6 + 1]);
        newState[startIndex * 6 + 2] += h * (newState[startIndex * 6 + 2] + 0.5f * h * deriv[startIndex * 6 + 2]);
        newState[startIndex * 6 + 3] += h * (newState[startIndex * 6 + 3] + 0.5f * h * deriv[startIndex * 6 + 3]);
        newState[startIndex * 6 + 4] += h * (newState[startIndex * 6 + 4] + 0.5f * h * deriv[startIndex * 6 + 4]);
        newState[startIndex * 6 + 5] += h * (newState[startIndex * 6 + 5] + 0.5f * h * deriv[startIndex * 6 + 5]);
        newState[startIndex * 6 + 6] += h * (newState[startIndex * 6 + 6] + 0.5f * h * deriv[startIndex * 6 + 6]);
        newState[startIndex * 6 + 7 ] = newState[startIndex * 6 + 7 ] + 0.5f * h * deriv[startIndex * 6 + 7 ] + h * deriv[i * 6 + 7 ];
        newState[startIndex * 6 + 8 ] = newState[startIndex * 6 + 8 ] + 0.5f * h * deriv[startIndex * 6 + 8 ] + h * deriv[i * 6 + 8 ];
        newState[startIndex * 6 + 9 ] = newState[startIndex * 6 + 9 ] + 0.5f * h * deriv[startIndex * 6 + 9 ] + h * deriv[i * 6 + 9 ];
        newState[startIndex * 6 + 10] = newState[startIndex * 6 + 10] + 0.5f * h * deriv[startIndex * 6 + 10] + h * deriv[i * 6 + 10];
        newState[startIndex * 6 + 11] = newState[startIndex * 6 + 11] + 0.5f * h * deriv[startIndex * 6 + 11] + h * deriv[i * 6 + 11];
        newState[startIndex * 6 + 12] = newState[startIndex * 6 + 12] + 0.5f * h * deriv[startIndex * 6 + 12] + h * deriv[i * 6 + 12];
    }
    system->setState(newState, oldTime + h);

}
