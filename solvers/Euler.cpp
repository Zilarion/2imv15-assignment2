//
// Created by Ruud Andriessen on 03/05/2017.
//

#include "Euler.h"
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

using namespace Eigen;

Euler::Euler(Euler::TYPE type) : type(type) {}

void Euler::simulateStep(System *system, float h) {
    // Get the old state
    VectorXf oldState = system->getState();

    // Evaluate derivative
    VectorXf deriv = system->derivEval();

    // Compute the new state
    VectorXf newState = oldState + h * deriv;

    if (type == SEMI) {
        // If we are running semi implicit euler, use the new velocity instead
        VectorXf semiImpl(system->getDim());
        for (int i = 0; i < newState.size(); i += 6) {
            semiImpl[i + 0] = oldState[i + 0] + h * newState[i + 3];  // Xnew implicit, using Vnew
            semiImpl[i + 1] = oldState[i + 1] + h * newState[i + 4];  // Xold + h * Vnew
            semiImpl[i + 2] = oldState[i + 2] + h * newState[i + 5];
            semiImpl[i + 3] = newState[i + 3];  // Keep Vnew
            semiImpl[i + 4] = newState[i + 4];
            semiImpl[i + 5] = newState[i + 5];
        }

        semiImpl = system->checkCollisions(semiImpl);
        // Set the new state, using semi implicit computation
        system->setState(semiImpl, system->getTime() + h);
    } else {
        system->setState(newState, system->getTime() + h);
    }
}
