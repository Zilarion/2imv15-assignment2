//
// Created by Ruud Andriessen on 10/05/2017.
//

#include "SystemBuilder.h"
#include "solvers/Euler.h"
#include "solvers/Midpoint.h"
#include "solvers/RungeKutta.h"
#include "forces/PressureForce.h"

System* SystemBuilder::get(AvailableSystems s) {
    System* sys;
    switch (s) {
        case BASIC:
            sys = initBasic();
            sys->type = BASIC;
            return sys;
    }
    return NULL;
}

System* SystemBuilder::initBasic()
{
    System* sys = new System(new RungeKutta());

    int dimensions = 10;
    float mass = 1.0f;
    float density = 1.0f;
    int index = 0;
    for (int i = -dimensions; i < dimensions; i++) {
        for (int j = -dimensions; j < dimensions; j++) {
            sys->addParticle(new Particle(Vector3f(i * .1f, .0f, j * .1f), density, mass, index));
            index++;
        }
    }

    sys->addForce(new PressureForce(sys->particles));

    return sys;
}