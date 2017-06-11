//
// Created by Ruud Andriessen on 10/05/2017.
//

#include "SystemBuilder.h"
#include "solvers/Euler.h"
#include "solvers/Midpoint.h"
#include "solvers/RungeKutta.h"

System* SystemBuilder::get(AvailableSystems s) {
    switch (s) {
        System* sys;
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

    int dimensions = 20;
    int index = 0;
    for (int i = -dimensions; i < dimensions; i++) {
        for (int j = -dimensions; j < dimensions; j++) {
            sys->addParticle(new Particle(Vec3f(i * .1f, .0f, j * .1f), 1.0f, index));
            index++;
        }
    }

    return sys;
}