//
// Created by Ruud Andriessen on 10/05/2017.
//

#include "SystemBuilder.h"
#include "solvers/Euler.h"
#include "solvers/Midpoint.h"
#include "solvers/RungeKutta.h"
#include "forces/PressureForce.h"
#include "forces/ViscosityForce.h"
#include "forces/DirectionalForce.h"

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

    int dimensions = 3;
    float mass = 1.0f;
    int index = 0;

    // Movable particles
    for (int i = -dimensions; i < dimensions; i++) {
        for (int j = -dimensions; j < dimensions; j++) {
            sys->addParticle(new Particle(Vector3f(i * .1f, .0f, j * .1f), mass, index, true));
            index++;
        }
    }

    dimensions += 1;
    // A small static particle set
    for (int i = -dimensions; i < dimensions; i++) {
        for (int j = -dimensions; j < dimensions; j++) {
            sys->addParticle(new Particle(Vector3f(i * .1f, -2.f, j * .1f), mass, index, false));
            index++;
        }
    }

    sys->addForce(new DirectionalForce(sys->particles, Vector3f(0.0f, -0.0981f, 0.0f)));
    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));

    return sys;
}