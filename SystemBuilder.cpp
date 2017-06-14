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
#include "forces/DragForce.h"

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
    System* sys = new System(new Euler(Euler::TYPE::SEMI));

    int dimensions = 2;
    float mass = 1.f;
    float staticMass = 10.f;
    int index = 0;
    float d = 0.2f;
    float ds = 0.1f;

    // Movable particles
    for (int i = -dimensions; i < dimensions; i++) {
        for (int j = -dimensions; j < dimensions; j++) {
            sys->addParticle(new Particle(Vector3f(i * d + 0.01f, .0f, j * d), mass, index++, true));
        }
    }

    dimensions += 1;
    // A small static particle set
    for (int i = -dimensions; i < dimensions; i++) {
        for (int j = -dimensions; j < dimensions; j++) {
            sys->addParticle(new Particle(Vector3f(i * ds, -2.f, j * ds), staticMass, index++, false));
            sys->addParticle(new Particle(Vector3f(-ds * dimensions, -2.f + i * ds, j * ds), staticMass, index++, false));
            sys->addParticle(new Particle(Vector3f(ds * dimensions, -2.f + i * ds, j * ds), staticMass, index++, false));
        }
    }

    sys->addForce(new DirectionalForce(sys->particles, Vector3f(0.0f, -.981f, 0.0f)));
    sys->addForce(new DragForce(sys->particles, 0.5f));
    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));

    return sys;
}