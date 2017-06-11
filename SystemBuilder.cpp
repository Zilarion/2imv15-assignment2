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
    System* sys = new System(new Euler(Euler::SEMI));

    const float dist = 0.5f;
    const Vec3f center(0.0, 0.0, 0.0);
    const Vec3f offset(dist, 0.0, 0.0);

    sys->addParticle(new Particle(center + offset, 1.0f, 0));
    sys->addParticle(new Particle(center + offset * 2, 1.0f, 1));
    sys->addParticle(new Particle(center + offset * 3, 1.0f, 2));
    sys->addParticle(new Particle(center + offset * 3, 1.0f, 3));
    sys->addParticle(new Particle(center + offset * 4, 1.0f, 4));
    sys->addParticle(new Particle(center + offset * 4, 1.0f, 5));

    return sys;
}