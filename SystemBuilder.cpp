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
#include "forces/SurfaceForce.h"

System* SystemBuilder::get(AvailableSystems s) {
    System* sys;
    switch (s) {
        case WATER:
            sys = initBasic();
            sys->type = WATER;
            return sys;
        case SMOKE:
            sys = initSmoke();
            sys->type = SMOKE;
            return sys;
    }
    return NULL;
}

System* SystemBuilder::initBasic()
{
    System* sys = new System(new Euler(Euler::SEMI));
//    System* sys = new System(new RungeKutta());

    int dimensions = 8;
    float mass = 1.f;
    float massStatic = 100.f;
    int index = 0;
    float d = 0.03f;
    float ds = 0.05f;

    // Movable particles
    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
            sys->addParticle(new Particle(Vector3f(i * d + (rand() % 10 + 1) * 0.001f, -0.4f + (rand() % 10 + 1) * 0.001f, j * d + (rand() % 10 + 1) * 0.001f), mass, index++, true));
        }
    }

    // These forces only apply to non-static particles
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, -9.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 0.3f));

    // Static particles
    float delta = ds * dimensions* 2;
    dimensions *= 4;
    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
            sys->addParticle(new Particle(Vector3f(i * ds, -delta, j * ds), massStatic, index++, false));
        }
    }

//    for (int i = -dimensions / 2; i < 0; i++) {
//        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
//            sys->addParticle(new Particle(Vector3f(-delta, i * ds, j * ds), massStatic, index++, false));
//            sys->addParticle(new Particle(Vector3f(delta, i * ds, j * ds), massStatic, index++, false));
//        }
//    }
//
//
//    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
//        for (int j = -dimensions / 2; j < 0; j++) {
//            sys->addParticle(new Particle(Vector3f(i * ds, j * ds, delta), massStatic, index++, false));
//            sys->addParticle(new Particle(Vector3f(i * ds, j * ds, -delta), massStatic, index++, false));
//        }
//    }


    //Add a rigid body
//    RigidBody* r = new RigidBody(Vector3f(0,2,0), Vector3f(1,1,1), Vector3f(5,5,5), massStatic);
//    sys->addRigidBody(r);
//    sys->addForce(new DirectionalForce(r->particles, Vector3f(0.0f, -9.81f, 0.0f)));
//    sys->addForce(new DragForce(r->particles, 0.9f));

    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));
    sys->addForce(new SurfaceForce(sys->particles));

    return sys;
}


System* SystemBuilder::initSmoke()
{
    System* sys = new System(new Euler(Euler::SEMI));
//    System* sys = new System(new RungeKutta());

    int dimensions = 2;
    float mass = 1.f;
    int index = 0;
    float d = 0.03f;

    // Movable particles
    for (int i = -dimensions; i < dimensions; i++) {
        for (int j = -dimensions; j < dimensions; j++) {
            sys->addParticle(new Particle(Vector3f(i * d, -1.f, j * d), mass, index++, true));
        }
    }

    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, 3.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 0.2f));
    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));
    sys->addForce(new SurfaceForce(sys->particles));

    return sys;
}