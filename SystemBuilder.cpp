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

#include <iostream>

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

    int dimensions = 20;
    float mass = 1.f;
    float massStatic = 50.f;
    int index = 0;
    float d = 0.03f;
    float ds = 0.04f;

    // Movable particles
    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
            float x = i * d + (rand() % 10 + 1) * 0.001f;
            float y = -0.7f + (rand() % 10 + 1) * 0.001f;
            float z = j * d + (rand() % 10 + 1) * 0.001f;
            sys->addParticle(new Particle(Vector3f(x, y, z), mass, index++, true));
        }
    }

    // These forces only apply to non-static particles
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, -9.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 0.5f));

    // Static particles

    float radius = .5f;

    int angleSteps = 120;
    float angleStep = M_PI * 2 / angleSteps;
    for (int i = 0; i < angleSteps; i++) {
        float angle = angleStep * i;
        for (int y = -20; y < 10; y++) {
            float x = cos(angle) * radius;
            float z = sin(angle) * radius;
            sys->addParticle(new Particle(Vector3f(x, y * ds, z), mass, index++, false));
        }
        for (float rd = 0; rd < radius; rd+=ds) {
            float x = cos(angle) * rd;
            float z = sin(angle) * rd;
            sys->addParticle(new Particle(Vector3f(x, -20 * ds, z), mass, index++, false));
        }
    }

//    float delta = ds * 20;
//    dimensions = 40;
//    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
//        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
//            sys->addParticle(new Particle(Vector3f(i * ds, -delta, j * ds), massStatic, index++, false));
//        }
//    }
//
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

    std::cout << "Running with: " << sys->particles.size() << " particles" << std::endl;
    return sys;
}


System* SystemBuilder::initSmoke()
{
    System* sys = new System(new Euler(Euler::SEMI));
//    System* sys = new System(new RungeKutta());

    int dimensions = 2;
    float mass = 1.f;
    int index = 0;
    float d = 0.05f;

    // Movable particles
//    for (int i = -dimensions; i < dimensions; i++) {
//        for (int j = -dimensions; j < dimensions; j++) {
//            sys->addParticle(new Particle(Vector3f(i * d, -1.f, j * d), mass, index++, true));
//        }
//    }

    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, 1.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 0.2f));
    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));
    sys->addForce(new SurfaceForce(sys->particles));

    return sys;
}