//
// Created by Ruud Andriessen on 10/05/2017.
//

#include <iostream>
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

System *SystemBuilder::get(AvailableSystems s) {
    System *sys;
    switch (s) {
        case BASIC:
            sys = initBasic();
            sys->type = BASIC;
            return sys;
        case WATER:
            sys = initTrechter();
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

    int dimensions = 10;
    float mass = 1.f;
    float massStatic = 50.f;
    int index = 0;
    float d = 0.03f;
    float ds = 0.04f;

    // Movable particles
    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
            float x = i * d + (rand() % 10 + 1) * 0.001f;
            float y = -0.4f + (rand() % 10 + 1) * 0.001f;
            float z = j * d + (rand() % 10 + 1) * 0.001f;
            sys->addParticle(new Particle(Vector3f(x, y, z), mass, index++, true));
        }
    }

    // These forces only apply to non-static particles
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, -9.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 0.5f));

    // Static particles
    float delta = ds * 20 / 2;
    dimensions = 20;
    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
            sys->addParticle(new Particle(Vector3f(i * ds, -delta - .5f, j * ds), massStatic, index++, false));
        }
    }

    for (int i = -dimensions / 2; i < 0; i++) {
        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
            sys->addParticle(new Particle(Vector3f(-delta, i * ds - .5f, j * ds), massStatic, index++, false));
            sys->addParticle(new Particle(Vector3f(delta, i * ds - .5f, j * ds), massStatic, index++, false));
        }
    }


    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
        for (int j = -dimensions / 2; j < 0; j++) {
            sys->addParticle(new Particle(Vector3f(i * ds, j * ds - .5f, delta), massStatic, index++, false));
            sys->addParticle(new Particle(Vector3f(i * ds, j * ds - .5f, -delta), massStatic, index++, false));
        }
    }


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
System *SystemBuilder::initTrechter() {
    System *sys = new System(new Euler(Euler::SEMI));
//    System* sys = new System(new RungeKutta());

    int dimensions = 7;
    float mass = 1.f;
    float massStatic = 50.f;
    int index = 0;
    float d = 0.04f;
    float ds = 0.04f;

    // Movable particles
    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
            for(int k = 0; k<10;k++) {
                float x = i * d + (rand() % 10 + 1) * 0.001f;
                float y = k*d-.6f + (rand() % 10 + 1) * 0.001f;
                float z = j * d + (rand() % 10 + 1) * 0.001f;
                sys->addParticle(new Particle(Vector3f(x, y, z), mass, index++, true));
            }
        }
    }

    // These forces only apply to non-static particles
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, -9.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 0.5f));

    // Static particles
    float topRadius = .3f;
    float bottomRadius = .1f;
    float radiusStep = (topRadius - bottomRadius)/30;
    int angleSteps = 120;
    float angleStep = M_PI * 2 / angleSteps;
    for (int i = 0; i < angleSteps; i++) {
        float angle = angleStep * i;
        float radius = topRadius;
        for (int y = -20; y < 0; y++) {
            float x = cos(angle) * radius;
            float z = sin(angle) * radius;
            sys->addParticle(new Particle(Vector3f(x, y * ds, z), mass, index++, false));
//            radius+=radiusStep;
        }
    }
    //bottom

    for (int i = -20; i < 20; i++) {
        for (int j = -20; j < 20; j++) {
            sys->addParticle(new Particle(Vector3f(i * .015f, -20*ds, j * .015f), massStatic, index++, false));
        }
    }

    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));
    sys->addForce(new SurfaceForce(sys->particles));

    std::cout << "Running with: " << sys->particles.size() << " particles" << std::endl;
    return sys;
}

System *SystemBuilder::initSmoke() {
    System *sys = new System(new Euler(Euler::SEMI));
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