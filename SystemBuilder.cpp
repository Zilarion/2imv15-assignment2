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

    int dimensions = 20;
    float mass = 1.f;
    float staticMass = 1000.f;
    int index = 0;
    float d = 0.03f;
    float ds = 0.1f;

    // Movable particles
    for (int i = -dimensions; i < dimensions; i++) {
        for (int j = -dimensions; j < dimensions; j++) {
            sys->addParticle(new Particle(Vector3f(i * d + (rand() % 10 + 1) * 0.001f, 0.f, j * d + (rand() % 10 + 1) * 0.001f), mass, index++, true));
        }
    }

    /*/Add a rigid body
    RigidBody* r = new RigidBody(Vector3f(0,0,0), Vector3f(1,1,1), Vector3f(5,5,5), mass);
    sys->addRigidBody(r);
    sys->addForce(new DirectionalForce(r->particles, Vector3f(0.0f, -9.81f, 0.0f)));
//    sys->addForce(new DragForce(r->particles, 0.9f));
    //*/

    dimensions += 1;
    // A small static particle set
//    for (int i = -dimensions; i < dimensions; i++) {
//        for (int j = -dimensions; j < dimensions; j++) {
//            sys->addParticle(new Particle(Vector3f(i * ds, -2.f, j * ds), staticMass, index++, false));
//            sys->addParticle(new Particle(Vector3f(-ds * dimensions, -2.f + ds * dimensions + i * ds, j * ds), staticMass, index++, false));
//            sys->addParticle(new Particle(Vector3f( ds * dimensions, -2.f + ds * dimensions + i * ds, j * ds), staticMass, index++, false));
//        }
//    }

    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, -2.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 0.9f));
    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));
    sys->addForce(new SurfaceForce(sys->particles));

    return sys;
}