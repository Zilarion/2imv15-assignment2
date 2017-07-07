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
#include "forces/SpringForce.h"
#include "forces/AngularSpringForce.h"
#include "constraints/RodConstraint.h"
#include "constraints/CircularWireConstraint.h"

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
        case GLASS:
            sys = initGlass();
            sys->type = GLASS;
            return sys;
        case CLOTH:
            sys = initCloth();
            sys->type = CLOTH;
            return sys;
        case HAIR:
            sys = initHair();
            sys->type = HAIR;
            return sys;
    }
    return NULL;
}

System* SystemBuilder::initBasic()
{
    System* sys = new System(new Euler(Euler::SEMI));

    int dimensions = 20;
    float mass = 1.f;
    float massStatic = 10.f;
    int index = 0;
    float d = 0.015f;
    float ds = 0.04f;

    // Movable particles
    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
            float x = i * d * 2 + (rand() % 10 + 1) * 0.01f;
            float y = -0.6f + (rand() % 10 + 1) * 0.05f;
            float z = j * d + (rand() % 10 + 1) * 0.01f;
            sys->addParticle(new Particle(Vector3f(x, y, z), mass, index++, true));
        }
    }

    // These forces only apply to non-static particles
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, -9.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 2.0f));

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
            // Layer one
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

    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));
    sys->addForce(new SurfaceForce(sys->particles));

    std::cout << "Running with: " << sys->particles.size() << " particles" << std::endl;
    return sys;
}
System *SystemBuilder::initTrechter() {
    System *sys = new System(new Euler(Euler::SEMI));

    int dimensions = 4;
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
                float y = k*d-.4f + (rand() % 10 + 1) * 0.001f;
                float z = j * d + (rand() % 10 + 1) * 0.001f;
                sys->addParticle(new Particle(Vector3f(x, y + 0.5f, z), mass, index++, true));
            }
        }
    }

    // These forces only apply to non-static particles
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, -9.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 2.f));

    // Static particles
    float topRadius = .2f;
    float bottomRadius = .075f;
    float radiusStep = (topRadius - bottomRadius)/30;
    int angleSteps = 120;
    float angleStep = M_PI * 2 / angleSteps;
    for (int i = 0; i < angleSteps; i++) {
        float angle = angleStep * i;
        float radius = bottomRadius;
        for (int y = -20; y < 0; y++) {
            float x = cos(angle) * radius;
            float z = sin(angle) * radius;
            sys->addParticle(new Particle(Vector3f(x, y * ds + 0.5f, z), mass, index++, false));
            radius+=radiusStep;
        }
    }
    //bottom

    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));
    sys->addForce(new SurfaceForce(sys->particles));

    std::cout << "Running with: " << sys->particles.size() << " particles" << std::endl;
    return sys;
}

System *SystemBuilder::initGlass() {
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
                float y = k*d-.4f + (rand() % 10 + 1) * 0.001f;
                float z = j * d + (rand() % 10 + 1) * 0.001f;
                Particle *p = new Particle(Vector3f(x, y, z), mass, index++, true);
                sys->addParticle(p);
            }
        }
    }

    // These forces only apply to non-static particles
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, -9.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 0.5f));

    // Static particles
    float topRadius = .3f;
    float bottomRadius = .05f;
    int angleSteps = 120;
    float angleStep = M_PI * 2 / angleSteps;
    for (int i = 0; i < angleSteps; i++) {
        float angle = angleStep * i;
        float radius = topRadius;
        for (int y = -20; y < 0; y++) {
            float x = cos(angle) * radius;
            float z = sin(angle) * radius;
            sys->addParticle(new Particle(Vector3f(x, y * ds, z), mass, index++, false));
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

    sys->addForce(new DirectionalForce(sys->particles, Vector3f(.0f, 9.81f, .0f)));
    sys->addForce(new DragForce(sys->particles, 2.f));
    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));
    sys->addForce(new SurfaceForce(sys->particles));

    std::cout << "Smoke scene created, press 'o' to spawn smoke." << std::endl;

    return sys;
}

System* SystemBuilder::initBasicCloth()
{
    System* sys = new System(new RungeKutta());

    const float dist = 0.5f;
    Vector3f center(0.0, 0.0, 0.0);
    Vector3f offset(dist, 0.0, 0.0);

    sys->addParticle(new Particle(center + offset, 1.0f, 0, true, false, true));
    sys->addParticle(new Particle(center + (2.f * offset), 1.0f, 1, true, false, true));
    sys->addParticle(new Particle(center + (3.f * offset), 1.0f, 2, true, false, true));
    sys->addParticle(new Particle(center + (3.f * offset), 1.0f, 3, true, false, true));
    sys->addParticle(new Particle(center + (4.f * offset), 1.0f, 4, true, false, true));
    sys->addParticle(new Particle(center + (4.f * offset), 1.0f, 5, true, false, true));

    sys->addForce(new DragForce(sys->particles, 0.5f));
    sys->addForce(new SpringForce(sys->particles[0], sys->particles[1], dist, 150.f, 1.5f));
    sys->addForce(new SpringForce(sys->particles[2], sys->particles[4], dist, 150.f, 1.5f));
    sys->addForce(new SpringForce(sys->particles[3], sys->particles[5], dist, 50.f, 1.5f));
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(0, -9.81f, 0)));

    sys->addConstraint(new RodConstraint(sys->particles[1], sys->particles[2], dist));
    sys->addConstraint(new RodConstraint(sys->particles[1], sys->particles[3], dist));
    sys->addConstraint(new CircularWireConstraint(sys->particles[0], center, dist));

    return sys;
}

System* SystemBuilder::initCloth() {
    System* sys = new System(new Euler(Euler::SEMI));

    const int xSize = 20, ySize = 20;
    const float deltaX = .5f/xSize, deltaZ = .5f/ySize;
    int pindex = 0;
    // Initialize particles
    for (int y = 0; y < ySize; y++) {
        for (int x = 0; x < xSize; x++) {
            //cloth particles have constant density
            Particle *p = new Particle(Vector3f(-.3f + x * deltaX, 0.15f, -.3f + deltaZ * y), 3.0f, pindex++, true, false, true);
            p->density = 10000.f;
            sys->addParticle(p);
        }
    }

    // Add gravity and drag to all particles
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(0, -9.81f, 0)));
    sys->addForce(new DragForce(sys->particles, 2.f));

    float spr = 8000.0f;
    float dmp = 80.5f;

    for (int y = 0; y < ySize; y++)  {
        for (int x = 0; x < xSize - 1; x++) {
            sys->addForce(new SpringForce(sys->particles[x + y * xSize],
                                          sys->particles[x + 1 + y * xSize],
                                          deltaX, spr, dmp));
        }
    }

    for (int y = 0; y < ySize - 1; y++) {
        for (int x = 0; x < xSize; x++) {
            sys->addForce(new SpringForce(sys->particles[x + y * xSize],
                                          sys->particles[x + (y + 1) * xSize],
                                          deltaZ, spr, dmp));
        }
    }

    for (int y = 0; y < ySize - 1; y++) {
        for (int x = 0; x < xSize - 1; x++) {
            sys->addForce(new SpringForce(sys->particles[x + y * xSize],
                                          sys->particles[x + 1 + (y + 1) * xSize],
                                          sqrt(pow(deltaX, 2) + pow(deltaZ, 2)), spr, dmp));
        }
    }



    for (int y = 0; y < ySize - 1; y++) {
        for (int x = 1; x < xSize; x++) {
            sys->addForce(new SpringForce(sys->particles[x + y * xSize],
                                          sys->particles[x - 1 + (y + 1) * xSize],
                                          sqrt(pow(deltaX, 2) + pow(deltaZ, 2)), spr, dmp));
        }
    }

    float r = 0.05f;
    sys->addConstraint(new CircularWireConstraint(sys->particles[0],
                                                  sys->particles[0]->startPos + Vector3f(0.f, r, 0.f),
                                                  r));
    sys->addConstraint(new CircularWireConstraint(sys->particles[xSize-1],
                                                  sys->particles[xSize-1]->startPos + Vector3f(0.f, r, 0.f),
                                                  r));
    sys->addConstraint(new CircularWireConstraint(sys->particles[xSize * ySize - 1],
                                                  sys->particles[xSize * ySize - 1]->startPos + Vector3f(0.f, r, 0.f),
                                                  r));
    sys->addConstraint(new CircularWireConstraint(sys->particles[xSize * (ySize - 1)],
                                                  sys->particles[xSize * (ySize - 1)]->startPos + Vector3f(0.f, r, 0.f),
                                                  r));

    int dimensions = 3;
    float mass = 1.f;
    float d = 0.06;

    // Movable particles
    for (int i = -dimensions / 2; i < dimensions / 2; i++) {
        for (int j = -dimensions / 2; j < dimensions / 2; j++) {
            for(int k = 0; k<1;k++) {
                float x = i * d + (rand() % 10 + 1) * 0.001f;
                float y = k*d+.2f + (rand() % 10 + 1) * 0.001f;
                float z = j * d + (rand() % 10 + 1) * 0.001f;
                Particle *p = new Particle(Vector3f(x, y, z), mass, pindex++, true);
                sys->addParticle(p);
            }
        }
    }

    sys->addForce(new PressureForce(sys->particles));
    sys->addForce(new ViscosityForce(sys->particles));
    sys->addForce(new SurfaceForce(sys->particles));

    return sys;
}


System* SystemBuilder::initHair() {
    System* sys = new System(new Euler(Euler::SEMI));

    const int ySize = 10;
    const float deltaY = 3.0f/ySize;
    const int numHairs = 8;


    for (int k = 0; k < numHairs; k++) {
        // Initialize particles
        for (int y = 0; y < ySize; y++) {
            sys->addParticle(new Particle(Vector3f(-0.5f + 0.03f*k, 0.5f - y * deltaY, deltaY * y), 0.2f, k * ySize + y, true, false, true));
        }

        float spr = 120.0f;
        float dmp = 1.5f;


        for (int y = 0; y < ySize - 1; y++) {
            sys->addForce(new SpringForce(sys->particles[k * ySize + y],
                                          sys->particles[k * ySize + y + 1],
                                          deltaY, spr, dmp));
        }

        for (int y = 2; y < ySize - 2; y++) {
            sys->addForce(new AngularSpringForce(sys->particles[k * ySize + y],
                                                 sys->particles[k * ySize + y + 1],
                                                 sys->particles[k * ySize + y + 2],
                                                 2.5f, 20.f, dmp));
        }

        float r = 0.05f;
        sys->addConstraint(new CircularWireConstraint(sys->particles[k * ySize],
                                                      sys->particles[k * ySize]->startPos + Vector3f(-r, 0.f, 0.f),
                                                      r));
    }


    // Add gravity and drag to all particles
    sys->addForce(new DirectionalForce(sys->particles, Vector3f(0, -9.81f, 0)));
    sys->addForce(new DragForce(sys->particles, 0.5f));

    return sys;
}