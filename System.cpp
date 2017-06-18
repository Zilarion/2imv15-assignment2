//
// Created by Ruud Andriessen on 03/05/2017.
//

#include "System.h"
#include "solvers/Solver.h"
#include "solvers/ConstraintSolver.h"
#include "fields/PressureField.h"
#include "fields/ColorField.h"
#include "MarchingCubes.h"

#if defined(_WIN32) || defined(WIN32)

#include <GL/glut.h>

#else
#include <GLUT/glut.h>
#endif

System::System(Solver *solver) : solver(solver), time(0.0f), wallExists(false), dt(0.005),
                                 grid(50, 50, 50, 0.1f, Vector3f(2.5f, 2.5f, 2.5f)) {
    densityField = new DensityField(this);
    pressureField = new PressureField(this);
    colorField = new ColorField(this);
}

System::~System() {
    delete densityField;
    delete pressureField;
    delete colorField;
}

/**
 * Adds a given particle to the system
 * @param p The particle to add
 */
void System::addParticle(Particle *p) {
    particles.push_back(p);
    for (Force *f : forces) {
        f->addAsTarget(p);
    }
}

/**
 * Adds a given particle to the system
 * @param p The particle to add
 */
void System::addRigidBody(RigidBody *r) {
    rigidBodies.push_back(r);
}

/**
 * Adds a force to use in the system when advancing a time step
 * @param f The new force to use in the system
 */
void System::addForce(Force *f) {
    forces.push_back(f);
}

/**
 * Adds a constraint to use in the system when advancing a time step
 * @param c The new constraint to use in the system
 */
void System::addConstraint(Constraint *c) {
    constraints.push_back(c);
}

/**
 * Frees all system data
 */
void System::free() {
    particles.clear();
    forces.clear();
}

/**
 * Resets all the system to it's initial state
 */
void System::reset() {
    for (Particle *p : particles) {
        p->reset();
    }
    for (RigidBody *r : rigidBodies) {
        r->reset();
    }
}

/**
 * Draws the forces
 */
void System::draw(bool drawVelocity, bool drawForce, bool drawConstraint) {
    drawParticles(drawVelocity, drawForce);
    drawRigidBodies(drawVelocity, drawForce);
    if (drawForce) {
        drawForces();
    }
    if (drawConstraint) {
        drawConstraints();
    }

    // draw marching cubes
    Vector3f cubeStart = Vector3f(-1.f, -3.f, -1.f);
    Vector3f cubeEnd = Vector3f(1.f, 1.f, 1.f);
    Vector3i cubeStartInt = Vector3i(-10, -30, -10);
    Vector3i cubeEndInt = Vector3i(10, 10, 10);
    int cubeCornerDimX = 21;
    int cubeCornerDimY = 41;
    int cubeCornerDimZ = 21;
    float cubeCorners[cubeCornerDimX * cubeCornerDimY * cubeCornerDimZ] = {};
    float cubeStep = .1f; //10^k
    float particleRange = .15f;
    for (Particle *p: particles) {
        Vector3f pos = p->position;
        // only apply marching cube to particle when it is inside the rendering volume
        if (pos[0] > cubeStart[0] && pos[1] > cubeStart[1] && pos[2] > cubeStart[2]
            && pos[0] < cubeEnd[0] && pos[1] < cubeEnd[1] && pos[2] < cubeEnd[2]) {
            // add the distance between the point and each of the 8 corners of its surrounding gridcube
            // to the grid values

            // lower corner [0, 0, 0] of gridcube
            Vector3f lowerGridPos = Vector3f(floor(pos[0] / cubeStep) * cubeStep, floor(pos[1] / cubeStep) * cubeStep,
                                             floor(pos[2] / cubeStep) * cubeStep);
            int lowerCubePos = (int) (floor(pos[0] * 10.f) + -cubeStartInt[0] +
                                      cubeCornerDimX * (floor(pos[1] * 10.f) + -cubeStartInt[1]
                                                       + (cubeCornerDimY * (floor(pos[2] * 10.f) + -cubeStartInt[2]))));

            //fill in all the gridcube values
            int steps = (int) ceilf(particleRange / cubeStep);
            for (int x = -steps; x <= steps + 1; x++) {
                for (int y = -steps; y <= steps + 1; y++) {
                    for (int z = -steps; z <= steps + 1; z++) {
                        Vector3f gridPos = lowerGridPos + Vector3f(x * cubeStep, y * cubeStep, z * cubeStep);
                        int cubePos = lowerCubePos + x + (cubeCornerDimX * (y + cubeCornerDimY * z));
                        if (cubePos < cubeCornerDimX * cubeCornerDimY * cubeCornerDimX && cubePos >= 0) {
                            cubeCorners[cubePos] = min(
                                    cubeCorners[cubePos] +
                                    max(particleRange - (pos - gridPos).norm(), 0.f) / particleRange, 1.f);
                        }
                    }
                }
            }
        }
    }

    double iso = .8;
    std::vector<TRIANGLE> triangles = {};
    std::map<string, std::vector> normals = {};

    for (int x = cubeStartInt[0]; x < cubeEndInt[0]; x++) {
        for (int y = cubeStartInt[1]; y < cubeEndInt[1]; y++) {
            for (int z = cubeStartInt[2]; z < cubeEndInt[2]; z++) {
                int cubePos0 = x + -cubeStartInt[0] + cubeCornerDimX * (y + -cubeStartInt[1] + (cubeCornerDimY * (z +
                                                                                                                -cubeStartInt[2]))); // [0,0,0]
                int cubePos1 = cubePos0 + 1; // [1,0,0]
                int cubePos2 = cubePos0 + 1 + cubeCornerDimX; // [1,1,0]
                int cubePos3 = cubePos0 + cubeCornerDimX; // [0,1,0]
                int cubePos4 = cubePos0 + cubeCornerDimX * cubeCornerDimY; // [0,0,1]
                int cubePos5 = cubePos0 + 1 + cubeCornerDimX * cubeCornerDimY; // [1,0,1]
                int cubePos6 = cubePos0 + 1 + cubeCornerDimX + cubeCornerDimX * cubeCornerDimY; // [1,1,1]
                int cubePos7 = cubePos0 + cubeCornerDimX + cubeCornerDimX * cubeCornerDimY; // [0,1,1]

                GRIDCELL cell = {
                        {
                                Vector3f(x / 10.f, y / 10.f, z / 10.f) +
                                Vector3f(0 * cubeStep, 0 * cubeStep, 0 * cubeStep), //[0,0,0]
                                Vector3f(x / 10.f, y / 10.f, z / 10.f) +
                                Vector3f(1 * cubeStep, 0 * cubeStep, 0 * cubeStep), //[1,0,0]
                                Vector3f(x / 10.f, y / 10.f, z / 10.f) +
                                Vector3f(1 * cubeStep, 1 * cubeStep, 0 * cubeStep), //[1,1,0]
                                Vector3f(x / 10.f, y / 10.f, z / 10.f) +
                                Vector3f(0 * cubeStep, 1 * cubeStep, 0 * cubeStep), //[0,1,0]
                                Vector3f(x / 10.f, y / 10.f, z / 10.f) +
                                Vector3f(0 * cubeStep, 0 * cubeStep, 1 * cubeStep), //[0,0,1]
                                Vector3f(x / 10.f, y / 10.f, z / 10.f) +
                                Vector3f(1 * cubeStep, 0 * cubeStep, 1 * cubeStep), //[1,0,1]
                                Vector3f(x / 10.f, y / 10.f, z / 10.f) +
                                Vector3f(1 * cubeStep, 1 * cubeStep, 1 * cubeStep), //[1,1,1]
                                Vector3f(x / 10.f, y / 10.f, z / 10.f) +
                                Vector3f(0 * cubeStep, 1 * cubeStep, 1 * cubeStep)  //[0,1,1]
                        },
                        {
                                cubeCorners[cubePos0],
                                cubeCorners[cubePos1],
                                cubeCorners[cubePos2],
                                cubeCorners[cubePos3],
                                cubeCorners[cubePos4],
                                cubeCorners[cubePos5],
                                cubeCorners[cubePos6],
                                cubeCorners[cubePos7]
                        }
                };

                TRIANGLE tris[5] = {};

                int n = Polygonise(cell, iso, tris);

                if (n > 0) {
                    //printf("%i triangles for gridcell centered around %f,%f,%f:\n", n, cell.p[0][0], cell.p[0][1],
                    //cell.p[0][2]);

                    for (int i = 0; i < n; i++) {
                        triangles.push_back(tris[i]);
                    }
                }

                /* draw each grid cell
                if (cubeCorners[cubePos0] > 0.f) {
                    glColor3f(cubeCorners[cubePos0], 0.f, 0.f);
                    glPushMatrix();
                    glTranslated(x*cubeStep, y*cubeStep, z*cubeStep);
                    glBegin(GL_POINTS);
                        glVertex3f(0.f, 0.f, 0.f);
                    glEnd();
                    glPopMatrix();
                }
                //*/
            }
        }
    }

    //* draw triangles
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < triangles.size(); i++) {
        TRIANGLE triangle = triangles[i];
        Vector3f a = triangle.p[0];
        Vector3f b = triangle.p[1];
        Vector3f c = triangle.p[2];
        //printf("%f,%f,%f;%f,%f,%f;%f,%f,%f\n", a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]);
        Vector3f norm = (b - a).cross(c - b);
        norm.normalize();

        glNormal3f(norm[0], norm[1], norm[2]);
        glVertex3f(a[0], a[1], a[2]);
        glVertex3f(b[0], b[1], b[2]);
        glVertex3f(c[0], c[1], c[2]);
    }
    glEnd();
}

/**
 * Runs the active solver on the system to progress it's state by dt time
 * @param dt the amount of time to advance the system
 */
void System::step(bool adaptive) {
    if (adaptive) {
        VectorXf before = this->getState();
        solver->simulateStep(this, dt);
        VectorXf xa = this->getState();
        this->setState(before);

        solver->simulateStep(this, dt / 2);
        solver->simulateStep(this, dt / 2);
        VectorXf xb = this->getState();

        float err = (xa - xb).norm();
        if (err > 0)
            dt *= pow(0.001f / err, .5f);

        this->setState(before);
    }

    solver->simulateStep(this, dt);
}


unsigned long System::getParticleDim() {
    return particles.size() * 3 * 2; // 3 dimensions, velocity and position
}

/**
 * Constructs a state given the current system
 * @return A copy of the current state of the system
 */
VectorXf System::getState() {
    VectorXf state(this->getParticleDim() + rigidBodies.size() * 18);

    for (int i = 0; i < this->particles.size(); i++) {
        Particle *p = particles[i];
        state[i * 6 + 0] = p->position[0];
        state[i * 6 + 1] = p->position[1];
        state[i * 6 + 2] = p->position[2];
        state[i * 6 + 3] = p->velocity[0];
        state[i * 6 + 4] = p->velocity[1];
        state[i * 6 + 5] = p->velocity[2];
    }

    for (int i = 0; i < rigidBodies.size(); i++) {
        RigidBody *r = rigidBodies[i];
        VectorXf rState = r->getState();
        for (int j = 0; j < rState.size(); j++) {
            state[getParticleDim() + 18 * i + j] = rState[j];
        }
    }
    return state;
}

float System::getTime() {
    return time;
}

/**
 * Evaluates a derivative
 * @param dst The destination vector
 */
VectorXf System::derivEval() {
    clearForces();
    computeForces();
//    ConstraintSolver::solve(this, 100.0f, 10.0f);
    return computeDerivative();
}

void System::setState(VectorXf src) {
    this->setState(src, this->getTime());
}

void System::setState(VectorXf src, float t) {
    for (int i = 0; i < particles.size(); i++) {
        if (particles[i]->movable) {
            particles[i]->position[0] = src[i * 6 + 0];
            particles[i]->position[1] = src[i * 6 + 1];
            particles[i]->position[2] = src[i * 6 + 2];
            particles[i]->velocity[0] = src[i * 6 + 3];
            particles[i]->velocity[1] = src[i * 6 + 4];
            particles[i]->velocity[2] = src[i * 6 + 5];
        }
    }
    for (int i = 0; i < rigidBodies.size(); i++) {
        RigidBody *r = rigidBodies[i];
        VectorXf rState(18);
        for (int j = 0; j < rState.size(); j++) {
            rState[j] = src[getParticleDim() + 18 * i + j];
        }
        r->setState(rState);
    }
    this->time = t;
}

/// Private ///

void System::computeForces() {
    grid.clear();
    grid.insert(particles);

    // Compute all densities
    float restDensity = 50;
    for (Particle *p : particles) {
        p->density = densityField->eval(p, grid);
    }

    float k = .1f;

    // Compute all pressures at each particle
    for (Particle *p : particles) {
        p->pressure = k * (p->density - restDensity);
    }

    // Apply all forces
    for (Force *f : forces) {
        f->apply(this);
    }
}

void System::clearForces() {
    for (Particle *p : particles) {
        p->force = Vector3f(0.0f, 0.0f, 0.0f);
    }
    for (RigidBody *r : rigidBodies) {
        for (Particle *p : r->particles) {
            p->force = Vector3f(0.0f, 0.0f, 0.0f);
        }
    }
}

VectorXf System::computeDerivative() {
    VectorXf dst(this->getParticleDim() + rigidBodies.size() * 18);
    for (int i = 0; i < particles.size(); i++) {
        Particle *p = particles[i];
        dst[i * 6 + 0] = p->velocity[0];            /* Velocity */
        dst[i * 6 + 1] = p->velocity[1];
        dst[i * 6 + 2] = p->velocity[2];
        dst[i * 6 + 3] = p->force[0] / p->density;  /* new acceleration is F/density */
        dst[i * 6 + 4] = p->force[1] / p->density;
        dst[i * 6 + 5] = p->force[2] / p->density;
    }
    for (int i = 0; i < rigidBodies.size(); i++) {
        RigidBody *r = rigidBodies[i];
        VectorXf rDeriv = r->getDerivativeState();
        for (int j = 0; j < rDeriv.size(); j++) {
            dst[getParticleDim() + 18 * i + j] = rDeriv[j];
        }
    }
    return dst;
}

void System::drawParticles(bool drawVelocity, bool drawForce) {
    for (Particle *p : particles) {
        p->draw(drawVelocity, drawForce);
    }
}

void System::drawRigidBodies(bool drawVelocity, bool drawForce) {
    for (RigidBody *r:rigidBodies) {
        r->draw(drawVelocity, drawForce);
    }
}

void System::drawForces() {
    for (Force *f : forces) {
        f->draw();
    }
}

void System::drawConstraints() {
    for (Constraint *c : constraints) {
        c->draw();
    }
}

VectorXf System::checkCollisions(VectorXf newState) {
    //collision from x side
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6] < -0.2f) {
            newState[i * 6] = -0.2f;
            if (newState[i * 6 + 3] < 0) {
                newState[i * 6 + 3] = -newState[i * 6 + 3];
            }
        }
    }
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6] > 0.2f) {
            newState[i * 6] = 0.2f;
            if (newState[i * 6 + 3] > 0) {
                newState[i * 6 + 3] = -newState[i * 6 + 3];
            }
        }
    }
    //collision from z side
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6 + 2] < -0.2f) {
            newState[i * 6 + 2] = -0.2f;
            if (newState[i * 6 + 5] < 0) {
                newState[i * 6 + 5] = -newState[i * 6 + 3];
            }
        }
    }
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6 + 2] > 0.2f) {
            newState[i * 6 + 2] = 0.2f;
            if (newState[i * 6 + 5] > 0) {
                newState[i * 6 + 5] = -newState[i * 6 + 3];
            }
        }
    }
    //Check collision with y side
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6 + 1] < -2.0f) {
            newState[i * 6 + 1] = -2.0f;
            if (newState[i * 6 + 4] < 0) {
                newState[i * 6 + 4] = -newState[i * 6 + 4];
            }
        }
    }
    return newState;
}
