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
                                 grid(80, 80, 80, 0.025f, Vector3f(1.f, 1.f, 1.f)) {
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
void System::draw(bool drawVelocity, bool drawForce, bool drawConstraint, bool drawMarchingCubes) {
    if (!drawMarchingCubes)
        drawParticles(drawVelocity, drawForce);
    drawRigidBodies(drawVelocity, drawForce);
    drawBorder();
    if (drawForce) {
        drawForces();
    }
    if (drawConstraint) {
        drawConstraints();
    }

    // draw marching cubes
    if (drawMarchingCubes) {
        Vector3f cubeStart = Vector3f(-1.1f, -1.1f, -1.1f);
        Vector3f cubeEnd = Vector3f(1.1f, 1.1f, 1.1f);
        float cubeStep = .05f; // a whole number of steps should fit into interval

        Vector3i cubeStartInt = Vector3i((int)roundf(cubeStart[0] / cubeStep), (int)roundf(cubeStart[1] / cubeStep), (int)roundf(cubeStart[2] / cubeStep));
        Vector3i cubeEndInt = Vector3i((int)roundf(cubeEnd[0] / cubeStep), (int)roundf(cubeEnd[1] / cubeStep), (int)roundf(cubeEnd[2] / cubeStep));
        int cubeCornerDim[3] = {cubeEndInt[0] - cubeStartInt[0] + 1, cubeEndInt[1] - cubeStartInt[1] + 1, cubeEndInt[2] - cubeStartInt[2] + 1};
        int size = cubeCornerDim[0] * cubeCornerDim[1] * cubeCornerDim[2];
        float cubeCorners[size];
        Vector3f gradientCorners[size];

        for (int i = 0; i < size; i++) {
            cubeCorners[i] = 0;
            gradientCorners[i] = Vector3f(0.f,0.f,0.f);
        }

        float particleRange = 1.f; //percentage of step size for speed increase
        particleRange = particleRange * cubeStep;
        for (Particle *p: particles) {
            Vector3f pos = p->position;
            // only apply marching cube to particle when it is inside the rendering volume
            if (pos[0] > cubeStart[0] && pos[1] > cubeStart[1] && pos[2] > cubeStart[2]
                && pos[0] < cubeEnd[0] && pos[1] < cubeEnd[1] && pos[2] < cubeEnd[2]) {
                // add the distance between the point and each of the 8 corners of its surrounding gridcube
                // to the grid values

                // lower corner [0, 0, 0] of gridcube
                Vector3f lowerGridPos = Vector3f(floor(pos[0] / cubeStep) * cubeStep,
                                                 floor(pos[1] / cubeStep) * cubeStep,
                                                 floor(pos[2] / cubeStep) * cubeStep);
                int lowerCubePos = (int) (floor(pos[0] / cubeStep) + -cubeStartInt[0] +
                                          cubeCornerDim[0] * (floor(pos[1] / cubeStep) + -cubeStartInt[1]
                                                            + (cubeCornerDim[1] *
                                                               (floor(pos[2] / cubeStep) + -cubeStartInt[2]))));

                //fill in all the gridcube values
//                int steps = (int) ceilf(particleRange / cubeStep);
                for (int x = 0; x <= 1; x++) {
                    for (int y = 0; y <= 1; y++) {
                        for (int z = 0; z <= 1; z++) {
                            Vector3f gridPos = lowerGridPos + Vector3f(x * cubeStep, y * cubeStep, z * cubeStep);
                            int cubePos = lowerCubePos + x + (cubeCornerDim[0] * (y + cubeCornerDim[1] * z));
                            if (cubePos < cubeCornerDim[0] * cubeCornerDim[1] * cubeCornerDim[0] && cubePos >= 0) {
                                cubeCorners[cubePos] =
                                        cubeCorners[cubePos] +
                                        max(particleRange - (pos - gridPos).norm(), 0.f) / particleRange;
                                // update gradients based on change in grid
                                updateGradient(cubeCorners, cubeCornerDim, cubePos, gradientCorners);
                            }
                        }
                    }
                }
            }
        }

        double iso = 0.1;
        vector<TRIANGLE> triangles = {};
        //unordered_map<string, Vector3f> normals = {};

        for (int x = cubeStartInt[0]; x < cubeEndInt[0] - 1; x++) {
            for (int y = cubeStartInt[1]; y < cubeEndInt[1] - 1; y++) {
                for (int z = cubeStartInt[2]; z < cubeEndInt[2] - 1; z++) {
                    int cubePos0 =
                            x + -cubeStartInt[0] + cubeCornerDim[0] * (y + -cubeStartInt[1] +
                                    (cubeCornerDim[1] * (z + -cubeStartInt[2])));
                    int cubePos[8] = {
                            cubePos0, //[0,0,0]
                            cubePos0 + 1, // [1,0,0]
                            cubePos0 + 1 + cubeCornerDim[0], // [1,1,0]
                            cubePos0 + cubeCornerDim[0], // [0,1,0]
                            cubePos0 + cubeCornerDim[0] * cubeCornerDim[1], // [0,0,1]
                            cubePos0 + 1 + cubeCornerDim[0] * cubeCornerDim[1], // [1,0,1]
                            cubePos0 + 1 + cubeCornerDim[0] + cubeCornerDim[0] * cubeCornerDim[1], // [1,1,1]
                            cubePos0 + cubeCornerDim[0] + cubeCornerDim[0] * cubeCornerDim[1] // [0,1,1]
                    };

                    GRIDCELL cell = {
                            {
                                Vector3f(x * cubeStep, y * cubeStep, z * cubeStep) +
                                Vector3f(0 * cubeStep, 0 * cubeStep, 0 * cubeStep), //[0,0,0]
                                Vector3f(x * cubeStep, y * cubeStep, z * cubeStep) +
                                Vector3f(1 * cubeStep, 0 * cubeStep, 0 * cubeStep), //[1,0,0]
                                Vector3f(x * cubeStep, y * cubeStep, z * cubeStep) +
                                Vector3f(1 * cubeStep, 1 * cubeStep, 0 * cubeStep), //[1,1,0]
                                Vector3f(x * cubeStep, y * cubeStep, z * cubeStep) +
                                Vector3f(0 * cubeStep, 1 * cubeStep, 0 * cubeStep), //[0,1,0]
                                Vector3f(x * cubeStep, y * cubeStep, z * cubeStep) +
                                Vector3f(0 * cubeStep, 0 * cubeStep, 1 * cubeStep), //[0,0,1]
                                Vector3f(x * cubeStep, y * cubeStep, z * cubeStep) +
                                Vector3f(1 * cubeStep, 0 * cubeStep, 1 * cubeStep), //[1,0,1]
                                Vector3f(x * cubeStep, y * cubeStep, z * cubeStep) +
                                Vector3f(1 * cubeStep, 1 * cubeStep, 1 * cubeStep), //[1,1,1]
                                Vector3f(x * cubeStep, y * cubeStep, z * cubeStep) +
                                Vector3f(0 * cubeStep, 1 * cubeStep, 1 * cubeStep)  //[0,1,1]
                            },
                            {
                                cubeCorners[cubePos[0]],
                                cubeCorners[cubePos[1]],
                                cubeCorners[cubePos[2]],
                                cubeCorners[cubePos[3]],
                                cubeCorners[cubePos[4]],
                                cubeCorners[cubePos[5]],
                                cubeCorners[cubePos[6]],
                                cubeCorners[cubePos[7]]
                            }
                    };

                    double cellsum = 0;
                    for (int i = 0; i < 8; i++) {
                        cellsum += cubeCorners[cubePos[i]];
                    }

                    if (cellsum > 0) {
                        TRIANGLE tris[5] = {};
                        int n = Polygonise(cell, iso, tris);

                        if (n > 0) {
                            for (int i = 0; i < n; i++) {
                                TRIANGLE tri = tris[i];
                                triangles.push_back(tri);

                                /*add normal to combined normals of each point in triangle. (per-vertex normals) DEPRECATED
                                Vector3f a = tri.p[0];
                                Vector3f b = tri.p[1];
                                Vector3f c = tri.p[2];
                                Vector3f norm = (b - a).cross(c - b);
                                string aindex = VectorToString(a, 10.f);
                                if (!normals.emplace(aindex, norm).second) {
                                    normals[aindex] += norm;
                                }
                                string bindex = VectorToString(b, 10.f);
                                if (!normals.emplace(bindex, norm).second) {
                                    normals[bindex] += norm;
                                }
                                string cindex = VectorToString(c, 10.f);
                                if (!normals.emplace(cindex, norm).second) {
                                    normals[cindex] += norm;
                                }
                                //*/
                            }
                        }
                    }

                    /* draw each grid cell DEBUG
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
        glColor4f(.8f, .7f, .9f, 1.f);
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < triangles.size(); i++) {
            TRIANGLE triangle = triangles[i];
            Vector3f a = triangle.p[0];
            Vector3f b = triangle.p[1];
            Vector3f c = triangle.p[2];
            /* per-face normals
            Vector3f norm = (b - a).cross(c - b);
            glNormal3f(norm[0], norm[1], norm[2]);
            //// per vertex normals
            Vector3f anorm = normals[VectorToString(a, 10.f)];
            Vector3f bnorm = normals[VectorToString(b, 10.f)];
            Vector3f cnorm = normals[VectorToString(c, 10.f)];
            anorm.normalize();
            bnorm.normalize();
            cnorm.normalize();
            //*/
            //* per vertex normals 2
            Vector3f anorm = getEdgeNormal(a, cubeStart, cubeEnd, cubeCornerDim, cubeStep, gradientCorners);
            anorm.normalize();
            glNormal3f(anorm[0], anorm[1], anorm[2]);
            glVertex3f(a[0], a[1], a[2]);

            Vector3f bnorm = getEdgeNormal(b, cubeStart, cubeEnd, cubeCornerDim, cubeStep, gradientCorners);
            bnorm.normalize();
            glNormal3f(bnorm[0], bnorm[1], bnorm[2]);
            glVertex3f(b[0], b[1], b[2]);

            Vector3f cnorm = getEdgeNormal(c, cubeStart, cubeEnd, cubeCornerDim, cubeStep, gradientCorners);
            cnorm.normalize();
            glNormal3f(cnorm[0], cnorm[1], cnorm[2]);
            glVertex3f(c[0], c[1], c[2]);
        }
        glEnd();
    }
    glNormal3f(1.f, 0.f, 0.f);
    glColor4f(1.f, 1.f, 1.f, 1.f);
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
    VectorXf state(this->getParticleDim() + rigidBodies.size() * 13);

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
            state[getParticleDim() + 13 * i + j] = rState[j];
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
        VectorXf rState(13);
        for (int j = 0; j < rState.size(); j++) {
            rState[j] = src[getParticleDim() + 13 * i + j];
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
    float restDensity = 1000;
    for (Particle *p : particles) {
        p->density = densityField->eval(p, grid);
        meanDensity += p->density;
    }
    meanDensity /= particles.size();

    float k = 2.f;

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
    VectorXf dst(this->getParticleDim() + rigidBodies.size() * 13);
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
            dst[getParticleDim() + 13 * i + j] = rDeriv[j];
        }
    }
    return dst;
}

void System::drawParticles(bool drawVelocity, bool drawForce) {
    for (Particle *p : particles) {
        p->draw(drawVelocity, drawForce, meanDensity);
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

VectorXf System::checkBoundingBox(VectorXf newState) {
    float dist = .95f;
    float dec = .8f;
    //collision from x side
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6] < -dist) {
            newState[i * 6] = -dist;
            if (newState[i * 6 + 3] < 0) {
                newState[i * 6 + 3] = -newState[i * 6 + 3] * dec;
            }
        }
    }
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6] > dist) {
            newState[i * 6] = dist;
            if (newState[i * 6 + 3] > 0) {
                newState[i * 6 + 3] = -newState[i * 6 + 3] * dec;
            }
        }
    }
    //collision from z side
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6 + 2] < -dist) {
            newState[i * 6 + 2] = -dist;
            if (newState[i * 6 + 5] < 0) {
                newState[i * 6 + 5] = -newState[i * 6 + 3] * dec;
            }
        }
    }
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6 + 2] > dist) {
            newState[i * 6 + 2] = dist;
            if (newState[i * 6 + 5] > 0) {
                newState[i * 6 + 5] = -newState[i * 6 + 3] * dec;
            }
        }
    }
    //Check collision with y side
    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6 + 1] < -dist) {
            newState[i * 6 + 1] = -dist;
            if (newState[i * 6 + 4] < 0) {
                newState[i * 6 + 4] = -newState[i * 6 + 4] * dec;
            }
        }
    }

    for (int i = 0; i < particles.size(); i++) {
        if (newState[i * 6 + 1] > dist) {
            newState[i * 6 + 1] = dist;
            if (newState[i * 6 + 4] > 0) {
                newState[i * 6 + 4] = -newState[i * 6 + 4] * dec;
            }
        }
    }

    return newState;
}

vector<Contact *> System::findContacts(VectorXf newState) {
    vector<Contact *> contacts;
    //sweep sort
    //bool indicates start or end, start=true
    map<float, pair<Object *, bool>> xMap;
    map<float, pair<Object *, bool>> yMap;
    map<float, pair<Object *, bool>> zMap;
    for (RigidBody *r:rigidBodies) {
        VectorXf boundingBox = r->getBoundingBox();
        xMap[boundingBox[0]] = make_pair(r, true);
        xMap[boundingBox[3]] = make_pair(r, false);
        yMap[boundingBox[1]] = make_pair(r, true);
        yMap[boundingBox[4]] = make_pair(r, false);
        zMap[boundingBox[2]] = make_pair(r, true);
        zMap[boundingBox[5]] = make_pair(r, false);
//        printf("minY: %f\n",boundingBox[1]);
    }
    for (Particle *p:particles) {
        xMap[p->position[0]] = make_pair(p, true);
        yMap[p->position[1]] = make_pair(p, true);
        zMap[p->position[2]] = make_pair(p, true);
    }
    vector<RigidBody *> activeRigidBodies;
    //keep track of particles that are in x/y/z range of a rigid body
    //there is a collision if a  particle is present in all 3 ranges
    vector<pair<RigidBody *, Particle *>> xRange;
    vector<pair<RigidBody *, Particle *>> yRange;
    vector<pair<RigidBody *, Particle *>> zRange;
    for (pair<int, pair<Object *, bool>> xPair:xMap) {
        xPair.second.first->handleSweep(xPair.second.second, &activeRigidBodies, &xRange);
    }
    for (pair<int, pair<Object *, bool>> yPair:yMap) {
        yPair.second.first->handleSweep(yPair.second.second, &activeRigidBodies, &yRange);
    }
    for (pair<int, pair<Object *, bool>> zPair:zMap) {
        zPair.second.first->handleSweep(zPair.second.second, &activeRigidBodies, &zRange);
    }
    for (pair<RigidBody *, Particle *> xPair:xRange) {
        //check if there is a collision in all three directions x,y,z
        if (find(yRange.begin(), yRange.end(), xPair) != yRange.end() &&
            find(zRange.begin(), zRange.end(), xPair) != zRange.end()) {
            contacts.push_back(new Contact(xPair.first, xPair.second, xPair.first->getNormal(xPair.second->position)));
        }
    }
    return contacts;
}
void System::drawBorder() {
    glBegin(GL_LINES);
        glColor3f(.8f, .8f, .8f);
        glVertex3f(-0.95f, -0.95f, -0.95f);
        glVertex3f(0.95f, -0.95f, -0.95f);
        glVertex3f(0.95f, -0.95f, 0.95f);
        glVertex3f(-0.95f, -0.95f, 0.95f);
    glEnd();
}
