//
// Created by Ruud Andriessen on 03/05/2017.
//

#ifndef PARTICLETOY_SYSTEM_H
#define PARTICLETOY_SYSTEM_H

#include "Eigen/Dense"

#include "Particle.h"
#include "SystemBuilder.h"
#include "constraints/Constraint.h"
#include "forces/Force.h"
#include "fields/DensityField.h"
#include "fields/PressureField.h"
#include "fields/ColorField.h"
#include "data/UniformGrid.h"
#include "RigidBody.h"
#include "MarchingCubes.h"
#include "Contact.h"

#include <vector>

using namespace Eigen;

class Solver;
class System {
private:
    void drawParticles(bool drawVelocity, bool drawForce, bool drawMarchingCubes);
    void drawRigidBodies(bool drawVelocity, bool drawForce);
    void drawForces();
    void drawConstraints();

    void computeForces();
    void clearForces();

    float time;
public:
    System(Solver* solver);
    ~System();

    std::vector<Particle*> particles;
    std::vector<RigidBody*> rigidBodies;
    std::vector<Force*> forces;
    std::vector<Constraint*> constraints;

    DensityField* densityField;
    PressureField* pressureField;
    ColorField* colorField;
    UniformGrid grid;
    MarchingCubes* marchingCubes;

    bool springsCanBreak = false;
    float dt;
    SystemBuilder::AvailableSystems type;
    Solver* solver;

    void addParticle(Particle* p);
    void addRigidBody(RigidBody* r);
    void addForce(Force* f);
    void addConstraint(Constraint* c);

    // ODE interface
    VectorXf derivEval();
    VectorXf computeDerivative();
    VectorXf getState();
    float getTime();
    void setState(VectorXf src);
    void setState(VectorXf newState, float time);
    unsigned long getParticleDim();
    vector<Contact*> findContacts(VectorXf newState);   //bool indicates if there are any

    void step(bool adaptive);
    void free();
    void reset();
    void draw(bool drawVelocity, bool drawForces = false, bool drawConstraints = false, bool drawMarchingCubes = false);
};


#endif //PARTICLETOY_SYSTEM_H
