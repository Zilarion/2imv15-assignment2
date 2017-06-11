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

#include <vector>

using namespace Eigen;

class Solver;
class System {
private:
    void drawParticles(bool drawVelocity, bool drawForce);
    void drawForces();
    void drawConstraints();

    void computeForces();
    void clearForces();

    float time;
public:
    System(Solver* solver);
    ~System();

    std::vector<Particle*> particles;
    std::vector<Force*> forces;
    std::vector<Constraint*> constraints;

    DensityField* densityField;
    PressureField* pressureField;

    bool wallExists;
    bool springsCanBreak = false;
    float dt;
    SystemBuilder::AvailableSystems type;
    Solver* solver;

    void addParticle(Particle* p);
    void addForce(Force* f);
    void addConstraint(Constraint* c);

    // ODE interface
    VectorXf derivEval();
    VectorXf computeDerivative();
    VectorXf getState();
    float getTime();
    void setState(VectorXf src);
    void setState(VectorXf newState, float time);
    unsigned long getDim();

    void step(bool adaptive);
    void free();
    void reset();
    void draw(bool drawVelocity, bool drawForces = false, bool drawConstraints = false);
};


#endif //PARTICLETOY_SYSTEM_H
