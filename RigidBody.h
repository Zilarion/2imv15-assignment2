//
// Created by s130604 on 16-6-2017.
//

#ifndef FLUIDS_RIGIDBODY_H
#define FLUIDS_RIGIDBODY_H

#include <Eigen/Dense>
#include "Particle.h"
#include "forces/Force.h"

using namespace Eigen;

class RigidBody {
public:

    RigidBody(Vector3f startPos, Vector3f dimensions, Vector3f numParticles, float particleMass);
    virtual ~RigidBody(void);
    void reset();
    void draw(bool drawVelocity, bool drawForce);

    VectorXf getState();
    VectorXf getDerivativeState();
    void setState(VectorXf newState);

    std::vector<Particle*> particles;
    Vector3f startPos;
    Vector3f dimensions;  //lengths of the edges

    //Constants
    double M;                    //totalMass
    Matrix3f Ibody, IbodyInv;

    //State variables
    Vector3f x;                 //position x(t)
    Quaternionf q;              //quaternion representing R
    Vector3f P;                 //linear momentum P(t)
    Vector3f L;                 //angular momentum L(t)

    //Derived quantities
    Matrix3f R;                   //rotation R(t)
    Matrix3f Iinv;              //I^-1(t)
    Vector3f v;                 //velocity v(t)
    Vector3f omega;             //angular velocity omega(t)

    //Computed quantities
    Vector3f force;
    Vector3f torque;
private:
    Matrix3f star(Vector3f a);
    void updateForce();
    void updateTorque();
    void initializeVariables();
};


#endif //FLUIDS_RIGIDBODY_H
