//
// Created by s130604 on 16-6-2017.
//
#ifndef FLUIDS_RIGIDBODY_H
#define FLUIDS_RIGIDBODY_H

#include <Eigen/Dense>
#include "Particle.h"
#include "forces/Force.h"
#include "Object.h"


using namespace Eigen;

class RigidBody : public Object {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RigidBody(Eigen::Vector3f startPos, Vector3f dimensions, Vector3f numParticles, float particleMass);

    virtual ~RigidBody(void);

    void reset();

    void draw(bool drawVelocity, bool drawForce);

    //from object
    void handleSweep(bool start, vector<RigidBody *>* activeRigidBodies, vector<pair<RigidBody *, Particle *>> *range) override;

    Vector3f pt_velocity(Vector3f p);   //get velocity at location in world space coordinates
    Vector3f getNormal(Vector3f p);     //unit normal of closest plane
    VectorXf getBoundingBox();          //minX, minY, minZ, maxX, maxY, maxZ
    Vector3f getBodyCoordinates(Vector3f world);

    VectorXf getState();

    VectorXf getDerivativeState();

    void setState(VectorXf newState);
    void recomputeAuxiliaryVars();
    bool isPenetrating(float epsilon, Particle *p);
    bool isContact(float epsilon, Particle *p);
    Particle* getClosestParticle(Vector3f bodyCoords);

    std::vector<Particle *> particles;
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
