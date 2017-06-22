//
// Created by s130604 on 21-6-2017.
//

#ifndef FLUIDS_CONTACT_H
#define FLUIDS_CONTACT_H


#include "RigidBody.h"

class Contact {
public:
    Contact(RigidBody *r, Particle *p, Vector3f n);

    RigidBody *r;
    Particle *p;
    Vector3f n;     //normal of the face colliding with the particle
};


#endif //FLUIDS_CONTACT_H
