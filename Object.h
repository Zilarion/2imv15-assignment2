//
// Created by s130604 on 21-6-2017.
//

#ifndef FLUIDS_OBJECT_H
#define FLUIDS_OBJECT_H

#include <vector>

using namespace std;

class RigidBody;
class Particle;

class Object {
public:
    virtual void handleSweep(bool start, vector<RigidBody *>* activeRigidBodies, vector<pair<RigidBody *, Particle *>> *range) = 0;
};


#endif //FLUIDS_OBJECT_H
