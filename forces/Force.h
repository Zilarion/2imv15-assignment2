//
// Created by Ruud Andriessen on 03/05/2017.
//

#ifndef PARTICLETOY_FORCE_H
#define PARTICLETOY_FORCE_H

#include "../Particle.h"
#include <vector>

#include <Eigen/Dense>
#include <map>

class System;

using namespace Eigen;
using namespace std;

class Force {
protected:
    bool active = true;

public:
    vector<Particle*> particles;
    virtual void setTarget(const vector<Particle*> &particles) = 0;
    void addAsTarget(Particle* p) {
        particles.push_back(p);
    }
    virtual void apply(System* s) = 0;
    virtual void draw() = 0;

    void setActive(bool state);
    bool toggle();
};


#endif //PARTICLETOY_FORCE_H
