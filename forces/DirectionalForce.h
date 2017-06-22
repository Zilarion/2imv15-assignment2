//
// Created by Ruud Andriessen on 03/05/2017.
//

#ifndef PARTICLETOY_DIRECTIONALFORCE_H
#define PARTICLETOY_DIRECTIONALFORCE_H

#include "Force.h"

using namespace Eigen;

class DirectionalForce : public Force {
public:
    DirectionalForce(const vector<Particle*> &particles, const Vector3f &direction);

    void setTarget(const vector<Particle*> &particles) override;
    void apply(System* s) override;
    void draw() override;

    Vector3f direction;
private:
};


#endif //PARTICLETOY_DIRECTIONALFORCE_H
