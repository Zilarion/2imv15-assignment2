//
// Created by Ruud Andriessen on 06/05/2017.
//

#ifndef PARTICLETOY_DRAGFORCE_H
#define PARTICLETOY_DRAGFORCE_H


#include "Force.h"

using namespace Eigen;

class DragForce : public Force {
public:
    DragForce(std::vector<Particle*> particles, float amount);

    void setTarget(std::vector<Particle*> particles) override;
    void apply(System* s) override;
    void draw() override;
private:
    float amount;
};


#endif //PARTICLETOY_DRAGFORCE_H
