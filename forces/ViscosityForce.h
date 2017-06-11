//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_VISCOSITYFORCE_H
#define FLUIDS_VISCOSITYFORCE_H

#include "Force.h"

class ViscosityForce : public Force {
public:
    ViscosityForce(std::vector<Particle*> particles);
    void setTarget(std::vector<Particle*> particles) override;
    void apply(System* s) override;
    void draw() override;

};


#endif //FLUIDS_VISCOSITYFORCE_H
