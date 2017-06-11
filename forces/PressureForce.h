//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_PRESSUREFORCE_H
#define FLUIDS_PRESSUREFORCE_H


#include "Force.h"

class PressureForce : public Force {
public:
    PressureForce(std::vector<Particle*> particles);
    void setTarget(std::vector<Particle*> particles) override;
    void apply(System* s) override;
    void draw() override;
};


#endif //FLUIDS_PRESSUREFORCE_H
