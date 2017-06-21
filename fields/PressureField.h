//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_PRESSUREFIELD_H
#define FLUIDS_PRESSUREFIELD_H

#include "../Particle.h"
#include "../data/UniformGrid.h"

class System;

class PressureField {
public:
    PressureField(System* s) : sys(s) {};
    Vector3f eval(Particle * pi);

private:
    System* sys;
};


#endif //FLUIDS_PRESSUREFIELD_H
