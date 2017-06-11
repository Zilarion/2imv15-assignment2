//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_DENSITYFIELD_H
#define FLUIDS_DENSITYFIELD_H

#include "../Particle.h"

class System;

class DensityField {
public:
    DensityField(System* s) : sys(s) {}
    float eval(Particle * pi);

private:
    System* sys;
};


#endif //FLUIDS_DENSITYFIELD_H
