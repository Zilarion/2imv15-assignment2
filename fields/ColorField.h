#ifndef FLUIDS_COLORFIELD_H
#define FLUIDS_COLORFIELD_H

#include "../Particle.h"

class System;

class ColorField {
public:
    ColorField(System* s) : sys(s) {}
    float eval(Vector3f position);
    Vector3f dEval(Vector3f position);
    float ddEval(Vector3f position);

private:
    System* sys;
};


#endif //FLUIDS_DENSITYFIELD_H
