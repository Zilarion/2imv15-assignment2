#ifndef FLUIDS_COLORFIELD_H
#define FLUIDS_COLORFIELD_H

#include "../Particle.h"

class System;

class ColorField {
public:
    ColorField(System* s) : sys(s) {}
    float eval(const Vector3f &position);
    Vector3f dEval(const Vector3f &position);
    float ddEval(const Vector3f &position);

private:
    System* sys;
};


#endif //FLUIDS_DENSITYFIELD_H
