#ifndef FLUIDS_COLORFIELD_H
#define FLUIDS_COLORFIELD_H

#include "../Particle.h"

class System;

class ColorField {
public:
    ColorField(System* s) : sys(s) {}
    Vector3f eval(Vector3f position);
    Vector3f dEval(Vector3f position);
    Vector3f ddEval(Vector3f position);

private:
    System* sys;
};


#endif //FLUIDS_DENSITYFIELD_H
