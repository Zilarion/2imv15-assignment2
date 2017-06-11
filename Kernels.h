//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_KERNEL_H
#define FLUIDS_KERNEL_H

#include <Eigen/Dense>

using namespace Eigen;

static class Poly6 {
    static float W(float r, float h);
};

static class Spiky {
    static float W(float r, float h);
};

static class Viscosity {
    static float W(float r, float h);
};

#endif //FLUIDS_KERNEL_H
