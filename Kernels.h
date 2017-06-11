//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_KERNEL_H
#define FLUIDS_KERNEL_H

#include <Eigen/Dense>

using namespace Eigen;

static float H = 0.2f;

static class Poly6 {
public:
    static float W(float r, float h = H);
};

static class Spiky {
public:
    static float W(float r, float h = H);
    static float dW(float r, float h = H);
};

static class Viscosity {
public:
    static float W(float r, float h = H);
    static float ddW(float r, float h = H);
};

#endif //FLUIDS_KERNEL_H
