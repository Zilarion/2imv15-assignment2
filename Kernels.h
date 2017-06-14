//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_KERNEL_H
#define FLUIDS_KERNEL_H

#include <Eigen/Dense>

using namespace Eigen;

static float H = .15f;

class Poly6 {
public:
    static float W(Vector3f r, float h = H);
    static Vector3f dW(Vector3f r, float h = H);
    static float ddW(Vector3f r, float h = H);
};

class Spiky {
public:
    static float W(Vector3f r, float h = H);
    static Vector3f dW(Vector3f r, float h = H);
};

class Viscosity {
public:
    static float W(Vector3f r, float h = H);

    static float ddW(Vector3f r, float h = H);
};

#endif //FLUIDS_KERNEL_H
