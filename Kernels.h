//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_KERNEL_H
#define FLUIDS_KERNEL_H

#include <Eigen/Dense>

using namespace Eigen;
static float hdef = 0.05f;

class Poly6 {
public:
    static float W(const Vector3f &r, float h = hdef);
    static Vector3f dW(const Vector3f &r, float h = hdef);
    static float ddW(const Vector3f &r, float h = hdef);
};

class Spiky {
public:
    static float W(const Vector3f &r, float h = hdef);
    static Vector3f dW(const Vector3f &r, float h = hdef);
};

class Viscosity {
public:
    static float W(const Vector3f &r, float h = hdef);
    static float ddW(const Vector3f &r, float h = hdef);
};

#endif //FLUIDS_KERNEL_H
