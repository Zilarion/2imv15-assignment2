//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_KERNEL_H
#define FLUIDS_KERNEL_H

#include <Eigen/Dense>

using namespace Eigen;

class Poly6 {
public:
    static float W(Vector3f r, float h);
    static Vector3f dW(Vector3f r, float h);
    static float ddW(Vector3f r, float h);
};

class Spiky {
public:
    static float W(Vector3f r, float h);
    static Vector3f dW(Vector3f r, float h);
};

class Viscosity {
public:
    static float W(Vector3f r, float h);
    static float ddW(Vector3f r, float h);
};

#endif //FLUIDS_KERNEL_H
