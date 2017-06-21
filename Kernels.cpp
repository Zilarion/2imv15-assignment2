//
// Created by Ruud Andriessen on 11/06/2017.
//

#include <iostream>
#include "Kernels.h"

// Use poly for all computations except for visc. and pressure
float Poly6::W(const Vector3f &r, float h) {
    float rd = r.norm();
    if (rd >= 0 && rd <= h) {
        return 315 / (64 * M_PI * pow(h, 9)) * pow(h * h - rd * rd, 3);
    }
    return 0;
}

Vector3f Poly6::dW(const Vector3f &r, float h) {
    float rd = r.norm();
    if (rd >= 0 && rd <= h) {
        return 315 / (64 * M_PI * pow(h, 9)) * -6  * pow(h * h - rd * rd, 2) * r;
    }
    return Vector3f(0,0,0);
}

float Poly6::ddW(const Vector3f &r, float h) {
    float rd = r.norm();
    if (rd >= 0 && rd <= h) {
        return 315 / (64 * M_PI * pow(h, 9)) * (24 * rd * rd * (h * h - rd * rd) - 6 * pow(h * h - rd * rd, 2));
    }
    return 0;
}

// Use spiky for pressure computations
float Spiky::W(const Vector3f &r, float h) {
    float rd = r.norm();
    if (rd >= 0 && rd <= h) {
        return 15 / (M_PI * pow(h, 6)) * pow(h - rd, 3);
    }
    return 0;
}

Vector3f Spiky::dW(const Vector3f &r, float h) {
    float rd = r.norm();
    if (rd > 0 && rd <= h) {
        return - 45 / (M_PI * pow(h, 6) * rd) * pow(h - rd, 2) * r;
    }
    return Vector3f(0,0,0);
};

// Use viscosity for viscosity computations
float Viscosity::W(const Vector3f &r, float h) {
    float rd = r.norm();
    if (rd >= 0 && rd <= h) {
        return 15 / (2 * M_PI * pow(h, 3)) * (-pow(rd, 3) / (2 * pow(h, 3)) + (rd * rd) / (h * h) + h / (2 * rd) - 1);
    }
    return 0;
}

float Viscosity::ddW(const Vector3f &r, float h) {
    float rd = r.norm();
    if (rd >= 0 && rd <= h) {
        return 45 / (M_PI * pow(h, 6)) * (h - rd);
    }
    return 0;
}
