//
// Created by Ruud Andriessen on 11/06/2017.
//

#include <iostream>
#include "Kernels.h"

// Use poly for all computations except for visc. and pressure
float Poly6::W(float r, float h) {
    if (r >= 0 && r <= h) {
        return 315/(64 * M_PI * pow(h, 9)) * pow(h*h - r *r, 3);
    }
    return 0;
}

// Use spiky for pressure computations
float Spiky::W(float r, float h) {
    if (r >= 0 && r <= h) {
        return 15/(M_PI * pow(h, 6)) * pow(h - r, 3);
    }
    return 0;
}

float Spiky::dW(float r, float h) {
    if (r >= 0 && r <= h) {
//        return 45 * (h - 2 * r) * pow(h-r, 2) /(M_PI * pow(h, 7)); // deriv r
        return 45 * pow(h - r, 2) * pow(h-r, 2) /(M_PI * pow(h, 6)); // deriv h
    }
    return 0;
};

// Use viscosity for viscosity computations
float Viscosity::W(float r, float h) {
    if (r >= 0 && r <= h) {
        return 15/(2 * M_PI * pow(h, 3)) * (-pow(r, 3)/(2 * pow(h, 3)) + (r*r)/(h*h) + h/(2 * r) - 1);
    }
    return 0;
}

float Viscosity::ddW(float r, float h) {
    return 45/(M_PI * pow(h, 6)) * (h - r);
}
