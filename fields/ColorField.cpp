#include "ColorField.h"
#include "../Kernels.h"
#include "../System.h"

float ColorField::eval(Vector3f pos) {
    float color = 0;
    for (Particle *pj : sys->particles) {
        color += pj->mass / pj->density * Poly6::W(pos - pj->position, 1.f);
    }
    return color;
}

Vector3f ColorField::dEval(Vector3f pos) {
    Vector3f color = Vector3f(0, 0, 0);
    for (Particle *pj : sys->particles) {
        color += pj->mass / pj->density * Poly6::dW(pos - pj->position, 1.f);
    }
    return color;
}

float ColorField::ddEval(Vector3f pos) {
    float color = 0;
    for (Particle *pj : sys->particles) {
        color += pj->mass / pj->density * Poly6::ddW(pos - pj->position, 1.f);
    }
    return color;
}