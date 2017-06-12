#include "ColorField.h"
#include "../Kernels.h"
#include "../System.h"

Vector3f ColorField::eval(Vector3f pos) {
    Vector3f color = Vector3f(0, 0, 0);
    for (Particle *pj : sys->particles) {
        for (int i = 0; i < 3; i++) {
            color[i] += pj->mass / pj->density * Poly6::W(pos[i] - pj->position[i]);
        }
    }
    return color;
}
Vector3f ColorField::dEval(Vector3f pos) {
    Vector3f color = Vector3f(0, 0, 0);
    for (Particle *pj : sys->particles) {
        for (int i = 0; i < 3; i++) {
            color[i] += pj->mass / pj->density * Poly6::dW(pos[i] - pj->position[i]);
        }
    }
    return color;
}
Vector3f ColorField::ddEval(Vector3f pos) {
    Vector3f color = Vector3f(0, 0, 0);
    for (Particle *pj : sys->particles) {
        for (int i = 0; i < 3; i++) {
            color[i] += pj->mass / pj->density * Poly6::ddW(pos[i] - pj->position[i]);
        }
    }
    return color;
}