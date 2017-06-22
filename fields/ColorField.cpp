#include "ColorField.h"
#include "../Kernels.h"
#include "../System.h"

float ColorField::eval(const Vector3f &pos) {
    float color = 0;
    for (Particle *pj : sys->grid.query(pos)) {
        color += pj->mass / pj->density * Poly6::W(pos - pj->position);
    }
    return color;
}

Vector3f ColorField::dEval(const Vector3f &pos) {
    Vector3f color = Vector3f(0, 0, 0);
    vector<Particle*> targets = sys->grid.query(pos);
    for (Particle *pj : targets) {
        color += pj->mass / pj->density * Poly6::dW(pos - pj->position);
    }
    return color;
}

float ColorField::ddEval(const Vector3f &pos) {
    float color = 0;
    vector<Particle*> targets = sys->grid.query(pos);
    for (Particle *pj : targets) {
        color += pj->mass / pj->density * Poly6::ddW(pos - pj->position);
    }
    return color;
}