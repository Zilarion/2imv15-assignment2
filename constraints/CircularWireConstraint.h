#pragma once

#include "../Particle.h"
#include "Constraint.h"
#include <vector>

using namespace std;

class CircularWireConstraint : public Constraint {
public:
    CircularWireConstraint(Particle *p, const Vector3f & center, const float radius);

    void draw() override;

    float C() override;
    float Cd() override;
    vector<Vector3f> j() override;
    vector<Vector3f> jd() override;

private:
    Particle * const particle;
    Vector3f const center;
    float const radius;
};
