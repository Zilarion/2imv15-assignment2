#pragma once

#include "../Particle.h"
#include "Constraint.h"

class RodConstraint : public Constraint {
public:
    RodConstraint(Particle *p1, Particle * p2, float dist);

    void draw() override;

    float C() override;
    float Cd() override;
    vector<Vector3f> j() override;
    vector<Vector3f> jd() override;

 private:
  Particle * const p1;
  Particle * const p2;
  float const dist;
};
