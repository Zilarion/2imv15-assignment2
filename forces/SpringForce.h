#pragma once

#include "../Particle.h"
#include "Force.h"

class SpringForce : public Force {

public:
    SpringForce(Particle *p1, Particle * p2, float dist, float ks, float kd);
    SpringForce(std::vector<Particle*> particles, float dist, float ks, float kd);

    void draw() override;
    void setTarget(const std::vector<Particle*> &particles) override;
    void apply(System *s) override;
    //map<int, map<int, float>>  jx() override;
    //MatrixXf jv() override;
private:
    float const dist;     // rest length
    float const ks, kd; // spring strength constants
};
