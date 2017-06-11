#pragma once

#include <gfx/vec3.h>
#include <Eigen/Dense>

using namespace Eigen;

class Particle
{
public:

	Particle(const Vector3f & startPosition, float density, float mass, int index);
	virtual ~Particle(void);

	void reset();
	void draw(bool drawVelocity, bool drawForce);

    Vector3f startPos;
	Vector3f position;
    Vector3f force;
    Vector3f velocity;
    float pressure;
    float density;
    int index;
    float mass;
};
