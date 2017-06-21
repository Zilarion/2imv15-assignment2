#pragma once

#include <gfx/vec3.h>
#include <Eigen/Dense>

using namespace Eigen;

class Particle
{
public:

	Particle(const Vector3f & startPosition, float mass, int index, bool movable);
	virtual ~Particle(void);

	void reset();
	void draw(bool drawVelocity, bool drawForce, float meanDensity);

    Vector3f startPos;
    Vector3f force;
    Vector3f velocity;
    Vector3f position;
    float pressure;
    float density;
    int index;
    float mass;
    bool movable;
};
