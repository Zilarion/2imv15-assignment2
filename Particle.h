#pragma once

#include <Eigen/Dense>
#include "Object.h"

class RigidBody;

using namespace Eigen;
using namespace std;

class Particle:public Object
{
public:

	Particle(const Vector3f & startPosition, float mass, int index, bool movable, bool rigid = false, bool cloth = false);
	virtual ~Particle(void);


	void reset();
	void draw(bool drawVelocity, bool drawForce);

	//from object

	void handleSweep(bool start, vector<RigidBody *>* activeRigidBodies, vector<pair<RigidBody *, Particle *>> *range) override;

    Vector3f startPos;
    Vector3f force;
    Vector3f velocity;
    Vector3f position;
    float pressure;
    float density;
    int index;
    float mass;
    bool movable;
    bool rigid;
	bool cloth;
	Vector3f sForce, pForce, vForce;
};
