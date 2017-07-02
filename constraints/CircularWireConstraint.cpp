#include "CircularWireConstraint.h"

#if defined(__CYGWIN__) || defined(WIN32)
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif


CircularWireConstraint::CircularWireConstraint(Particle *p, const Vector3f & center,
                                               const float radius) :
        Constraint({p}), particle(p), center(center), radius(radius) {}


void CircularWireConstraint::draw()
{
    glBegin(GL_LINE_LOOP);
    glColor3f(0.0,0.7,0.0);
    for (int i=0; i < 360; i=i+18)
    {
        float degInRad = i * M_PI / 180.0f;
        glVertex3f(center[0]+cos(degInRad)*radius,center[1]+sin(degInRad)*radius, center[2]);
    }
    glEnd();

    glBegin(GL_LINES);
    glColor3f(0.7,0.7,0.0);
        glVertex3f(particle->position[0], particle->position[1], particle->position[2]);
        glVertex3f(center[0], center[1], center[2]);
    glEnd();
}

/**
 * Compute C of this constraint
 * @return x * x - r * r
 */
float CircularWireConstraint::C() {
    Vector3f delta = particle->position - center;
    return delta.dot(delta) - radius * radius;
}

/**
 * Computes Cd of this constraint
 * @return x * xd
 */
float CircularWireConstraint::Cd() {
    Vector3f pVector = (particle->position - center);
    Vector3f vVector = particle->velocity;

    return 2 * pVector.dot(vVector);
}

/**
 * Computes j of this constraint
 * @return
 */
vector<Vector3f> CircularWireConstraint::j() {
    vector<Vector3f> j;
    j.push_back((particle->position - center) * 2);
    return j;
}

/**
 * Computes jd of this constraint
 * @return
 */
vector<Vector3f> CircularWireConstraint::jd() {
    vector<Vector3f> jd;
//     TODO fix this? I don't know why it doesn't work
    jd.push_back(Vector3f(2.f * particle->velocity[0], 2.f * particle->velocity[1], 2.f * particle->velocity[2]));
    return jd;
}
