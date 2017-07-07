//
// Created by Ruud Andriessen on 03/05/2017.
//

#include "DirectionalForce.h"

#if defined(__CYGWIN__) || defined(WIN32)
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif
using namespace Eigen;

DirectionalForce::DirectionalForce(const vector<Particle*> &particles, const Vector3f &direction) : direction(direction)
{
    this->setTarget(particles);
}

void DirectionalForce::setTarget(const vector<Particle*> &particles)
{
    this->particles = particles;
}

void DirectionalForce::apply(System * s)
{
    if (!active)
        return;
    for (Particle* p : particles) {
        if (p->rigid || p->cloth)
            p->force += p->mass * direction;
        else
            p->force += p->mass * p->density * direction;
    }
}

void DirectionalForce::draw()
{
    if (active) {
        glColor4f(.8f, .5f, 0.f, .3f);
        glBegin(GL_LINES);
        Vector3f normDirection = direction / direction.norm();
        for (Particle *p : particles) {
            glVertex3f(p->position[0], p->position[1], p->position[2]);
            glVertex3f(p->position[0] + normDirection[0], p->position[1] + normDirection[1], p->position[2] + normDirection[2]);
        }
        glEnd();
    }
}