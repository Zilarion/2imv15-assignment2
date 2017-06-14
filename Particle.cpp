#include "Particle.h"
#if defined(_WIN32) || defined(WIN32)
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif

Particle::Particle(const Vector3f & startPosition, float mass, int index, bool movable) :
	startPos(startPosition), density(0), position(Vector3f(0.0, 0.0, 0.0)),
    velocity(Vector3f(0.0, 0.0, 0.0)), force(Vector3f(0.0,0.0,0.0)), mass(mass), index(index),
    movable(movable)
{
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
    position = startPos;
    velocity = Vector3f(0.0, 0.0, 0.0);
    force = Vector3f(0.0, 0.0, 0.0);
}

void Particle::draw(bool drawVelocity, bool drawForce) {
    const float h = .05f;
    if (movable) {
        glColor3f(0.f, 1.f, 1.f);
    } else {
        glColor3f(1.f, 1.f, 1.f);
    }

    glPushMatrix();
    glTranslated(position[0], position[1], position[2]);
    glutSolidSphere(h, 10, 10);

//    glColor4f(1.f, 0.f, 0.f, 0.4f);
//    glutSolidSphere(.15f, 10, 10);
    glPopMatrix();

    if (drawVelocity && movable) {
        glColor3f(0.0, 0.6, 0.0);
        glBegin(GL_LINES);
        glVertex3f(position[0], position[1], position[2]);
        glVertex3f(position[0] + velocity[0] * 0.2f, position[1] + velocity[1] * 0.2f,
                   position[2] + velocity[2] * 0.2f);
        glEnd();
    }
    if (drawForce && movable) {
        glColor3f(0.0, 0.6, 0.6);
        glBegin(GL_LINES);
        glVertex3f(position[0], position[1], position[2]);
        glVertex3f(position[0] + force[0] * 0.2f, position[1] + force[1] * 0.2f,
                   position[2] + force[2] * 0.2f);
        glEnd();
    }
}
