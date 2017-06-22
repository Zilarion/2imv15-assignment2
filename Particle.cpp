#include "Particle.h"

#if defined(_WIN32) || defined(WIN32)
#include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif

Particle::Particle(const Vector3f &startPosition, float mass, int index, bool movable) :
        startPos(startPosition), density(0), position(Vector3f(0.0, 0.0, 0.0)),
        velocity(Vector3f(0.0, 0.0, 0.0)), force(Vector3f(0.0, 0.0, 0.0)), mass(mass), index(index),
        movable(movable) {
    position = startPos;
}

Particle::~Particle(void) {
}

void Particle::reset()
{
    position = startPos;
    velocity = Vector3f(0.0, 0.0, 0.0);
    force = Vector3f(0.0, 0.0, 0.0);
}

void Particle::draw(bool drawVelocity, bool drawForce, float meanDensity) {
    glPushMatrix();
        float sphereSize = 0.01f;
        if (movable) {
            glColor3f(.2f, 1.f, 1.f);
        } else {
            sphereSize = 0.01f;
            glColor4f(1.f, 1.f, 1.f, 0.1f);
        }
        glTranslated(position[0], position[1], position[2]);
        glutSolidSphere(sphereSize, 3, 3);
    glPopMatrix();

    float fMult = 0.00001f;

    if (drawVelocity && movable) {
        glColor3f(0.0, 0.6, 0.0);
        glBegin(GL_LINES);
        glVertex3f(position[0], position[1], position[2]);
        glVertex3f(position[0] + velocity[0] * 0.2f, position[1] + velocity[1] * 0.2f,
                   position[2] + velocity[2] * 0.2f);
        glEnd();
    }
    if (drawForce && movable) {
        glColor3f(0.0, 1.f, 1.f);
        glBegin(GL_LINES);
        glVertex3f(position[0], position[1], position[2]);
        glVertex3f(position[0] + force[0] * fMult, position[1] + force[1] * fMult,
                   position[2] + force[2] * fMult);
        glEnd();
    }
}

void Particle::handleSweep(bool isStart, vector<RigidBody *> *activeRigidBodies,
                           vector<pair<RigidBody *, Particle *>> *range) {
    for (RigidBody *r:(*activeRigidBodies)) {
        (*range).push_back(make_pair(r, this));
    }
}
