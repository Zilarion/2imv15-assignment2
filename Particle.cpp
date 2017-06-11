#include "Particle.h"
#if defined(_WIN32) || defined(WIN32)
    #include <GL/glut.h>
#else
    #include <GLUT/glut.h>
#endif

Particle::Particle(const Vector3f & startPosition, float density, float mass, int index) :
	startPos(startPosition), density(density), position(Vector3f(0.0, 0.0, 0.0)), velocity(Vector3f(0.0, 0.0, 0.0)), force(Vector3f(0.0,0.0,0.0)), mass(mass), index(index)
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
void Particle::draw(bool drawVelocity, bool drawForce)
{
	const float h = .025f;
	glColor3f(1.f, 1.f, 1.f);
    glPointSize(h);
	glBegin(GL_POINTS);
        glVertex3f(position[0], position[1], position[2]);
	glEnd();

//    glPushMatrix();
//    glMaterialfv(GL_FRONT, GL_DIFFUSE, Vec3f(0.88,0.08,0.88));
//    glTranslated(position[0], position[1], position[2]);
//    glutSolidSphere(h, 10, 10);
//    glPopMatrix();

    if (drawVelocity) {
        glColor3f(0.0, 0.6, 0.0);
        glBegin(GL_LINES);
        glVertex3f(position[0], position[1], position[2]);
        glVertex3f(position[0] + velocity[0] * 0.2f, position[1] + velocity[1] * 0.2f,
                   position[2] + velocity[2] * 0.2f);
        glEnd();
    }
    if (drawForce) {
        glColor3f(0.0, 0.6, 0.6);
        glBegin(GL_LINES);
        glVertex3f(position[0], position[1], position[2]);
        glVertex3f(position[0] + force[0], position[1] + force[1], position[2] + force[2]);
        glEnd();
    }
}
