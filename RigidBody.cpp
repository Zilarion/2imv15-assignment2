

#include <GL/gl.h>
#include "RigidBody.h"
#if defined(_WIN32) || defined(WIN32)
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif

RigidBody::RigidBody(Vector3f startPos, Vector3f dimensions, Vector3f numParticles, float particleMass) :
        startPos(startPos), dimensions(dimensions) {
    initializeVariables();

    //generate particles with body coordinates
    int index = 0;
    for (int x = 0; x < numParticles[0]; x++) {
        for (int y = 0; y < numParticles[1]; y++) {
            for (int z = 0; z < numParticles[2]; z++) {
                float xStart = -dimensions[0] / 2 + dimensions[0] * (float) x / float(numParticles[0]);
                float yStart = -dimensions[1] / 2 + dimensions[1] * (float) y / float(numParticles[1]);
                float zStart = -dimensions[2] / 2 + dimensions[2] * (float) z / float(numParticles[2]);
                Particle* p = new Particle(Vector3f(xStart, yStart, zStart), particleMass, index, true);
                //A rigid body has constant density
                p->density = 1.0f;
                particles.push_back(p);
                index++;
            }
        }
    }

    //Calculate total mass
    for (Particle *p : particles) {
        M += p->mass;
    }

    //Calculate Ibody
    for (Particle *p : particles) {
        Vector3f r0i = p->position;             //position in body coordinates
        RowVector3f r0iT = r0i.transpose();     //position in body coordinates
        Ibody += p->mass * (r0i * r0iT);
    }
    IbodyInv = Ibody.inverse();
}

RigidBody::~RigidBody(void) {

}

void RigidBody::reset() {
    initializeVariables();
    for (Particle *p : particles) {
        p->reset();
    }
}

void RigidBody::initializeVariables() {
    x = startPos;
    R = Matrix3f::Identity();
    P = Vector3f(0, 0, 0);
    L = Vector3f(0, 0, 0);
    Iinv = IbodyInv;
    v = Vector3f(0, 0, 0);
    omega = Iinv * L;
    force = Vector3f(0, 0, 0);
    torque = Vector3f(0, 0, 0);
}

void RigidBody::draw(bool drawVelocity, bool drawForce) {
    glBegin(GL_POINTS);
    for(Particle* p:particles){
        Vector3f ri =  R* p->position + x;
        printf("ri: [%f, %f, %f]\n", ri[0], ri[1], ri[2]);
        printf("ri: [%f, %f, %f]\n", ri[0], ri[1], ri[2]);
        glVertex3f(ri[0], ri[1], ri[2]);
    }
    glEnd();
}

void RigidBody::updateForce() {
    force = Vector3f(0, 0, 0);
    printf("Rigid body net forces\n");
    for (Particle *p : particles) {
        force += p->force;
        printf("%f ", p->force[1]);
    }
    printf("\n total net force: %f\n", force[1]);
}

void RigidBody::updateTorque() {
    torque = Vector3f(0, 0, 0);
    for (Particle *p : particles) {
        Vector3f ri =  R* p->position + x;
        torque += (ri - x).cross(p->force);
    }
}

void RigidBody::setState(VectorXf newState) {
    x[0] = newState[0];
    x[1] = newState[1];
    x[2] = newState[2];
    printf("x: %f\n", x[1]);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R(i, j) = newState[i * 3 + j + 3];
        }
    }
    P[0] = newState[12];
    P[1] = newState[13];
    P[2] = newState[14];

    L[0] = newState[15];
    L[1] = newState[16];
    L[2] = newState[17];

    //Compute auxiliary variables
    v = P / M;
    Iinv = R * IbodyInv * R.transpose();
    omega = Iinv * L;
}

/*
 * pack x, R, P and L into a single vector
 */
VectorXf RigidBody::getState() {
    VectorXf y(18);
    y[0] = x[0];
    y[1] = x[1];
    y[2] = x[2];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            y[i * 3 + j + 3] = R(i, j);
        }
    }
    y[12] = P[0];
    y[13] = P[1];
    y[14] = P[2];

    y[15] = L[0];
    y[16] = L[1];
    y[17] = L[2];
    return y;
}

VectorXf RigidBody::getDerivativeState() {
    updateForce();
    updateTorque();
    VectorXf y(18);
    //xdot
    y[0] = v[0];
    y[1] = v[1];
    y[2] = v[2];

    //Rdot
    Matrix3f Rdot = star(omega) * R;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            y[i * 3 + j + 3] = Rdot(i, j);
        }
    }

    printf("\n total net force at deriv: %f\n", force[1]);
    //Pdot = F
    y[12] = force[0];
    y[13] = force[1];
    y[14] = force[2];

    //Ldot = torque
    y[15] = torque[0];
    y[16] = torque[1];
    y[17] = torque[2];
    return y;
}

Matrix3f RigidBody::star(Vector3f a) {
    Matrix3f m;
    m << 0, -a[2], a[1],
            a[2], 0, -a[0],
            -a[1], a[0], 0;
    return m;
}



