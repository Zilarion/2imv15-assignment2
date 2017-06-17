

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
                Particle *p = new Particle(Vector3f(xStart, yStart, zStart), particleMass, index, true);
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
//    glBegin(GL_POINTS);
//    for (Particle *p:particles) {
//        Vector3f ri = R * p->position + x;
//        glVertex3f(ri[0], ri[1], ri[2]);
//    }
//    glEnd();
    Vector3f v1 = R * Vector3f(-dimensions[0] / 2, -dimensions[1] / 2, -dimensions[2] / 2) + x;
    Vector3f v2 = R * Vector3f(dimensions[0] / 2, -dimensions[1] / 2, -dimensions[2] / 2) + x;
    Vector3f v3 = R * Vector3f(-dimensions[0] / 2, -dimensions[1] / 2, dimensions[2] / 2) + x;
    Vector3f v4 = R * Vector3f(dimensions[0] / 2, -dimensions[1] / 2, dimensions[2] / 2) + x;
    Vector3f v5 = R * Vector3f(-dimensions[0] / 2, dimensions[1] / 2, -dimensions[2] / 2) + x;
    Vector3f v6 = R * Vector3f(dimensions[0] / 2, dimensions[1] / 2, -dimensions[2] / 2) + x;
    Vector3f v7 = R * Vector3f(-dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2) + x;
    Vector3f v8 = R * Vector3f(dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2) + x;
    glBegin(GL_LINES);
    glColor3f(1.f, 1.f, 1.f);
    glVertex3f(v1[0], v1[1], v1[2]);
    glVertex3f(v2[0], v2[1], v2[2]);
    glVertex3f(v1[0], v1[1], v1[2]);
    glVertex3f(v3[0], v3[1], v3[2]);
    glVertex3f(v2[0], v2[1], v2[2]);
    glVertex3f(v4[0], v4[1], v4[2]);
    glVertex3f(v3[0], v3[1], v3[2]);
    glVertex3f(v4[0], v4[1], v4[2]);

    glVertex3f(v5[0], v5[1], v5[2]);
    glVertex3f(v6[0], v6[1], v6[2]);
    glVertex3f(v5[0], v5[1], v5[2]);
    glVertex3f(v7[0], v7[1], v7[2]);
    glVertex3f(v6[0], v6[1], v6[2]);
    glVertex3f(v8[0], v8[1], v8[2]);
    glVertex3f(v7[0], v7[1], v7[2]);
    glVertex3f(v8[0], v8[1], v8[2]);

    glVertex3f(v1[0], v1[1], v1[2]);
    glVertex3f(v5[0], v5[1], v5[2]);
    glVertex3f(v2[0], v2[1], v2[2]);
    glVertex3f(v6[0], v6[1], v6[2]);
    glVertex3f(v3[0], v3[1], v3[2]);
    glVertex3f(v7[0], v7[1], v7[2]);
    glVertex3f(v4[0], v4[1], v4[2]);
    glVertex3f(v8[0], v8[1], v8[2]);
    glEnd();
}

void RigidBody::updateForce() {
    force = Vector3f(0, 0, 0);
    for (Particle *p : particles) {
        force += p->force;
    }
}

void RigidBody::updateTorque() {
    torque = Vector3f(0, 0, 0);
    for (Particle *p : particles) {
        Vector3f ri = R * p->position + x;
        torque += (ri - x).cross(p->force);
    }
}

void RigidBody::setState(VectorXf newState) {
    x[0] = newState[0];
    x[1] = newState[1];
    x[2] = newState[2];

    q.w() = newState[3];
    q.x() = newState[4];
    q.y() = newState[5];
    q.z() = newState[6];

    P[0] = newState[7];
    P[1] = newState[8];
    P[2] = newState[9];

    L[0] = newState[10];
    L[1] = newState[11];
    L[2] = newState[12];

    //Compute auxiliary variables
    R = q.normalized().toRotationMatrix();
    v = P / M;
    Iinv = R * IbodyInv * R.transpose();
    omega = Iinv * L;
}

/*
 * pack x, R, P and L into a single vector
 */
VectorXf RigidBody::getState() {
    VectorXf y(13);
    y[0] = x[0];
    y[1] = x[1];
    y[2] = x[2];

    y[3] = q.w();
    y[4] = q.x();
    y[5] = q.y();
    y[6] = q.z();

    y[7] = P[0];
    y[8] = P[1];
    y[9] = P[2];

    y[10] = L[0];
    y[11] = L[1];
    y[12] = L[2];
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

    //calculate product, convert to resulting matrix to quaternion
    Quaternionf omegaQuaternion(0, omega[0], omega[1], omega[2]);
    Quaternionf qdot(omegaQuaternion * q);
    y[3] = qdot.w() * 0.5f;
    y[4] = qdot.x() * 0.5f;
    y[5] = qdot.y() * 0.5f;
    y[6] = qdot.z() * 0.5f;

    //Pdot = F
    y[7] = force[0];
    y[8] = force[1];
    y[9] = force[2];

    //Ldot = torque
    y[10] = torque[0];
    y[11] = torque[1];
    y[12] = torque[2];
    return y;
}

Matrix3f RigidBody::star(Vector3f a) {
    Matrix3f m;
    m << 0, -a[2], a[1],
            a[2], 0, -a[0],
            -a[1], a[0], 0;
    return m;
}



