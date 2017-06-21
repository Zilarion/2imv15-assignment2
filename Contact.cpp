//
// Created by s130604 on 21-6-2017.
//
#include "Contact.h"


Contact::Contact(RigidBody *r, Particle *p, Vector3f n) : r(r), p(p), n(n) {}

bool Contact::isPenetrating(float epsilon) {
    Vector3f bodyCoords = r->getBodyCoordinates(p->position);
    VectorXf boundingBox = r->getBoundingBox();
    return bodyCoords[0] - boundingBox[0] > epsilon && boundingBox[3] - bodyCoords[0] > epsilon &&
           bodyCoords[1] - boundingBox[1] > epsilon && boundingBox[4] - bodyCoords[1] > epsilon &&
           bodyCoords[2] - boundingBox[2] > epsilon && boundingBox[5] - bodyCoords[2] > epsilon;
}

bool Contact::isContact(float epsilon) {
    Vector3f bodyCoords = r->getBodyCoordinates(p->position);
    VectorXf boundingBox = r->getBoundingBox();
    return (((abs(bodyCoords[0] - boundingBox[0]) < epsilon) || (abs(bodyCoords[0] - boundingBox[3]) < epsilon)) &&
            bodyCoords[1] - boundingBox[1] > epsilon && boundingBox[4] - bodyCoords[1] > epsilon &&
            bodyCoords[2] - boundingBox[2] > epsilon && boundingBox[5] - bodyCoords[2] > epsilon) ||
           (((abs(bodyCoords[1] - boundingBox[1]) < epsilon) || (abs(bodyCoords[1] - boundingBox[4]) < epsilon)) &&
            bodyCoords[0] - boundingBox[0] > epsilon && boundingBox[3] - bodyCoords[0] > epsilon &&
            bodyCoords[2] - boundingBox[2] > epsilon && boundingBox[5] - bodyCoords[2] > epsilon) ||
           (((abs(bodyCoords[2] - boundingBox[2]) < epsilon) || (abs(bodyCoords[2] - boundingBox[5]) < epsilon)) &&
            bodyCoords[1] - boundingBox[1] > epsilon && boundingBox[4] - bodyCoords[1] > epsilon &&
            bodyCoords[0] - boundingBox[0] > epsilon && boundingBox[3] - bodyCoords[0] > epsilon);
}


