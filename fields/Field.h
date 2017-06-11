//
// Created by Ruud Andriessen on 11/06/2017.
//

#ifndef FLUIDS_FIELD_H
#define FLUIDS_FIELD_H

#include <Eigen/Dense>
#include "../System.h"

using namespace Eigen;

class Field {
    Field(System* s);
    Vector3f W(Vector3f pos);
    Vector3f dW(Vector3f pos);
    Vector3f ddW(Vector3f pos);

private:
    System* sys;

};


#endif //FLUIDS_FIELD_H
