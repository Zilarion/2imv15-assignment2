//
// Created by Ruud Andriessen on 11/06/2017.
//

#include "Field.h"

Field::Field(System* s): sys(s) {

}

Vector3f Field::W(Vector3f pos) {
    return Eigen::Vector3f();
}

Vector3f Field::dW(Vector3f pos) {
    return Eigen::Vector3f();
}

Vector3f Field::ddW(Vector3f pos) {
    return Eigen::Vector3f();
}
