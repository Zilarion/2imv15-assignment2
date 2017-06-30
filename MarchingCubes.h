//
// Created by s132054 on 18-6-2017.
//

#ifndef FLUIDS_MARCHINGCUBES_H
#define FLUIDS_MARCHINGCUBES_H

#include "Eigen/Dense"
#include <vector>
#include <string>

using namespace Eigen;
using namespace std;

bool operator<(const Vector3f &left, const Vector3f &right);

class MarchingCubes {

public:

    struct XYZ {
        float x, y, z;

        XYZ operator+(const XYZ &other) const {
            return XYZ{x + other.x, y + other.y, z + other.z};
        }
        void operator+=(const XYZ &other) {
            x += other.x;
            y += other.y;
            z += other.z;
        }
        XYZ operator^(const XYZ &other) const {
            return XYZ{y*other.z - z*other.y, z*other.x - x*other.z, x*other.y - y*other.x};
        }
        XYZ operator-(const XYZ &other) const{
            return XYZ{x - other.x, y - other.y, z - other.z};
        }
        XYZ mult(float left) const {
            return XYZ{left * x, left * y, left * z};
        }
        XYZ div(float right) const {
            return XYZ{x / right, y / right, z / right};
        }

        string toString(float prec) {
            return to_string((int) (x * prec)) + "," + to_string((int) (y * prec)) + "," +
                   to_string((int) (z * prec));
        }

        float operator[](int i) const {
            if (i == 0) {
                return x;
            } else if (i == 1) {
                return y;
            } else {
                return z;
            }
        }
        bool operator<(XYZ &other) const{
            return x - other.x < .005f && y - other.y < .005f && z - other.z < .005f;
        }

        long toLongHash() {
            float p = 100.f;
            long w = 1000;
            return (long)(x * p) + (long)(y * p) * w + (long)(z * p) * w * w;
        }

        float size() const {
            return sqrt(x * x + y * y + z * z);
        }

        XYZ normalize(){
            float s = size();
            x = x / s;
            y = y / s;
            z = z / s;
            return *this;
        }

    };

    struct XYZA {
        XYZ v;
        float a;
    };

    struct TRIANGLE {
        //EIGEN MAKE ALIGNED OPERATOR NEW
        XYZ p[3];
        XYZ n[3];
    };

    struct GRIDCELL {
        //EIGEN MAKE ALIGNED OPERATOR NEW
        XYZA p[8];
        XYZ n[8];
    };

    MarchingCubes(System* system);
    ~MarchingCubes();

    int Polygonise(GRIDCELL &grid, TRIANGLE *triangles);

    XYZ VertexInterp(float isovalue, XYZ &p1, XYZ &p2, float val1, float val2);
    XYZ VertexInterpNormal(float isolevel, XYZ &np1, XYZ &np2, float valp1, float valp2);

    //void updateGradient(int index);

    //XYZ getEdgeNormal(XYZ edgePoint);

    void drawMarching();

    System* system;

    XYZ cubeStart;
    XYZ cubeEnd;
    float cubeStep;

    int* cubeStartInt;
    int* cubeEndInt;
    int *cubeCornerDim;
    int size;
    XYZA *cubeCorners;
    //XYZ *gradientCorners;

    float particleRange;
    float iso;
    vector<TRIANGLE> triangles;
    //map<long, XYZ> normals;

};
#endif //FLUIDS_MARCHINGCUBES_H
