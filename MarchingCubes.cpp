//
// Created by s132054 on 18-6-2017.
//
// source: http://paulbourke.net/geometry/polygonise/ (with minor changes to work with our codebase)
//

#include "System.h"
#include <random>
#include <string>
#include "MarchingCubes.h"

#if defined(__CYGWIN__) || defined(WIN32)

#include <GL/glut.h>

#else
#include <GLUT/glut.h>
#endif

const int edgeTable[256] = {
        0x0, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
        0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
        0x190, 0x99, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
        0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
        0x230, 0x339, 0x33, 0x13a, 0x636, 0x73f, 0x435, 0x53c,
        0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
        0x3a0, 0x2a9, 0x1a3, 0xaa, 0x7a6, 0x6af, 0x5a5, 0x4ac,
        0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
        0x460, 0x569, 0x663, 0x76a, 0x66, 0x16f, 0x265, 0x36c,
        0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
        0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff, 0x3f5, 0x2fc,
        0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
        0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55, 0x15c,
        0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
        0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc,
        0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
        0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
        0xcc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
        0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
        0x15c, 0x55, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
        0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
        0x2fc, 0x3f5, 0xff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
        0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
        0x36c, 0x265, 0x16f, 0x66, 0x76a, 0x663, 0x569, 0x460,
        0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
        0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa, 0x1a3, 0x2a9, 0x3a0,
        0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
        0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33, 0x339, 0x230,
        0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
        0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99, 0x190,
        0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
        0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0};
const int triTable[256][16] =
        {{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  8,  3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  1,  9,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  8,  3,  9,  8,  1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  2,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  8,  3,  1,  2,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9,  2,  10, 0,  2,  9,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2,  8,  3,  2,  10, 8,  10, 9,  8,  -1, -1, -1, -1, -1, -1, -1},
         {3,  11, 2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  11, 2,  8,  11, 0,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  9,  0,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  11, 2,  1,  9,  11, 9,  8,  11, -1, -1, -1, -1, -1, -1, -1},
         {3,  10, 1,  11, 10, 3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  10, 1,  0,  8,  10, 8,  11, 10, -1, -1, -1, -1, -1, -1, -1},
         {3,  9,  0,  3,  11, 9,  11, 10, 9,  -1, -1, -1, -1, -1, -1, -1},
         {9,  8,  10, 10, 8,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4,  7,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4,  3,  0,  7,  3,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  1,  9,  8,  4,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4,  1,  9,  4,  7,  1,  7,  3,  1,  -1, -1, -1, -1, -1, -1, -1},
         {1,  2,  10, 8,  4,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3,  4,  7,  3,  0,  4,  1,  2,  10, -1, -1, -1, -1, -1, -1, -1},
         {9,  2,  10, 9,  0,  2,  8,  4,  7,  -1, -1, -1, -1, -1, -1, -1},
         {2,  10, 9,  2,  9,  7,  2,  7,  3,  7,  9,  4,  -1, -1, -1, -1},
         {8,  4,  7,  3,  11, 2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {11, 4,  7,  11, 2,  4,  2,  0,  4,  -1, -1, -1, -1, -1, -1, -1},
         {9,  0,  1,  8,  4,  7,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1},
         {4,  7,  11, 9,  4,  11, 9,  11, 2,  9,  2,  1,  -1, -1, -1, -1},
         {3,  10, 1,  3,  11, 10, 7,  8,  4,  -1, -1, -1, -1, -1, -1, -1},
         {1,  11, 10, 1,  4,  11, 1,  0,  4,  7,  11, 4,  -1, -1, -1, -1},
         {4,  7,  8,  9,  0,  11, 9,  11, 10, 11, 0,  3,  -1, -1, -1, -1},
         {4,  7,  11, 4,  11, 9,  9,  11, 10, -1, -1, -1, -1, -1, -1, -1},
         {9,  5,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9,  5,  4,  0,  8,  3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  5,  4,  1,  5,  0,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {8,  5,  4,  8,  3,  5,  3,  1,  5,  -1, -1, -1, -1, -1, -1, -1},
         {1,  2,  10, 9,  5,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3,  0,  8,  1,  2,  10, 4,  9,  5,  -1, -1, -1, -1, -1, -1, -1},
         {5,  2,  10, 5,  4,  2,  4,  0,  2,  -1, -1, -1, -1, -1, -1, -1},
         {2,  10, 5,  3,  2,  5,  3,  5,  4,  3,  4,  8,  -1, -1, -1, -1},
         {9,  5,  4,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  11, 2,  0,  8,  11, 4,  9,  5,  -1, -1, -1, -1, -1, -1, -1},
         {0,  5,  4,  0,  1,  5,  2,  3,  11, -1, -1, -1, -1, -1, -1, -1},
         {2,  1,  5,  2,  5,  8,  2,  8,  11, 4,  8,  5,  -1, -1, -1, -1},
         {10, 3,  11, 10, 1,  3,  9,  5,  4,  -1, -1, -1, -1, -1, -1, -1},
         {4,  9,  5,  0,  8,  1,  8,  10, 1,  8,  11, 10, -1, -1, -1, -1},
         {5,  4,  0,  5,  0,  11, 5,  11, 10, 11, 0,  3,  -1, -1, -1, -1},
         {5,  4,  8,  5,  8,  10, 10, 8,  11, -1, -1, -1, -1, -1, -1, -1},
         {9,  7,  8,  5,  7,  9,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9,  3,  0,  9,  5,  3,  5,  7,  3,  -1, -1, -1, -1, -1, -1, -1},
         {0,  7,  8,  0,  1,  7,  1,  5,  7,  -1, -1, -1, -1, -1, -1, -1},
         {1,  5,  3,  3,  5,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9,  7,  8,  9,  5,  7,  10, 1,  2,  -1, -1, -1, -1, -1, -1, -1},
         {10, 1,  2,  9,  5,  0,  5,  3,  0,  5,  7,  3,  -1, -1, -1, -1},
         {8,  0,  2,  8,  2,  5,  8,  5,  7,  10, 5,  2,  -1, -1, -1, -1},
         {2,  10, 5,  2,  5,  3,  3,  5,  7,  -1, -1, -1, -1, -1, -1, -1},
         {7,  9,  5,  7,  8,  9,  3,  11, 2,  -1, -1, -1, -1, -1, -1, -1},
         {9,  5,  7,  9,  7,  2,  9,  2,  0,  2,  7,  11, -1, -1, -1, -1},
         {2,  3,  11, 0,  1,  8,  1,  7,  8,  1,  5,  7,  -1, -1, -1, -1},
         {11, 2,  1,  11, 1,  7,  7,  1,  5,  -1, -1, -1, -1, -1, -1, -1},
         {9,  5,  8,  8,  5,  7,  10, 1,  3,  10, 3,  11, -1, -1, -1, -1},
         {5,  7,  0,  5,  0,  9,  7,  11, 0,  1,  0,  10, 11, 10, 0,  -1},
         {11, 10, 0,  11, 0,  3,  10, 5,  0,  8,  0,  7,  5,  7,  0,  -1},
         {11, 10, 5,  7,  11, 5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {10, 6,  5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  8,  3,  5,  10, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9,  0,  1,  5,  10, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  8,  3,  1,  9,  8,  5,  10, 6,  -1, -1, -1, -1, -1, -1, -1},
         {1,  6,  5,  2,  6,  1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  6,  5,  1,  2,  6,  3,  0,  8,  -1, -1, -1, -1, -1, -1, -1},
         {9,  6,  5,  9,  0,  6,  0,  2,  6,  -1, -1, -1, -1, -1, -1, -1},
         {5,  9,  8,  5,  8,  2,  5,  2,  6,  3,  2,  8,  -1, -1, -1, -1},
         {2,  3,  11, 10, 6,  5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {11, 0,  8,  11, 2,  0,  10, 6,  5,  -1, -1, -1, -1, -1, -1, -1},
         {0,  1,  9,  2,  3,  11, 5,  10, 6,  -1, -1, -1, -1, -1, -1, -1},
         {5,  10, 6,  1,  9,  2,  9,  11, 2,  9,  8,  11, -1, -1, -1, -1},
         {6,  3,  11, 6,  5,  3,  5,  1,  3,  -1, -1, -1, -1, -1, -1, -1},
         {0,  8,  11, 0,  11, 5,  0,  5,  1,  5,  11, 6,  -1, -1, -1, -1},
         {3,  11, 6,  0,  3,  6,  0,  6,  5,  0,  5,  9,  -1, -1, -1, -1},
         {6,  5,  9,  6,  9,  11, 11, 9,  8,  -1, -1, -1, -1, -1, -1, -1},
         {5,  10, 6,  4,  7,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4,  3,  0,  4,  7,  3,  6,  5,  10, -1, -1, -1, -1, -1, -1, -1},
         {1,  9,  0,  5,  10, 6,  8,  4,  7,  -1, -1, -1, -1, -1, -1, -1},
         {10, 6,  5,  1,  9,  7,  1,  7,  3,  7,  9,  4,  -1, -1, -1, -1},
         {6,  1,  2,  6,  5,  1,  4,  7,  8,  -1, -1, -1, -1, -1, -1, -1},
         {1,  2,  5,  5,  2,  6,  3,  0,  4,  3,  4,  7,  -1, -1, -1, -1},
         {8,  4,  7,  9,  0,  5,  0,  6,  5,  0,  2,  6,  -1, -1, -1, -1},
         {7,  3,  9,  7,  9,  4,  3,  2,  9,  5,  9,  6,  2,  6,  9,  -1},
         {3,  11, 2,  7,  8,  4,  10, 6,  5,  -1, -1, -1, -1, -1, -1, -1},
         {5,  10, 6,  4,  7,  2,  4,  2,  0,  2,  7,  11, -1, -1, -1, -1},
         {0,  1,  9,  4,  7,  8,  2,  3,  11, 5,  10, 6,  -1, -1, -1, -1},
         {9,  2,  1,  9,  11, 2,  9,  4,  11, 7,  11, 4,  5,  10, 6,  -1},
         {8,  4,  7,  3,  11, 5,  3,  5,  1,  5,  11, 6,  -1, -1, -1, -1},
         {5,  1,  11, 5,  11, 6,  1,  0,  11, 7,  11, 4,  0,  4,  11, -1},
         {0,  5,  9,  0,  6,  5,  0,  3,  6,  11, 6,  3,  8,  4,  7,  -1},
         {6,  5,  9,  6,  9,  11, 4,  7,  9,  7,  11, 9,  -1, -1, -1, -1},
         {10, 4,  9,  6,  4,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4,  10, 6,  4,  9,  10, 0,  8,  3,  -1, -1, -1, -1, -1, -1, -1},
         {10, 0,  1,  10, 6,  0,  6,  4,  0,  -1, -1, -1, -1, -1, -1, -1},
         {8,  3,  1,  8,  1,  6,  8,  6,  4,  6,  1,  10, -1, -1, -1, -1},
         {1,  4,  9,  1,  2,  4,  2,  6,  4,  -1, -1, -1, -1, -1, -1, -1},
         {3,  0,  8,  1,  2,  9,  2,  4,  9,  2,  6,  4,  -1, -1, -1, -1},
         {0,  2,  4,  4,  2,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {8,  3,  2,  8,  2,  4,  4,  2,  6,  -1, -1, -1, -1, -1, -1, -1},
         {10, 4,  9,  10, 6,  4,  11, 2,  3,  -1, -1, -1, -1, -1, -1, -1},
         {0,  8,  2,  2,  8,  11, 4,  9,  10, 4,  10, 6,  -1, -1, -1, -1},
         {3,  11, 2,  0,  1,  6,  0,  6,  4,  6,  1,  10, -1, -1, -1, -1},
         {6,  4,  1,  6,  1,  10, 4,  8,  1,  2,  1,  11, 8,  11, 1,  -1},
         {9,  6,  4,  9,  3,  6,  9,  1,  3,  11, 6,  3,  -1, -1, -1, -1},
         {8,  11, 1,  8,  1,  0,  11, 6,  1,  9,  1,  4,  6,  4,  1,  -1},
         {3,  11, 6,  3,  6,  0,  0,  6,  4,  -1, -1, -1, -1, -1, -1, -1},
         {6,  4,  8,  11, 6,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {7,  10, 6,  7,  8,  10, 8,  9,  10, -1, -1, -1, -1, -1, -1, -1},
         {0,  7,  3,  0,  10, 7,  0,  9,  10, 6,  7,  10, -1, -1, -1, -1},
         {10, 6,  7,  1,  10, 7,  1,  7,  8,  1,  8,  0,  -1, -1, -1, -1},
         {10, 6,  7,  10, 7,  1,  1,  7,  3,  -1, -1, -1, -1, -1, -1, -1},
         {1,  2,  6,  1,  6,  8,  1,  8,  9,  8,  6,  7,  -1, -1, -1, -1},
         {2,  6,  9,  2,  9,  1,  6,  7,  9,  0,  9,  3,  7,  3,  9,  -1},
         {7,  8,  0,  7,  0,  6,  6,  0,  2,  -1, -1, -1, -1, -1, -1, -1},
         {7,  3,  2,  6,  7,  2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2,  3,  11, 10, 6,  8,  10, 8,  9,  8,  6,  7,  -1, -1, -1, -1},
         {2,  0,  7,  2,  7,  11, 0,  9,  7,  6,  7,  10, 9,  10, 7,  -1},
         {1,  8,  0,  1,  7,  8,  1,  10, 7,  6,  7,  10, 2,  3,  11, -1},
         {11, 2,  1,  11, 1,  7,  10, 6,  1,  6,  7,  1,  -1, -1, -1, -1},
         {8,  9,  6,  8,  6,  7,  9,  1,  6,  11, 6,  3,  1,  3,  6,  -1},
         {0,  9,  1,  11, 6,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {7,  8,  0,  7,  0,  6,  3,  11, 0,  11, 6,  0,  -1, -1, -1, -1},
         {7,  11, 6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {7,  6,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3,  0,  8,  11, 7,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  1,  9,  11, 7,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {8,  1,  9,  8,  3,  1,  11, 7,  6,  -1, -1, -1, -1, -1, -1, -1},
         {10, 1,  2,  6,  11, 7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  2,  10, 3,  0,  8,  6,  11, 7,  -1, -1, -1, -1, -1, -1, -1},
         {2,  9,  0,  2,  10, 9,  6,  11, 7,  -1, -1, -1, -1, -1, -1, -1},
         {6,  11, 7,  2,  10, 3,  10, 8,  3,  10, 9,  8,  -1, -1, -1, -1},
         {7,  2,  3,  6,  2,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {7,  0,  8,  7,  6,  0,  6,  2,  0,  -1, -1, -1, -1, -1, -1, -1},
         {2,  7,  6,  2,  3,  7,  0,  1,  9,  -1, -1, -1, -1, -1, -1, -1},
         {1,  6,  2,  1,  8,  6,  1,  9,  8,  8,  7,  6,  -1, -1, -1, -1},
         {10, 7,  6,  10, 1,  7,  1,  3,  7,  -1, -1, -1, -1, -1, -1, -1},
         {10, 7,  6,  1,  7,  10, 1,  8,  7,  1,  0,  8,  -1, -1, -1, -1},
         {0,  3,  7,  0,  7,  10, 0,  10, 9,  6,  10, 7,  -1, -1, -1, -1},
         {7,  6,  10, 7,  10, 8,  8,  10, 9,  -1, -1, -1, -1, -1, -1, -1},
         {6,  8,  4,  11, 8,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3,  6,  11, 3,  0,  6,  0,  4,  6,  -1, -1, -1, -1, -1, -1, -1},
         {8,  6,  11, 8,  4,  6,  9,  0,  1,  -1, -1, -1, -1, -1, -1, -1},
         {9,  4,  6,  9,  6,  3,  9,  3,  1,  11, 3,  6,  -1, -1, -1, -1},
         {6,  8,  4,  6,  11, 8,  2,  10, 1,  -1, -1, -1, -1, -1, -1, -1},
         {1,  2,  10, 3,  0,  11, 0,  6,  11, 0,  4,  6,  -1, -1, -1, -1},
         {4,  11, 8,  4,  6,  11, 0,  2,  9,  2,  10, 9,  -1, -1, -1, -1},
         {10, 9,  3,  10, 3,  2,  9,  4,  3,  11, 3,  6,  4,  6,  3,  -1},
         {8,  2,  3,  8,  4,  2,  4,  6,  2,  -1, -1, -1, -1, -1, -1, -1},
         {0,  4,  2,  4,  6,  2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  9,  0,  2,  3,  4,  2,  4,  6,  4,  3,  8,  -1, -1, -1, -1},
         {1,  9,  4,  1,  4,  2,  2,  4,  6,  -1, -1, -1, -1, -1, -1, -1},
         {8,  1,  3,  8,  6,  1,  8,  4,  6,  6,  10, 1,  -1, -1, -1, -1},
         {10, 1,  0,  10, 0,  6,  6,  0,  4,  -1, -1, -1, -1, -1, -1, -1},
         {4,  6,  3,  4,  3,  8,  6,  10, 3,  0,  3,  9,  10, 9,  3,  -1},
         {10, 9,  4,  6,  10, 4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4,  9,  5,  7,  6,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  8,  3,  4,  9,  5,  11, 7,  6,  -1, -1, -1, -1, -1, -1, -1},
         {5,  0,  1,  5,  4,  0,  7,  6,  11, -1, -1, -1, -1, -1, -1, -1},
         {11, 7,  6,  8,  3,  4,  3,  5,  4,  3,  1,  5,  -1, -1, -1, -1},
         {9,  5,  4,  10, 1,  2,  7,  6,  11, -1, -1, -1, -1, -1, -1, -1},
         {6,  11, 7,  1,  2,  10, 0,  8,  3,  4,  9,  5,  -1, -1, -1, -1},
         {7,  6,  11, 5,  4,  10, 4,  2,  10, 4,  0,  2,  -1, -1, -1, -1},
         {3,  4,  8,  3,  5,  4,  3,  2,  5,  10, 5,  2,  11, 7,  6,  -1},
         {7,  2,  3,  7,  6,  2,  5,  4,  9,  -1, -1, -1, -1, -1, -1, -1},
         {9,  5,  4,  0,  8,  6,  0,  6,  2,  6,  8,  7,  -1, -1, -1, -1},
         {3,  6,  2,  3,  7,  6,  1,  5,  0,  5,  4,  0,  -1, -1, -1, -1},
         {6,  2,  8,  6,  8,  7,  2,  1,  8,  4,  8,  5,  1,  5,  8,  -1},
         {9,  5,  4,  10, 1,  6,  1,  7,  6,  1,  3,  7,  -1, -1, -1, -1},
         {1,  6,  10, 1,  7,  6,  1,  0,  7,  8,  7,  0,  9,  5,  4,  -1},
         {4,  0,  10, 4,  10, 5,  0,  3,  10, 6,  10, 7,  3,  7,  10, -1},
         {7,  6,  10, 7,  10, 8,  5,  4,  10, 4,  8,  10, -1, -1, -1, -1},
         {6,  9,  5,  6,  11, 9,  11, 8,  9,  -1, -1, -1, -1, -1, -1, -1},
         {3,  6,  11, 0,  6,  3,  0,  5,  6,  0,  9,  5,  -1, -1, -1, -1},
         {0,  11, 8,  0,  5,  11, 0,  1,  5,  5,  6,  11, -1, -1, -1, -1},
         {6,  11, 3,  6,  3,  5,  5,  3,  1,  -1, -1, -1, -1, -1, -1, -1},
         {1,  2,  10, 9,  5,  11, 9,  11, 8,  11, 5,  6,  -1, -1, -1, -1},
         {0,  11, 3,  0,  6,  11, 0,  9,  6,  5,  6,  9,  1,  2,  10, -1},
         {11, 8,  5,  11, 5,  6,  8,  0,  5,  10, 5,  2,  0,  2,  5,  -1},
         {6,  11, 3,  6,  3,  5,  2,  10, 3,  10, 5,  3,  -1, -1, -1, -1},
         {5,  8,  9,  5,  2,  8,  5,  6,  2,  3,  8,  2,  -1, -1, -1, -1},
         {9,  5,  6,  9,  6,  0,  0,  6,  2,  -1, -1, -1, -1, -1, -1, -1},
         {1,  5,  8,  1,  8,  0,  5,  6,  8,  3,  8,  2,  6,  2,  8,  -1},
         {1,  5,  6,  2,  1,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  3,  6,  1,  6,  10, 3,  8,  6,  5,  6,  9,  8,  9,  6,  -1},
         {10, 1,  0,  10, 0,  6,  9,  5,  0,  5,  6,  0,  -1, -1, -1, -1},
         {0,  3,  8,  5,  6,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {10, 5,  6,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {11, 5,  10, 7,  5,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {11, 5,  10, 11, 7,  5,  8,  3,  0,  -1, -1, -1, -1, -1, -1, -1},
         {5,  11, 7,  5,  10, 11, 1,  9,  0,  -1, -1, -1, -1, -1, -1, -1},
         {10, 7,  5,  10, 11, 7,  9,  8,  1,  8,  3,  1,  -1, -1, -1, -1},
         {11, 1,  2,  11, 7,  1,  7,  5,  1,  -1, -1, -1, -1, -1, -1, -1},
         {0,  8,  3,  1,  2,  7,  1,  7,  5,  7,  2,  11, -1, -1, -1, -1},
         {9,  7,  5,  9,  2,  7,  9,  0,  2,  2,  11, 7,  -1, -1, -1, -1},
         {7,  5,  2,  7,  2,  11, 5,  9,  2,  3,  2,  8,  9,  8,  2,  -1},
         {2,  5,  10, 2,  3,  5,  3,  7,  5,  -1, -1, -1, -1, -1, -1, -1},
         {8,  2,  0,  8,  5,  2,  8,  7,  5,  10, 2,  5,  -1, -1, -1, -1},
         {9,  0,  1,  5,  10, 3,  5,  3,  7,  3,  10, 2,  -1, -1, -1, -1},
         {9,  8,  2,  9,  2,  1,  8,  7,  2,  10, 2,  5,  7,  5,  2,  -1},
         {1,  3,  5,  3,  7,  5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  8,  7,  0,  7,  1,  1,  7,  5,  -1, -1, -1, -1, -1, -1, -1},
         {9,  0,  3,  9,  3,  5,  5,  3,  7,  -1, -1, -1, -1, -1, -1, -1},
         {9,  8,  7,  5,  9,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {5,  8,  4,  5,  10, 8,  10, 11, 8,  -1, -1, -1, -1, -1, -1, -1},
         {5,  0,  4,  5,  11, 0,  5,  10, 11, 11, 3,  0,  -1, -1, -1, -1},
         {0,  1,  9,  8,  4,  10, 8,  10, 11, 10, 4,  5,  -1, -1, -1, -1},
         {10, 11, 4,  10, 4,  5,  11, 3,  4,  9,  4,  1,  3,  1,  4,  -1},
         {2,  5,  1,  2,  8,  5,  2,  11, 8,  4,  5,  8,  -1, -1, -1, -1},
         {0,  4,  11, 0,  11, 3,  4,  5,  11, 2,  11, 1,  5,  1,  11, -1},
         {0,  2,  5,  0,  5,  9,  2,  11, 5,  4,  5,  8,  11, 8,  5,  -1},
         {9,  4,  5,  2,  11, 3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2,  5,  10, 3,  5,  2,  3,  4,  5,  3,  8,  4,  -1, -1, -1, -1},
         {5,  10, 2,  5,  2,  4,  4,  2,  0,  -1, -1, -1, -1, -1, -1, -1},
         {3,  10, 2,  3,  5,  10, 3,  8,  5,  4,  5,  8,  0,  1,  9,  -1},
         {5,  10, 2,  5,  2,  4,  1,  9,  2,  9,  4,  2,  -1, -1, -1, -1},
         {8,  4,  5,  8,  5,  3,  3,  5,  1,  -1, -1, -1, -1, -1, -1, -1},
         {0,  4,  5,  1,  0,  5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {8,  4,  5,  8,  5,  3,  9,  0,  5,  0,  3,  5,  -1, -1, -1, -1},
         {9,  4,  5,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4,  11, 7,  4,  9,  11, 9,  10, 11, -1, -1, -1, -1, -1, -1, -1},
         {0,  8,  3,  4,  9,  7,  9,  11, 7,  9,  10, 11, -1, -1, -1, -1},
         {1,  10, 11, 1,  11, 4,  1,  4,  0,  7,  4,  11, -1, -1, -1, -1},
         {3,  1,  4,  3,  4,  8,  1,  10, 4,  7,  4,  11, 10, 11, 4,  -1},
         {4,  11, 7,  9,  11, 4,  9,  2,  11, 9,  1,  2,  -1, -1, -1, -1},
         {9,  7,  4,  9,  11, 7,  9,  1,  11, 2,  11, 1,  0,  8,  3,  -1},
         {11, 7,  4,  11, 4,  2,  2,  4,  0,  -1, -1, -1, -1, -1, -1, -1},
         {11, 7,  4,  11, 4,  2,  8,  3,  4,  3,  2,  4,  -1, -1, -1, -1},
         {2,  9,  10, 2,  7,  9,  2,  3,  7,  7,  4,  9,  -1, -1, -1, -1},
         {9,  10, 7,  9,  7,  4,  10, 2,  7,  8,  7,  0,  2,  0,  7,  -1},
         {3,  7,  10, 3,  10, 2,  7,  4,  10, 1,  10, 0,  4,  0,  10, -1},
         {1,  10, 2,  8,  7,  4,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4,  9,  1,  4,  1,  7,  7,  1,  3,  -1, -1, -1, -1, -1, -1, -1},
         {4,  9,  1,  4,  1,  7,  0,  8,  1,  8,  7,  1,  -1, -1, -1, -1},
         {4,  0,  3,  7,  4,  3,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {4,  8,  7,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {9,  10, 8,  10, 11, 8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3,  0,  9,  3,  9,  11, 11, 9,  10, -1, -1, -1, -1, -1, -1, -1},
         {0,  1,  10, 0,  10, 8,  8,  10, 11, -1, -1, -1, -1, -1, -1, -1},
         {3,  1,  10, 11, 3,  10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  2,  11, 1,  11, 9,  9,  11, 8,  -1, -1, -1, -1, -1, -1, -1},
         {3,  0,  9,  3,  9,  11, 1,  2,  9,  2,  11, 9,  -1, -1, -1, -1},
         {0,  2,  11, 8,  0,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {3,  2,  11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2,  3,  8,  2,  8,  10, 10, 8,  9,  -1, -1, -1, -1, -1, -1, -1},
         {9,  10, 2,  0,  9,  2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {2,  3,  8,  2,  8,  10, 0,  1,  8,  1,  10, 8,  -1, -1, -1, -1},
         {1,  10, 2,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {1,  3,  8,  9,  1,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  9,  1,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {0,  3,  8,  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
         {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};

/*
   Given a grid cell and an isolevel, calculate the triangular
   facets required to represent the isosurface through the cell.
   Return the number of triangular facets, the array "triangles"
   will be loaded up with the vertices at most 5 triangular facets.
	0 will be returned if the grid cell is either totally above
   of totally below the isolevel.
*/
int MarchingCubes::Polygonise(GRIDCELL &grid, TRIANGLE *triangles) {
    int i, ntriang;
    int cubeindex;
    XYZ vertlist[12];
    XYZ normlist[12];

    /*
       Determine the index into the edge table which
       tells us which vertices are inside of the surface
    */
    cubeindex = 0;
    if (grid.p[0].a < iso) cubeindex |= 1;
    if (grid.p[1].a < iso) cubeindex |= 2;
    if (grid.p[2].a < iso) cubeindex |= 4;
    if (grid.p[3].a < iso) cubeindex |= 8;
    if (grid.p[4].a < iso) cubeindex |= 16;
    if (grid.p[5].a < iso) cubeindex |= 32;
    if (grid.p[6].a < iso) cubeindex |= 64;
    if (grid.p[7].a < iso) cubeindex |= 128;

    /* Cube is entirely in/out of the surface */
    if (edgeTable[cubeindex] == 0)
        return (0);

    /* Find the vertices where the surface intersects the cube */
    if (edgeTable[cubeindex] & 1) {
        vertlist[0] =
                VertexInterp(iso, grid.p[0].v, grid.p[1].v, grid.p[0].a, grid.p[1].a);
        normlist[0] =
                VertexInterpNormal(iso, grid.n[0], grid.n[1], grid.p[0].a, grid.p[1].a);
    }
    if (edgeTable[cubeindex] & 2) {
        vertlist[1] =
                VertexInterp(iso, grid.p[1].v, grid.p[2].v, grid.p[1].a, grid.p[2].a);
        normlist[1] =
                VertexInterpNormal(iso, grid.n[1], grid.n[2], grid.p[1].a, grid.p[2].a);
    }
    if (edgeTable[cubeindex] & 4) {
        vertlist[2] =
                VertexInterp(iso, grid.p[2].v, grid.p[3].v, grid.p[2].a, grid.p[3].a);
        normlist[2] =
                VertexInterpNormal(iso, grid.n[2], grid.n[3], grid.p[2].a, grid.p[3].a);
    }
    if (edgeTable[cubeindex] & 8) {
        vertlist[3] =
                VertexInterp(iso, grid.p[3].v, grid.p[0].v, grid.p[3].a, grid.p[0].a);
        normlist[3] =
                VertexInterpNormal(iso, grid.n[3], grid.n[0], grid.p[3].a, grid.p[0].a);
    }
    if (edgeTable[cubeindex] & 16) {
        vertlist[4] =
                VertexInterp(iso, grid.p[4].v, grid.p[5].v, grid.p[4].a, grid.p[5].a);
        normlist[4] =
                VertexInterpNormal(iso, grid.n[4], grid.n[5], grid.p[4].a, grid.p[5].a);
    }
    if (edgeTable[cubeindex] & 32) {
        vertlist[5] =
                VertexInterp(iso, grid.p[5].v, grid.p[6].v, grid.p[5].a, grid.p[6].a);
        normlist[5] =
                VertexInterpNormal(iso, grid.n[5], grid.n[6], grid.p[5].a, grid.p[6].a);
    }
    if (edgeTable[cubeindex] & 64) {
        vertlist[6] =
                VertexInterp(iso, grid.p[6].v, grid.p[7].v, grid.p[6].a, grid.p[7].a);
        normlist[6] =
                VertexInterpNormal(iso, grid.n[6], grid.n[7], grid.p[6].a, grid.p[7].a);
    }
    if (edgeTable[cubeindex] & 128) {
        vertlist[7] =
                VertexInterp(iso, grid.p[7].v, grid.p[4].v, grid.p[7].a, grid.p[4].a);
        normlist[7] =
                VertexInterpNormal(iso, grid.n[7], grid.n[4], grid.p[7].a, grid.p[4].a);
    }
    if (edgeTable[cubeindex] & 256) {
        vertlist[8] =
                VertexInterp(iso, grid.p[0].v, grid.p[4].v, grid.p[0].a, grid.p[4].a);
        normlist[8] =
                VertexInterpNormal(iso, grid.n[0], grid.n[4], grid.p[0].a, grid.p[4].a);
    }
    if (edgeTable[cubeindex] & 512) {
        vertlist[9] =
                VertexInterp(iso, grid.p[1].v, grid.p[5].v, grid.p[1].a, grid.p[5].a);
        normlist[9] =
                VertexInterpNormal(iso, grid.n[1], grid.n[5], grid.p[1].a, grid.p[5].a);
    }
    if (edgeTable[cubeindex] & 1024) {
        vertlist[10] =
                VertexInterp(iso, grid.p[2].v, grid.p[6].v, grid.p[2].a, grid.p[6].a);
        normlist[10] =
                VertexInterpNormal(iso, grid.n[2], grid.n[6], grid.p[2].a, grid.p[6].a);
    }
    if (edgeTable[cubeindex] & 2048) {
        vertlist[11] =
                VertexInterp(iso, grid.p[3].v, grid.p[7].v, grid.p[3].a, grid.p[7].a);
        normlist[11] =
                VertexInterpNormal(iso, grid.n[3], grid.n[7], grid.p[3].a, grid.p[7].a);
    }

    /* Create the triangle */
    ntriang = 0;
    for (i = 0; triTable[cubeindex][i] != -1; i += 3) {
        triangles[ntriang].p[0] = vertlist[triTable[cubeindex][i]];
        triangles[ntriang].p[1] = vertlist[triTable[cubeindex][i + 1]];
        triangles[ntriang].p[2] = vertlist[triTable[cubeindex][i + 2]];
        triangles[ntriang].n[0] = normlist[triTable[cubeindex][i]];
        triangles[ntriang].n[1] = normlist[triTable[cubeindex][i + 1]];
        triangles[ntriang].n[2] = normlist[triTable[cubeindex][i + 2]];
        ntriang++;
    }

    return (ntriang);
}

bool operator<(const Vector3f &left, const Vector3f &right) {
    if (left[0] < right[0])
        return true;
    else if (left[0] > right[0])
        return false;

    if (left[1] < right[1])
        return true;
    else if (left[1] > right[1])
        return false;

    if (left[2] < right[2])
        return true;
    else if (left[2] > right[2])
        return false;

    return false;
}

/*
   Linearly interpolate the position where an isosurface cuts
   an edge between two vertices, each with their own scalar value
*/
MarchingCubes::XYZ MarchingCubes::VertexInterp(float isolevel, MarchingCubes::XYZ &p1, MarchingCubes::XYZ &p2,
                                               float valp1, float valp2) {
    float mu;
    XYZ p;

    if (fabs(isolevel - valp1) < 0.00001)
        return (p1);
    if (fabs(isolevel - valp2) < 0.00001)
        return (p2);
    if (fabs(valp1 - valp2) < 0.00001)
        return (p1);
    mu = (isolevel - valp1) / (valp2 - valp1);

    p = p1 + (p2 - p1).mult(mu);

    return p;
}

MarchingCubes::XYZ MarchingCubes::VertexInterpNormal(float isolevel, MarchingCubes::XYZ &np1, MarchingCubes::XYZ &np2,
                                                     float valp1, float valp2) {
    float mu;
    XYZ p;

    if (fabs(isolevel - valp1) < 0.00001)
        return (np1);
    if (fabs(isolevel - valp2) < 0.00001)
        return (np2);
    if (fabs(valp1 - valp2) < 0.00001)
        return (np1);
    mu = (isolevel - valp1) / (valp2 - valp1);

    p = np1.mult(1.f - mu) + np2.mult(mu);

    return p;
}

/* DEPRECATED
void MarchingCubes::updateGradient(int index) {
    //only update normals if center index is within cube grid
    if (0 <= index && index < size) {
        //for each neighbouring grid cell
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                for (int z = -1; z <= 1; z++) {
                    // only neighbouring cells though, not the cell itself
                    if (abs(x) + abs(y) + abs(z) == 1) {
                        //calculate grid cell index
                        int centerIndex = index + x + cubeCornerDim[0] * (y + cubeCornerDim[1] * z);
                        //only update normals if grid cell is within cube grid
                        if (0 <= centerIndex && centerIndex < size) {
                            //get all the indices of grid cell-neighbouring cells, setting them to the cell
                            // itself when they are outside the cube grid
                            int plusXIndex = index + 1;
                            plusXIndex = plusXIndex < size ? plusXIndex : index;
                            int minusXIndex = index - 1;
                            minusXIndex = minusXIndex >= 0 ? minusXIndex : index;
                            int plusYIndex = index + cubeCornerDim[0];
                            plusYIndex = plusYIndex < size ? plusYIndex : index;
                            int minusYIndex = index - cubeCornerDim[0];
                            minusYIndex = minusYIndex >= 0 ? minusYIndex : index;
                            int plusZIndex = index + cubeCornerDim[0] * cubeCornerDim[1];
                            plusZIndex = plusZIndex < size ? plusZIndex : index;
                            int minusZIndex = index - cubeCornerDim[0] * cubeCornerDim[1];
                            minusZIndex = minusZIndex >= 0 ? minusZIndex : index;

                            //calculate gradient from neighbouring cell difference
                            float dx = cubeCorners[plusXIndex] - cubeCorners[minusXIndex];
                            float dy = cubeCorners[plusYIndex] - cubeCorners[minusYIndex];
                            float dz = cubeCorners[plusZIndex] - cubeCorners[minusZIndex];
                            //save gradient
                            gradientCorners[index] = XYZ{dx, dy, dz};
                        }
                    }
                }
            }
        }
    }
}
*/
/* DEPRECATED
MarchingCubes::XYZ MarchingCubes::getEdgeNormal(MarchingCubes::XYZ edgePoint) {
    XYZ edgeStart = XYZ{floorf(edgePoint[0] / cubeStep) * cubeStep,
                        floorf(edgePoint[1] / cubeStep) * cubeStep,
                        floorf(edgePoint[2] / cubeStep) * cubeStep};

    XYZ edgeEnd = XYZ{ceilf(edgePoint[0] / cubeStep) * cubeStep,
                      ceilf(edgePoint[1] / cubeStep) * cubeStep,
                      ceilf(edgePoint[2] / cubeStep) * cubeStep};

    //edge should fall inside of grid
    if (cubeStart < edgeStart && edgeStart < cubeEnd
        && cubeStart < edgeEnd && edgeEnd < cubeEnd) {
        // get normal at edge start
        XYZ relEdgeStart = edgeStart - cubeStart;
        int edgeStartGridPos[3] = {(int) lroundf(relEdgeStart[0] / cubeStep),
                                   (int) lroundf(relEdgeStart[1] / cubeStep),
                                   (int) lroundf(relEdgeStart[2] / cubeStep)};
        int edgeStartIndex = edgeStartGridPos[0] + cubeCornerDim[0] * (edgeStartGridPos[1] + (cubeCornerDim[1] * edgeStartGridPos[2]));
        XYZ startNormal = gradientCorners[edgeStartIndex];

        // get normal at edge end
        XYZ relEdgeEnd = edgeEnd - cubeStart;
        int edgeEndGridPos[3] = {(int) lroundf(relEdgeEnd[0] / cubeStep),
                                 (int) lroundf(relEdgeEnd[1] / cubeStep),
                                 (int) lroundf(relEdgeEnd[2] / cubeStep)};
        int edgeEndIndex = edgeEndGridPos[0] + cubeCornerDim[0] * (edgeEndGridPos[1] + (cubeCornerDim[1] * edgeEndGridPos[2]));
        XYZ endNormal = gradientCorners[edgeEndIndex];

        // return average of start end end normal
        XYZ normal = (startNormal + endNormal).div(2.f);
        normal.normalize();
        return normal;
    } else {
        return XYZ{0.f, 0.f, 0.f};
    }
}
*/

void MarchingCubes::drawMarching() {

    for (int i = 0; i < size; i++) {
        cubeCorners[i] = XYZA{XYZ{.0f, .0f, .0f}, .0f};
        //gradientCorners[i] = XYZ{0.f,0.f,0.f};
    }

    for (Particle *p: system->particles) {
        if (!p->movable) continue;
        Vector3f vecPos = p->position;
        XYZ pos = {vecPos[0], vecPos[1], vecPos[2]};
        // only apply marching cube to particle when it is inside the rendering volume
        if (pos[0] > cubeStart[0] && pos[1] > cubeStart[1] && pos[2] > cubeStart[2]
            && pos[0] < cubeEnd[0] && pos[1] < cubeEnd[1] && pos[2] < cubeEnd[2]) {
            // add the distance between the point and each of the 8 corners of its surrounding gridcube
            // to the grid values

            // lower corner [0, 0, 0] of gridcube
            XYZ lowerGridPos = XYZ{floor(pos[0] / cubeStep) * cubeStep,
                                   floor(pos[1] / cubeStep) * cubeStep,
                                   floor(pos[2] / cubeStep) * cubeStep};
            int lowerCubePos = (int) (floor(pos[0] / cubeStep) + -cubeStartInt[0] +
                                      cubeCornerDim[0] * (floor(pos[1] / cubeStep) + -cubeStartInt[1]
                                                          + (cubeCornerDim[1] *
                                                             (floor(pos[2] / cubeStep) + -cubeStartInt[2]))));

            //fill in all the gridcube values
            int steps = (int) ceilf(particleRange / cubeStep);
            for (int x = -steps; x <= steps + 1; x++) {
                for (int y = -steps; y <= steps + 1; y++) {
                    for (int z = -steps; z <= steps + 1; z++) {
                        XYZ gridPos = lowerGridPos + XYZ{x * cubeStep, y * cubeStep, z * cubeStep};
                        int cubePos = lowerCubePos + x + (cubeCornerDim[0] * (y + cubeCornerDim[1] * z));
                        if (cubePos < cubeCornerDim[0] * cubeCornerDim[1] * cubeCornerDim[0] && cubePos >= 0) {
                            XYZ relPos = pos - gridPos;
                            float relDist = relPos.size();
                            cubeCorners[cubePos].a = min(
                                    cubeCorners[cubePos].a +
                                    max(particleRange -  relDist, 0.f) / particleRange, 1.f);
                            cubeCorners[cubePos].v += relPos.normalize().mult(max(particleRange -  relDist, 0.f) / -particleRange);
                            // update gradients based on change in grid
                            // updateGradient(cubePos);
                        }
                    }
                }
            }
        }
    }

    triangles.clear();
    //normals.clear();

    for (int x = cubeStartInt[0]; x < cubeEndInt[0] - 1; x++) {
        for (int y = cubeStartInt[1]; y < cubeEndInt[1] - 1; y++) {
            for (int z = cubeStartInt[2]; z < cubeEndInt[2] - 1; z++) {
                int cubePos0 =
                        x + -cubeStartInt[0] + cubeCornerDim[0] * (y + -cubeStartInt[1] +
                                                                   (cubeCornerDim[1] * (z + -cubeStartInt[2])));
                int cubePos1 = cubePos0 + 1; // [1,0,0]
                int cubePos2 = cubePos0 + 1 + cubeCornerDim[0]; // [1,1,0]
                int cubePos3 = cubePos0 + cubeCornerDim[0]; // [0,1,0]
                int cubePos4 = cubePos0 + cubeCornerDim[0] * cubeCornerDim[1]; // [0,0,1]
                int cubePos5 = cubePos0 + 1 + cubeCornerDim[0] * cubeCornerDim[1]; // [1,0,1]
                int cubePos6 = cubePos0 + 1 + cubeCornerDim[0] + cubeCornerDim[0] * cubeCornerDim[1]; // [1,1,1]
                int cubePos7 = cubePos0 + cubeCornerDim[0] + cubeCornerDim[0] * cubeCornerDim[1]; // [0,1,1]

                GRIDCELL cell = {
                        {
                                XYZA{XYZ{x * cubeStep, y * cubeStep, z * cubeStep}, cubeCorners[cubePos0].a}, //[0,0,0]
                                XYZA{XYZ{x * cubeStep + cubeStep, y * cubeStep, z * cubeStep},
                                     cubeCorners[cubePos1].a}, //[1,0,0]
                                XYZA{XYZ{x * cubeStep + cubeStep, y * cubeStep + cubeStep, z * cubeStep},
                                     cubeCorners[cubePos2].a}, //[1,1,0]
                                XYZA{XYZ{x * cubeStep, y * cubeStep + cubeStep, z * cubeStep},
                                     cubeCorners[cubePos3].a}, //[0,1,0]
                                XYZA{XYZ{x * cubeStep, y * cubeStep, z * cubeStep + cubeStep},
                                     cubeCorners[cubePos4].a}, //[0,0,1]
                                XYZA{XYZ{x * cubeStep + cubeStep, y * cubeStep, z * cubeStep + cubeStep},
                                     cubeCorners[cubePos5].a}, //[1,0,1]
                                XYZA{XYZ{x * cubeStep + cubeStep, y * cubeStep + cubeStep, z * cubeStep + cubeStep},
                                     cubeCorners[cubePos6].a}, //[1,1,1]
                                XYZA{XYZ{x * cubeStep, y * cubeStep + cubeStep, z * cubeStep + cubeStep},
                                     cubeCorners[cubePos7].a} //[0,1,1]
                        }, {
                                cubeCorners[cubePos0].v,
                                cubeCorners[cubePos1].v,
                                cubeCorners[cubePos2].v,
                                cubeCorners[cubePos3].v,
                                cubeCorners[cubePos4].v,
                                cubeCorners[cubePos5].v,
                                cubeCorners[cubePos6].v,
                                cubeCorners[cubePos7].v,
                        }
                };

                double cellsum = 0;
                cellsum += cubeCorners[cubePos0].a;
                cellsum += cubeCorners[cubePos1].a;
                cellsum += cubeCorners[cubePos2].a;
                cellsum += cubeCorners[cubePos3].a;
                cellsum += cubeCorners[cubePos4].a;
                cellsum += cubeCorners[cubePos5].a;
                cellsum += cubeCorners[cubePos6].a;
                cellsum += cubeCorners[cubePos7].a;

                if (cellsum > 0) {
                    TRIANGLE tris[5] = {};
                    int n = Polygonise(cell, tris);

                    if (n > 0) {
                        for (int i = 0; i < n; i++) {
                            TRIANGLE tri = tris[i];
                            triangles.push_back(tri);

                            /* add normal to combined normals of each point in triangle. (per-vertex normals) DEPRECATED
                            XYZ a = tri.p[0];
                            XYZ b = tri.p[1];
                            XYZ c = tri.p[2];
                            XYZ norm = (b - a)^(c - b);
                            long akey = a.toLongHash();
                            if (!normals.emplace(akey, norm).second) {
                                normals[akey] += norm;
                            }
                            long bkey = b.toLongHash();
                            if (!normals.emplace(bkey, norm).second) {
                                normals[bkey] += norm;
                            }
                            long ckey = c.toLongHash();
                            if (!normals.emplace(ckey, norm).second) {
                                normals[ckey] += norm;
                            }
                            //*/
                        }
                    }
                }

                /* draw each grid cell DEBUG
                if (cubeCorners[cubePos0] > 0.f) {
                    glColor3f(cubeCorners[cubePos0], 0.f, 0.f);
                    glPushMatrix();
                    glTranslated(x*cubeStep, y*cubeStep, z*cubeStep);
                    glBegin(GL_POINTS);
                        glVertex3f(0.f, 0.f, 0.f);
                    glEnd();
                    glPopMatrix();
                }
                //*/
            }
        }
    }

    //// draw triangles
    if (system->type == SystemBuilder::SMOKE)
        glColor4f(0.9f, 0.9f, 0.9f, 0.2f);
    else
        glColor4f(0.1f, 0.9f, 0.9f, 1.0f);
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < triangles.size(); i++) {
        TRIANGLE triangle = triangles[i];
        XYZ a = triangle.p[0];
        XYZ b = triangle.p[1];
        XYZ c = triangle.p[2];
        //*/// per vertex normals
        XYZ anorm = triangle.n[0].normalize();
        XYZ bnorm = triangle.n[1].normalize();
        XYZ cnorm = triangle.n[2].normalize();
        /*/// per-face normals
        XYZ norm = (b - a)^(c - b);
        norm.normalize();
        XYZ anorm = norm;
        XYZ bnorm = norm;
        XYZ cnorm = norm;
        /* /// per vertex normals
        XYZ anorm = normals[a.toLongHash()];
        XYZ bnorm = normals[b.toLongHash()];
        XYZ cnorm = normals[c.toLongHash()];
        anorm.normalize();
        bnorm.normalize();
        cnorm.normalize();
        ///
        /* per vertex normals 2 DEPRECATED
        XYZ anorm = getEdgeNormal(a).mult(-1.f);
        XYZ bnorm = getEdgeNormal(b).mult(-1.f);
        XYZ cnorm = getEdgeNormal(c).mult(-1.f);
        //*/
        glNormal3f(anorm[0], anorm[1], anorm[2]);
        glVertex3f(a[0], a[1], a[2]);

        glNormal3f(bnorm[0], bnorm[1], bnorm[2]);
        glVertex3f(b[0], b[1], b[2]);

        glNormal3f(cnorm[0], cnorm[1], cnorm[2]);
        glVertex3f(c[0], c[1], c[2]);
    }
    glEnd();
    //*/
}

MarchingCubes::MarchingCubes(System *system) : system(system) {
    cubeStart = XYZ{-1.1f, -1.1f, -1.1f};
    cubeEnd = XYZ{1.1f, 1.1f, 1.1f};
    cubeStep = .0125f; // a whole number of steps should fit into interval

    cubeStartInt = new int[3];
    cubeEndInt = new int[3];
    cubeCornerDim = new int[3];
    for (int i = 0; i < 3; i++) {
        cubeStartInt[i] = (int) roundf(cubeStart[i] / cubeStep);
        cubeEndInt[i] = (int) roundf(cubeEnd[i] / cubeStep);
        cubeCornerDim[i] = cubeEndInt[0] - cubeStartInt[0] + 1;
    }
    size = cubeCornerDim[0] * cubeCornerDim[1] * cubeCornerDim[2];

    cubeCorners = new XYZA[size];
    //gradientCorners = new XYZ[size];

    for (int i = 0; i < size; i++) {
        cubeCorners[i] = XYZA{XYZ{0.f, 0.f, 0.f},0.f};
        //gradientCorners[i] = XYZ{0.f, 0.f, 0.f};
    }

    particleRange = .1f;
    iso = .3f;
    triangles = {};
    //normals = {};
}

MarchingCubes::~MarchingCubes() {
    delete cubeCorners;
    //delete gradientCorners;
    delete[] cubeStartInt;
    delete[] cubeEndInt;
    delete[] cubeCornerDim;
}
