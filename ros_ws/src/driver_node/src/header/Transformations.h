#pragma once
#include "Vector.h"
#include <algorithm>

using namespace std;

namespace Transformations {

    float radians(float direction);

    float degrees(float radians);

    Vector getVectorOfDirection(float Direction);

    float dotProduct(Vector v1, Vector v2);

    Vector crossProduct(Vector v1, Vector v2);

    float SignedAngleBetweenVectors(Vector v1, Vector v2, Vector vn);

    float SignedAngleBetweenVectorAndDirection(Vector v, float direction);

    float SignedAngleBetweenVectorAndDirection(Vector v, float direction, bool debug);
};
