#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <algorithm>

class Vector {
public:
    double x, y, z;

    Vector() {};

    Vector(double x, double y, double z);

    Vector(double x, double y);

    double magnitude();

    Vector add(Vector vectorToAdd);

    Vector sub(Vector vectorToSub);

    Vector mul(double value);

    void rotate(double direction);

    Vector normalize();
};
