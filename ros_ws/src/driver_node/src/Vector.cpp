#include "header/Vector.h"

Vector::Vector(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

Vector::Vector(double x, double y) {
    this->x = x;
    this->y = y;
    this->z = 0.0f;
}

double Vector::magnitude() {
    return sqrt(x * x + y * y + z * z);
}

Vector Vector::add(Vector vectorToAdd) {
    return Vector(this->x + vectorToAdd.x, this->y + vectorToAdd.y, this->z + vectorToAdd.z);
}

Vector Vector::sub(Vector vectorToSub) {
    return Vector(this->x - vectorToSub.x, this->y - vectorToSub.y, this->z - vectorToSub.z);
}

Vector Vector::mul(double value) {
    return Vector(this->x * value, this->y * value, this->z * value);
}

float radians(float direction) {
    return direction / 180.0f * M_PI;
}

void Vector::rotate(double direction) {
    double rad = radians(direction);
    double nx = cos(rad) * x - sin(rad) * y;
    double ny = cos(rad) * y + sin(rad) * x;
    x = nx;
    y = ny;
}

Vector Vector::normalize() {
    double mag = this->magnitude();
    return Vector(this->x / mag, this->y / mag, this->z / mag);
}
